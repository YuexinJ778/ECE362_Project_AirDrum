#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "imu_io.h"
#include "customize.h"
#include <stdlib.h>
#include "hardware/pwm.h"
#include "hardware/adc.h"  
#include "sound.h"

// ========= I2C / LSM6DSOX =========
#define I2C_PORT      i2c0
#define I2C_SDA_PIN   40
#define I2C_SCL_PIN   41
#define I2C_BAUD      (400 * 1000)
#define LSM6DSOX_ADDR           0x6A
#define LSM6DSOX_REG_WHO_AM_I   0x0F
#define LSM6DSOX_WHOAMI_VAL     0x6C
#define REG_CTRL1_XL            0x10
#define REG_CTRL2_G             0x11
#define REG_CTRL3_C             0x12
#define REG_OUTX_L_G            0x22
#define REG_OUTX_L_A            0x28

// ========= 采样/换算 =========
#define DT_MS               20
#define DT_SEC              (DT_MS/1000.0f)
#define ACC_LSB_TO_G        0.000061f     // ±2g
#define GYRO_LSB_TO_DPS     0.00875f      // 250 dps

// ========= 命中检测参数 =========
#define EMA_ALPHA              0.24f
#define MIN_PEAK_LSB           2800.0f
#define MIN_DROP_LSB           1100.0f
#define D_EPS                  1.2f
#define MIN_INTERVAL_SAMPLES   16
#define HIT_MIN_GAP_MS         220

// 命中需要一定下挥角速度
#define HIT_GY_DPS             100.0f   // 上排击打最低俯仰下挥
#define DOWN_PITCH_DPS         90.0f    // floor tom 最低下挥

// ========= 姿态（不做位移积分） =========
#define GHAT_ALPHA           0.03f    // ĝ 低通
#define YAW_LEAK             0.08f    // yaw 泄漏
#define UP_IS_POSITIVE       0        // 取 -Z 为“上”

// —— 现在做 2/3/4/5 + CYMBAL（6） —— //
// 倾角分层（结合你调过的数据）
#define FLOOR_TILT_MAX_DEG    18.0f   // floor：<=18°
#define TOM_TILT_MIN_DEG      26.0f   // 上排 tom：>=26°
#define CYMBAL_TILT_MIN_DEG   60.0f   // cymbal：抬得很高（>=60° 左右）

// 左右滞回（更容易进入左右槽）
#define YAW_ON_DEG             2.2f   // 进入左/右：|yaw| >= 2.2°
#define YAW_OFF_DEG            1.4f   // 回到中：  |yaw| <  1.4f

//gpio parameters
const uint audio_pin = 36;                
const uint pwm_slice_num = 10;           
const uint pwm_channel = PWM_CHAN_A;           
//PWM parameters
const float pwm_clk_div = 13.25f;          
const uint16_t pwm_wrap_value = 255;  
//ADC parameters
const uint adc_pin = 45;
const uint adc_input = 5;
const float adc_clk_div = 11.34f;
uint volume = 4095;

/* ==================== I2C helpers ==================== */
static inline int16_t u8pair_to_i16(uint8_t lo, uint8_t hi){ return (int16_t)((hi<<8)|lo); }
static void i2c_bus_init(void){
    i2c_init(I2C_PORT, I2C_BAUD);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN); gpio_pull_up(I2C_SCL_PIN);
}
static int i2c_write_reg_addr(uint8_t reg, bool nostop){ return i2c_write_blocking(I2C_PORT, LSM6DSOX_ADDR, &reg, 1, nostop); }
static int read_reg(uint8_t reg, uint8_t *val){
    if (i2c_write_reg_addr(reg, true) != 1) return PICO_ERROR_GENERIC;
    return (i2c_read_blocking(I2C_PORT, LSM6DSOX_ADDR, val, 1, false) == 1) ? 0 : PICO_ERROR_GENERIC;
}
static int write_reg(uint8_t reg, uint8_t val){
    uint8_t buf[2] = {reg, val};
    return (i2c_write_blocking(I2C_PORT, LSM6DSOX_ADDR, buf, 2, false) == 2) ? 0 : PICO_ERROR_GENERIC;
}
static bool probe_whoami(void){
    uint8_t who=0;
    if (read_reg(LSM6DSOX_REG_WHO_AM_I, &who)==0 && who==LSM6DSOX_WHOAMI_VAL) return true;
    printf("WHO_AM_I=0x%02X (expect 0x6C)\n", who);
    return false;
}
static void enable_odo_208hz(void){
    write_reg(REG_CTRL3_C, 0b01000100);   // IF_INC/BDU
    write_reg(REG_CTRL1_XL, 0b01100000);  // ACC 208Hz ±2g
    write_reg(REG_CTRL2_G,  0b01100000);  // GYR 208Hz 250dps
    sleep_ms(20);
}
void read_accel_raw(int16_t* ax,int16_t* ay,int16_t* az){
    uint8_t buf[6];
    i2c_write_reg_addr(REG_OUTX_L_A, true);
    i2c_read_blocking(I2C_PORT, LSM6DSOX_ADDR, buf, 6, false);
    if (ax) *ax = u8pair_to_i16(buf[0], buf[1]);
    if (ay) *ay = u8pair_to_i16(buf[2], buf[3]);
    if (az) *az = u8pair_to_i16(buf[4], buf[5]);
}
void read_gyro_raw (int16_t* gx,int16_t* gy,int16_t* gz){
    uint8_t buf[6];
    i2c_write_reg_addr(REG_OUTX_L_G, true);
    i2c_read_blocking(I2C_PORT, LSM6DSOX_ADDR, buf, 6, false);
    if (gx) *gx = u8pair_to_i16(buf[0], buf[1]);
    if (gy) *gy = u8pair_to_i16(buf[2], buf[3]);
    if (gz) *gz = u8pair_to_i16(buf[4], buf[5]);
}

/* ==================== 命中状态机 ==================== */
typedef struct{
    float ema,last,peak,trough;
    int   state,cooldown;
    bool  fired_in_descent;
} HitState;
enum{ SEARCH_PEAK=0, DESCENT=1 };
static inline void hit_init(HitState* s,float x0){
    s->ema=s->last=s->peak=s->trough=x0;
    s->state=SEARCH_PEAK;
    s->cooldown=0;
    s->fired_in_descent=false;
}
static inline bool hit_step(HitState* s,float x){
    s->ema=EMA_ALPHA*x+(1.0f-EMA_ALPHA)*s->ema;
    float dx=s->ema-s->last; s->last=s->ema;
    if(s->cooldown>0){ s->cooldown--; return false; }
    if(s->state==SEARCH_PEAK){
        if(s->ema>s->peak) s->peak=s->ema;
        if(s->peak>=MIN_PEAK_LSB && dx<-D_EPS){
            s->trough=s->ema;
            s->fired_in_descent=false;
            s->state=DESCENT;
        }
        return false;
    }else{
        if(s->ema<s->trough) s->trough=s->ema;
        if(!s->fired_in_descent && (s->peak-s->ema)>=MIN_DROP_LSB){
            s->fired_in_descent=true;
            s->cooldown=MIN_INTERVAL_SAMPLES;
            return true;
        }
        if(dx>D_EPS){
            s->peak=s->ema;
            s->state=SEARCH_PEAK;
        }
        return false;
    }
}

/* ==================== 姿态跟踪器 ==================== */
typedef struct { float x,y,z; } Vec3;
static Vec3 ghat = {0,0,1};
static float yaw_rel_deg = 0.0f;
static bool pose_inited = false;
static float acc_bias_[3]={0}, gyr_bias_[3]={0};
static float tilt_zero_deg = 0.0f;
static bool  tilt_zero_done = false;
static float gyro_y_dps_last = 0.0f;

// 全局命中计数
static uint32_t g_hit_count = 0;

static inline void pose_reset(void){
    ghat.x=0; ghat.y=0; ghat.z=1;
    yaw_rel_deg=0;
    pose_inited=false;
}

static inline void pose_update(){
    int16_t ax_i, ay_i, az_i, gx_i, gy_i, gz_i;
    read_accel_raw(&ax_i,&ay_i,&az_i);
    read_gyro_raw (&gx_i,&gy_i,&gz_i);

    float ax_g = ((float)ax_i - acc_bias_[0]) * ACC_LSB_TO_G;
    float ay_g = ((float)ay_i - acc_bias_[1]) * ACC_LSB_TO_G;
    float az_g = ((float)az_i - acc_bias_[2]) * ACC_LSB_TO_G;

    if(!pose_inited){ ghat.x=ax_g; ghat.y=ay_g; ghat.z=az_g; pose_inited=true; }
    ghat.x = (1.0f-GHAT_ALPHA)*ghat.x + GHAT_ALPHA*ax_g;
    ghat.y = (1.0f-GHAT_ALPHA)*ghat.y + GHAT_ALPHA*ay_g;
    ghat.z = (1.0f-GHAT_ALPHA)*ghat.z + GHAT_ALPHA*az_g;
    float n = sqrtf(ghat.x*ghat.x + ghat.y*ghat.y + ghat.z*ghat.z) + 1e-9f;
    ghat.x/=n; ghat.y/=n; ghat.z/=n;

    float gz_dps = ((float)gz_i - gyr_bias_[2]) * GYRO_LSB_TO_DPS;
    yaw_rel_deg = (1.0f - YAW_LEAK)*yaw_rel_deg + gz_dps*DT_SEC;

    // 俯仰角速度（下挥为正，按你装配的 y 轴方向）
    gyro_y_dps_last = ((float)gy_i - gyr_bias_[1]) * GYRO_LSB_TO_DPS;
}

// 倾角：点积法，[-90,+90], +90=直上, 0=水平, -90=直下
static inline float pose_tilt_deg(void){
#if UP_IS_POSITIVE
    const float ux=0.0f, uy=0.0f, uz=+1.0f;
#else
    const float ux=0.0f, uy=0.0f, uz=-1.0f;
#endif
    float dot = ux*ghat.x + uy*ghat.y + uz*ghat.z;
    if (dot >  1.0f) dot =  1.0f;
    if (dot < -1.0f) dot = -1.0f;
    float theta = acosf(dot) * 180.0f/(float)M_PI; // [0,180]
    float sign = (ghat.z * uz >= 0.0f) ? +1.0f : -1.0f;
    return sign * (90.0f - theta);
}

static void pose_warmup_and_zero(void){
    for(int i=0;i<50;i++){ pose_update(); sleep_ms(DT_MS); } // ~1s 预热
    float acc = 0.0f;
    for(int i=0;i<50;i++){ pose_update(); acc += pose_tilt_deg(); sleep_ms(DT_MS); } // ~1s 平均
    tilt_zero_deg = acc / 50.0f;
    tilt_zero_done = true;
    yaw_rel_deg = 0.0f;
}

/* ==================== 分区（2/3/4/5 + CYMBAL 6） ==================== */
static int last_side = 0; // -1:左, 0:中, +1:右

// 2=SNARE, 3=HI_TOM, 4=MEDIUM_TOM, 5=FLOOR_TOM, 6=CYMBAL
static int slot_from_pose(float tilt_corr_deg, float yaw_deg, float gy_dps) {
    // 5: floor tom（低倾角 + 明显下挥）
    if (tilt_corr_deg <= FLOOR_TILT_MAX_DEG && gy_dps >= DOWN_PITCH_DPS) {
        return 5;
    }

    // 6: cymbal — 抬得很高的上挥（你高处打 c ymbal 时 tilt_corr 大约 60+）
    if (tilt_corr_deg >= CYMBAL_TILT_MIN_DEG) {
        return 6;
    }

    // 上排 tom：SNARE/HI_TOM/MEDIUM_TOM
    if (tilt_corr_deg >= TOM_TILT_MIN_DEG) {
        // 左右滞回
        if (last_side == 0) {
            if (yaw_deg <= -YAW_ON_DEG)      last_side = -1;
            else if (yaw_deg >=  YAW_ON_DEG) last_side = +1;
        } else if (last_side == -1) {
            if (yaw_deg > -YAW_OFF_DEG)      last_side = 0;
        } else { // last_side == +1
            if (yaw_deg <  YAW_OFF_DEG)      last_side = 0;
        }

        if      (last_side == -1) return 2; // SNARE (左)
        else if (last_side == +1) return 4; // MEDIUM_TOM (右)
        else                      return 3; // HI_TOM (中)
    }

    // 其他角度暂时忽略
    return -1;
}

static void print_pose_at_hit(uint32_t hit_id,
                              int slot,
                              float tilt_raw, float tilt_corr,
                              float yaw_deg, Vec3 g, float gy_dps) {
    if (slot == 2 || slot == 3 || slot == 4 || slot == 5 || slot == 6) {
        printf("[HIT#%lu] slot=%d  tilt_raw=%.1f  tilt_corr=%.1f  yaw=%.1f  gy=%.1f  g=[%.2f,%.2f,%.2f]\n",
               (unsigned long)hit_id,
               slot, tilt_raw, tilt_corr, yaw_deg, gy_dps, g.x, g.y, g.z);
        if      (slot==2) puts("SNARE");
        else if (slot==3) puts("HI_TOM");
        else if (slot==4) puts("MEDIUM_TOM");
        else if (slot==5) puts("FLOOR_TOM");
        else if (slot==6) puts("CYMBAL");
    }
}

/* ==================== VGA 可视化 ==================== */
static bool g_vga_ready = false;

static void vga_init_display(void){
    ensure_pin_layout();
    initVGA();
    show_project_name();
    sleep_ms(3000);
    show_loading_sequence();
    show_drum_intro();
    show_drum();
    draw_hit_label("Ready");
    g_vga_ready = true;
}

static void vga_hit_animation(int slot, float yaw_deg, bool animate){
    switch (slot) {
        case 2:  // SNARE -> 左上鼓
            snare_played = true;
            if (animate && g_vga_ready) hit_left_upper_tom();
            break;
        case 3:  // HI_TOM -> 中间两上鼓
            hitom_played = true;
            if (animate && g_vga_ready) hit_upper_toms();
            break;
        case 4:  // MEDIUM_TOM -> 右上鼓
            midtom_played = true;
            if (animate && g_vga_ready) hit_right_upper_tom();
            break;
        case 5:  // FLOOR_TOM -> 左/右地鼓
            midtom_played = true;
            if (animate && g_vga_ready) {
                if (yaw_deg < 0.0f) hit_left_floor_tom();
                else                hit_right_floor_tom();
            }
            break;
        case 6:  // CYMBAL -> 左/中/右镲
            hihat_played = true;
            if (animate && g_vga_ready) {
                if      (yaw_deg <= -YAW_ON_DEG) hit_left_cymbal();
                else if (yaw_deg >=  YAW_ON_DEG) hit_right_cymbal();
                else                             hit_middle_cymbal();
            }
            break;
        default:
            if (animate && g_vga_ready) {
                show_drum();
                draw_hit_label("Ready");
            }
            break;
    }
}


// PWM Output

//PWM Part
void init_pwm_audio() {
    gpio_set_function(audio_pin, GPIO_FUNC_PWM);
    pwm_set_clkdiv(pwm_slice_num, pwm_clk_div);
    pwm_set_wrap(pwm_slice_num, pwm_wrap_value);
    pwm_set_chan_level(pwm_slice_num, pwm_channel, 0);

    pwm_set_irq_enabled(pwm_slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP_0, pwm_audio_handler);
    irq_set_enabled(PWM_IRQ_WRAP_0, true);
    pwm_set_enabled(pwm_slice_num, true);
}



void pwm_audio_handler() {

    pwm_clear_irq(pwm_slice_num);

    if(snare_played) {
        offset_snare += 1;
        if(offset_snare >= SNARE_LEN) {
            snare_played = 0;
            offset_snare = -1;
        }
    }

    if(hitom_played) {
        offset_hitom += 1;
        if(offset_hitom >= HITOM_LEN) {
            hitom_played = 0;
            offset_hitom = -1;
        }
    }

    if(midtom_played) {
        offset_midtom += 1;
        if(offset_midtom >= MIDTOM_LEN) {
            midtom_played = 0;
            offset_midtom = -1;
        }
    }

    if (hihat_played) {
        offset_hihat += 1;
        if(offset_hihat >= HIHATS_LEN) {
            hihat_played = 0;
            offset_hihat = -1;
        }
    }

    uint32_t samp = 0;
    samp += (offset_snare != -1) ? ((uint32_t)(wavetable_snare[offset_snare] + 32768)) : 0;
    samp += (offset_hitom != -1) ? ((uint32_t)(wavetable_hitom[offset_hitom] + 32768)) : 0;
    samp += (offset_midtom != -1) ? ((uint32_t)(wavetable_midtom[offset_midtom] + 32768)) : 0;
    samp += (offset_hihat != -1) ? ((uint32_t)(wavetable_hihat[offset_hihat] + 32768)) : 0;
    int count = (offset_snare != -1) + (offset_hitom != -1) + (offset_midtom != -1) + (offset_hihat != -1);
    samp /= (count > 0) ? count : 1;

    //set volume ratio
    samp = (samp * volume) / 4095;

    samp = (samp * pwm_hw->slice[pwm_slice_num].top) / (1<<16);

    pwm_set_chan_level(pwm_slice_num, pwm_channel, samp);
}

//ADC Part
void init_adc() {
    adc_init();
    adc_gpio_init(adc_pin);
    adc_select_input(adc_input);

    adc_set_clkdiv(adc_clk_div);
    adc_fifo_setup(true, false, 1, false, false);

    adc_run(true);
}

int adc_fifo_read(void) {
    int volume = 4095;
    adc_fifo_drain();
    volume = adc_fifo_get_blocking();
    return volume;
}


/* ==================== 主程序 ==================== */
int main(void){
    stdio_init_all();

    // PWM Init Begin
    stdio_init_all();
    init_pwm_audio();
    init_adc();
    // PWM Init End

    sleep_ms(300);

    i2c_bus_init();
    if(!probe_whoami()){ while(true){ sleep_ms(1000);} }
    enable_odo_208hz();

    // 偏置校准（静止 1~2s）
    calibrate_bias(acc_bias_, gyr_bias_);
    printf("ACC bias: %.1f %.1f %.1f | GYR bias: %.1f %.1f %.1f\n",
           acc_bias_[0], acc_bias_[1], acc_bias_[2],
           gyr_bias_[0], gyr_bias_[1], gyr_bias_[2]);

    // 姿态初始化 + 预热&零位
    pose_reset();
    pose_warmup_and_zero();

    // 启动 VGA 显示
    vga_init_display();

    // 初始化命中检测
    int16_t ax0,ay0,az0;
    read_accel_raw(&ax0,&ay0,&az0);
    float amag0 = sqrtf((float)ax0*ax0 + (float)ay0*ay0 + (float)az0*az0);
    HitState hs; hit_init(&hs, amag0);

    uint32_t last_hit_ms = 0;
    int settle_hits = 2; // 丢掉前两次命中，避免刚起步抖动

    while(true){
        // 姿态更新
        pose_update();

        // 命中检测
        int16_t ax,ay,az;
        read_accel_raw(&ax,&ay,&az);
        float amag = sqrtf((float)ax*ax + (float)ay*ay + (float)az*az);
        bool hit = hit_step(&hs, amag);

        if (hit){
            float tilt_raw  = pose_tilt_deg();
            float tilt_corr = tilt_zero_done ? (tilt_raw - tilt_zero_deg) : tilt_raw;
            float yaw_now   = yaw_rel_deg;
            float gy_now    = gyro_y_dps_last;

            // 下挥门槛：减少抖动误触
            if (gy_now < HIT_GY_DPS) {
                sleep_ms(DT_MS);
                continue;
            }

            if (settle_hits > 0){
                settle_hits--;
            } else {
                int slot = slot_from_pose(tilt_corr, yaw_now, gy_now);

                //PWM sound output
                volume = adc_fifo_read(); // read volume from ADC

                uint32_t now=to_ms_since_boot(get_absolute_time());
                bool can_animate = (now - last_hit_ms >= HIT_MIN_GAP_MS);

                // Always trigger audio; animate only when gap允许
                vga_hit_animation(slot, yaw_now, can_animate);

                if(can_animate){
                    last_hit_ms = now;

                    // 命中编号 +1
                    g_hit_count++;

                    // 打印当前第几次 hit + 姿态
                    print_pose_at_hit(g_hit_count, slot,
                                      tilt_raw, tilt_corr,
                                      yaw_now, ghat, gy_now);

                    // 命中后轻度衰减 yaw，避免不断累积
                    yaw_rel_deg *= 0.6f;
                }
            }
        }

        sleep_ms(DT_MS);
    }
}
