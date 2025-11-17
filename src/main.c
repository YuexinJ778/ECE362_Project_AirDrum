#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "imu_io.h"

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

// ========= 你“原来的”命中检测参数（原封不动） =========
#define EMA_ALPHA              0.28f
#define MIN_PEAK_LSB           2800.0f
#define MIN_DROP_LSB           1500.0f
#define D_EPS                  1.5f
#define MIN_INTERVAL_SAMPLES   18
#define HIT_MIN_GAP_MS         200

// ========= 新的“仅姿态”位置判定（不做位移积分） =========
// ĝ 的 LPF（越小越稳，越大越灵敏）
#define GHAT_ALPHA           0.02f
// yaw 短窗泄漏（越大越快清零）
#define YAW_LEAK             0.15f
// 互补角轴组合（按你当前设置）
#define ANGLE_USE_AZ_AY 1
#define ANGLE_USE_AX_AZ 0
#define ANGLE_USE_AX_AY 0
#define SIGN_AX              (+1.0f)
#define SIGN_AY              (1.0f)
#define SIGN_AZ              (1.0f)
// 门槛（只在命中时使用）
#define CYMBAL_TILT_DEG      20.0f   // 倾角接近“竖直向上”
#define UNDER_TILT_DEG       25.0f   // 倾角接近“竖直向下”
#define YAW_UPPER_DEG        28.0f   // 左右外摆足够大
// 方向定义：把“向上”当正，把“向下”当负（按装配需要改符号）
#define UP_IS_POSITIVE       0       // 1: 上是正；0: 上是负

// ========= I2C helpers =========
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
    write_reg(REG_CTRL3_C, 0b01000100);
    write_reg(REG_CTRL1_XL, 0b01100000);
    write_reg(REG_CTRL2_G,  0b01100000);
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

// ========= 你原来的“命中”状态机（保留） =========
typedef struct{
    float ema,last,peak,trough;
    int   state,cooldown;
    bool  fired_in_descent;
} HitState;
enum{ SEARCH_PEAK=0, DESCENT=1 };
static inline void hit_init(HitState* s,float x0){
    s->ema=s->last=s->peak=s->trough=x0; s->state=SEARCH_PEAK; s->cooldown=0; s->fired_in_descent=false;
}
static inline bool hit_step(HitState* s,float x){
    s->ema=EMA_ALPHA*x+(1.0f-EMA_ALPHA)*s->ema;
    float dx=s->ema-s->last; s->last=s->ema;
    if(s->cooldown>0){ s->cooldown--; return false; }
    if(s->state==SEARCH_PEAK){
        if(s->ema>s->peak) s->peak=s->ema;
        if(s->peak>=MIN_PEAK_LSB && dx<-D_EPS){ s->trough=s->ema; s->fired_in_descent=false; s->state=DESCENT; }
        return false;
    }else{
        if(s->ema<s->trough) s->trough=s->ema;
        if(!s->fired_in_descent && (s->peak-s->ema)>=MIN_DROP_LSB){ s->fired_in_descent=true; s->cooldown=MIN_INTERVAL_SAMPLES; return true; }
        if(dx>D_EPS){ s->peak=s->ema; s->state=SEARCH_PEAK; }
        return false;
    }
}

// ========= 新：姿态跟踪器（仅 ĝ + 短窗 yaw） =========
typedef struct { float x,y,z; } Vec3;
static Vec3 ghat = {0,0,1};      // 重力方向（机体系测得的重力，经 LPF）
static float yaw_rel_deg = 0.0f;  // 短窗相对 yaw（积分并泄漏）
static bool pose_inited = false;
static float acc_bias_[3]={0}, gyr_bias_[3]={0}; // 从 calibrate_bias 拿

// ★★★ 新增：倾角零位
static float tilt_zero_deg = 0.0f;
static bool  tilt_zero_done = false;

static inline void pose_reset(void){
    ghat.x=0; ghat.y=0; ghat.z=1; yaw_rel_deg=0; pose_inited=false;
}

static inline void pose_update(){
    int16_t ax_i, ay_i, az_i, gx_i, gy_i, gz_i;
    read_accel_raw(&ax_i,&ay_i,&az_i);
    read_gyro_raw (&gx_i,&gy_i,&gz_i);

    // 扣偏置 → g 单位
    float ax_g = ((float)ax_i - acc_bias_[0]) * ACC_LSB_TO_G;
    float ay_g = ((float)ay_i - acc_bias_[1]) * ACC_LSB_TO_G;
    float az_g = ((float)az_i - acc_bias_[2]) * ACC_LSB_TO_G;

    // LPF 更新 ĝ（单位向量）
    if(!pose_inited){ ghat.x=ax_g; ghat.y=ay_g; ghat.z=az_g; pose_inited=true; }
    ghat.x = (1.0f-GHAT_ALPHA)*ghat.x + GHAT_ALPHA*ax_g;
    ghat.y = (1.0f-GHAT_ALPHA)*ghat.y + GHAT_ALPHA*ay_g;
    ghat.z = (1.0f-GHAT_ALPHA)*ghat.z + GHAT_ALPHA*az_g;
    float n = sqrtf(ghat.x*ghat.x + ghat.y*ghat.y + ghat.z*ghat.z) + 1e-9f;
    ghat.x/=n; ghat.y/=n; ghat.z/=n;

    // yaw 短窗：仅用 gz（扣偏置）
    float gz_dps = ((float)gz_i - gyr_bias_[2]) * GYRO_LSB_TO_DPS;
    yaw_rel_deg = (1.0f - YAW_LEAK)*yaw_rel_deg + gz_dps*DT_SEC;
}

// 用 ĝ 计算“与上、下”的相对角度（不依赖积分）
static inline float pose_tilt_deg(void){
#if ANGLE_USE_AZ_AY
    float az = SIGN_AZ*ghat.z;
    float ay = SIGN_AY*ghat.y;
    float ang = atan2f(az, ay) * 180.0f / (float)M_PI; // [-180,180]
#else
    float ax = SIGN_AX*ghat.x, az = SIGN_AZ*ghat.z;
    float ang = atan2f(ax, az) * 180.0f / (float)M_PI;
#endif
#if UP_IS_POSITIVE
    float rel_up   = fabsf(ang -  90.0f);
    float rel_down = fabsf(ang - (-90.0f));
    return (rel_up <= rel_down) ? (90.0f - rel_up) : -(90.0f - rel_down);
#else
    float rel_up   = fabsf(ang - (-90.0f));
    float rel_down = fabsf(ang - ( 90.0f));
    return (rel_up <= rel_down) ? (90.0f - rel_up) : -(90.0f - rel_down);
#endif
}

// ★★★ 新增：预热 + 零位标定（约 2 秒，保持起手姿势不动）
static void pose_warmup_and_zero(void){
    for(int i=0;i<50;i++){ pose_update(); sleep_ms(DT_MS); } // 1s 预热
    float acc = 0.0f;
    for(int i=0;i<50;i++){ pose_update(); acc += pose_tilt_deg(); sleep_ms(DT_MS); } // 1s 平均
    tilt_zero_deg = acc / 50.0f;
    tilt_zero_done = true;
    yaw_rel_deg = 0.0f; // 同步清零 yaw
}

// 命中瞬间的分区：0=BASS,1=CYMBAL,2=UNDER_TOM,3=UPPER_TOM
static inline int classify_zone_from_pose(void){
    float tilt_raw = pose_tilt_deg();
    float tilt = tilt_zero_done ? (tilt_raw - tilt_zero_deg) : tilt_raw; // ★ 以零位为基准
    if (tilt >=  CYMBAL_TILT_DEG)           return 1; // CYMBAL
    if (tilt <= -UNDER_TILT_DEG)            return 2; // UNDER_TOM
    if (fabsf(yaw_rel_deg) >= YAW_UPPER_DEG) return 3; // UPPER_TOM
    return 0; // BASS
}

// ========= 主程序 =========
int main(void){
    stdio_init_all();
    sleep_ms(300);

    i2c_bus_init();
    if(!probe_whoami()){ while(true){ sleep_ms(1000);} }
    enable_odo_208hz();

    // 校准偏置（静止 1~2s）
    calibrate_bias(acc_bias_, gyr_bias_);

    // 姿态初始化 + 预热&零位
    pose_reset();
    pose_warmup_and_zero();

    // 初始化命中检测（用原始 LSB 幅值）
    int16_t ax0,ay0,az0;
    read_accel_raw(&ax0,&ay0,&az0);
    float amag0 = sqrtf((float)ax0*ax0 + (float)ay0*ay0 + (float)az0*az0);
    HitState hs; hit_init(&hs, amag0);

    uint32_t last_hit_ms = 0;
    int settle_hits = 3; // ★ 刚开机丢掉前3次命中

    while(true){
        // 姿态每帧更新（不触发输出）
        pose_update();

        // 命中检测（保持你原参数/逻辑）
        int16_t ax,ay,az;
        read_accel_raw(&ax,&ay,&az);
        float amag = sqrtf((float)ax*ax + (float)ay*ay + (float)az*az);
        bool hit = hit_step(&hs, amag);

        if (hit){
            printf("tilt_raw=%.1f  tilt_corr=%.1f  yaw=%.1f  ghat=[%.2f,%.2f,%.2f]\n",
            pose_tilt_deg(),
            tilt_zero_done ? (pose_tilt_deg()-tilt_zero_deg) : pose_tilt_deg(),
            yaw_rel_deg, ghat.x, ghat.y, ghat.z);

            if (!tilt_zero_done || settle_hits > 0){
                if (settle_hits > 0) settle_hits--;
                // 保护期：命中但不输出
            } else {
                int zone = classify_zone_from_pose();
                uint32_t now=to_ms_since_boot(get_absolute_time());
                if(now - last_hit_ms >= HIT_MIN_GAP_MS){
                    last_hit_ms = now;
                    switch(zone){
                        case 0: puts("BASS");      break;
                        case 1: puts("CYMBAL");    break;
                        case 2: puts("UNDER_TOM"); break;
                        case 3: puts("UPPER_TOM"); break;
                    }
                    // 命中后把 yaw_rel 软复位，避免越积越大
                    yaw_rel_deg *= 0.3f;
                }
            }
        }

        sleep_ms(DT_MS);
    }
}
