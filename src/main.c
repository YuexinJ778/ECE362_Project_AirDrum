#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "imu_io.h"

// ---------- I2C ----------
#define I2C_PORT      i2c0
#define I2C_SDA_PIN   40
#define I2C_SCL_PIN   41
#define I2C_BAUD      (400 * 1000)

// ---------- LSM6DSOX ----------
#define LSM6DSOX_ADDR           0x6A
#define LSM6DSOX_REG_WHO_AM_I   0x0F
#define LSM6DSOX_WHOAMI_VAL     0x6C
#define REG_CTRL1_XL            0x10
#define REG_CTRL2_G             0x11
#define REG_CTRL3_C             0x12
#define REG_OUTX_L_G            0x22
#define REG_OUTX_L_A            0x28

// ---------- “稍微更敏感”的简化检测参数（仅下落触发） ----------
#define EMA_ALPHA              0.28f     // 原来 0.20，略提一点，响应更快但不过敏
#define MIN_PEAK_LSB           2500.0f   // 原来 3000，略降一点，便于识别有效峰
#define MIN_DROP_LSB           1300.0f   // 原来 1500，略降一点，稍微更敏感
#define D_EPS                  1.0f      // 导数阈值，保持温和去抖
#define MIN_INTERVAL_SAMPLES   14        // 原来 20，略缩短，避免“只打一枪”但不至于连发

static inline int16_t u8pair_to_i16(uint8_t lo, uint8_t hi) {
    return (int16_t)((hi << 8) | lo);
}

static void i2c_bus_init(void) {
    i2c_init(I2C_PORT, I2C_BAUD);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
}

static int i2c_write_reg_addr(uint8_t reg, bool nostop) {
    return i2c_write_blocking(I2C_PORT, LSM6DSOX_ADDR, &reg, 1, nostop);
}

static int read_reg(uint8_t reg, uint8_t *val) {
    if (i2c_write_reg_addr(reg, true) != 1) return PICO_ERROR_GENERIC;
    return (i2c_read_blocking(I2C_PORT, LSM6DSOX_ADDR, val, 1, false) == 1) ? 0 : PICO_ERROR_GENERIC;
}

static int write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return (i2c_write_blocking(I2C_PORT, LSM6DSOX_ADDR, buf, 2, false) == 2) ? 0 : PICO_ERROR_GENERIC;
}

static bool probe_whoami(void) {
    uint8_t who = 0;
    if (read_reg(LSM6DSOX_REG_WHO_AM_I, &who) == 0 && who == LSM6DSOX_WHOAMI_VAL) return true;
    printf("WHO_AM_I=0x%02X (expect 0x6C). Check wiring/addr.\n", who);
    return false;
}

static void enable_odo_208hz(void) {
    write_reg(REG_CTRL3_C, 0b01000100); // IF_INC=1, BDU=1
    write_reg(REG_CTRL1_XL, 0b01100000); // Accel 208Hz, ±2g
    write_reg(REG_CTRL2_G,  0b01100000); // Gyro  208Hz, 250dps
    sleep_ms(20);
}

void read_accel_raw(int16_t* ax, int16_t* ay, int16_t* az) {
    uint8_t buf[6];
    i2c_write_reg_addr(REG_OUTX_L_A, true);
    i2c_read_blocking(I2C_PORT, LSM6DSOX_ADDR, buf, 6, false);
    if (ax) *ax = u8pair_to_i16(buf[0], buf[1]);
    if (ay) *ay = u8pair_to_i16(buf[2], buf[3]);
    if (az) *az = u8pair_to_i16(buf[4], buf[5]);
}

void read_gyro_raw(int16_t* gx, int16_t* gy, int16_t* gz) {
    uint8_t buf[6];
    i2c_write_reg_addr(REG_OUTX_L_G, true);
    i2c_read_blocking(I2C_PORT, LSM6DSOX_ADDR, buf, 6, false);
    if (gx) *gx = u8pair_to_i16(buf[0], buf[1]);
    if (gy) *gy = u8pair_to_i16(buf[2], buf[3]);
    if (gz) *gz = u8pair_to_i16(buf[4], buf[5]);
}

// ---------- 仅“下落触发”的简洁两态机 ----------
typedef struct {
    float ema, last, peak, trough;
    int   state;        // 0:找峰  1:下落中
    int   cooldown;     // 冷却计数，避免抖动多触发
    bool  fired_in_descent; // 本次下落已触发过
} HitState;

enum { SEARCH_PEAK = 0, DESCENT = 1 };

static inline void hit_init(HitState* s, float x0) {
    s->ema = s->last = s->peak = s->trough = x0;
    s->state = SEARCH_PEAK;
    s->cooldown = 0;
    s->fired_in_descent = false;
}

// 只在“峰->谷的下落途中”且跌幅首次超过阈值时触发；回升不触发
static inline bool hit_step(HitState* s, float x) {
    s->ema = EMA_ALPHA * x + (1.0f - EMA_ALPHA) * s->ema;
    float dx = s->ema - s->last;
    s->last = s->ema;

    if (s->cooldown > 0) { s->cooldown--; return false; }

    if (s->state == SEARCH_PEAK) {
        if (s->ema > s->peak) s->peak = s->ema;
        // 只在出现明显“向下”时进入下落态
        if (s->peak >= MIN_PEAK_LSB && dx < -D_EPS) {
            s->trough = s->ema;
            s->fired_in_descent = false;
            s->state = DESCENT;
        }
        return false;
    } else { // DESCENT
        if (s->ema < s->trough) s->trough = s->ema;

        // ★ 核心：第一次超过跌幅阈值则触发（只触发一次）
        if (!s->fired_in_descent && (s->peak - s->ema) >= MIN_DROP_LSB) {
            s->fired_in_descent = true;
            s->cooldown = MIN_INTERVAL_SAMPLES; // 小冷却抑制抖动
            return true; // 只在“放下去”时打一次
        }

        // 当明显出现回升时，重新回到找峰，等待下一次动作
        if (dx > D_EPS) {
            s->peak = s->ema;         // 以当前位置作为下一轮起点
            s->state = SEARCH_PEAK;
        }
        return false;
    }
}

int main() {
    stdio_init_all();
    sleep_ms(300);

    i2c_bus_init();
    if (!probe_whoami()) { while (true) { sleep_ms(1000); } }
    enable_odo_208hz();

    float acc_bias[3], gyr_bias[3];
    calibrate_bias(acc_bias, gyr_bias);   // 由你的 imu_io.h 提供

    // 初始化检测器
    int16_t ax0, ay0, az0;
    read_accel_raw(&ax0, &ay0, &az0);
    float axf0 = (float)ax0 - acc_bias[0];
    float ayf0 = (float)ay0 - acc_bias[1];
    float azf0 = (float)az0 - acc_bias[2];
    float mag0 = sqrtf(axf0*axf0 + ayf0*ayf0 + azf0*azf0);
    HitState hs; hit_init(&hs, mag0);

    while (true) {
        int16_t ax, ay, az, gx, gy, gz;
        read_accel_raw(&ax, &ay, &az);
        read_gyro_raw (&gx, &gy, &gz);

        float axf = (float)ax - acc_bias[0];
        float ayf = (float)ay - acc_bias[1];
        float azf = (float)az - acc_bias[2];
        float amag = sqrtf(axf*axf + ayf*ayf + azf*azf);

        if (hit_step(&hs, amag)) {
            printf("HIT\n");  // 只在“放下去”时触发
        }

        // 回到原先的用户态频率（更稳更省资源）
        sleep_ms(20);  // ≈50 Hz
    }
}
