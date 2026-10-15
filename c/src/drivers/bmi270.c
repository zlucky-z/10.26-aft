/* =============================================================================
 * bmi270.c - BMI270 六轴 IMU 驱动实现（I2C，最小可用配置）
 * -----------------------------------------------------------------------------
 * BMI270 关键寄存器（数据手册，小端）：
 *   CHIP_ID     0x00  (读，应为 0x24)
 *   ERR_REG     0x02  (错误/状态)
 *   STATUS      0x03
 *   ACC_DATA    0x0C~0x11  (ax,ay,az，各 int16 小端)
 *   GYR_DATA    0x12~0x17  (gx,gy,gz，各 int16 小端)
 *   CMD         0x7E  (写命令寄存器)
 *   PWR_CTRL    0x7D  (acc_en/gry_en 使能位)
 *   ACC_CONF    0x40  (accel 量程 + ODR)
 *   GYR_CONF    0x42  (gyro 量程)
 *   ACC_RANGE   0x41  (accel 量程)
 *   GYR_RANGE   0x43  (gyro 量程)
 *   INIT_CTRL   0x59  (初始化控制)
 *   INIT_DATA   0x5E  (初始化数据，加载 config 用；最小配置下不需要)
 *
 * 软复位：CMD(0x7E) 写 0xB6；等待 1ms。
 * 使能：PWR_CTRL(0x7D) 写 0x0E（acc_en | aux_en | 保留?）实际 acc_en=bit0, gry_en=bit2。
 *        BMI270 正确使能值：acc_en(0x01) + gry_en(0x04) = 0x05；temp_en 0x08。
 *
 * 注意：BMI270 官方流程要求加载 config 文件才能稳定输出；最小配置仅设
 *   使能位 + 量程，数据可读但精度/稳定性略低于官方初始化。绕障够用。
 * =========================================================================== */

/* A4(10.09)：从基线工程 ltw_dwa 原样移植（实车验证版）。
 * 板端=Linux I2C；本地 Windows 对拍编译走 #else 桩（init 恒失败 →
 * robot_main 自动禁用 IMU 补偿，其余行为不变）。 */
#if defined(__linux__)

#include "bmi270.h"
#include "bmi270_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>      /* usleep */
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* BMI270 寄存器地址 */
#define BMI_REG_CHIP_ID   0x00
#define BMI_REG_ERR_REG   0x02
#define BMI_REG_STATUS    0x03
#define BMI_REG_ACC_X_LSB 0x0C
#define BMI_REG_GYR_X_LSB 0x12
#define BMI_REG_ACC_CONF  0x40
#define BMI_REG_ACC_RANGE 0x41
#define BMI_REG_GYR_CONF  0x42
#define BMI_REG_GYR_RANGE 0x43
#define BMI_REG_PWR_CONF  0x7C
#define BMI_REG_PWR_CTRL  0x7D
#define BMI_REG_CMD       0x7E

/* 初始化/config 加载相关 */
#define BMI_REG_INIT_CTRL      0x59   /* bit0 = LOAD_DONE：0=准备，1=完成 */
#define BMI_REG_INIT_ADDR_0    0x5B   /* config 块地址（2字节：低位+高位） */
#define BMI_REG_INIT_DATA      0x5E   /* config 数据写入口 */
#define BMI_REG_INTERNAL_STATUS 0x21  /* bit[3:0]=消息码，0x01=INIT_OK */

/* 软复位命令 */
#define BMI_CMD_SOFTRESET 0xB6

/* accel 量程寄存器值：±4g -> mask GENMASK(1,0)，值 0x01 */
#define BMI_ACC_RANGE_4G  0x01
/* gyro 量程寄存器值：±2000dps -> 0x00 */
#define BMI_GYR_RANGE_2000 0x00

/* accel ODR 100Hz @ ACC_CONF(0x40)：odr=0x08，BWP 正常模式=0x02<<4 */
#define BMI_ACC_ODR_100HZ  0x08
#define BMI_ACC_BWP_NORMAL (0x02 << 4)
/* gyro ODR 200Hz @ GYR_CONF(0x42)：odr=0x09，BWP 正常模式=0x02<<4 */
#define BMI_GYR_ODR_200HZ  0x09
#define BMI_GYR_BWP_NORMAL (0x02 << 4)

/* 全局状态 */
static int g_fd = -1;
static float g_gyro_bias[3] = {0.0f, 0.0f, 0.0f};  /* gyro 零偏 (rad/s) */
static float g_pitch = 0.0f;   /* IMU 绕 X 轴倾斜角(rad)，安装倾斜 */
static float g_roll  = 0.0f;   /* IMU 绕 Y 轴倾斜角(rad) */
static float g_yaw   = 0.0f;   /* 相对航向角(rad)，正=左转 */

/* ------------------------------------------------------------------------- */
/* I2C 底层读写（利用 Linux SMBus，7bit 地址） */
/* ------------------------------------------------------------------------- */
static int i2c_write_reg(uint8_t reg, uint8_t val)
{
    if (g_fd < 0) return -1;
    if (ioctl(g_fd, I2C_SLAVE, BMI270_I2C_ADDR) < 0) return -1;
    unsigned char buf[2] = { reg, val };
    if (write(g_fd, buf, 2) != 2) return -1;
    return 0;
}

static int i2c_read_reg(uint8_t reg, uint8_t *val)
{
    if (g_fd < 0) return -1;
    if (ioctl(g_fd, I2C_SLAVE, BMI270_I2C_ADDR) < 0) return -1;
    if (write(g_fd, &reg, 1) != 1) return -1;
    if (read(g_fd, val, 1) != 1) return -1;
    return 0;
}

/* 连续读取多个寄存器（一次起始地址） */
static int i2c_read_burst(uint8_t reg, uint8_t *buf, int len)
{
    if (g_fd < 0) return -1;
    if (ioctl(g_fd, I2C_SLAVE, BMI270_I2C_ADDR) < 0) return -1;
    if (write(g_fd, &reg, 1) != 1) return -1;
    if (read(g_fd, buf, len) != len) return -1;
    return 0;
}

/* 连续写多个寄存器（固定起始地址，数据连续推送，芯片内部地址自动递增）。
 * BMI270 的 INIT_DATA(0x5E) 支持一次大 burst write：首字节固定 0x5E，
 * 后续数据字节芯片内部自动推进。用 i2c-dev 原生 write 一次写完整个 config。
 * （Linux 内核用 regmap_bulk_write 也是单次地址 + 连续数据） */
static int i2c_write_burst(uint8_t reg, const uint8_t *buf, int len)
{
    if (g_fd < 0) return -1;
    if (ioctl(g_fd, I2C_SLAVE, BMI270_I2C_ADDR) < 0) return -1;

    /* 一次 I2C burst write：首字节地址 + 全部数据 */
    unsigned char *tmp = (unsigned char *)malloc(len + 1);
    if (!tmp) return -1;
    tmp[0] = reg;
    memcpy(tmp + 1, buf, len);
    ssize_t wr = write(g_fd, tmp, len + 1);
    free(tmp);
    if (wr != len + 1) return -1;
    return 0;
}

/* ------------------------------------------------------------------------- */
/* 初始化与校准 */
/* ------------------------------------------------------------------------- */
int bmi270_init_and_calibrate(void)
{
    uint8_t chipid = 0;

    g_fd = open(BMI270_I2C_DEV, O_RDWR);
    if (g_fd < 0) {
        printf("[bmi270] open %s failed: %s\n", BMI270_I2C_DEV, strerror(errno));
        return -1;
    }

    /* 软复位 */
    i2c_write_reg(BMI_REG_CMD, BMI_CMD_SOFTRESET);
    usleep(2000);   /* 复位后等待 2ms */

    /* 读 chip id */
    if (i2c_read_reg(BMI_REG_CHIP_ID, &chipid) != 0) {
        printf("[bmi270] read chip id failed\n");
        return -1;
    }
    printf("[bmi270] chip id = 0x%02X\n", chipid);
    if (chipid != 0x24) {
        printf("[bmi270] unexpected chip id (expect 0x24)\n");
        return -1;
    }

    /* ---- 加载官方 config 文件（BMI270 必须加载才能出数据） ----
     * 协议参考 Zephyr RTOS 的 bmi270 驱动 write_config_file：
     *   1. PWR_CONF(0x7C)=0x00 关 adv_pwr_save，等 1ms
     *   2. INIT_CTRL(0x59)=0x00（准备加载）
     *   3. 循环每 32 字节一块：
     *        先写 INIT_ADDR_0(0x5B) 2字节块地址（index/2 的低8位+高8位）
     *        再写 INIT_DATA(0x5E) 32字节数据
     *   4. INIT_CTRL(0x59)=0x01（完成加载）
     *   5. 轮询 INTERNAL_STATUS(0x21) 直到 bit[3:0]==0x01 */
    i2c_write_reg(BMI_REG_PWR_CONF, 0x00);   /* 关 adv_pwr_save */
    usleep(1000);

    i2c_write_reg(BMI_REG_INIT_CTRL, 0x00);  /* 准备加载 */

    {
        const int wr_len = 32;
        int total = (int)BMI270_CONFIG_SIZE;
        for (int index = 0; index < total; index += wr_len) {
            /* 块地址：按半字(2字节)编址，故 index/2 */
            uint16_t addr_hm = (uint16_t)(index / 2);
            uint8_t addr2[2];
            addr2[0] = (uint8_t)(addr_hm & 0x0F);
            addr2[1] = (uint8_t)(addr_hm >> 4);

            if (i2c_write_burst(BMI_REG_INIT_ADDR_0, addr2, 2) != 0) {
                printf("[bmi270] write init addr failed @%d\n", index);
                return -1;
            }

            int chunk = wr_len;
            if (index + chunk > total) chunk = total - index;
            if (i2c_write_burst(BMI_REG_INIT_DATA,
                                &bmi270_config_file[index], chunk) != 0) {
                printf("[bmi270] write init data failed @%d\n", index);
                return -1;
            }
        }
        printf("[bmi270] config file written: %d bytes\n", total);
    }

    i2c_write_reg(BMI_REG_INIT_CTRL, 0x01);  /* 完成加载，触发初始化 */

    /* 轮询 internal_status，最多等约 300ms（每 10ms 一次，最多 30 次） */
    {
        int ok = 0;
        for (int i = 0; i < 30; i++) {
            usleep(10000);
            uint8_t st = 0;
            if (i2c_read_reg(BMI_REG_INTERNAL_STATUS, &st) == 0) {
                printf("[bmi270] internal_status = 0x%02X\n", st);
                if ((st & 0x0F) == 0x01) { ok = 1; break; }
            }
        }
        if (!ok) {
            printf("[bmi270] init failed (internal_status msg != 0x01)\n");
            return -1;
        }
    }

    /* ---- 配置量程 + ODR + 使能（规则同内核 bmi270_configure_imu） ---- */
    /* 使能 accel + gyro + temp：aux_en(0x01)|gyr_en(0x02)|accel_en(0x04)|temp_en(0x08) */
    i2c_write_reg(BMI_REG_PWR_CTRL, 0x0E);
    /* accel：ODR 100Hz + BWP 正常模式 */
    i2c_write_reg(BMI_REG_ACC_CONF, BMI_ACC_ODR_100HZ | BMI_ACC_BWP_NORMAL);
    /* gyro：ODR 200Hz + BWP 正常模式 */
    i2c_write_reg(BMI_REG_GYR_CONF, BMI_GYR_ODR_200HZ | BMI_GYR_BWP_NORMAL);
    /* 量程：accel ±4g(0x01)，gyro ±2000dps(0x00) */
    i2c_write_reg(BMI_REG_ACC_RANGE, BMI_ACC_RANGE_4G);
    i2c_write_reg(BMI_REG_GYR_RANGE, BMI_GYR_RANGE_2000);
    /* 最终 PWR_CONF：使能 FIFO_WKUP(bit1)=0x02，关 adv_pwr_save/fup_en */
    i2c_write_reg(BMI_REG_PWR_CONF, 0x02);

    /* 等待 gyro/accel 起振稳定。BMI270 gyro 模拟前端起振需要数十~上百 ms，
     * 传统固定 50ms 延时太短会导致 gyro 数据仍为 0（accel 稳定更快，故先就绪）。
     * 这里主动轮询 STATUS 的 GYR_DRDY(bit6)，最多等 500ms。 */
    {
        uint8_t st = 0;
        int gyr_ready = 0;
        for (int i = 0; i < 50; i++) {
            usleep(10000);   /* 每 10ms 查一次 */
            if (i2c_read_reg(BMI_REG_STATUS, &st) == 0) {
                if (st & 0x40) { gyr_ready = 1; break; }  /* bit6 = DRDY_GYR */
            }
        }
        printf("[bmi270] gyro ready wait: %s (STATUS=0x%02X)\n",
               gyr_ready ? "OK" : "timeout", st);
    }

    /* ---- 寄存器回读诊断：确认配置是否真的写进去了 ---- */
    {
        uint8_t r;
        printf("[bmi270] reg readback");
        if (i2c_read_reg(BMI_REG_PWR_CONF, &r) == 0) printf(" PWR_CONF=0x%02X", r);
        if (i2c_read_reg(BMI_REG_PWR_CTRL, &r) == 0) printf(" PWR_CTRL=0x%02X", r);
        if (i2c_read_reg(BMI_REG_ACC_CONF, &r) == 0) printf(" ACC_CONF(0x40)=0x%02X", r);
        if (i2c_read_reg(BMI_REG_ACC_RANGE, &r) == 0) printf(" ACC_RANGE=0x%02X", r);
        if (i2c_read_reg(BMI_REG_GYR_CONF, &r) == 0) printf(" GYR_CONF(0x42)=0x%02X", r);
        if (i2c_read_reg(BMI_REG_GYR_RANGE, &r) == 0) printf(" GYR_RANGE=0x%02X", r);
        if (i2c_read_reg(BMI_REG_ERR_REG, &r) == 0) printf(" ERR_REG(0x02)=0x%02X", r);
        if (i2c_read_reg(BMI_REG_STATUS, &r) == 0) printf(" STATUS(0x03)=0x%02X", r);
        printf("\n");
        /* 单采样读 12 字节原始数据，看是否全 0 */
        uint8_t buf[12];
        if (i2c_read_burst(BMI_REG_ACC_X_LSB, buf, 12) == 0) {
            printf("[bmi270] raw12: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                   buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],
                   buf[6],buf[7],buf[8],buf[9],buf[10],buf[11]);
        }
    }

    /* ---- 开机静止校正：采样求 gyro 零偏 + 重力方向（pitch/roll） ---- */
    {
        float sum_ax = 0.0, sum_ay = 0.0, sum_az = 0.0;
        float sum_gx = 0.0, sum_gy = 0.0, sum_gz = 0.0;
        int ok = 0;

        for (int i = 0; i < BMI270_CALIB_SAMPLES; i++) {
            uint8_t buf[12];
            if (i2c_read_burst(BMI_REG_ACC_X_LSB, buf, 12) == 0) {
                /* accel 3 轴 (int16 LE) */
                int16_t ax = (int16_t)((buf[1] << 8) | buf[0]);
                int16_t ay = (int16_t)((buf[3] << 8) | buf[2]);
                int16_t az = (int16_t)((buf[5] << 8) | buf[4]);
                /* gyro 3 轴 (int16 LE) */
                int16_t gx = (int16_t)((buf[7] << 8) | buf[6]);
                int16_t gy = (int16_t)((buf[9] << 8) | buf[8]);
                int16_t gz = (int16_t)((buf[11] << 8) | buf[10]);

                sum_ax += (float)ax;
                sum_ay += (float)ay;
                sum_az += (float)az;
                sum_gx += (float)gx;
                sum_gy += (float)gy;
                sum_gz += (float)gz;
                ok++;
            }
            usleep(BMI270_CALIB_INTERVAL_US);
        }

        if (ok == 0) {
            printf("[bmi270] calibrate: no sample\n");
            return -1;
        }

        /* 平均 -> 物理单位 */
        float ax = (float)(sum_ax / ok) / BMI270_ACC_LSB_PER_G;   /* g */
        float ay = (float)(sum_ay / ok) / BMI270_ACC_LSB_PER_G;
        float az = (float)(sum_az / ok) / BMI270_ACC_LSB_PER_G;

        /* gyro 零偏 (rad/s)：LSB/dps -> deg/s -> rad/s */
        const float dps2rad = (float)M_PI / 180.0f;
        g_gyro_bias[0] = (float)(sum_gx / ok) / BMI270_GYRO_LSB_PER_DPS * dps2rad;
        g_gyro_bias[1] = (float)(sum_gy / ok) / BMI270_GYRO_LSB_PER_DPS * dps2rad;
        g_gyro_bias[2] = (float)(sum_gz / ok) / BMI270_GYRO_LSB_PER_DPS * dps2rad;

        /* 由重力向量求安装倾斜角（pitch/roll）
         * pitch = atan2(ax, sqrt(ay^2+az^2))  绕 X
         * roll  = atan2(-ay, az)              绕 Y
         * 注意：不同文献符号约定略有差异，此处按常见 aeronautical 约定。 */
        g_pitch = atan2f(ax, sqrtf(ay * ay + az * az));
        g_roll  = atan2f(-ay, az);

        printf("[bmi270] calibrate ok: samples=%d\n", ok);
        printf("[bmi270] gyro_bias = %.4f %.4f %.4f rad/s\n",
               g_gyro_bias[0], g_gyro_bias[1], g_gyro_bias[2]);
        printf("[bmi270] install tilt: pitch=%.2fdeg roll=%.2fdeg\n",
               g_pitch * 180.0f / (float)M_PI, g_roll * 180.0f / (float)M_PI);

        /* ---- 校准自检（不改行为，仅诊断）-------
         * 校完零偏后，立即再采一段“静止”样本，重新计算残留 wz_up漂移，
         * 用来量化“零偏到底准不准”。若车并非真静止（上电振动/目检自检），
         * 这里能直接看出残留漂移。不写入任何全局状态。 */
        {
            const float dps2rad_s = (float)M_PI / 180.0f;
            float r_gx = 0.0, r_gy = 0.0, r_gz = 0.0;
            int r_ok = 0;
            for (int i = 0; i < BMI270_CALIB_SAMPLES; i++) {
                uint8_t rb[12];
                if (i2c_read_burst(BMI_REG_ACC_X_LSB, rb, 12) == 0) {
                    int16_t rgx = (int16_t)((rb[7] << 8) | rb[6]);
                    int16_t rgy = (int16_t)((rb[9] << 8) | rb[8]);
                    int16_t rgz = (int16_t)((rb[11] << 8) | rb[10]);
                    r_gx += (float)rgx;
                    r_gy += (float)rgy;
                    r_gz += (float)rgz;
                    r_ok++;
                }
                usleep(BMI270_CALIB_INTERVAL_US);
            }
            if (r_ok > 0) {
                float bgx = (float)(r_gx / r_ok) / BMI270_GYRO_LSB_PER_DPS * dps2rad_s;
                float bgy = (float)(r_gy / r_ok) / BMI270_GYRO_LSB_PER_DPS * dps2rad_s;
                float bgz = (float)(r_gz / r_ok) / BMI270_GYRO_LSB_PER_DPS * dps2rad_s;
                float sp = sinf(g_pitch), cp = cosf(g_pitch);
                float sr = sinf(g_roll),  cr = cosf(g_roll);
                float wz_res = -sp * bgx + cp * sr * bgy + cp * cr * bgz;
                printf("[bmi270校准自检] 静止重采样 %d 次: 残留wz_up=%.5f rad/s (%.3f deg/s)  [理想应≈0]\n",
                       r_ok, wz_res, wz_res * 180.0f / (float)M_PI);
            }
        }
    }

    g_yaw = 0.0f;   /* 以开机静止朝向为 0 基准 */
    return 0;
}

/* ------------------------------------------------------------------------- */
/* 读取一帧 IMU（已减零偏 + 倾斜投影修正） */
/* ------------------------------------------------------------------------- */
int bmi270_read(Bmi270ImuSample *out)
{
    uint8_t buf[12];
    if (!out || g_fd < 0) return -1;
    if (i2c_read_burst(BMI_REG_ACC_X_LSB, buf, 12) != 0) return -1;

    int16_t ax = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t ay = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t az = (int16_t)((buf[5] << 8) | buf[4]);
    int16_t gx = (int16_t)((buf[7] << 8) | buf[6]);
    int16_t gy = (int16_t)((buf[9] << 8) | buf[8]);
    int16_t gz = (int16_t)((buf[11] << 8) | buf[10]);

    const float dps2rad = (float)M_PI / 180.0f;

    out->ax = (float)ax / BMI270_ACC_LSB_PER_G;
    out->ay = (float)ay / BMI270_ACC_LSB_PER_G;
    out->az = (float)az / BMI270_ACC_LSB_PER_G;

    /* 减零偏后转 rad/s */
    out->gx = (float)gx / BMI270_GYRO_LSB_PER_DPS * dps2rad - g_gyro_bias[0];
    out->gy = (float)gy / BMI270_GYRO_LSB_PER_DPS * dps2rad - g_gyro_bias[1];
    out->gz = (float)gz / BMI270_GYRO_LSB_PER_DPS * dps2rad - g_gyro_bias[2];

    return 0;
}

/* ------------------------------------------------------------------------- */
/* 航向积分：把 gyro 投影到竖直轴得到绕竖直方向的角速度，积分成 yaw */
/* ------------------------------------------------------------------------- */
int bmi270_update_yaw(float dt)
{
    Bmi270ImuSample s;
    if (bmi270_read(&s) != 0) {
        printf("[bmi270] read failed\n");
        return -1;
    }

    float sp = sinf(g_pitch);
    float cp = cosf(g_pitch);
    float sr = sinf(g_roll);
    float cr = cosf(g_roll);

    float wz_up = -sp * s.gx + cp * sr * s.gy + cp * cr * s.gz;

    g_yaw += wz_up * dt;

    /* 调试：前 200 帧打印原始 gyro（rad/s）与 yaw，定位 imu_yaw 恒为 0 的根因 */
    static int dbg_cnt = 0;
    if (dbg_cnt < 200) {
        printf("[bmi270] gx=%.4f gy=%.4f gz=%.4f wz_up=%.4f yaw=%.4f\n",
               s.gx, s.gy, s.gz, wz_up, g_yaw);
        dbg_cnt++;
    }
    return 0;
}

float bmi270_get_yaw(void)
{
    return g_yaw;
}

void bmi270_close(void)
{
    if (g_fd >= 0) {
        close(g_fd);
        g_fd = -1;
    }
}

#else  /* !__linux__ —— 本地 Windows 对拍桩：全部失败，调用方自动禁用 */

#include <stdio.h>

int bmi270_init_and_calibrate(void)
{
    fprintf(stderr, "[bmi270] 非 Linux 平台，IMU 不可用（桩）\n");
    return -1;
}

int bmi270_read(Bmi270ImuSample *out)
{
    (void)out;
    return -1;
}

int bmi270_update_yaw(float dt)
{
    (void)dt;
    return -1;
}

float bmi270_get_yaw(void)
{
    return 0.0f;
}

void bmi270_close(void)
{
}

#endif /* __linux__ */
