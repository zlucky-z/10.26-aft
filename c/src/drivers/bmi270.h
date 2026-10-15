/* =============================================================================
 * bmi270.h - BMI270 六轴 IMU 驱动接口（I2C）
 * -----------------------------------------------------------------------------
 * 功能：
 *   1. 通过 Linux I2C 节点（/dev/i2c-N）读写 BMI270 寄存器；
 *   2. 最小可用配置：上电软复位 -> 读 chip id -> 配置 accel/gyro 范围与
 *      ODR -> 使能 accel + gyro 电源与数据通道；
 *   3. 开机倾斜/零偏自动校正：静止时采样求重力向量算出 pitch/roll 安装角，
 *      并求 gyro 三轴零偏；以当前朝向作为相对航向 0 基准；
 *   4. 运行时输出绕【竖直轴】的角速度积分得到的相对航向角 yaw(rad)。
 *
 * 使用约定：
 *   几何量角度一律 rad；只有 cfg 里的角度阈值/范围用宏可调。
 *   坐标系：yaw 为相对开机零基准的转弯角，正=左转(逆时针，符合车身 +Y=左)。
 *
 * 注意：BMI270 是复杂 6 轴 IMU，本驱动采用【最小可用配置】，不加载 Bosch
 *   官方 config 文件，直接设置基本寄存器让 accel/gyro 出数据。对底盘绕障
 *   所需的 z 轴角速度积分已足够；如需更高精度姿态，可后续按官方 config 升级。
 * =========================================================================== */

#ifndef BMI270_H
#define BMI270_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================ 可调参数 ================================== */
/* I2C 设备节点路径（板子实际接线位置） */
#define BMI270_I2C_DEV          "/dev/i2c-3"
/* BMI270 I2C 从机地址（SDO 接地为 0x68，拉高为 0x69） */
#define BMI270_I2C_ADDR         0x68

/* 采样/校正参数 */
#define BMI270_CALIB_SAMPLES    	50     /* 开机校正采样次数（10Hz 下约 2 秒） */
#define BMI270_CALIB_INTERVAL_US 	10000  /* 每次采样间隔 10ms */

/* gyro 量程与灵敏度（±2000dps -> 16.4 LSB/dps，需与寄存器配置一致） */
#define BMI270_GYRO_RANGE_DPS   2000.0f
#define BMI270_GYRO_LSB_PER_DPS 16.4f

/* accel 量程与灵敏度（±4g -> 8192 LSB/g，需与寄存器配置一致） */
#define BMI270_ACC_RANGE_G      4.0f
#define BMI270_ACC_LSB_PER_G    8192.0f

/* ============================ 数据结构 ================================== */
/* 三轴原始读数（已归一化为物理单位） */
typedef struct {
    float ax, ay, az;   /* 加速度 (g) */
    float gx, gy, gz;   /* 角速度 (rad/s) */
} Bmi270ImuSample;

/* ============================ 接口 ====================================== */

/* 打开 I2C、软复位、校验 chip id、配置并校准。返回 0 成功，非 0 失败。
 * 开机时车必须静止，以便采样重力方向与 gyro 零偏。 */
int bmi270_init_and_calibrate(void);

/* 读取一帧 IMU（已减零偏、已做倾斜投影修正），供主循环调用。返回 0 成功。 */
int bmi270_read(Bmi270ImuSample *out);

/* 更新航向积分（内部用当前 gyro 竖直分量累加 yaw）。返回 0 成功。
 * 调用方需按主循环周期传入 dt(秒)。 */
int bmi270_update_yaw(float dt);

/* 获取当前相对航向角(rad)，正=左转。 */
float bmi270_get_yaw(void);

/* 关闭 I2C。 */
void bmi270_close(void);

#ifdef __cplusplus
}
#endif

#endif /* BMI270_H */
