#ifndef __MahonyAHRS_H
#define __MahonyAHRS_H

// 更换为 IMU660RC 的头文件
#include "zf_device_imu660rc.h"

// 将延时宏对应到你系统的实际延时函数（请确保 system_delay_ms 存在或修改为你的延时）
#define Delay(time)     system_delay_ms(time)

#define M_PI 3.1415926535f

// 建议将原本的 icm_param_t 重命名为通用名称 imu_param_t
typedef struct
{
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
} imu_param_t;

typedef struct
{
    float pitch;
    float roll;
    float yaw;
} euler_param_t;

extern imu_param_t imu_data;
extern euler_param_t eulerAngle;
extern float GZ;

// 暴露出动态校准计算出的零偏值，方便上位机调试观察
extern float gyro_offset_x;
extern float gyro_offset_y;
extern float gyro_offset_z;

void Mahony_Init(float sampleFrequency);
void Mahony_CalibrateGyro(void); // 新增：动态校准陀螺仪零漂
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void Mahony_GetAngles(void);
float get_yaw(void);
void imu660rc_init_main();

#endif
