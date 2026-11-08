#include "MahonyAHRS.h"
#include "zf_device_imu660rc.h"
#include "zf_driver_pit.h"
#include <math.h>

//-------------------------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------------------------
float twoKi;                                  // 2 * integral gain (Ki)
float q0, q1, q2, q3;                         // quaternion of sensor frame relative to auxiliary frame
float integralFBx, integralFBy, integralFBz;  // integral error terms scaled by Ki
float invSampleFreq;
float roll_mahony, pitch_mahony, yaw_mahony;
float GZ;

imu_param_t imu_data;
euler_param_t eulerAngle;

// 动态零漂变量
float gyro_offset_x = 0.0f;
float gyro_offset_y = 0.0f;
float gyro_offset_z = 0.0f;

//-------------------------------------------------------------------------------------------
#define twoKpDef    (2.0f * 1.0f)   // 2 * proportional gain
#define twoKiDef    (2.0f * 0.0f)   // 2 * integral gain (建议后续可改为 0.005f 以消除长期静态误差)

// 快速平方根倒数
float Mahony_invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}

// ======================= 新增：陀螺仪上电去零偏校准 =======================
void Mahony_CalibrateGyro(void)
{
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    uint16_t sample_count = 200; // 采集200次取平均

    for(uint16_t i = 0; i < sample_count; i++)
    {
        imu660rc_get_gyro(); // 获取 IMU660RC 陀螺仪原始数据

        // 累加转换为物理量(DPS)后的数据
        sum_x += imu660rc_gyro_transition(imu660rc_gyro_x);
        sum_y += imu660rc_gyro_transition(imu660rc_gyro_y);
        sum_z += imu660rc_gyro_transition(imu660rc_gyro_z);

        Delay(5); // 每次读取间隔5ms，总校准耗时约1秒
    }

    // 计算平均零漂值 (单位：DPS)
    gyro_offset_x = sum_x / sample_count;
    gyro_offset_y = sum_y / sample_count;
    gyro_offset_z = sum_z / sample_count;
}

// 获取传感器数据并对齐单位
void IMU_GetValue()
{
    // 1. 读取 IMU660RC 原始数据
    imu660rc_get_gyro();
    imu660rc_get_acc();

    // 2. 加速度计数据转换 (直接用宏转为 g 作为物理量向量即可，后续滤波会自动归一化)
    imu_data.acc_x = imu660rc_acc_transition(imu660rc_acc_x);
    imu_data.acc_y = imu660rc_acc_transition(imu660rc_acc_y);
    imu_data.acc_z = imu660rc_acc_transition(imu660rc_acc_z);

    // 3. 陀螺仪数据转换并去零漂
    // 获取当前物理量(DPS) - 动态校准的零偏(DPS)
    float gyro_x_dps = imu660rc_gyro_transition(imu660rc_gyro_x) - gyro_offset_x;
    float gyro_y_dps = imu660rc_gyro_transition(imu660rc_gyro_y) - gyro_offset_y;
    float gyro_z_dps = imu660rc_gyro_transition(imu660rc_gyro_z) - gyro_offset_z;

    // 4. 将角速度(DPS)转化为 Mahony 要求的 rad/s (弧度每秒)
    imu_data.gyro_x = gyro_x_dps * (M_PI / 180.0f);
    imu_data.gyro_y = gyro_y_dps * (M_PI / 180.0f);
    imu_data.gyro_z = gyro_z_dps * (M_PI / 180.0f);

    // 记录去零漂后的Z轴角速度(DPS，用于某些特定控制)
    GZ = gyro_z_dps;
}

// IMU姿态更新互补滤波算法
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc, qd;

    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        recipNorm = Mahony_invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        if(twoKi > 0.0f)
        {
            integralFBx += twoKi * halfex  * invSampleFreq;
            integralFBy += twoKi * halfey  * invSampleFreq;
            integralFBz += twoKi * halfez  * invSampleFreq;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        gx += twoKpDef * halfex;
        gy += twoKpDef * halfey;
        gz += twoKpDef * halfez;
    }

    gx *= (0.5f  * invSampleFreq);
    gy *= (0.5f  * invSampleFreq);
    gz *= (0.5f  * invSampleFreq);

    qa = q0;
    qb = q1;
    qc = q2;
    qd = q3;

    q0 += (-qb * gx - qc * gy - qd * gz);
    q1 += (qa * gx + qc * gz - qd * gy);
    q2 += (qa * gy - qb * gz + qd * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    recipNorm = Mahony_invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void QuaternionsToEulerAngle()
{
    roll_mahony = asinf(-2.0f * (q1*q3 - q0*q2));
    eulerAngle.roll = 57.29578f * roll_mahony;

    pitch_mahony = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
    eulerAngle.pitch = 57.29578f * pitch_mahony;

    yaw_mahony = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
    eulerAngle.yaw = 57.29578f * yaw_mahony;
}

void Mahony_GetAngles()
{
    IMU_GetValue();
    MahonyAHRSupdateIMU(imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z, imu_data.acc_x, imu_data.acc_y, imu_data.acc_z);
    QuaternionsToEulerAngle();
}

void Get_InitAngle(float ax, float ay, float az)
{
    float recipNorm = Mahony_invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    float roll_init = atan2f(ay, az);
    float pitch_init = atan2f(-ax, sqrtf(ay*ay + az*az));

    float cr = cosf(roll_init * 0.5f);
    float sr = sinf(roll_init * 0.5f);
    float cp = cosf(pitch_init * 0.5f);
    float sp = sinf(pitch_init * 0.5f);

    q0 = cr * cp;
    q1 = sr * cp;
    q2 = cr * sp;
    q3 = -sr * sp;

    recipNorm = Mahony_invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void Mahony_Init(float sampleFrequency)
{
    // 如果 IMU660RC 尚未在外部初始化，这里应调用硬件初始化：
    imu660rc_init(IMU660RC_QUARTERNION_DISABLE);

    twoKi = twoKiDef;
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
    invSampleFreq = 1.0f / sampleFrequency;

    Delay(100); // 稳定等待

    // 1. 动态校准陀螺仪去零偏 (此时务必保持小车/飞机绝对静止)
    Mahony_CalibrateGyro();

    // 2. 利用加速度计获取第一组初始角度
    imu660rc_get_acc();
    float init_ax = imu660rc_acc_transition(imu660rc_acc_x);
    float init_ay = imu660rc_acc_transition(imu660rc_acc_y);
    float init_az = imu660rc_acc_transition(imu660rc_acc_z);

    Get_InitAngle(init_ax, init_ay, init_az);
}

float get_yaw(){
    return eulerAngle.yaw;
}

void imu660rc_init_main(){
    Mahony_Init(500);
    pit_ms_init(CCU60_CH1,2);
}
