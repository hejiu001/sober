#ifndef CODE_ADRC_H_
#define CODE_ADRC_H_

#include "zf_common_headfile.h"

// ==========================================
// 一阶 LADRC 结构体定义
// ==========================================
typedef struct {
    // 1. 核心可调参数
    float b0;           // 系统增益 (PWM到加速度的放大倍数)
    float wc;           // 控制器带宽 (决定响应速度，类似PID的P)
    float wo;           // 观测器带宽 (决定抗干扰速度，通常设为 wc 的 3~5 倍)
    float h;            // 控制周期 (单位：秒，例如 1ms = 0.001)

    // 2. 输出限幅
    float max_pwm;      // 正向最大PWM
    float min_pwm;      // 反向最大PWM (负数)

    // 3. 内部自动计算的观测器增益
    float beta1;
    float beta2;

    // 4. 状态变量
    float z1;           // 观测器估计的平滑转速
    float z2;           // 观测器估计的总扰动 (负压阻力、摩擦力等)
    float u;            // 最终输出的控制量 (PWM)
} LADRC_Struct;

// ==========================================
// 函数声明
// ==========================================
void LADRC_Init(LADRC_Struct *adrc, float b0, float wc, float wo, float h, float max_pwm, float min_pwm);
int LADRC_Calculate(LADRC_Struct *adrc, float target_speed, float actual_speed);
void LADRC_Reset(LADRC_Struct *adrc);

#endif /* CODE_ADRC_H_ */
