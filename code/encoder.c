/**************************************************************
 * 文件名:  encoder.c
 * 创建时间: 2026年3月6日 下午3:20:21
 * 描述:
 **************************************************************/
#include "encoder.h"
#include "zf_driver_encoder.h"

void Encoder_Init(void){
    encoder_dir_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);
    encoder_dir_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B);
}


#include <stdlib.h> // 需要用到 abs() 函数求绝对值

// 定义允许的最大速度突变阈值 (你需要根据实际情况微调)
// 如果你的控制周期是5ms，电机在5ms内速度变化绝对不可能超过150！超过了肯定是硬件噪点。
#define ENCODER_SPIKE_THRESHOLD 300

/**************************************************************
 * 获取左编码器速度 (带限幅保护与低通滤波)
 **************************************************************/
int Get_Encoder_L() {
    static float filtered_L = 0;  // 修正变量名
    static int last_raw = 0;      // 新增：保存上一次可靠的原始值

    // 1. 获取原始值并清零计数器
    int raw = (int)encoder_get_count(ENCODER_1);
    encoder_clear_count(ENCODER_1);

    // 2. 【核心保护：防突变限幅滤波】
    // 比较这次读数和上次可靠读数的差值。如果大得离谱（超过物理极限），说明遇到静电或干扰了！
    if (abs(raw - last_raw) > ENCODER_SPIKE_THRESHOLD) {
        raw = last_raw;  // 霸道做法：认定它是噪点，直接丢弃，沿用上一次的正常值
    } else {
        last_raw = raw;  // 如果值正常，就更新历史记录
    }

    // 3. 应用低通滤波公式 (此时喂进去的 raw 绝对是干净安全的)
    filtered_L = FILTER_ALPHA * raw + (1.0f - FILTER_ALPHA) * filtered_L;

    // 4. 返回整数值（四舍五入）
    return -(int)(filtered_L + 0.5f);
}

/**************************************************************
 * 获取右编码器速度 (带限幅保护与低通滤波)
 **************************************************************/
int Get_Encoder_R() {
    static float filtered_R = 0;  // 修正变量名
    static int last_raw = 0;      // 新增：保存上一次可靠的原始值

    // 1. 获取原始值并清零计数器
    int raw = (int)encoder_get_count(ENCODER_2);
    encoder_clear_count(ENCODER_2);

    // 2. 【核心保护：防突变限幅滤波】
    if (abs(raw - last_raw) > ENCODER_SPIKE_THRESHOLD) {
        raw = last_raw;  // 丢弃噪点
    } else {
        last_raw = raw;  // 更新正常值
    }

    // 3. 应用低通滤波公式
    filtered_R = FILTER_ALPHA * raw + (1.0f - FILTER_ALPHA) * filtered_R;

    // 4. 返回整数值（四舍五入）
    return (int)(filtered_R + 0.5f);
}
