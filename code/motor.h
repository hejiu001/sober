#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_

#include "zf_common_headfile.h"
#include"tool.h"
// 硬件引脚定义
#define DIR_R1             (P11_11)
#define PWM_R1              (ATOM2_CH7_P11_12)
#define DIR_L1               (P11_9)
#define PWM_L1              (ATOM3_CH5_P11_10)


// ************************* 全局变量声明 *************************
extern volatile  int stop_flag;


// ************************* 函数声明 *************************
void Motor_Init(void);
void L_set_PWM(int Speed);
void R_set_PWM(int Speed);
void speed_control_L(int target_speed_L);
void speed_control_R(int target_speed_R);
void combined_control(int base_speed, int path_deviation);
void send_speed_to_pc(void);
float steering_control(float path_deviation);
void car_stop();
void negative_pressure_init();
void negative_pressure_start();
void negative_pressure_stop();
void Motor_Reset_State(void);



#endif /* CODE_MOTOR_H_ */
