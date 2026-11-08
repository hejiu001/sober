/**************************************************************
 * 文件名:  state_control.c
 * 创建时间: 2026年2月24日 下午6:53:14
 * 描述:
 **************************************************************/

#include "element_line.h"
#include "state_control.h"
#include "line.h"
#include "element_line.h"
#include "zf_device_ips114.h"
#include "maze.h"
#include "MahonyAHRS.h"
#include "LED.h"
#include "menu.h"
// state_control.c 顶部变量
state car_state = normal;
element_state line_state = none;
//int fork_type[20] = {1,1,1,1,1,1,1,0,1,0,0,0,0,0,0,0,0,0,0,0};
// --- 修改部分开始 ---
// 定义不同科目的路口类型 (0:直行, 1:转弯/其他)

int fork_type_sub2[20] = {1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

int fork_type_sub3[20] = {1,1,1,1,1,1,1,0,1,0,0,0,0,0,0,0,0,0,0,0};

float current_angle = 0;
int count = 0;
static int straight_count = 0;
int current_fork_type = 1;
int angle_num = 0;

#define FORK_MAX 20
#define INS_EXIT_CONFIRM_FRAMES 5
static int ins_exit_confirm_count = 0; // 连续确认帧数计数器
extern Point out_L,out_R;

static void fork_num_increase(void){
    fork_num++;
    if(fork_num >= FORK_MAX){
        fork_num = FORK_MAX - 1;
       // car_stop();
    }
}
void state_update(){
    float end_angle = 0;
    int angle_difference = 0;

    int element_count = count_valid_exits();


    if(subject_id == 2) current_fork_type = fork_type_sub2[fork_num];
    else if(subject_id == 3) current_fork_type = fork_type_sub3[fork_num];


    switch(car_state){
        case normal:{
            // 基础条件判断
            if(element_count==1 && R_point >= 100&& L_point >= 100 && trend.slope>60){
                count++; // 满足条件，计数器累加
                if (count >= 0) { // 连续3次满足条件，才执行状态转换
                    if (subject_id != 1) {
                        if(current_fork_type == 0){
                            car_state = fork_straight;
                            straight_count = 0;
                        }else{
                            car_state = fork_turn;
                            current_angle = get_yaw();
                        }
                    }
                    count = 0;
                    if (run_mode == 1) {
                        INS_Relocalize_At_Flag(fork_num);
                    }
                }
            }else{
                count = 0;
            }
//
            LED1_off();
            LED2_off();
            break;
        }
        case fork_straight:{
            if(element_count < 1){
                straight_count++;
                if(straight_count > 0){
                    car_state = normal;
                    fork_num_increase();
                    straight_count = 0;
                }
            }else{
                straight_count = 0;
            }
            LED1_on();
            LED2_on();
            break;
        }
        case fork_turn:{
            end_angle = get_yaw();
            angle_difference = abs(((int)end_angle - (int)current_angle));
            if (angle_difference > 180.0) {
                angle_difference = 360.0 - angle_difference;
            }
            if(angle_difference > 50){
                car_state = normal;
                fork_num_increase();
                count = 0;
            }
            LED1_on();
            LED2_on();
            break;

        }

    }
    switch(line_state){
        case none:{
            if(car_state==normal){
                if(check_sharp_turn_state(35)==1){
                    line_state = angle;
                    current_angle = get_yaw();
                    if (run_mode == 1) {
                        INS_Relocalize_At_Flag(200 + angle_num);
                    }
//                    LED1_on();
//                    LED2_on();
                }
            }
            break;
        }
        case angle:{
            if(car_state==normal){
                end_angle = get_yaw();
                angle_difference = abs(((int)end_angle - (int)current_angle));
                if (angle_difference > 180.0) {
                        angle_difference = 360.0 - angle_difference;
                    }
                if(angle_difference>60){
                    line_state = none;
                    angle_num++;
//                    LED1_off();
//                    LED2_off();
                }
            }else{
                line_state = none;
                angle_num++;
                LED1_off();
                LED2_off();
            }
            break;
        }
    }
}

void show_state(){
    ips200_show_int(0,120,fork_num,2);
    switch(car_state){
        case normal:{
            ips114_show_string(0,140,"normal");
            break;
        }
        case fork_turn:{
            ips114_show_string(0,140,"fork_turn");
            break;
        }
        case fork_straight:{
            ips114_show_string(0,140,"fork_straight");
            break;
        }

    }
    switch(line_state){
        case none:{
            ips114_show_string(0,180,"none");
            break;
        }
        case angle:{
            ips114_show_string(0,180,"element");
            break;
        }
    }

}

