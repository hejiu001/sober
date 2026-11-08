/**************************************************************
 * 文件名:  line.c
 * 创建时间: 2026年2月18日 下午3:57:37
 * 描述:
 **************************************************************/
#include "maze.h"
#include "image.h"
#include "stdio.h"
#include "line.h"
#include "tool.h"
#include "zf_device_ips114.h"
#include "state_control.h"
#include "menu.h"
volatile int error=0;
int last_error = 0;
int L_R_difference = 0;

Point target_L,target_R,target_M;
extern Point out_L,out_R;
//int fork_seq[20] = {1,0,1,1,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0};//0左1右

// 定义不同科目的路径序列 (0:左, 1:右)


int fork_seq_sub2[20] = {1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

int fork_seq_sub3[20] = {1,0,1,1,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0};

int fork_num = 0;

int front_point = 95;
int fork_front_point = 125;
Point R,L,M;


// 辅助函数：根据当前科目获取 fork_seq 的值
int get_current_fork_seq(int index) {
    if (subject_id == 2) {
        return fork_seq_sub2[index];
    } else if (subject_id == 3) {
        return fork_seq_sub3[index];
    }
    return 0; // 默认返回
}
//-------------------------------------------------------------------------------------------------------------------
//  函数简介     计算 某个点与底边中点的连线 与 中点法线的夹角
//  返回类型     int
//  参数说明        x，y该点坐标
//  使用样例        get_angle（M.x,M.y）;
//---------------------------------------------------------------------------------------------------------------
int get_angle(int x,int y){
    x = (MT9V03X_W/2)-x;
    y = MT9V03X_H - y;
    int result = (int)(atan2f(x, y) * 180.0f / M_PI);
    return result;
}



//-------------------------------------------------------------------------------------------------------------------
//  函数简介      计算一组边界点的平均坐标
//  返回类型     Side（包含平均x,y坐标的结构体）
//  参数说明        sidein 边界点数组指针
//              min 起始索引
//              max 结束索引（不包括）
//  使用样例        Side avg = average_point(Side_L, 10, 20);
//---------------------------------------------------------------------------------------------------------------
Point average_point(Point *Pointin,int min,int max){
    Point sum;
    sum.x = 0;
    sum.y = 0;
    int count = 0;

    for(int i=min;i<max;i++){
        sum.x += Pointin[i].x;
        sum.y += Pointin[i].y;
        count +=  1;
    }

    sum.x = sum.x/count;
    sum.y = sum.y/count;
    return sum;
}
//-------------------------------------------------------------------------------------------------------------------
//  函数名称     在屏幕上可视化显示检测到的边缘线
//---------------------------------------------------------------------------------------------------------------
void show_line(){
    for(int i=0;i<L_point;i++){
        // 将原始坐标映射到显示区域
        int draw_x = Side_L[i].x * 135 / MT9V03X_W;
        int draw_y = Side_L[i].y * 86 / MT9V03X_H;

        // 【增加保护】：确保坐标在屏幕最大分辨率范围内
        if(draw_x >= 0 && draw_x < ips114_width_max && draw_y >= 0 && draw_y < ips114_height_max){
            show_point_red(draw_x, draw_y);
        }
    }

    for(int i=0;i<R_point;i++){
        // 将原始坐标映射到显示区域
        int draw_x = Side_R[i].x * 135 / MT9V03X_W;
        int draw_y = Side_R[i].y * 86 / MT9V03X_H;

        // 【增加保护】：确保坐标在屏幕最大分辨率范围内
        if(draw_x >= 0 && draw_x < ips114_width_max && draw_y >= 0 && draw_y < ips114_height_max){
            show_point_blue(draw_x, draw_y);
        }
    }
    ips114_show_int(0,100,R_point,3);
    ips114_show_int(0,114,L_point,3);
}

//-------------------------------------------------------------------------------------------------------------------
//  函数名称     显示中线
//---------------------------------------------------------------------------------------------------------------
void show_middle_line(){
    for(int i=0;i<min(L_point,R_point);i++){
        Point M;
        M.x = (int)((Side_L[i].x+Side_R[i].x)/2);
        M.y = (int)((Side_L[i].y+Side_R[i].y)/2);

        // 判断是否使用原始坐标定义，防止越界
        if(M.x > 0 && M.x < MT9V03X_W && M.y > 0 && M.y < MT9V03X_H){
            // 将中线的原始坐标映射到显示区域
            int draw_x = M.x * 135 / MT9V03X_W;
            int draw_y = M.y * 86 / MT9V03X_H;

            // 【增加保护】：确保坐标在屏幕最大分辨率范围内
            if(draw_x >= 0 && draw_x < ips114_width_max && draw_y >= 0 && draw_y < ips114_height_max){
                show_point_yellow(draw_x, draw_y);
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  函数名称     显示目标点
//---------------------------------------------------------------------------------------------------------------
void show_target_point(Point M,Point L,Point R){
    // 将目标点坐标映射到显示区域
    int draw_Mx = M.x * 135 / MT9V03X_W;
    int draw_My = M.y * 86 / MT9V03X_H;

    int draw_Lx = L.x * 135 / MT9V03X_W;
    int draw_Ly = L.y * 86 / MT9V03X_H;

    int draw_Rx = R.x * 135 / MT9V03X_W;
    int draw_Ry = R.y * 86 / MT9V03X_H;

    if(car_state == normal){
        if(draw_Mx >= 0 && draw_Mx < ips114_width_max && draw_My >= 0 && draw_My < ips114_height_max)
            show_point(draw_Mx, draw_My);
    }else{
        int current_seq = get_current_fork_seq(fork_num);

        if(current_seq == 0){
            if(draw_Lx >= 0 && draw_Lx < ips114_width_max && draw_Ly >= 0 && draw_Ly < ips114_height_max)
                show_point(draw_Lx, draw_Ly);
            // 保护字符串打印（确保高度 95+16 不超过屏幕高度）
            //if(95 + 16 < ips114_height_max) ips114_show_string(0, 95, "L");
        }else{
            if(draw_Rx >= 0 && draw_Rx < ips114_width_max && draw_Ry >= 0 && draw_Ry < ips114_height_max)
                show_point(draw_Rx, draw_Ry);
            // 保护字符串打印
            //if(95 + 16 < ips114_height_max) ips114_show_string(0, 95, "R");
        }
    }
}
void caculate_error(){
    int average_range = 5;
    error=0;
    uint8_t R_fail = 0,L_fail = 0;

    if(car_state==fork_turn){
        front_point = fork_front_point;
    }
    else if(line_state==angle){
        front_point = min(L_point,R_point);
    }else{
        front_point = 105;
    }

    if (L_point>front_point){
        L = average_point(Side_L,front_point-average_range,front_point);

    }else if(L_point>40){
        L = average_point(Side_L,L_point-average_range,L_point);
    }else if(L_point>20){
        L = average_point(Side_L,15,20);
    }else{
        if(L_point>0){
            L = Side_L[L_point-1];
        }else{
            L_fail = 1;
        }
    }
    if (R_point>front_point){
        R = average_point(Side_R,front_point-average_range,front_point);
    }else if(R_point>40){
        R = average_point(Side_R,R_point-average_range,R_point);
    }else if(R_point>20){
        R = average_point(Side_R,15,20);
    }else{
        if(R_point>0){
            R = Side_R[R_point-1];
        }else{
            R_fail = 1;
        }
    }

    M.x = (L.x+R.x)/2;
    M.y = (L.y+R.y)/2;
    int M_result = get_angle(M.x,M.y);
    int L_result = get_angle(L.x,L.y);
    int R_result = get_angle(R.x,R.y);

//    if(abs(M_result-last_error)>60){
//        M_result = last_error;
//    }else{
//        last_error = M_result;
//    }
    L_R_difference = L_result-R_result;
    int current_seq = get_current_fork_seq(fork_num);
    if(car_state == normal){
        if(R_fail==1){
            error = L_result;
            L_R_difference = 0;
        }else if(L_fail==1){
            error = R_result;
            L_R_difference = 0;
        }else{
            error = M_result;
        }
    }else{
        if(current_seq == 0){
           error = L_result;
        }else{
           error = R_result;
        }
    }

    //ips114_show_int(0,200,L_R_difference,3);

}

int return_error(){
    return error;
}
