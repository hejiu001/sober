#ifndef CODE_INERTIAL_NAVIGATION_H_
#define CODE_INERTIAL_NAVIGATION_H_

#include "zf_common_headfile.h"
#include "state_control.h"

// ==============================================================
// 结构体 1：实时车辆状态 (在内存中实时刷新，不需要存入 Flash)
// ==============================================================
typedef struct {
    float mileage_ticks;   // 当前累积总里程 (用float防止积分丢失精度)
    float yaw;             // 当前绝对偏航角 (由IMU提供)
    int32_t error;         // 当前摄像头/电磁偏差
    int32_t element_flag;  // 当前所处元素状态 (0:普通, >0:特定岔路)
} Car_State_t;

// ==============================================================
// 结构体 2：赛道轨迹节点 (严丝合缝的 16 字节，专供 Flash 存储和规划)
// ==============================================================
typedef struct {
    int32_t mileage_ticks; // 1. 节点里程 (4字节)
    float   delta_yaw;     // 2. 局部曲率特征（角度变化量） (4字节)
    int32_t target_speed;  // 3. 规划好的目标速度 (4字节)
    int32_t element_flag;  // 4. 元素特征 (4字节)
} INS_Pos_t;


// 全局状态变量暴露
extern Car_State_t car_pos;              // 【修改点】改为专属结构体
extern INS_Pos_t Path_Memory[];          // 轨迹内存池
extern int Total_Path_Points;
extern int Current_Path_Index;
extern uint8_t run_mode;                 // 0: 探索建图, 1: 查表竞速

// --- 函数声明区域 ---
void INS_Init(void);
void INS_Update(int enc_L, int enc_R, float imu_yaw);

void INS_Clear_Map(void);
void INS_Record_Trajectory(void);

void INS_Generate_Velocity_Table(int min_speed, int max_speed);

void    INS_Sync_Tracking_Index(void);
int32_t INS_Get_Target_Speed(void);
INS_Pos_t INS_Get_Lookahead_Point(float lookahead_dist_ticks);

void INS_Relocalize_At_Flag(int target_flag);

void Save_Trajectory_To_Flash(void);
void Load_Trajectory_From_Flash(void);

#endif
