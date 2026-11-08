#include "inertial_navigation.h"
#include "line.h"
#include "state_control.h"
#include "zf_driver_flash.h"
extern float basic_speed;
#include <math.h>

#define PI 3.1415926535f


#define R_STRAIGHT 1000.0f // 判定为“纯直道”的最小转弯半径 (给最大速度)
#define R_MIN      500.0f  // 判定为“最急弯道”的最大转弯半径 (给最小速度)
#define MAX_PATH_POINTS 2000
#define RECORD_INTERVAL_TICKS 200  // 每行驶多远记录一个点 (打点精度)
#define DECEL_STEP 3         // 极限刹车能力：每个打点区间允许的最大减速量
#define ACCEL_STEP 10      // 极限加速能力：防止出弯瞬间打滑

int min_target_speed = 1000;   // 默认最低速度
int max_target_speed = 1600;  // 默认最高速度

uint8_t run_mode = 0; // 0: 探索录图, 1: 全速竞速
extern int current_fork_type;
extern int angle_num;

// ================== 内存与全局变量 ==================
Car_State_t car_pos;

#pragma section all "cpu0_dsram"
INS_Pos_t Path_Memory[MAX_PATH_POINTS];
#pragma section all restore

int Total_Path_Points = 0;
int Current_Path_Index = 0;


extern int fork_num; // 来自 state_control 的岔路编号

// 建图专用的基准静态变量
static float last_record_mileage = 0.0f;
static float last_record_yaw = 0.0f;


// ====================== 1. 基础里程计 (Odometry) ======================

void INS_Init(void) {
    // 1. 清空实时物理状态
    car_pos.mileage_ticks = 0.0f;
    car_pos.yaw = 0.0f;
    car_pos.error = 0;
    car_pos.element_flag = 0;

    // 2. 清空查表指针
    Current_Path_Index = 0;

    // 3. 重置打点记录器的基准状态
    last_record_mileage = 0.0f;
    last_record_yaw = 0.0f;

    // 4. 如果是探索发车，必须清空上一圈录的地图
    if (run_mode == 0) {
        Total_Path_Points = 0;
    }
}

void INS_Update(int enc_L, int enc_R, float imu_yaw) {
    float delta_ticks = (enc_L + enc_R) / 2.0f;
    car_pos.yaw = imu_yaw;
    car_pos.mileage_ticks += fabsf(delta_ticks);
    car_pos.error = return_error();

    // =============== 修改部分开始：支持岔路和直角双重 Flag ===============
    if (car_state != normal) {
        // 如果处于岔路状态，赋值为 岔路编号 + 1 (1~20)
        if(current_fork_type == 1){
            car_pos.element_flag = fork_num + 1;
        }else{
            car_pos.element_flag = 0;
        }
    } else if (line_state != none) {
        // 如果处于直角状态，赋值为 200 + 编号 (200~)
        car_pos.element_flag = 200 + angle_num;
    } else {
        // 回到普通直道，重置为 0
        car_pos.element_flag = 0;
    }
    // =============== 修改部分结束 ===============
}

// ====================== 2. 建图与记录 (Mapping) ======================

void INS_Clear_Map(void) {
    Total_Path_Points = 0;
}

void INS_Record_Trajectory(void) {
    if (run_mode != 0) return; // 【安全锁】竞速模式下绝不录图，防止污染内存

    if (car_pos.mileage_ticks - last_record_mileage < RECORD_INTERVAL_TICKS) return;
    if (Total_Path_Points >= MAX_PATH_POINTS) return;

    // 1. 计算这短短一段距离内的角度变化 (反映曲率)
    float current_delta_yaw = car_pos.yaw - last_record_yaw;

    // 2. 存入 Flash 专属结构体 (浮点转整型，严丝合缝存放)
    Path_Memory[Total_Path_Points].mileage_ticks = (int32_t)car_pos.mileage_ticks;
    Path_Memory[Total_Path_Points].delta_yaw     = current_delta_yaw;
    Path_Memory[Total_Path_Points].element_flag  = car_pos.element_flag;
    Path_Memory[Total_Path_Points].target_speed  = 0; // 留空，待跑完后离线规划

    float abs_d_yaw = fabsf(current_delta_yaw);
    if (abs_d_yaw < 0.001f) abs_d_yaw = 0.001f;

    // 计算瞬间转弯半径
    float current_radius = (float)RECORD_INTERVAL_TICKS / abs_d_yaw;

    if (current_radius > 1500.0f) {
        current_radius = 1500.0f;
    }

    static float display_radius = 1500.0f; // 初始默认大直道
    const float filter_alpha = 0.2f; // 滤波系数，越小越平滑，越大越滞后

    display_radius = filter_alpha * current_radius + (1.0f - filter_alpha) * display_radius;

    // 发送过滤后的平滑数据
    //sent_two_int(0, (int)display_radius);


    // 3. 更新打点基准
    last_record_mileage = car_pos.mileage_ticks;
    last_record_yaw = car_pos.yaw;
    Total_Path_Points++;
}

// ====================== 3. 离线全局速度规划 (基于元素的1/2处标定法) ======================

void INS_Generate_Velocity_Table(int min_speed, int max_speed) {
    if (Total_Path_Points == 0) return;
    min_target_speed = min_speed;   // 默认最低速度
    max_target_speed = max_speed;  // 默认最高速度

    // [第0步]: 统配基础速度与起点速度
    for (int i = 0; i < Total_Path_Points; i++) {
        // 初始化全部为最大速度。不论元素内外先拉满，后续靠包络线将速度压下来
        Path_Memory[i].target_speed = max_speed;
    }
    // 强制起点速度为0起步
    Path_Memory[0].target_speed = 100;

    // [第1步]: 寻找连续元素区间，并在 1/2 处标定最低速度
    int element_start_idx = -1;

    for (int i = 0; i <= Total_Path_Points; i++) {
        // 判定当前是否是有效元素点 (i == Total_Path_Points 用于结算最后一段)
        int is_element = (i < Total_Path_Points) && (Path_Memory[i].element_flag != 0);

        if (is_element) {
            // 记录某一段元素的起点
            if (element_start_idx == -1) {
                element_start_idx = i;
            }
        } else {
            // 当前非元素（直道）或到达终点，且之前处于元素区间，说明一个完整的元素段结束了
            if (element_start_idx != -1) {
                int element_end_idx = i - 1;
                int element_length = element_end_idx - element_start_idx + 1; // 该元素点数总计(n)

                // 计算 1/2 n 处的绝对索引位置
                int mid_index = element_start_idx + (element_length / 2);

                // ========================== 【核心修改区】 ==========================
                // 从弯道中央向前、向后各延展 10 个点设为强制最低速度平台
                int platform_start = mid_index - 50;
                int platform_end   = mid_index + 50;

                // 数组内存越界绝对保护（防止修改到还未记录或非法的内存区域）
                if (platform_start < 0) platform_start = 0;
                if (platform_end >= Total_Path_Points) platform_end = Total_Path_Points - 1;

                // 将这 20 多点的区间全部直接钉死底速
                for (int j = platform_start; j <= platform_end; j++) {
                    Path_Memory[j].target_speed = min_speed;
                }
                // ====================================================================

                // 重置状态，准备寻找下一个元素区间
                element_start_idx = -1;
            }
        }
    }

    // [第2步 - 逆向包络]: 限制减速梯度 (入弯前必须提前匀减速)
    // 从赛道终点往起点扫描，如果下一个点速度很低，当前点速度就不能太高，强行往下拉
    for (int i = Total_Path_Points - 2; i >= 0; i--) {
        int32_t safe_braking_speed = Path_Memory[i + 1].target_speed + DECEL_STEP;
        // 如果当前点原规划速度 > 安全刹车极限，则替换掉它
        if (Path_Memory[i].target_speed > safe_braking_speed) {
            Path_Memory[i].target_speed = safe_braking_speed;
        }
    }

    // [第3步 - 正向包络]: 限制加速梯度 (出弯防狂加速打滑，起点起步平顺)
    // 从起点向终点扫描，如果前一个点速度很低，当前点不能突变高，强行往下拉
    for (int i = 1; i < Total_Path_Points; i++) {
        int32_t safe_accel_speed = Path_Memory[i - 1].target_speed + ACCEL_STEP;
        // 如果当前点原规划速度 > 安全加速极限，则替换掉它
        if (Path_Memory[i].target_speed > safe_accel_speed) {
            Path_Memory[i].target_speed = safe_accel_speed;
        }
    }
}

// ====================== 4. 轨迹跟踪与查表 (Tracking) ======================

// 核心游标同步：确保指针咬紧当前里程
void INS_Sync_Tracking_Index(void) {
    if (Total_Path_Points == 0) return;
    while (Current_Path_Index < Total_Path_Points - 1 &&
           car_pos.mileage_ticks >= Path_Memory[Current_Path_Index + 1].mileage_ticks) {
        Current_Path_Index++;
    }
}

// 获取当前规划速度
int32_t INS_Get_Target_Speed(void) {
    // 1. 安全检查：如果从未录制轨迹，返回一个默认低速（这里假设是 100，请改成你的实际值）
    INS_Sync_Tracking_Index();

    if (Total_Path_Points <= 0) {
        return min_target_speed;
    }

    // 2. 终点判断：如果查表指针已经走到了地图的最后1个点
    // 这说明车子已经把地图跑完了，开始强制输出最低速度
    if (Current_Path_Index >= Total_Path_Points - 1) {
        return min_target_speed; // 请改成你需要的最低安全速度
    }
    sent_two_int(Current_Path_Index,0);

    // 3. 正常运行逻辑：返回当前查表指针对应的目标速度
    return Path_Memory[Current_Path_Index].target_speed;
}

// 供舵机前馈控制获取前瞻点
INS_Pos_t INS_Get_Lookahead_Point(float lookahead_dist_ticks) {
    if (Total_Path_Points == 0) {
        INS_Pos_t empty_point = {0};
        return empty_point;
    }
    INS_Sync_Tracking_Index();

    float target_mileage = car_pos.mileage_ticks + lookahead_dist_ticks;
    int lookahead_index = Current_Path_Index;

    while (lookahead_index < Total_Path_Points - 1 &&
           Path_Memory[lookahead_index].mileage_ticks < target_mileage) {
        lookahead_index++;
    }
    return Path_Memory[lookahead_index];
}

// ====================== 5. 绝对位置重定位 (Relocalization) ======================

void INS_Relocalize_At_Flag(int target_flag) {
    if (Total_Path_Points == 0 || run_mode != 1) return; // 仅竞速模式下生效

    // 搜索窗口：以当前查表指针为中心，前后各60个点
    int search_start = Current_Path_Index - 60;
    if (search_start < 0) search_start = 0;
    int search_end = Current_Path_Index + 60;
    if (search_end >= Total_Path_Points) search_end = Total_Path_Points - 1;

    for (int i = search_start; i <= search_end; i++) {
        // 匹配传入的 target_flag (不管是岔路还是直角)
        if (Path_Memory[i].element_flag == target_flag) {
            float drift = Path_Memory[i].mileage_ticks - car_pos.mileage_ticks;

            // 误差在合理范围内，实施强行对齐
            if (fabsf(drift) < 3500.0f) {
                car_pos.mileage_ticks = Path_Memory[i].mileage_ticks;
                Current_Path_Index = i; // 强行拉平查表指针
            }
            break; // 找到即退出，防止重复匹配
        }
    }
}

// ====================== 6. Flash 存储持久化 (Flash) ======================

#define PAGE_META  11
#define PAGE_DATA  5      // 5~10页
#define MAXIMUM_FLASH_POINTS 1536

void Save_Trajectory_To_Flash(void) {
    if (Total_Path_Points == 0) return;

    int save_count = Total_Path_Points > MAXIMUM_FLASH_POINTS ? MAXIMUM_FLASH_POINTS : Total_Path_Points;

    // 1. 写总点数
    uint32_t meta_data[1] = { (uint32_t)save_count };
    flash_write_page(0, PAGE_META, meta_data, 1);

    // 2. 写轨迹数据
    uint32_t total_words = save_count * (sizeof(INS_Pos_t) / 4);
    uint32_t *data_ptr = (uint32_t *)Path_Memory;
    uint32_t words_written = 0;
    uint32_t current_page = PAGE_DATA;

    while (words_written < total_words && current_page <= 10) {
        uint32_t words_to_write = total_words - words_written;
        if (words_to_write > EEPROM_PAGE_LENGTH) {
            words_to_write = EEPROM_PAGE_LENGTH;
        }
        flash_write_page(0, current_page, &data_ptr[words_written], words_to_write);
        words_written += words_to_write;
        current_page++;
    }
}

void Load_Trajectory_From_Flash(void) {
    uint32_t meta_data[1];
    flash_read_page(0, PAGE_META, meta_data, 1);
    int saved_count = (int)meta_data[0];

    if (saved_count > 0 && saved_count <= MAXIMUM_FLASH_POINTS) {
        Total_Path_Points = saved_count;

        uint32_t total_words = Total_Path_Points * (sizeof(INS_Pos_t) / 4);
        uint32_t *data_ptr = (uint32_t *)Path_Memory;
        uint32_t words_read = 0;
        uint32_t current_page = PAGE_DATA;

        while (words_read < total_words && current_page <= 10) {
            uint32_t words_to_read = total_words - words_read;
            if (words_to_read > EEPROM_PAGE_LENGTH) {
                words_to_read = EEPROM_PAGE_LENGTH;
            }
            flash_read_page(0, current_page, &data_ptr[words_read], words_to_read);
            words_read += words_to_read;
            current_page++;
        }

        if (words_read < total_words) {
            Total_Path_Points = words_read / (sizeof(INS_Pos_t) / 4);
        }

        ips114_show_string(0, 16*3, "Load OK!");
        ips114_show_int(60, 16*3, Total_Path_Points, 4);

    } else {
        Total_Path_Points = 0;
        ips114_show_string(0, 16*3, "ERR Val:");
        ips114_show_int(60, 16*3, saved_count, 10);
    }
}
