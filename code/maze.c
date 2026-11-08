#include "stdio.h"
#include "image.h"
#include "zf_device_mt9v03x.h"
#include "string.h"
#include "maze.h"
#include "line.h"
#include "zf_device_ips200.h"
#include "motor.h"

const int dir_front[4][2]= {{0,  -1},
                            {1,  0},
                            {0,  1},
                            {-1, 0}};

const int dir_frontleft[4][2] = {{-1, -1},
                                 {1,  -1},
                                 {1,  1},
                                 {-1, 1}};

const int dir_frontright[4][2] ={{1,  -1},
                                 {1,  1},
                                 {-1, 1},
                                 {-1, -1}};

Point StarM;
Point Max_Lx,Max_Ly,Max_Rx,Max_Ry;
#pragma section all "cpu1_dsram"
uint8_t L_visited[MT9V03X_H][MT9V03X_W];
uint8_t R_visited[MT9V03X_H][MT9V03X_W];
Point Side_L[300], Side_R[300];
#pragma section all restore
int L_point,R_point;
uint8_t touch = 0;
DistTrend trend;
int count_outside = 0;




void findline_twoside(Point L, Point R, int *Lnum, int *Rnum, int MAX_step) {

    maze L_maze = {0}, R_maze = {0};
    L_maze.step = *Lnum;
    R_maze.step = *Rnum;

    L_visited[L.y][L.x] = 1;
    R_visited[R.y][R.x] = 1;

    if (safe_access_binimg(L.x, L.y) == 0 || safe_access_binimg(R.x, R.y) == 0) {
        return;
    }

    uint8_t L_stopped = 0;
    uint8_t R_stopped = 0;

    while ((L_maze.step < MAX_step-2 || R_maze.step < MAX_step-2) &&
           (!L_stopped || !R_stopped)) {

        // ---------------- 左边巡线逻辑 ----------------
        if (!L_stopped) {
            // 1. 绝对停止条件（Y边界越界、打圈超限、步数超限）
            if (L.y < SAFE_MARGIN_Y || L.y >= MT9V03X_H ||
                L_maze.turn >= 4 || L_maze.step >= MAX_step-2) {
                L_stopped = 1;
            }
            // 2. 条件停止条件（碰到X左右边界，且前方y-10处没有检测到赛道特征）
            else if ((L.x <= 11 || L.x >= MT9V03X_W-11)&&L_maze.step>90) {
                L_stopped = 1;
            }
            // 3. 正常爬线 或 触边但前方有线(继续沿着画面边缘爬)
            else {
                int front_Lx = L.x + dir_front[L_maze.dir][0];
                int front_Ly = L.y + dir_front[L_maze.dir][1];
                int frontleft_x = L.x + dir_frontleft[L_maze.dir][0];
                int frontleft_y = L.y + dir_frontleft[L_maze.dir][1];

                int front_L_value = safe_access_binimg(front_Lx, front_Ly);
                int frontleft_value = safe_access_binimg(frontleft_x, frontleft_y);

                if (front_L_value == 0) {
                    L_maze.dir = (L_maze.dir + 1) % 4;
                    L_maze.turn++;
                } else if (frontleft_value == 0) {
                    L.x += dir_front[L_maze.dir][0];
                    L.y += dir_front[L_maze.dir][1];
                    Side_L[L_maze.step].x = L.x;
                    Side_L[L_maze.step].y = L.y;
                    L_maze.step++;
                    L_maze.turn = 0;
                } else {
                    L.x += dir_frontleft[L_maze.dir][0];
                    L.y += dir_frontleft[L_maze.dir][1];
                    L_maze.dir = (L_maze.dir + 3) % 4;
                    L_maze.step++;
                    Side_L[L_maze.step].x = L.x;
                    Side_L[L_maze.step].y = L.y;
                    L_maze.turn = 0;
                }

                if (Max_Lx.x > L.x) {
                    Max_Lx.x = L.x;
                    Max_Lx.y = L.y;
                }
                if (Max_Ly.y > L.y) {
                    Max_Ly.y = L.y;
                    Max_Ly.x = L.x;
                }

                if (R_visited[L.y][L.x] == 1) {
                    touch = 1;
                    break;
                } else {
                    L_visited[L.y][L.x] = 1;
                }
            }
        }

        // ---------------- 右边巡线逻辑 ----------------
        if (!R_stopped) {
            // 1. 绝对停止条件（Y边界越界、打圈超限、步数超限）
            if (R.y < SAFE_MARGIN_Y || R.y >= MT9V03X_H ||
                R_maze.turn >= 4 || R_maze.step >= MAX_step-2) {
                R_stopped = 1;
            }
            // 2. 条件停止条件（碰到X左右边界，且前方y-10处没有检测到赛道特征）
            else if ((R.x <= 11 || R.x >= MT9V03X_W-11)&& R_maze.step>90) {
                R_stopped = 1;
            }
            // 3. 正常爬线 或 触边但前方有线(继续沿着画面边缘爬)
            else {
                int front_x = R.x + dir_front[R_maze.dir][0];
                int front_y = R.y + dir_front[R_maze.dir][1];
                int frontright_x = R.x + dir_frontright[R_maze.dir][0];
                int frontright_y = R.y + dir_frontright[R_maze.dir][1];

                int front_value = safe_access_binimg(front_x, front_y);
                int frontright_value = safe_access_binimg(frontright_x, frontright_y);

                if (front_value == 0) {
                    R_maze.dir = (R_maze.dir + 3) % 4;
                    R_maze.turn++;
                } else if (frontright_value == 0) {
                    R.x += dir_front[R_maze.dir][0];
                    R.y += dir_front[R_maze.dir][1];
                    Side_R[R_maze.step].x = R.x;
                    Side_R[R_maze.step].y = R.y;
                    R_maze.step++;
                    R_maze.turn = 0;
                } else {
                    R.x += dir_frontright[R_maze.dir][0];
                    R.y += dir_frontright[R_maze.dir][1];
                    R_maze.dir = (R_maze.dir + 1) % 4;
                    R_maze.step++;
                    Side_R[R_maze.step].x = R.x;
                    Side_R[R_maze.step].y = R.y;
                    R_maze.turn = 0;
                }

                if (Max_Rx.x < R.x) {
                    Max_Rx.x = R.x;
                    Max_Rx.y = R.y;
                }
                if (Max_Ry.y > R.y) {
                    Max_Ry.y = R.y;
                    Max_Ry.x = R.x;
                }

                if (L_visited[R.y][R.x] == 1) {
                    touch = 1;
                    break;
                } else {
                    R_visited[R.y][R.x] = 1;
                }
            }
        }
    }

    *Lnum = L_maze.step;
    *Rnum = R_maze.step;
}

uint8_t First_Line_M(uint8_t n,uint8_t y){
    int sum = 0;
    int sum_max = 0;
    StarM.x = MT9V03X_W / 2;
    StarM.y = y;

    for(int i=0;i<MT9V03X_W-1;i++){
        for(int j=y;j>0;j--){
            if(safe_access_binimg(i, j) == 0) break;
            sum++;

        }
        if(sum>sum_max){
            sum_max = sum;
            StarM.x = i;
            StarM.y = y;

        }
        sum = 0;
    }
    Side_L[n].x = StarM.x;
    Side_L[n].y = StarM.y;
    Side_R[n].x = StarM.x;
    Side_R[n].y = StarM.y;

    if(sum_max<1) return 0;





    for(int i=StarM.x;i>2;i--){
        if(safe_access_binimg(i-1,StarM.y)==0){
            if(safe_access_binimg(i-2,StarM.y)==0){
                Side_L[n].x = i;
                break;
            }
        }
    }

    for(int i=StarM.x;i<MT9V03X_W-2;i++){
        if(safe_access_binimg(i+1,StarM.y)==0){
            if(safe_access_binimg(i+2,StarM.y)==0){
                Side_R[n].x = i;
                break;
            }
        }
    }
    return 1;

}


int find_line_edges_at_row(int y, Point *L, Point *R) {
    if (y < 0 || y >= MT9V03X_H) {
        return -1;
    }

    int found_L = 0, found_R = 0;


    for (int x = 2; x < MT9V03X_W - 2; x++) {
        uint8_t p0 = safe_access_binimg(x - 2, y);
        uint8_t p1 = safe_access_binimg(x - 1, y);
        uint8_t p2 = safe_access_binimg(x,     y);
        uint8_t p3 = safe_access_binimg(x + 1, y);

        if (p0 == 0 && p1 == 0 && p2 != 0 && p3 != 0) {
            L->x = x;
            L->y = y;
            found_L = 1;
            break;
        }
    }
    for (int x = MT9V03X_W - 3; x >= 2; x--) {
        uint8_t p0 = safe_access_binimg(x - 1, y);
        uint8_t p1 = safe_access_binimg(x,     y);
        uint8_t p2 = safe_access_binimg(x + 1, y);
        uint8_t p3 = safe_access_binimg(x + 2, y);

        if (p0 != 0 && p1 != 0 && p2 == 0 && p3 == 0) {
            R->x = x;
            R->y = y;
            found_R = 1;
            break;
        }
    }

    if (found_L && found_R && L->x < R->x) {
        return 0;
    } else {
        return -1;
    }
}

int find_outermost_edges_at_row(int y, Point *L, Point *R) {
    if (y < 0 || y >= MT9V03X_H) {
        return -1;
    }

    int found_L = 0, found_R = 0;

    for (int x = 2; x < MT9V03X_W - 2; x++) {
        uint8_t p0 = safe_access_binimg(x + 3, y);
        uint8_t p1 = safe_access_binimg(x + 2, y);
        uint8_t p2 = safe_access_binimg(x,     y);
        uint8_t p3 = safe_access_binimg(x + 1, y);

        if (p2 != 0 && p3 != 0) {
            L->x = x;
            L->y = y;
            found_L = 1;
            break;
        }
    }
    for (int x = MT9V03X_W - 3; x >= 2; x--) {
        uint8_t p0 = safe_access_binimg(x - 1, y);
        uint8_t p1 = safe_access_binimg(x,     y);
        uint8_t p2 = safe_access_binimg(x - 2, y);
        uint8_t p3 = safe_access_binimg(x - 3, y);

        if (p0 != 0 && p1 != 0) {
            R->x = x;
            R->y = y;
            found_R = 1;
            break;
        }
    }

    if (found_L && found_R && L->x < R->x) {
        return 0;
    } else {
        return -1;
    }
}

int calc_side_distance_trend(int Lnum, int Rnum, DistTrend *result)
{
    if (!result) return -1;

    result->slope       = 0.0f;
    result->avg_dist    = 0.0f;
    result->first_dist  = 0.0f;
    result->last_dist   = 0.0f;
    result->valid_pairs = 0;

    if (Lnum <= 0 || Rnum <= 0) return -1;

    float dist_buf[MAX_MATCH_POINTS];
    int   pair_count = 0;
    float sum_dist   = 0.0f;

    int max_i = (Lnum < Rnum) ? Lnum : Rnum;
    if (max_i > MAX_MATCH_POINTS) max_i = MAX_MATCH_POINTS;

    // 依然取最后 30 个点分析趋势
    int start_i = max_i - 30;
    if (start_i < 0) start_i = 0;

    for (int i = start_i; i < max_i; i++)
    {
        int lx = Side_L[i].x;
        int rx = Side_R[i].x;
        int ly = Side_L[i].y;
        int ry = Side_R[i].y;

        float dx = (float)(rx - lx);
        float dy = (float)(ry - ly);

        // --- 修改处：增加 sqrtf，计算真实物理距离 ---
        float dist = dx * dx + dy * dy;

        dist_buf[pair_count] = dist;
        sum_dist += dist;
        pair_count++;
    }

    if (pair_count < MIN_VALID_PAIRS) return -1;

    // 线性回归计算
    float n      = (float)pair_count;
    float sum_x  = 0.0f;
    float sum_y  = 0.0f;
    float sum_xx = 0.0f;
    float sum_xy = 0.0f;

    for (int i = 0; i < pair_count; i++)
    {
        float xi = (float)i;
        float yi = dist_buf[i];
        sum_x  += xi;
        sum_y  += yi;
        sum_xx += xi * xi;
        sum_xy += xi * yi;
    }

    float denom = n * sum_xx - sum_x * sum_x;
    float slope = 0.0f;
    if (denom > 1e-6f || denom < -1e-6f)
    {
        slope = (n * sum_xy - sum_x * sum_y) / denom;
    }

    result->slope       = slope;
    result->avg_dist    = sum_dist / n;
    result->first_dist  = dist_buf[0];
    result->last_dist   = dist_buf[pair_count - 1];
    result->valid_pairs = pair_count;

    return 0;
}

void outside_protect(void){
    if(L_point<20&&R_point<20){
        if(count_outside>4||abs(imu660rc_pitch)>20){
            if(set_f == 1) {   // 只有在真正发车后，才执行强制停车
                car_stop();
            }

        }else{
            count_outside++;
        }
    }
}

/**
 * @brief 评估单边爬线的空间占用密度 (质量评估)
 *
 * @param step_num  爬线的总步数 (Lnum 或 Rnum)
 * @param start_pt  爬线起点 (Side_L[0] 或 Side_R[0])
 * @param extreme_x 维护的极值X坐标 (Max_Lx.x 或 Max_Rx.x)
 * @param extreme_y 维护的极值Y坐标 (Max_Ly.y 或 Max_Ry.y)
 * @return float    返回密度值，正常线通常 < 1.5，绕圈烂线通常 > 2.5
 */
float evaluate_line_density(int step_num, Point start_pt, int extreme_x, int extreme_y) {
    // 步数小于20不具备统计意义，直接默认其合理，防止误删小段好线
    if (step_num <= 20) {
        return 1.0f;
    }

    // 利用起点和极值计算包围框跨度
    int width  = abs(start_pt.x - extreme_x) + 1;
    int height = abs(start_pt.y - extreme_y) + 1;

    // 密度 = 总点数 / 包围盒横纵尺寸总和
    float density = (float)step_num / (float)(width + height);

    return density;
}
#define density_threshold 2.5f

void search_line_main(void) {
    uint8_t y = MT9V03X_H - 10;
    int MAX_step = 300;
    Max_Lx.x = MT9V03X_W - 1;
    Max_Rx.x = 0;
    Max_Ly.y = MT9V03X_H - 1;
    Max_Ry.y = MT9V03X_H - 1;

    int Lnum = 0, Rnum = 0;


    memset(L_visited, 0, sizeof(L_visited));
    memset(R_visited, 0, sizeof(R_visited));

    Point init_L, init_R;
    int start_found = find_outermost_edges_at_row(y, &init_L, &init_R);

    if (start_found != 0) {
        for (int i = MT9V03X_H - 10; i > MT9V03X_H - 100; i -= 2) {
            start_found = find_outermost_edges_at_row(i, &init_L, &init_R);
            if (start_found == 0) {
                break;
            }
        }
    }

    if (start_found != 0) {
        L_point = 0;
        R_point = 0;
        outside_protect();
        return;
    }

    Side_L[0] = init_L;
    Side_R[0] = init_R;

    touch = 0;
    findline_twoside(Side_L[0], Side_R[0], &Lnum, &Rnum, MAX_step);

    int search_attempts = 0;
    const int MAX_ATTEMPTS = 5;

    while (touch == 1 && search_attempts < MAX_ATTEMPTS) {
        search_attempts++;
        Point start_point = Max_Ly.y < Max_Ry.y ? Max_Ly : Max_Ry;

        Point new_L, new_R;
        int reconnect_found = find_outermost_edges_at_row(start_point.y - 5, &new_L, &new_R);

        if (reconnect_found != 0) {
            for (int i = start_point.y - 12; i > start_point.y - 30 && i >= SAFE_MARGIN_Y; i -= 2) {
                reconnect_found = find_outermost_edges_at_row(i, &new_L, &new_R);
                if (reconnect_found == 0) {
                    break;
                }
            }
        }

        if (reconnect_found != 0) {
            break;
        }

        Side_L[Lnum] = new_L;
        Side_R[Rnum] = new_R;

        touch = 0;
        findline_twoside(Side_L[Lnum], Side_R[Rnum], &Lnum, &Rnum, MAX_step);
    }
    count_outside = 0;

//    ips114_show_float(0, 132, L_density, 2, 2);
//    ips114_show_float(0, 148, R_density, 2, 2);


    L_point = Lnum;
    R_point = Rnum;
    //ips114_show_float(0,200,trend.slope,3,3);

    const float FIXED_DIST_THRES = 20.0f; // 绝对距离阈值：首尾直线距离小于20像素，认为是原地打转
    const float DIST_RATIO_THRES = 0.25f; // 比例阈值：直线距离 / 总步数 < 0.25 (即绕了很远的路但推进极少)

    // 校验左线 (给点数一个基础门槛，如果本来就极短，比如才不到10个点，交给后续逻辑自然淘汰)
    if (L_point > 10) {
        float dx = (float)(Side_L[L_point - 1].x - Side_L[0].x);
        float dy = (float)(Side_L[L_point - 1].y - Side_L[0].y);
        float start_end_dist = sqrtf(dx * dx + dy * dy); // 计算首尾欧氏距离

        if (start_end_dist < FIXED_DIST_THRES || start_end_dist < (L_point * DIST_RATIO_THRES)) {
            L_point = 0; // 判定为烂线，直接宣告无效
        }
    }

    // 校验右线
    if (R_point > 10) {
        float dx = (float)(Side_R[R_point - 1].x - Side_R[0].x);
        float dy = (float)(Side_R[R_point - 1].y - Side_R[0].y);
        float start_end_dist = sqrtf(dx * dx + dy * dy);

        if (start_end_dist < FIXED_DIST_THRES || start_end_dist < (R_point * DIST_RATIO_THRES)) {
            R_point = 0; // 判定为烂线，直接宣告无效
        }
    }

    calc_side_distance_trend(L_point, R_point, &trend);
    outside_protect();

}
