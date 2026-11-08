/**************************************************************
 * 文件名:  element_line.c
 * 创建时间: 2026年2月20日 下午7:41:59
 * 描述:
 **************************************************************/
#include "image.h"
#include "tool.h"
#include "maze.h"
#include "zf_device_ips200.h"
#include "element_line.h"

int element_line_L=0,element_line_R=0,element_line_U=0,element_line_M=0;

Point L_edge[10],R_edge[10],U_edge[10],B_edge[10],M_edge[10];

extern uint8_t R_visited[MT9V03X_H][MT9V03X_W],L_visited[MT9V03X_H][MT9V03X_W];

Point out_L,out_R;


//-------------------------------------------------------------------------------------------------------------------
//  函数简介      在指定高度（Y坐标）的水平方向检测跳变点（黑到白）
//  返回类型     uint8_t（检测到的跳变点数量或最大值）
//  参数说明        img 图像数据
//              hight 检测的Y坐标高度
//              max 当前最大值（用于比较）
//              *edge 存储检测到的边界点
//  使用样例        line_W(img, 50, 0, U_edge);
//---------------------------------------------------------------------------------------------------------------

uint8_t line_W(int hight){
    for(int i=30;i<MT9V03X_W-30;i++){
        if(safe_access_binimg(i-2,hight)==0){
            if(safe_access_binimg(i-1,hight)==0){
                if(safe_access_binimg(i,hight)==255){
                    if(safe_access_binimg(i+1,hight)==255){
                        if(safe_access_binimg(i+2,hight)==255){
                            return 1;
                        }
                    }
                }
            }
        }
    }
    return 0;
}
//-------------------------------------------------------------------------------------------------------------------
//  函数简介      在指定X坐标的垂直方向检测跳变点（黑到白）
//  返回类型     uint8_t（检测到的跳变点数量）
//  参数说明        img 图像数据
//              x 检测的X坐标位置
//  使用样例        int count = line_H(img_data, 100);
//---------------------------------------------------------------------------------------------------------------
uint8_t line_H(int x){
    for(int i=10+2;i<MT9V03X_H-30;i++){
        if(safe_access_binimg(x,i-2)==0){
            if(safe_access_binimg(x,i-1)==0){
                if(safe_access_binimg(x,i)==255){
                    if(safe_access_binimg(x,i+1)==255){
                        if(safe_access_binimg(x,i+2)==255){
                            return 1;
                        }
                    }
                }
            }
        }
    }

    return 0;
}

int element_L(){
    int count = 0;
    count = line_H(25);
    return count;
}

int element_R(){
    return line_H(MT9V03X_W-25);
}

int element_U(){
    return line_W(10);
}

int element_B(){
    return line_W(MT9V03X_H-40);
}

int element_M(){
    return line_W(MT9V03X_H-20);
}

void show_element_line(){
    //ips200_show_int(0,140,element_line_M,2);
    ips200_show_int(0,160,element_line_L,2);
    ips200_show_int(0,180,element_line_R,2);
    ips200_show_int(0,200,element_line_U,2);
    ips200_show_int(0,200,element_line_M,2);
}

void element_update(){
    element_line_L = element_L();
    element_line_R = element_R();
    element_line_U = element_U();
    element_line_M = element_M();
    //element_line_B = element_B();
    //show_element_line();
    //show_element_first_point();
    //show_circie_line();
}


//-------------------------------------------------------------------------------------------------------------------
//  函数名称：    check_sharp_turn_state
//  功能：       判定极端的“双线侧穿”拓扑特征。
//  触发条件：    L和R线同时穿出了"下+左" (此时上和右必须全空)
//              或者 L和R线同时穿出了"下+右" (此时上和左必须全空)
//  返回值：     1 (满足单侧双穿特征), 0 (不满足)
//---------------------------------------------------------------------------------------------------------------
int check_sharp_turn_state(int lookahead_y) {
    if (lookahead_y < 5) lookahead_y = 5;
    if (lookahead_y >= MT9V03X_H - 10) lookahead_y = MT9V03X_H - 10;

    int left_x = 11;
    int right_x = MT9V03X_W - 11;
    int base_y = MT9V03X_H - 10;

    // 分别记录 左线(L) 和 右线(R) 在四条边上的命中情况
    int L_hit_U = 0, R_hit_U = 0; // 上边 (Up)
    int L_hit_B = 0, R_hit_B = 0; // 下边 (Bottom)
    int L_hit_L = 0, R_hit_L = 0; // 左边 (Left)
    int L_hit_R = 0, R_hit_R = 0; // 右边 (Right)

    // 1. 扫描下边 (包含底整行)
    for (int x = left_x; x <= right_x; x++) {
        if (!L_hit_B && L_visited[base_y][x]) L_hit_B = 1;
        if (!R_hit_B && R_visited[base_y][x]) R_hit_B = 1;
    }

    // 2. 扫描上边 (包含顶整行)
    for (int x = left_x; x <= right_x; x++) {
        if (!L_hit_U && L_visited[lookahead_y][x]) L_hit_U = 1;
        if (!R_hit_U && R_visited[lookahead_y][x]) R_hit_U = 1;
    }

    // 3. 扫描左边 (掐头去尾，不扫角点，防交叉)
    for (int y = base_y - 1; y >= lookahead_y + 1; y--) {
        if (!L_hit_L && L_visited[y][left_x]) L_hit_L = 1;
        if (!R_hit_L && R_visited[y][left_x]) R_hit_L = 1;
    }

    // 4. 扫描右边 (掐头去尾，不扫角点，防交叉)
    for (int y = base_y - 1; y >= lookahead_y + 1; y--) {
        if (!L_hit_R && L_visited[y][right_x]) L_hit_R = 1;
        if (!R_hit_R && R_visited[y][right_x]) R_hit_R = 1;
    }

    // ============ 核心逻辑判定 ============

    // 【一票否决】：只要有任何一根线碰到了顶边，绝对不是“全侧穿”，直接返回 0
    if (L_hit_U == 1 || R_hit_U == 1) {
        return 0;
    }

    // 场景 A：【下左组合】 -> L和R都在下边和左边出没；右边绝对干净。
    int is_all_out_left = (L_hit_B && L_hit_L) && (R_hit_B && R_hit_L) && (L_hit_R == 0 && R_hit_R == 0);

    // 场景 B：【下右组合】 -> L和R都在下边和右边出没；左边绝对干净。
    int is_all_out_right = (L_hit_B && L_hit_R) && (R_hit_B && R_hit_R) && (L_hit_L == 0 && R_hit_L == 0);

    // 只要满足其中一种极端组合，就触发标志位！
    if (is_all_out_left || is_all_out_right) {
        return 1;
    }

    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数名称：    count_valid_exits_multi_layer
//  功能：       4层倒U型无交叉防线扫描。满足最严苛的“一票否决”和“组合高度差”验证。
//  返回值：     1=真三叉，0=假三叉或直角弯
//---------------------------------------------------------------------------------------------------------------
int count_valid_exits() {
    int base_y = MT9V03X_H - 10;
    Point best_L = {0}, best_R = {0};

    // 循环 3 次，向内推进布置 3 道倒 U 型防线
    for (int step = 0; step < 3; step++) {

        int current_left_x  = 11 + step * 2;                    // 左墙推进: 11, 14, 17
        int current_right_x = MT9V03X_W - current_left_x;       // 右墙推进: W-11, W-14, W-17
        int current_up_y    = 15;                               // 顶墙固定高度: 15 (注：此处你原代码写的是30)

        int L_hit_L = 0, R_hit_L = 0; Point pL_L = {0}, pR_L = {0};
        int L_hit_R = 0, R_hit_R = 0; Point pL_R = {0}, pR_R = {0};
        int L_hit_U = 0, R_hit_U = 0; Point pL_U = {0}, pR_U = {0};

        // ============ 严防交叉的拼接扫描 ============

        // 1. 扫描左侧 (注：y >= current_up_y + 1，把最高那一行让给顶墙，防交叉！)
        for (int y = base_y; y >= current_up_y + 1; y--) {
            if (!L_hit_L && L_visited[y][current_left_x]) { L_hit_L = 1; pL_L.x = current_left_x; pL_L.y = y; }
            if (!R_hit_L && R_visited[y][current_left_x]) { R_hit_L = 1; pR_L.x = current_left_x; pR_L.y = y; }
        }

        // 2. 扫描右侧 (同理，把最高一行让出来)
        for (int y = base_y; y >= current_up_y + 1; y--) {
            if (!L_hit_R && L_visited[y][current_right_x]) { L_hit_R = 1; pL_R.x = current_right_x; pL_R.y = y; }
            if (!R_hit_R && R_visited[y][current_right_x]) { R_hit_R = 1; pR_R.x = current_right_x; pR_R.y = y; }
        }

        // 3. 扫描顶侧 (独占完整的一行)
        for (int x = current_left_x; x <= current_right_x; x++) {
            if (!L_hit_U && L_visited[current_up_y][x]) { L_hit_U = 1; pL_U.x = x; pL_U.y = current_up_y; }
            if (!R_hit_U && R_visited[current_up_y][x]) { R_hit_U = 1; pR_U.x = x; pR_U.y = current_up_y; }
        }

        // ============ 逻辑判定与一票否决 ============

        int state_L = (L_hit_L && !R_hit_L) ? 1 : ( (!L_hit_L && R_hit_L) ? 2 : 0 );
        int state_R = (L_hit_R && !R_hit_R) ? 1 : ( (!L_hit_R && R_hit_R) ? 2 : 0 );
        int state_U = (L_hit_U && !R_hit_U) ? 1 : ( (!L_hit_U && R_hit_U) ? 2 : 0 );

        int has_only_L = (state_L == 1) || (state_R == 1) || (state_U == 1);
        int has_only_R = (state_L == 2) || (state_R == 2) || (state_U == 2);

        // 如果本层没有发生完美的左右撕裂 -> 不是三叉，一票否决！
        if (!(has_only_L && has_only_R)) {
            return 0;
        }

        Point final_L, final_R;
        int src_L = 0, src_R = 0; // 记录突破口: 1=顶边, 2=侧边

        // 分配最后通过的点
        if (state_U == 1)      { final_L = pL_U; src_L = 1; }
        else if (state_L == 1) { final_L = pL_L; src_L = 2; }
        else                   { final_L = pL_R; src_L = 2; }

        if (state_U == 2)      { final_R = pR_U; src_R = 1; }
        else if (state_R == 2) { final_R = pR_R; src_R = 2; }
        else                   { final_R = pR_L; src_R = 2; }

        // ============ 高度差与拓扑组合校验 (核心修改区) ============

        // 提取左右两个穿出点在 Y 轴上的绝对高度差
        int y_diff = abs(final_L.y - final_R.y);

        // 新增特赦条件：统计左、右、上 三面墙是否都被触碰了
        int hit_Left  = L_hit_L || R_hit_L;
        int hit_Right = L_hit_R || R_hit_R;
        int hit_Top   = L_hit_U || R_hit_U;
        int hit_all_three = (hit_Left && hit_Right && hit_Top);

        // 【新增逻辑】如果左右上都有线穿出，说明赛道极度发散，直接无视高度差，特赦判定为三叉！
        if (hit_all_three) {
            // 什么都不做，直接放行进入 best_L 和 best_R 记录阶段
        }
        else {
            // 【原有逻辑保持不变】
            // 组合 1：只要有一边是从“顶边”穿出的 (即包含 上+左 或 上+右)
            if (src_L == 1 || src_R == 1) {
                // 斜向进入三叉，或者直道颠簸歪斜
                // 如果落差不够大 -> 判定为歪斜直道或干扰，一票否决！
                if (y_diff <= 20) {
                    return 0;
                }
            }
            // 组合 2：两边都是从“侧边”穿出的 (左+右 组合)
            else if (src_L == 2 && src_R == 2) {
                if (y_diff > 30) {
                    return 0;
                }

                if (final_L.x > final_R.x) {
                    return 0;
                }
            }
        }

        // 本层通过考验，暂存坐标
        best_L = final_L;
        best_R = final_R;
    }

    // 3 道防线全部通过！真·三叉路口确认！
    out_L = best_L;
    out_R = best_R;

    return 1;
}
