/*
 * menu.c V3
 * OUC 沧海一粟ovo
 * 带有flash读写功能的多级菜单 - 适配IncPID与SteeringPID参数
 *
 * 2025.7.2
 * 2026.4.7 适配IPS114屏幕
 */

#include "zf_common_headfile.h"
#include "zf_device_ips114.h"   // 新增 IPS114 驱动
#include <string.h>
#include <math.h>

extern IncPID pid_left, pid_right;
extern SteeringPID turnPID;
extern float steer_Kp2;
extern float SCALE_KD;
extern int min_target_speed;
extern int max_target_speed;

// 限制数据范围宏
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

int8 set_f = 0;
int32 Save=0;
KEY_STATE key = 0;
MENU menu = MENU1_1;
float params[40]={0}; //用于转换投递flash的数组
uint8 subject_id = 1; // 默认科目1
float basic_speed = 0; // 基础速度

extern Point R,L,M;

uint32 time_use = 0;

static int8 menu1_flag = 0;
static int8 menu2_flag = 0;

//-------------------------------------------------------------------------------------------------------------------
// 按键扫描
//-------------------------------------------------------------------------------------------------------------------
void key_scan(void)
{
  if(!gpio_get_level(KEY_UP))
   {
        system_delay_ms(20);
        while(!gpio_get_level(KEY_UP)) ;
        system_delay_ms(20);
        key=UP;
        return;
  }
  if(!gpio_get_level(KEY_DOWN))
   {
        system_delay_ms(20);
        while(!gpio_get_level(KEY_DOWN)) ;
        system_delay_ms(20);
        key=DOWN;
        return;
  }
  if(!gpio_get_level(KEY_RIGHT))
   {
        system_delay_ms(20);
        while(!gpio_get_level(KEY_RIGHT)) ;
        system_delay_ms(20);
        key=RIGHT;
        return;
  }
  if(!gpio_get_level(KEY_LEFT))
   {
        system_delay_ms(20);
        while(!gpio_get_level(KEY_LEFT)) ;
        system_delay_ms(20);
        key=LEFT;
        return;
  }
  if(!gpio_get_level(KEY_PRESS))
   {
        system_delay_ms(20);
        while(!gpio_get_level(KEY_PRESS)) ;
        system_delay_ms(20);
        key=PRESS;
        return;
  }
  key = EMPTY;
}

//-------------------------------------------------------------------------------------------------------------------
// 菜单初始化 (读取Flash写入到相应的新结构体中)
//-------------------------------------------------------------------------------------------------------------------
void menu_init(void)
{
  // --- 读取速度环参数 ---
  memset(params,0,40*sizeof(float));
  param_r_flash(SPEED_PAGE);
  basic_speed = params[0];

  memcpy(&pid_left, &params[1], sizeof(IncPID));
  memcpy(&pid_right, &params[1 + sizeof(IncPID)/4], sizeof(IncPID));

  int offset_spd = 1 + (sizeof(IncPID)/4)*2;
  if(params[offset_spd] != 0 || params[offset_spd+1] != 0) {
      max_target_speed = (int)params[offset_spd];
      min_target_speed = (int)params[offset_spd+1];
  } else {
      max_target_speed = 1200;
      min_target_speed = 400;
  }

  // --- 读取转向环参数 ---
  memset(params,0,40*sizeof(float));
  param_r_flash(DIR_PAGE);
  memcpy(&turnPID, params, sizeof(SteeringPID));

  // --- 【修改】：读取平方项系数 steer_Kp2 ---
  int offset = sizeof(SteeringPID) / 4;
  if(params[offset] != 0 || params[offset+1] != 0) {
      steer_Kp2 = params[offset];
      SCALE_KD = params[offset+1];
  } else {
      steer_Kp2 = 0.05f; // 默认初值
      SCALE_KD = 0.1f;
  }

  memset(params, 0, 40*sizeof(float));
    param_r_flash(SUB_PAGE);
    if (!isnan(params[0]) && params[0] >= 1 && params[0] <= 3) {
        subject_id = (uint8_t)params[0];
    } else {
        subject_id = 1;
        params[0] = (float)subject_id;
        param_w_flash(SUB_PAGE);
    }

    // --- 读取前瞻点参数 ---
    memset(params, 0, 40*sizeof(float));
    param_r_flash(IMAGE_PAGE);
    if (0){
        front_point = params[0];
        fork_front_point = params[1];
    } else {
        front_point = 90.0f;
        fork_front_point = 125.0f;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 菜单显示主循环
//-------------------------------------------------------------------------------------------------------------------
void menu_display(void)
{
  while(!set_f)
  {
   key_scan();
   switch (menu)
   {
   case MENU1_1:
    menu1();
    break;
   case MENU2_1:
    menu2_1();
    break;
   case MENU2_2:
    menu2_2();
    break;
   case MENU2_3:
    menu2_3();
    break;
   case MENU2_4:
    menu2_4();
    break;
   case MENU2_5:
    menu2_5();
    break;
   case MENU2_6:
    menu2_6();
    break;
   }
  }
  ips114_clear();
}

//-------------------------------------------------------------------------------------------------------------------
// 一级菜单
//-------------------------------------------------------------------------------------------------------------------
void menu1(void)
{
  uint8 line_index;
  ips114_show_string (48, 16*1, "1.Set");
  ips114_show_string (48, 16*2, "2.DL");
  ips114_show_string (48, 16*3, "3.SL");
  ips114_show_string (48, 16*4, "4.IMG");
  ips114_show_string (48, 16*5, "5.SUB");
  ips114_show_string (48, 16*6, "6.Path");
  ips114_show_string (48, 16*8, "V:");
  ips114_show_float(30, 16*8, valtage_now, 3, 3);

  if(key == DOWN)
  {
    menu1_flag++;
    menu1_flag > 5 ? menu1_flag = 0 : menu1_flag;
    ips114_clear();
  }
  else if(key == UP)
  {
    menu1_flag--;
    menu1_flag < 0 ? menu1_flag = 5 : menu1_flag;
    ips114_clear();
  }

  line_index = menu1_flag + 1;
  ips114_show_string(0, 16*line_index, "--->");

  if(key == PRESS)
  {
    switch(menu1_flag)
    {
     case 0: menu = MENU2_1; break;
     case 1: menu = MENU2_2; break;
     case 2: menu = MENU2_3; break;
     case 3: menu = MENU2_4; break;
     case 4: menu = MENU2_5; break;
     case 5: menu = MENU2_6; break;
    }
    menu2_flag = 0;
    ips114_clear();
  }
}

//-------------------------------------------------------------------------------------------------------------------
// 参数高亮显示函数 (ips114)
//-------------------------------------------------------------------------------------------------------------------
void show_parameter_with_highlight(uint16_t x, uint16_t y, float value,
                                  uint8_t digit_pos, uint8_t int_digits)
{
    char str[20];
    if (fabsf(value - (int)value) < 0.001f) {
        sprintf(str, "%d.00", (int)value);
    } else if (fabsf(value * 10 - (int)(value * 10)) < 0.001f) {
        sprintf(str, "%.1f0", value);
    } else {
        sprintf(str, "%.2f", value);
    }

    uint8_t total_chars = strlen(str);
    uint8_t decimal_index = 0;

    for (uint8_t i = 0; i < total_chars; i++) {
        if (str[i] == '.') {
            decimal_index = i;
            break;
        }
    }

    uint8_t highlight_index = 0;
    if (digit_pos < int_digits) {
        highlight_index = digit_pos;
    } else if (digit_pos == int_digits) {
        highlight_index = decimal_index;
    } else {
        highlight_index = decimal_index + 1 + (digit_pos - int_digits - 1);
        if (highlight_index >= total_chars) highlight_index = total_chars - 1;
    }

    for (uint8_t i = 0; i < total_chars; i++) {
        if (i == highlight_index) {
            ips114_set_color(RGB565_RED, RGB565_WHITE);
        } else {
            ips114_set_color(RGB565_BLACK, RGB565_WHITE);
        }
        ips114_show_char(x + i * 6, y, str[i]);
    }
    ips114_set_color(RGB565_BLACK, RGB565_WHITE);
}

//-------------------------------------------------------------------------------------------------------------------
// 步进调节参数
//-------------------------------------------------------------------------------------------------------------------
float adjust_parameter(float value, uint8_t digit, int8_t direction, uint8_t int_digits)
{
    int is_integer_part = (digit < int_digits);
    float step = 0.0f;

    if (is_integer_part) {
        step = powf(10.0f, int_digits - digit - 1);
    } else if (digit > int_digits) {
        step = powf(10.0f, -(digit - int_digits));
    }

    float new_value = value + (direction * step);
    new_value = CLAMP(new_value, 0.0f, 2000.00f);

    return new_value;
}

//-------------------------------------------------------------------------------------------------------------------
// 获取整数位数
//-------------------------------------------------------------------------------------------------------------------
uint8_t get_integer_digits(float value)
{
    if (value < 0) value = -value;
    int integer_part = (int)value;

    if (integer_part == 0) return 1;

    uint8_t digits = 0;
    while (integer_part > 0) {
        digits++;
        integer_part /= 10;
    }
    return digits;
}

//-------------------------------------------------------------------------------------------------------------------
// 子菜单1：发车
//-------------------------------------------------------------------------------------------------------------------
void menu2_1(void)
{
    ips114_show_string(48, 16*2, "P2Go!");

    if (key == PRESS) {
        if (run_mode == 0) {
            INS_Clear_Map();
        } else {
            Load_Trajectory_From_Flash();
        }

        INS_Init();
        Motor_Reset_State();

        fork_num = 0;
        stop_flag = 0;
        set_f = 1;
    } else if (key == LEFT) {
        menu = MENU1_1;
        ips114_clear();
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 转向环调试菜单 (基础Kp, 基础Kd, 平方项系数Kp2, 备用Kd, 陀螺仪GKD)
//-------------------------------------------------------------------------------------------------------------------
void menu2_2(void)
{
    static uint8_t edit_mode = 0;
    static uint8_t digit_pos = 0;
    static float last_value = 0;
    static uint8_t int_digits = 0;
    static uint8_t total_digits = 0;

    // 显示参数名称
    ips114_show_string(30, 16*0, "bKp");
    ips114_show_string(30, 16*1, "bKd");
    ips114_show_string(30, 16*2, "Kp2"); // 【修改】：显示名改为 Kp2
    ips114_show_string(30, 16*3, "sKd");
    ips114_show_string(30, 16*4, "Gkd");

    if (!edit_mode) {
        for (uint8_t i = 0; i < 5; i++) {
            float value = (i == 0) ? turnPID.Kp :
                          (i == 1) ? turnPID.Kd :
                          (i == 2) ? steer_Kp2 : // 【修改】：显示 steer_Kp2
                          (i == 3) ? SCALE_KD :
                          turnPID.GKD;

            char str[20];
            if (fabsf(value - (int)value) < 0.001f) {
                sprintf(str, "%d.00", (int)value);
            } else if (fabsf(value * 10 - (int)(value * 10)) < 0.001f) {
                sprintf(str, "%.1f0", value);
            } else {
                sprintf(str, "%.2f", value);
            }
            ips114_show_string(70, 16*i, str);
        }

        if (key == DOWN) {
            menu2_flag = (menu2_flag + 1) % 5;
            ips114_clear();
        }
        else if (key == UP) {
            menu2_flag = (menu2_flag + 4) % 5;
            ips114_clear();
        }
        else if (key == PRESS) {
            edit_mode = 1;
            digit_pos = 0;

            float current_value = (menu2_flag == 0) ? turnPID.Kp :
                                  (menu2_flag == 1) ? turnPID.Kd :
                                  (menu2_flag == 2) ? steer_Kp2 : // 【修改】：修改 steer_Kp2
                                  (menu2_flag == 3) ? SCALE_KD :
                                  turnPID.GKD;

            last_value = current_value;
            int_digits = get_integer_digits(current_value);
            total_digits = int_digits + 3;
            ips114_clear();
        }
        else if (key == LEFT) {
            menu = MENU1_1;
            memset(params, 0, 40*sizeof(float));

            // 保存转向PID结构体
            memcpy(params, &turnPID, sizeof(SteeringPID));

            // 【修改】：保存平方项系数到原本 SCALE_KP 的位置
            int offset = sizeof(SteeringPID) / 4;
            params[offset] = steer_Kp2;
            params[offset+1] = SCALE_KD;

            param_w_flash(DIR_PAGE);

            ips114_show_string(100, 16*6, "SAVE");
            system_delay_ms(500);

            ips114_clear();
            return;
        }

        ips114_show_string(0, 16*menu2_flag, "->");
    }
    else {
        float *current_param = NULL;
        switch (menu2_flag) {
            case 0: current_param = &turnPID.Kp; break;
            case 1: current_param = &turnPID.Kd; break;
            case 2: current_param = &steer_Kp2;  break; // 【修改】：绑定 steer_Kp2
            case 3: current_param = &SCALE_KD;   break;
            case 4: current_param = &turnPID.GKD; break;
        }

        int_digits = get_integer_digits(*current_param);
        total_digits = int_digits + 3;

        for (uint8_t i = 0; i < 5; i++) {
            float value = (i == 0) ? turnPID.Kp :
                          (i == 1) ? turnPID.Kd :
                          (i == 2) ? steer_Kp2 : // 【修改】
                          (i == 3) ? SCALE_KD :
                          turnPID.GKD;

            if (i == menu2_flag) {
                show_parameter_with_highlight(70, 16*i, value, digit_pos, int_digits);
            } else {
                char str[20];
                if (fabsf(value - (int)value) < 0.001f) {
                    sprintf(str, "%d.00", (int)value);
                } else if (fabsf(value * 10 - (int)(value * 10)) < 0.001f) {
                    sprintf(str, "%.1f0", value);
                } else {
                    sprintf(str, "%.2f", value);
                }
                ips114_show_string(70, 16*i, str);
            }
        }

        if (key == UP) {
            if (digit_pos != int_digits) {
                *current_param = adjust_parameter(*current_param, digit_pos, 1, int_digits);
            }
        }
        else if (key == DOWN) {
            if (digit_pos != int_digits) {
                *current_param = adjust_parameter(*current_param, digit_pos, -1, int_digits);
            }
        }
        else if (key == RIGHT) {
            digit_pos = (digit_pos + 1) % total_digits;
        }
        else if (key == LEFT) {
            digit_pos = (digit_pos - 1 + total_digits) % total_digits;
        }
        else if (key == PRESS) {
            edit_mode = 0;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 子菜单3：速度环调试 (调节 basic_speed 及左右电机的 Kp, Ki)
//-------------------------------------------------------------------------------------------------------------------
void menu2_3(void)
{
    static uint8_t edit_mode = 0;
    static uint8_t digit_pos = 0;
    static float last_value = 0;
    static uint8_t int_digits = 0;
    static uint8_t total_digits = 0;

    // 显示名称
    ips114_show_string(30, 16*0, "spd");
    ips114_show_string(30, 16*1, "lkp");
    ips114_show_string(30, 16*2, "lki");
    ips114_show_string(30, 16*3, "rkp");
    ips114_show_string(30, 16*4, "rki");

    if (!edit_mode) {
        for (uint8_t i = 0; i < 5; i++) {
            float value = (i == 0) ? basic_speed :
                          (i == 1) ? pid_left.Kp :
                          (i == 2) ? pid_left.Ki :
                          (i == 3) ? pid_right.Kp :
                          pid_right.Ki;

            char str[20];
            if (fabsf(value - (int)value) < 0.001f) {
                sprintf(str, "%d.00", (int)value);
            } else if (fabsf(value * 10 - (int)(value * 10)) < 0.001f) {
                sprintf(str, "%.1f0", value);
            } else {
                sprintf(str, "%.2f", value);
            }
            ips114_show_string(70, 16*i, str);
        }

        if (key == DOWN) {
            menu2_flag = (menu2_flag + 1) % 5;
            ips114_clear();
        }
        else if (key == UP) {
            menu2_flag = (menu2_flag + 4) % 5;
            ips114_clear();
        }
        else if (key == PRESS) {
            edit_mode = 1;
            digit_pos = 0;

            float current_value = (menu2_flag == 0) ? basic_speed :
                                  (menu2_flag == 1) ? pid_left.Kp :
                                  (menu2_flag == 2) ? pid_left.Ki :
                                  (menu2_flag == 3) ? pid_right.Kp :
                                  pid_right.Ki;

            last_value = current_value;
            int_digits = get_integer_digits(current_value);
            total_digits = int_digits + 3;
            ips114_clear();
        }
        else if (key == LEFT) {
            menu = MENU1_1;
            memset(params, 0, 40*sizeof(float));
            params[0] = basic_speed;
            memcpy(&params[1], &pid_left, sizeof(IncPID));
            memcpy(&params[1 + sizeof(IncPID)/4], &pid_right, sizeof(IncPID));
            param_w_flash(SPEED_PAGE);

            ips114_show_string(100, 16*6, "SAVE");
            system_delay_ms(500);

            ips114_clear();
            return;
        }
        ips114_show_string(0, 16*menu2_flag, "--->");
    }
    else {
        float *current_param = NULL;
        switch (menu2_flag) {
            case 0: current_param = &basic_speed; break;
            case 1: current_param = &pid_left.Kp; break;
            case 2: current_param = &pid_left.Ki; break;
            case 3: current_param = &pid_right.Kp; break;
            case 4: current_param = &pid_right.Ki; break;
        }

        int_digits = get_integer_digits(*current_param);
        total_digits = int_digits + 3;

        for (uint8_t i = 0; i < 5; i++) {
            float value = (i == 0) ? basic_speed :
                          (i == 1) ? pid_left.Kp :
                          (i == 2) ? pid_left.Ki :
                          (i == 3) ? pid_right.Kp :
                          pid_right.Ki;

            if (i == menu2_flag) {
                show_parameter_with_highlight(70, 16*i, value, digit_pos, int_digits);
            } else {
                char str[20];
                if (fabsf(value - (int)value) < 0.001f) {
                    sprintf(str, "%d.00", (int)value);
                } else if (fabsf(value * 10 - (int)(value * 10)) < 0.001f) {
                    sprintf(str, "%.1f0", value);
                } else {
                    sprintf(str, "%.2f", value);
                }
                ips114_show_string(70, 16*i, str);
            }
        }

        if (key == UP) {
            if (digit_pos != int_digits) {
                *current_param = adjust_parameter(*current_param, digit_pos, 1, int_digits);
            }
        }
        else if (key == DOWN) {
            if (digit_pos != int_digits) {
                *current_param = adjust_parameter(*current_param, digit_pos, -1, int_digits);
            }
        }
        else if (key == RIGHT) {
            digit_pos = (digit_pos + 1) % total_digits;
        }
        else if (key == LEFT) {
            digit_pos = (digit_pos - 1 + total_digits) % total_digits;
        }
        else if (key == PRESS) {
            edit_mode = 0;
        }
    }
}
//-------------------------------------------------------------------------------------------------------------------
// 子菜单4：图像显示
//-------------------------------------------------------------------------------------------------------------------

void menu2_4(void)
{

    if(mt9v03x_finish_flag)
    {
//        system_start();
        static uint8_t current_threshold = 128; // 缓存的阈值
        static uint8_t frame_count = 0;         // 帧计数器

        // 第0帧，或者每跑满50帧，重新计算一次阈值

        if (frame_count == 0) {
            current_threshold = otsuThreshold_fast();
        }
                // 帧计数器累加，到达50后归零
        frame_count++;
        if (frame_count >= 0) {
            frame_count = 0;
        }

        binarization(current_threshold);

        ips114_displayimage03x((const uint8 *)mt9v03x_image_bin, 135, 86);





        search_line_main();

        //element_update();
        state_update();
        caculate_error();
        //show_state();
//        show_line();
//        show_target_point(M,L,R);
//        uint32 tim = system_getval_us();
//        printf("Timer counted %lu us.\r\n", tim);

        ips114_show_int(0,86,return_error(),3);
        //ips114_draw_line(67,0,67,86,RGB565_RED);



        mt9v03x_finish_flag = 0;
    }

}
//-------------------------------------------------------------------------------------------------------------------
// 子菜单5：科目选择
//-------------------------------------------------------------------------------------------------------------------
void menu2_5(void)
{
    ips114_show_string(48, 16*1, " 1");
    ips114_show_string(48, 16*2, " 2");
    ips114_show_string(48, 16*3, " 3");

    // 显示当前选中项的指示符
    ips114_show_string(0, 16*(menu2_flag + 1), "--->");

    // 右上角提示当前已生效科目
    char str[20];
    sprintf(str, "Cur:%d", subject_id);
    ips114_show_string(40, 0, str);

    if(key == DOWN)
    {
        menu2_flag++;
        menu2_flag > 2 ? menu2_flag = 0 : menu2_flag;
        ips114_clear();
    }
    else if(key == UP)
    {
        menu2_flag--;
        menu2_flag < 0 ? menu2_flag = 2 : menu2_flag;
        ips114_clear();
    }
    else if(key == PRESS)
    {
        // 确认选中并保存
                subject_id = menu2_flag + 1;

                // --- 【新增】将当前选中的科目保存至 Flash ---
        memset(params, 0, 40*sizeof(float)); // 清空缓冲区
        params[0] = (float)subject_id;       // 将科目ID存入缓冲区
        param_w_flash(SUB_PAGE);           // 写入 IMAGE_PAGE

        ips114_show_string(80, 16 * 6, "SAVED!"); // 提示已保存
        system_delay_ms(300);
        ips114_clear();

//        subject_id = menu2_flag + 1;
//        ips114_show_string(140, 16*6, "OK!");
//        system_delay_ms(300);
//        ips114_clear();
    }
    else if(key == LEFT)
    {
        menu = MENU1_1;
        ips114_clear();
    }

}

//-------------------------------------------------------------------------------------------------------------------
// 子菜单6：路书与全局速度管理 (支持按位修改)
//-------------------------------------------------------------------------------------------------------------------
void menu2_6(void)
{
    extern int min_target_speed;
    extern int max_target_speed;

    static uint8_t edit_mode = 0;
    static uint8_t digit_pos = 0;
    static uint8_t int_digits = 0;
    static uint8_t total_digits = 0;

    char str[20];

    // ================== 非编辑模式 (浏览菜单) ==================
    if (!edit_mode) {
        ips114_show_string(10, 16*0, "Mode:");
        if(run_mode == 0) ips114_show_string(60, 16*0, "Record");
        else              ips114_show_string(60, 16*0, "Race  ");

        ips114_show_string(10, 16*1, "Load Map");
        ips114_show_string(10, 16*2, "Save Map");

        sprintf(str, "MaxV: %d   ", max_target_speed);
        ips114_show_string(10, 16*3, str);

        sprintf(str, "MinV: %d   ", min_target_speed);
        ips114_show_string(10, 16*4, str);

        ips114_show_string(10, 16*5, "Re-Plan Vel");

        sprintf(str, "Pts: %d   ", Total_Path_Points);
        ips114_show_string(10, 16*6, str);

        ips114_show_string(0, 16*menu2_flag, ">");

        if (key == DOWN) {
            menu2_flag = (menu2_flag + 1) % 6;
            ips114_clear();
        }
        else if (key == UP) {
            menu2_flag = (menu2_flag + 5) % 6;
            ips114_clear();
        }
        else if (key == LEFT) {
            // 退出并保存到 Flash
            menu = MENU1_1;
            memset(params, 0, 40*sizeof(float));
            params[0] = basic_speed;
            memcpy(&params[1], &pid_left, sizeof(IncPID));
            memcpy(&params[1 + sizeof(IncPID)/4], &pid_right, sizeof(IncPID));
            int offset_spd = 1 + (sizeof(IncPID)/4)*2;
            params[offset_spd] = (float)max_target_speed;
            params[offset_spd+1] = (float)min_target_speed;
            param_w_flash(SPEED_PAGE);

            ips114_show_string(100, 16*7, "SAVE");
            system_delay_ms(500);
            ips114_clear();
        }
        else if (key == PRESS) {
            if(menu2_flag == 0) {
                run_mode = !run_mode;
                ips114_clear();
            }
            else if(menu2_flag == 1) {
                ips114_show_string(80, 16*1, "Wait");
                Load_Trajectory_From_Flash();
                ips114_clear();
            }
            else if(menu2_flag == 2) {
                ips114_show_string(80, 16*2, "Wait");
                Save_Trajectory_To_Flash();
                system_delay_ms(500);
                ips114_clear();
            }
            else if(menu2_flag == 3 || menu2_flag == 4) {
                // 【进入编辑模式】
                edit_mode = 1;
                digit_pos = 0;
                float current_value = (menu2_flag == 3) ? (float)max_target_speed : (float)min_target_speed;
                int_digits = get_integer_digits(current_value);
                total_digits = int_digits + 3;
                ips114_clear();
            }
            else if(menu2_flag == 5) {
                ips114_show_string(100, 16*5, "Calc");
                INS_Generate_Velocity_Table(min_target_speed, max_target_speed);
                system_delay_ms(500);
                ips114_clear();
            }
        }
    }
    // ================== 编辑模式 (修改数值) ==================
    else {
        // 重绘静态文本
        ips114_show_string(10, 16*0, "Mode:");
        if(run_mode == 0) ips114_show_string(60, 16*0, "Record");
        else              ips114_show_string(60, 16*0, "Race  ");
        ips114_show_string(10, 16*1, "Load Map");
        ips114_show_string(10, 16*2, "Save Map");
        ips114_show_string(10, 16*5, "Re-Plan Vel");
        sprintf(str, "Pts: %d   ", Total_Path_Points);
        ips114_show_string(10, 16*6, str);
        ips114_show_string(0, 16*menu2_flag, ">");

        // 提取当前正在编辑的值 (转为浮点数方便复用底层高亮和步进函数)
        float current_val = (menu2_flag == 3) ? (float)max_target_speed : (float)min_target_speed;

        // 动态计算位数 (防止从 99 变成 100 时错位)
        int_digits = get_integer_digits(current_val);
        total_digits = int_digits + 3;

        // 绘制高亮项与非高亮项
        if (menu2_flag == 3) {
            ips114_show_string(10, 16*3, "MaxV: ");
            show_parameter_with_highlight(10 + 6*6, 16*3, current_val, digit_pos, int_digits);
            sprintf(str, "MinV: %d   ", min_target_speed);
            ips114_show_string(10, 16*4, str);
        } else {
            sprintf(str, "MaxV: %d   ", max_target_speed);
            ips114_show_string(10, 16*3, str);
            ips114_show_string(10, 16*4, "MinV: ");
            show_parameter_with_highlight(10 + 6*6, 16*4, current_val, digit_pos, int_digits);
        }

        // 按键处理逻辑
        if (key == UP) {
            if (digit_pos != int_digits) { // 跳过小数点
                current_val = adjust_parameter(current_val, digit_pos, 1, int_digits);
            }
        }
        else if (key == DOWN) {
            if (digit_pos != int_digits) {
                current_val = adjust_parameter(current_val, digit_pos, -1, int_digits);
            }
        }
        else if (key == RIGHT) {
            digit_pos = (digit_pos + 1) % total_digits;
        }
        else if (key == LEFT) {
            digit_pos = (digit_pos - 1 + total_digits) % total_digits;
        }
        else if (key == PRESS) {
            edit_mode = 0; // 退出编辑模式
            ips114_clear();
        }

        // 把修改后的值转回整型并保存
        if (menu2_flag == 3) {
            max_target_speed = (int)current_val;
        } else {
            min_target_speed = (int)current_val;
        }
    }
}

void param_w_flash(uint8 page)
{
    // 参数含义: sector恒写0, 页码page, 数据源地址, 长度(单位是 uint32 即 float 的个数)
    flash_write_page(0, page, (const uint32 *)params, 40);
}


void param_r_flash(uint8 page)
{
    uint8 need_init = 0;

    // 1. 检查Flash对应页是否有数据 (返回1说明有数据)
    if (flash_check(0, page) == 1)
    {
        // 2. 有数据则读取出来
        flash_read_page(0, page, (uint32 *)params, 40);

        // 3. 检查数据是否合法 (防止擦除后未写入时读出全0xFF导致NaN)
        if (isnan(params[0]) || isnan(params[1]))
        {
            need_init = 1;
        }
    }
    else
    {
        // 返回0说明没有数据
        need_init = 1;
    }

    // 4. 如果需要初始化，根据传入的page，把原本在代码里的默认值重新打包写入Flash修复
    if (need_init)
    {
        memset(params, 0, 40 * sizeof(float));

        switch (page)
        {
            case SPEED_PAGE:
                params[0] = basic_speed;
                memcpy(&params[1], &pid_left, sizeof(IncPID));
                memcpy(&params[1 + sizeof(IncPID)/4], &pid_right, sizeof(IncPID));
                break;

            case DIR_PAGE:
                memcpy(params, &turnPID, sizeof(SteeringPID));
                break;

            case IMAGE_PAGE:
                // TODO: 如果后续有图像参数，在这里填补默认值打包逻辑
                break;
        }

        // 写回Flash进行修复初始化
        param_w_flash(page);
    }
}
