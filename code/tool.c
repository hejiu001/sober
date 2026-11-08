/**************************************************************
 * 锟侥硷拷锟斤拷:  tool.c
 * 锟斤拷锟斤拷时锟斤拷: 2026锟斤拷2锟斤拷19锟斤拷  锟斤拷锟斤拷9:55:57
 * 锟斤拷锟斤拷: 通锟矫癸拷锟竭猴拷锟斤拷锟斤拷PID锟斤拷锟姐、锟剿诧拷锟斤拷图锟斤拷锟斤拷示锟饺ｏ拷
 * 2026.4.7 锟斤拷锟斤拷IPS114锟斤拷幕
 **************************************************************/
//#include "zf_common_headfile.h"
#include "tool.h"
#include "zf_device_ips114.h"   // 锟斤拷锟斤拷 IPS114 锟斤拷锟斤拷

// 全锟斤拷时锟戒步锟斤拷锟斤拷PID锟斤拷锟斤拷锟矫ｏ拷
float dt = 0.006f;

// 锟斤拷锟皆匡拷锟截ｏ拷0=锟截闭ｏ拷1=锟斤拷锟斤拷锟斤拷
#define DEBUG_PRINT 0

//float GZ;
/**************************************************************
 * 锟斤拷锟斤拷: 锟斤拷示锟斤拷色锟斤拷锟截碉拷
 * 锟斤拷锟斤拷: x/y - 锟斤拷锟斤拷锟斤拷锟斤拷
 * 说锟斤拷: 锟斤拷锟疥范围锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷越锟斤拷
 **************************************************************/
void show_point_red(int x, int y)
{
    if (x > 0 && x < MT9V03X_W && y > 0 && y < MT9V03X_H)
    {
        ips114_draw_point((uint16)x, (uint16)y, RGB565_RED);
    }
}

/**************************************************************
 * 锟斤拷锟斤拷: 锟斤拷示锟斤拷色锟斤拷锟截碉拷
 * 锟斤拷锟斤拷: x/y - 锟斤拷锟斤拷锟斤拷锟斤拷
 **************************************************************/
void show_point_blue(int x, int y)
{
    if (x > 0 && x < MT9V03X_W && y > 0 && y < MT9V03X_H)
    {
        ips114_draw_point((uint16)x, (uint16)y, RGB565_BLUE);
    }
}

/**************************************************************
 * 锟斤拷锟斤拷: 锟斤拷示锟斤拷色锟斤拷锟截碉拷
 * 锟斤拷锟斤拷: x/y - 锟斤拷锟斤拷锟斤拷锟斤拷
 **************************************************************/
void show_point_yellow(int x, int y)
{
    if (x > 0 && x < MT9V03X_W && y > 0 && y < MT9V03X_H)
    {
        ips114_draw_point((uint16)x, (uint16)y, RGB565_YELLOW);
    }
}

/**************************************************************
 * 锟斤拷锟斤拷: 锟斤拷锟斤拷锟斤拷锟斤拷
 * 锟斤拷锟斤拷: x/y - 锟斤拷锟斤拷锟斤拷锟斤拷
 **************************************************************/
void show_point(int x, int y)
{
    if (x > 10 && x < MT9V03X_W-10 && y > 10 && y < MT9V03X_H-10)
    {
        for(int i=-1;i<2;i++){
        ips114_draw_line((uint16)x-10, (uint16)y+i,(uint16)x+10, (uint16)y+i,RGB565_RED);
        ips114_draw_line((uint16)x+i, (uint16)y-10,(uint16)x+i, (uint16)y+10,RGB565_RED);
        }
    }
}

/**************************************************************
 * 锟斤拷锟斤拷: 位锟斤拷式PD锟斤拷锟斤拷锟斤拷锟斤拷转锟斤拷锟斤拷锟阶拷茫锟�
 * 锟斤拷锟斤拷: pid - PD锟斤拷锟斤拷锟斤拷锟结构锟斤拷指锟斤拷
 *       setpoint - 目锟斤拷值锟斤拷路锟斤拷偏锟斤拷目锟疥）
 *       actual - 实锟斤拷值锟斤拷锟斤拷前路锟斤拷偏锟筋）
 * 锟斤拷锟斤拷: PD锟斤拷锟斤拷锟斤拷锟街碉拷锟阶拷锟絇WM锟斤拷锟斤拷锟斤拷锟斤拷
 * 锟斤拷式: output = Kp*e(t) + Kd*[e(t)-e(t-1)]/dt
 **************************************************************/
float steering_pd_control(SteeringPID *pid, float setpoint, float actual)
{
    // 锟斤拷锟姐当前锟斤拷锟�
    float current_error = setpoint - actual;
    float GZ = -imu660rc_gyro_transition(imu660rc_gyro_z);

    // 1. 锟斤拷锟斤拷原始微锟斤拷锟斤拷 (锟斤拷锟戒化锟斤拷)
    float raw_derivative = (current_error - pid->last_error) / dt;

    // 2. 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟狡★拷锟斤拷锟皆硷拷锟斤拷锟斤拷锟叫碉拷微锟斤拷锟斤拷锟斤拷锟斤拷通锟剿诧拷
    // 专锟斤拷 5ms 锟斤拷频锟斤拷锟斤拷锟铰碉拷锟斤拷锟斤拷头/锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟侥筹拷锟斤拷锟斤拷锟斤拷
    static float filtered_derivative = 0.0f;
    // 0.3锟斤拷锟斤拷锟斤拷锟饺ｏ拷0.7锟斤拷平锟斤拷锟饺★拷锟斤拷锟斤拷锟斤拷锟矫筹拷锟斤拷锟角讹拷锟斤拷锟斤拷0.3锟斤拷小(锟斤拷0.15)锟斤拷
    filtered_derivative = 0.6f * raw_derivative + 0.4f * filtered_derivative;

    // 锟斤拷锟斤拷锟斤拷锟斤拷锟绞�
    pid->prev_error = pid->last_error;
    pid->last_error = current_error;
    pid->error = current_error;

    // 锟斤拷取锟斤拷锟斤拷锟角斤拷锟劫讹拷


    // 3. PD锟斤拷锟斤拷锟斤拷锟� (注锟斤拷锟斤拷锟斤拷使锟矫碉拷锟斤拷锟剿诧拷锟斤拷锟轿拷锟斤拷锟� filtered_derivative)
    float output = pid->Kp * current_error + pid->Kd * filtered_derivative + pid->GKD * GZ;

    return output;
}

/**************************************************************
 * 锟斤拷锟斤拷: 锟斤拷锟斤拷式PID锟斤拷锟姐（锟斤拷锟斤拷俣瓤锟斤拷锟阶拷茫锟�
 * 锟斤拷锟斤拷: pid - 锟斤拷锟斤拷式PID锟斤拷锟斤拷锟斤拷锟结构锟斤拷指锟斤拷
 *       current_speed - 锟斤拷前实锟斤拷锟劫度ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
 * 锟斤拷锟斤拷: 锟斤拷锟斤拷PID锟斤拷锟斤拷锟斤拷锟斤拷锟絇WM锟斤拷锟斤拷锟斤拷锟斤拷
 * 锟斤拷式: 锟斤拷u(t) = Kp*[e(t)-e(t-1)] + Ki*e(t) + Kd*[e(t)-2e(t-1)+e(t-2)]
 **************************************************************/
int IncPID_Calculate(IncPID *pid, int current_speed)
{
    if (pid == NULL) return 0;

    int error = pid->target - current_speed;
    float p_term = pid->Kp * (error - pid->last_error);
    float i_term = pid->Ki * error;
    float d_term = pid->Kd * (error - 2 * pid->last_error + pid->prev_error);

    int increment = (int)(p_term + i_term + d_term);
    int output = pid->last_output + increment;

    pid->prev_error = pid->last_error;
    pid->last_error = error;
    pid->last_output = output; // 锟斤拷时锟斤拷锟饺ワ拷木锟斤拷锟斤拷前锟饺拷锟斤拷锟街�

    return output;
}


float angle_difference(float angle1, float angle2) {
    // 锟斤拷锟斤拷直锟接诧拷值
    float diff = fabs(angle1 - angle2);

    // 锟斤拷锟斤拷锟街碉拷锟斤拷锟�180锟饺ｏ拷取360-锟斤拷值为锟斤拷小锟角度诧拷
    if (diff > 180.0) {
        diff = 360.0 - diff;
    }

    return diff;
}
#define DOB_LIMIT 2000
float DOB_Calculate(DOB_Controller *dob, int current_speed)
{
    // 1. 锟斤拷模锟酵硷拷锟斤拷 (锟斤拷锟斤拷锟较达到锟斤拷前锟劫讹拷状态锟斤拷要锟斤拷PWM)
    float speed_dot = (current_speed - dob->last_speed) / dob->dt;
    float u_ideal = (dob->T_nom / dob->K_nom) * speed_dot + (1.0f / dob->K_nom) * (float)current_speed;

    // 2. 锟斤拷锟斤拷原始锟脚讹拷
    float raw_dist = u_ideal - dob->last_pwm;

    // 3. Q-filter
    dob->dob_comp = dob->Q_alpha * raw_dist + (1.0f - dob->Q_alpha) * dob->dob_comp;

    // ==========================================

    // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟絇WM锟斤拷5000锟斤拷锟斤拷锟斤拷DOB锟斤拷锟街伙拷懿锟斤拷锟� 锟斤拷2500锟斤拷
    // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷恕锟轿拷直锟秸拷耍锟斤拷锟揭诧拷锟斤拷锟斤拷锟斤拷锟斤拷锟�
    if (dob->dob_comp > DOB_LIMIT) {
        dob->dob_comp = DOB_LIMIT;
    }
    if (dob->dob_comp < -DOB_LIMIT) {
        dob->dob_comp = -DOB_LIMIT;
    }
    // ==========================================

    // 4. 锟斤拷锟斤拷锟斤拷史状态锟斤拷锟斤拷一锟斤拷使锟斤拷
    dob->last_speed = current_speed;

    return dob->dob_comp;
}

// =========================================================================
// ==================== 专锟斤拷锟斤拷锟劫凤拷锟斤拷锟斤拷模锟斤拷锟斤拷锟狡诧拷锟斤拷锟斤拷 =======================
// =========================================================================


// 2. 锟斤拷锟斤拷锟斤拷锟斤拷 (锟斤拷锟斤拷锟斤拷锟斤拷锟接筹拷涞� [-3, 3] 锟斤拷位)
#define QUANTIZE_E   0.1f   // 锟角讹拷锟斤拷锟斤拷锟斤拷锟� (锟斤拷锟斤拷约为45锟斤拷)
#define QUANTIZE_EC  0.003f  // 锟角度变化锟斤拷锟斤拷锟斤拷

// 3. 锟斤拷态锟斤拷锟斤拷锟斤拷锟斤拷 (锟斤拷锟斤拷时唯一锟斤拷要锟睫改碉拷锟斤拷锟斤拷值锟斤拷)
float SCALE_KP = 8.0f;       // Kp锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
float SCALE_KD = 0.12f;       // Kd锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷

const float delta_Kp_table[7][7] = {
 // ec:NB    NM    NS    ZO    PS    PM    PB
    { 0.7,  0.7,  0.6,  0.6,  0.4,  0.2,  0.0}, // NB (锟斤拷锟斤拷锟斤拷锟斤拷牵锟斤拷锟斤拷旨锟斤拷锟� Kp 锟斤拷锟斤拷锟斤拷锟斤拷)
    { 0.5,  0.5,  0.4,  0.4,  0.2,  0.0, -0.2}, // NM
    { 0.4,  0.4,  0.3,  0.3,  0.1,  0.0, -0.3}, // NS (小锟斤拷偏锟襟：癸拷锟较革拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷)
    {-0.3, -0.4, -0.4, -0.5, -0.4, -0.4, -0.3}, // ZO (锟斤拷锟斤拷直锟斤拷锟斤拷锟捷狠斤拷锟斤拷 Kp锟斤拷锟斤拷锟斤拷锟斤拷一锟斤拷直锟竭ｏ拷锟斤拷止锟斤拷锟斤拷)
    {-0.3,  0.0,  0.1,  0.3,  0.3,  0.4,  0.4}, // PS (小锟斤拷偏锟揭ｏ拷锟斤拷锟较革拷锟斤拷锟斤拷锟斤拷锟斤拷瞬锟斤拷锟金反匡拷扭锟斤拷)
    {-0.2,  0.0,  0.2,  0.4,  0.4,  0.5,  0.5}, // PM
    { 0.0,  0.2,  0.4,  0.6,  0.6,  0.7,  0.7}  // PB (锟斤拷锟揭达拷锟斤拷锟角ｏ拷锟斤拷锟街硷拷锟斤拷 Kp 锟斤拷锟�90锟饺碉拷头)
};

// 5. Kd锟斤拷锟斤拷锟斤拷 (锟斤拷锟斤拷锟斤拷锟斤拷锟截伙拷锟芥：直锟斤拷锟斤拷锟斤拷锟结，锟斤拷锟斤拷卸锟斤拷锟结，锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷)
const float delta_Kd_table[7][7] = {
 // ec:NB    NM    NS    ZO    PS    PM    PB
    {-0.4, -0.2,  0.0,  0.1,  0.4,  0.5,  0.6}, // NB
    {-0.2, -0.1,  0.1,  0.2,  0.3,  0.4,  0.5}, // NM
    { 0.0,  0.1,  0.2,  0.3,  0.4,  0.3,  0.2}, // NS
    { 0.1,  0.2,  0.3,  0.4,  0.3,  0.2,  0.1}, // ZO (直锟斤拷)
    { 0.2,  0.3,  0.4,  0.3,  0.2,  0.1,  0.0}, // PS
    { 0.5,  0.4,  0.3,  0.2,  0.1, -0.1, -0.2}, // PM
    { 0.6,  0.5,  0.4,  0.1,  0.0, -0.2, -0.4}  // PB
};
// ==============================================================
// 目锟疥：锟斤拷锟斤拷模锟斤拷锟竭硷拷锟侥革拷锟斤拷锟解环转锟斤拷锟斤拷锟� (锟斤拷锟皆拷械锟� steering_pd_control)
//锟睫革拷为锟斤拷锟斤拷锟斤拷锟斤拷锟芥本锟斤拷锟斤拷时锟斤拷锟斤拷模锟斤拷
// ==============================================================
float steer_Kp2 = 0.00f;
float steering_fuzzy_pd_control(SteeringPID *pid, float setpoint, float actual)
{
    float current_error = setpoint - actual;
    float abs_e = fabsf(current_error);
    float GZ = -imu660rc_gyro_transition(imu660rc_gyro_z);

    // 微锟斤拷锟斤拷锟斤拷锟斤拷希ㄎ⒎窒刃校锟斤拷只锟斤拷实锟斤拷值锟斤拷锟斤拷锟斤拷
    float raw_derivative = -(actual - pid->last_actual) / dt;
    static float filtered_derivative = 0.0f;
    filtered_derivative = 0.8f * raw_derivative + 0.2f * filtered_derivative;

    // 锟斤拷锟斤拷状态
    pid->prev_error = pid->last_error;
    pid->last_error = current_error;
    pid->last_actual = actual;
    pid->error = current_error;

    // ==========================================================
    // 锟斤拷平锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷Output = Kp*e + Kp2*e*|e| + Kd*de + Gkd*GZ
    // current_error * abs_e 实锟斤拷锟剿★拷平锟斤拷锟脚达拷同时锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟脚★拷
    // ==========================================================
    float p_linear = pid->Kp * current_error;
    float p_square = steer_Kp2 * current_error * abs_e;

    float d_term = pid->Kd * filtered_derivative;
    float g_term = pid->GKD * GZ;

    // 锟斤拷锟斤拷锟斤拷锟�
    float output = p_linear + p_square + d_term + g_term;

    // 锟斤拷锟斤拷趴锟斤拷薹锟斤拷锟斤拷钥锟�1kg锟斤拷压
    return output;
}
//-------------------------------------------------------------------------------------------------------------------
//  锟斤拷锟斤拷锟斤拷锟�      锟斤拷锟斤拷转锟街凤拷锟斤拷锟斤拷锟斤拷全实锟街ｏ拷
//  锟斤拷锟斤拷锟斤拷锟斤拷     const char*锟斤拷锟斤拷态锟斤拷锟斤拷锟斤拷锟斤拷址锟斤拷
//  锟斤拷锟斤拷说锟斤拷        value 要转锟斤拷锟斤拷锟斤拷锟斤拷值
//  使锟斤拷锟斤拷锟斤拷        const char* str = int_to_str(-12345);
//  注锟解：       锟斤拷锟截撅拷态锟斤拷锟斤拷锟斤拷锟斤拷址锟斤拷每锟轿碉拷锟矫会覆锟斤拷之前锟斤拷锟�
//              支锟斤拷锟斤拷锟�10位锟斤拷锟街ｏ拷锟斤拷锟斤拷锟脚ｏ拷
//-------------------------------------------------------------------------------------------------------------------
const char* int_to_str(int value) {

    static char buffer[12];

    int len = snprintf(buffer, sizeof(buffer), "%d", value);

    if (len < 0 || len >= sizeof(buffer)) {
        return "ERR";
    }

    return buffer;
}
//-------------------------------------------------------------------------------------------------------------------
//  锟斤拷锟斤拷锟斤拷锟�      锟斤拷锟竭达拷锟节凤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷值
//  锟斤拷锟斤拷锟斤拷锟斤拷     void
//  锟斤拷锟斤拷说锟斤拷        a 锟斤拷一锟斤拷锟斤拷锟斤拷值
//              b 锟节讹拷锟斤拷锟斤拷锟斤拷值
//  使锟斤拷锟斤拷锟斤拷        sent_two_int(current_error, pwm_output);
//  锟斤拷锟斤拷锟绞斤拷锟�    "a,b\n"
//-------------------------------------------------------------------------------------------------------------------
void sent_two_int(int a,int b){
    wireless_uart_send_string(int_to_str(a));
    wireless_uart_send_string(",");
    wireless_uart_send_string(int_to_str(b));
    wireless_uart_send_string("\n");
}
