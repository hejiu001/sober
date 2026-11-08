/**************************************************************
 * 锟侥硷拷锟斤拷:  motor.c
 * 锟斤拷锟斤拷时锟斤拷: 2026锟斤拷3锟斤拷6锟斤拷
 * 锟斤拷锟斤拷: 直锟斤拷锟斤拷锟斤拷俣瓤锟斤拷啤锟絇ID锟秸伙拷锟斤拷路锟斤拷偏锟斤拷锟桔合匡拷锟狡★拷锟斤拷锟节诧拷锟轿凤拷锟斤拷
 **************************************************************/
#include "motor.h"
#include <stdio.h>                      // 锟斤拷锟斤拷 sprintf
#include "zf_device_wireless_uart.h"    // 锟斤拷锟斤拷锟斤拷锟竭达拷锟节凤拷锟斤拷
#include "isr_config.h"
#include "isr.h"
#include "motor.h"

// 锟斤拷锟皆匡拷锟狡ｏ拷0=锟截闭ｏ拷1=锟斤拷锟斤拷
#define DEBUG_PRINT_TARGET 0
#define DEBUG_PRINT_CURRENT 0

// 硬锟斤拷锟斤拷睾甓拷锟�
#define PWM_FREQ_HZ        17000    // PWM频锟斤拷
#define PWM_MAX            9500     // PWM锟斤拷锟街�
#define encoder_MAX        1000     // 锟斤拷锟斤拷锟斤拷锟斤拷锟街�
#define PWM_MIN            0        // PWM锟斤拷小值
#define PID_OUTPUT_MAX     9500     // PID锟斤拷锟斤拷锟斤拷锟�
#define PID_OUTPUT_MIN     -8000    // PID锟斤拷锟斤拷锟斤拷锟�
#define STEER_OUTPUT_MAX   600.0f   // 转锟津补筹拷锟斤拷锟斤拷薹锟�
#define SOFT_START_MIN     100       // 锟斤拷锟斤拷锟斤拷锟斤拷伲锟斤拷锟斤拷诳朔锟斤拷锟斤拷锟斤拷摩锟斤拷
#define ACCEL_STEP         5         // 每锟斤拷执锟斤拷锟斤拷锟斤拷锟斤拷锟接碉拷锟劫度诧拷锟斤拷锟斤拷值越小锟斤拷锟斤拷越平锟斤拷锟斤拷锟缴革拷锟斤拷实锟绞匡拷锟斤拷频锟斤拷微锟斤拷锟斤拷

// 转锟斤拷锟斤拷俚姆嵌猿撇锟斤拷锟� (锟斤拷锟斤拷锟斤拷锟斤拷惴�)
#define IN_SPD_PARAM  0.4f     // 锟节诧拷锟斤拷俦锟斤拷锟斤拷锟斤拷锟�
#define DE_SPD_PARAM  0.6f     // 锟斤拷锟斤拷锟劫憋拷锟斤拷锟斤拷锟斤拷

int current_ramp_speed = 0;    // 锟斤拷前平锟斤拷目锟斤拷锟劫讹拷

// 锟劫讹拷PID锟斤拷锟斤拷
IncPID pid_left = {0.0, 0.0f, 0.0f, 0, 0, 0, 0};
IncPID pid_right = {0.0f, 0.0f, 0.0f, 0, 0, 0, 0};
// ==========================================
// 锟斤拷锟斤拷锟斤拷DOB锟斤拷锟斤拷锟斤拷始锟斤拷
// K_nom = 锟斤拷锟斤拷1600 / 锟斤拷PWM5000 = 0.32
// T_nom = 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷时锟戒常锟斤拷为 0.05 锟斤拷
// Q_alpha = 0.1 (锟斤拷0.1锟斤拷始锟斤拷锟斤拷锟斤拷锟较硷拷)
// dt = 0.005 (锟斤拷锟斤拷5ms锟斤拷锟斤拷)
// ==========================================
// DOB锟脚讹拷锟桔诧拷锟斤拷锟斤拷锟斤拷 ( K_nom,, T_nom,Q_alpha dt)
DOB_Controller dob_left  = {0.20f, 0.5f, 0.2f, 0.005f, 0, 0.0f, 0.0f};
DOB_Controller dob_right = {0.20f, 0.5f, 0.2f, 0.005f, 0, 0.0f, 0.0f};

// 转锟斤拷PD锟斤拷锟斤拷
SteeringPID turnPID = {0};

int dis_R = 0, dis_L = 0;
volatile int stop_flag = 0;

// 锟斤拷锟睫改点】锟斤拷全锟街憋拷锟斤拷锟斤拷锟斤拷锟斤拷专锟斤拷锟斤拷锟节憋拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷摹锟绞碉拷锟斤拷俣取锟斤拷锟斤拷锟斤拷锟酵局诧拷锟斤拷锟斤拷锟斤拷突
int actual_right_speed = 0;
int actual_left_speed = 0;
int right_speed = 0;
int left_speed = 0;

// 专锟斤拷锟斤拷锟节凤拷锟斤拷瞬锟斤拷锟斤拷锟斤拷锟斤拷史锟斤拷锟斤拷
void Motor_Reset_State(void) {
    // 锟斤拷锟斤拷锟斤拷旨锟斤拷锟�
    pid_left.error = pid_left.last_error = pid_left.prev_error = 0;
    pid_left.last_output = 0; // 锟斤拷锟斤拷牡锟斤拷锟戒话锟斤拷

    // 锟斤拷锟斤拷锟斤拷旨锟斤拷锟�
    pid_right.error = pid_right.last_error = pid_right.prev_error = 0;
    pid_right.last_output = 0;

    // 锟斤拷锟阶拷锟斤拷锟斤拷
    turnPID.error = turnPID.last_error = turnPID.prev_error = 0;

    current_ramp_speed = 0;
}

void negative_pressure_init(){
    pwm_init(ATOM0_CH0_P22_1, 40000, 0);
    gpio_init(P23_1, GPO, GPIO_LOW, GPO_PUSH_PULL);
}

void negative_pressure_start(){
    int target_duty = 3500;
    int min_start_duty = 1000;   // 璧峰鍗犵┖姣旓紝鐩存帴璺宠繃鏃犳硶椹卞姩鐢垫満鐨勬鍖�
    int step = 50;
    int current_duty;

    for (current_duty = min_start_duty; current_duty <= target_duty; current_duty += step)
    {
        pwm_set_duty(ATOM0_CH0_P22_1, current_duty);
        system_delay_ms(20);
    }
    pwm_set_duty(ATOM0_CH0_P22_1, target_duty);
}

void negative_pressure_stop(){
    pwm_set_duty(ATOM0_CH0_P22_1,0);
    gpio_set_level(P23_1,0);
   // gpio_set_level(P22_1, GPIO_LOW);
  //  gpio_set_level(P23_1, GPIO_LOW);
}

/**************************************************************
 * 锟斤拷锟斤拷: 锟斤拷始锟斤拷GPIO/PWM/锟斤拷锟斤拷锟斤拷
 **************************************************************/
void Motor_Init(void)
{
    // 锟揭碉拷锟斤拷锟斤拷锟斤拷锟斤拷懦锟绞硷拷锟�
    gpio_init(DIR_R1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    // 锟揭碉拷锟絇WM锟斤拷锟脚筹拷始锟斤拷
    pwm_init(PWM_R1, PWM_FREQ_HZ, 0);

    // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟脚筹拷始锟斤拷
    gpio_init(DIR_L1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    // 锟斤拷锟斤拷PWM锟斤拷锟脚筹拷始锟斤拷
    pwm_init(PWM_L1, PWM_FREQ_HZ, 0);

    // 锟斤拷始锟斤拷转锟斤拷PID锟斤拷史锟斤拷锟�
    turnPID.error = turnPID.last_error = turnPID.prev_error = 0.0f;
    /////////锟斤拷压锟斤拷始锟斤拷////////
    negative_pressure_init();
}

/**************************************************************
 * 锟斤拷锟斤拷: 锟斤拷锟斤拷锟斤拷锟斤拷PWM值
 **************************************************************/
void L_set_PWM(int Speed)
{
    // 约锟斤拷PWM锟斤拷围
    int pwm = constrain(Speed, PWM_MIN, PWM_MAX);

    // 锟斤拷锟狡凤拷锟斤拷锟斤拷俣锟�
    if (pwm > 0)
    {
        gpio_set_level(DIR_L1, GPIO_HIGH);
        pwm_set_duty(PWM_L1, pwm);
    }
//    else if (pwm < 0)
//    {
//        gpio_set_level(DIR_L1, GPIO_LOW);
//        pwm_set_duty(PWM_L1, -pwm);
//    }
//    if (pwm < 0)
//        {
//            gpio_set_level(DIR_L1, GPIO_LOW);
//            pwm_set_duty(PWM_L1, -pwm);
//        }
    else
    {
        pwm_set_duty(PWM_L1, 0);
    }
}

/**************************************************************
 * 锟斤拷锟斤拷: 锟斤拷锟斤拷锟揭碉拷锟絇WM值
 **************************************************************/
void R_set_PWM(int Speed)
{
    // 约锟斤拷PWM锟斤拷围
    int pwm = constrain(Speed, PWM_MIN, PWM_MAX);

    // 锟斤拷锟狡凤拷锟斤拷锟斤拷俣锟�
    if (pwm > 0)
    {
        gpio_set_level(DIR_R1, GPIO_HIGH);
        pwm_set_duty(PWM_R1, pwm);
    }
//    else if (pwm < 0)
//    {
//        gpio_set_level(DIR_R1, GPIO_LOW);
//        pwm_set_duty(PWM_R1, -pwm);
//    }
    else
    {
        pwm_set_duty(PWM_R1, 0);
    }
}
#define K_forward 10      // 原锟叫碉拷锟劫讹拷前锟斤拷系锟斤拷锟斤拷锟剿凤拷摩锟斤拷锟斤拷锟斤拷
#define K_accel   0.0f    // 锟斤拷锟斤拷锟侥硷拷锟劫讹拷前锟斤拷系锟斤拷锟斤拷锟剿凤拷锟斤拷锟皆ｏ拷
/**************************************************************
 * 锟斤拷锟斤拷: 锟斤拷锟斤拷锟劫度匡拷锟斤拷 (PI + 锟劫讹拷前锟斤拷 + 锟斤拷锟劫讹拷前锟斤拷)
 **************************************************************/
void speed_control_L(int target_speed_L)
{
    static int last_target_L = 0; // 锟斤拷锟节硷拷锟斤拷锟斤拷俣龋锟侥匡拷锟戒化锟绞ｏ拷

    pid_left.target = target_speed_L;
    actual_left_speed = Get_Encoder_L();
    dis_L += actual_left_speed;

    // 1. 锟斤拷锟斤拷 PI 锟斤拷锟� (锟斤拷锟斤拷式PID锟斤拷锟斤拷锟斤拷)
    int pi_pwm = IncPID_Calculate(&pid_left, actual_left_speed);


    // 锟斤拷目锟斤拷锟劫度凤拷锟斤拷锟斤拷锟斤拷时锟斤拷锟斤拷锟斤拷一锟斤拷瞬时锟斤拷锟斤拷
    float accel_ff_L = (float)(target_speed_L - last_target_L) * K_accel;
    last_target_L = target_speed_L; // 锟斤拷锟斤拷锟斤拷史值

    // 3. 锟斤拷锟斤拷锟� = PI锟斤拷锟斤拷 + 锟劫讹拷前锟斤拷 + 锟斤拷锟劫讹拷前锟斤拷
    // 注锟解：K_forward 锟斤拷锟斤拷锟斤拷锟教拷锟斤拷锟斤拷锟絘ccel_ff_L 锟斤拷锟斤拷亩锟教拷锟斤拷锟�
    int total_pwm = pi_pwm + (int)(K_forward * target_speed_L) + (int)accel_ff_L;

    total_pwm = constrain(total_pwm, PID_OUTPUT_MIN, PID_OUTPUT_MAX);


    if(stop_flag == 1){
        L_set_PWM(0);
        dob_left.last_pwm = 0.0f;
        last_target_L = 0; // 停止时锟斤拷锟斤拷
    }else{
        L_set_PWM(total_pwm);
    }
}

/**************************************************************
 * 锟斤拷锟斤拷: 锟揭碉拷锟斤拷俣瓤锟斤拷锟� (PI + 锟劫讹拷前锟斤拷 + 锟斤拷锟劫讹拷前锟斤拷)
 **************************************************************/
void speed_control_R(int target_speed_R)
{
    static int last_target_R = 0; // 锟斤拷锟节硷拷锟斤拷锟斤拷俣锟�

    pid_right.target = target_speed_R;
    actual_right_speed = Get_Encoder_R();
    dis_R += actual_right_speed;

    // 1. 锟斤拷锟斤拷 PI 锟斤拷锟�
    int pi_pwm = IncPID_Calculate(&pid_right, actual_right_speed);

    // 2. 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷俣锟角帮拷锟�
    float accel_ff_R = (float)(target_speed_R - last_target_R) * K_accel;
    last_target_R = target_speed_R;

    // 3. 锟斤拷锟斤拷锟�
    int total_pwm = pi_pwm + (int)(K_forward * target_speed_R) + (int)accel_ff_R;

    total_pwm = constrain(total_pwm, PID_OUTPUT_MIN, PID_OUTPUT_MAX);

    if(stop_flag == 1){
        R_set_PWM(0);
        last_target_R = 0;
    }else{
        R_set_PWM(total_pwm);
    }
}
/**************************************************************
 * 锟斤拷锟斤拷: 锟桔合匡拷锟狡ｏ拷锟斤拷锟斤拷锟劫讹拷 + 路锟斤拷偏锟筋补锟斤拷锟斤拷锟斤拷锟皆讹拷锟斤拷锟斤拷锟劫ｏ拷
 **************************************************************/
void combined_control(int target_base_speed, int path_deviation)
{

    static uint8_t control_div_cnt = 0;

    // --- 1. 偏锟斤拷锟酵拷瞬锟� ---
    static float filtered_deviation = 0.0f;
    const float alpha = 0.7f;
    filtered_deviation = alpha * path_deviation + (1.0f - alpha) * filtered_deviation;

    // =========================================================================
    // --- 3. 目锟斤拷锟劫度硷拷锟斤拷 (锟金步伙拷锟斤拷锟斤拷锟竭硷拷) ---
    int base_speed;

    // 锟斤拷锟斤拷目锟斤拷锟斤拷锟诫范围锟斤拷锟斤拷止锟斤拷锟�
    target_base_speed = constrain(target_base_speed, 0, encoder_MAX);

    if (stop_flag == 1 || target_base_speed == 0) {
        // 停锟斤拷状态锟斤拷锟劫讹拷锟斤拷锟斤拷
        current_ramp_speed = 0;
    } else {
        if (run_mode == 1) {
            // 锟斤拷锟斤拷模式锟铰ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟轿惯碉拷锟芥划锟斤拷锟劫度ｏ拷锟竭碉拷锟窖撅拷锟斤拷锟斤拷锟斤拷锟斤拷锟侥加硷拷锟劫帮拷锟界，锟斤拷锟斤拷撞锟斤拷锟侥ミ�
            current_ramp_speed = INS_Get_Target_Speed();

        }
        else {
            // 探锟斤拷模式锟铰ｏ拷没锟斤拷图时锟斤拷锟斤拷使锟斤拷原锟斤拷锟侥伙拷锟斤拷锟斤拷锟竭硷拷锟斤拷锟斤拷锟斤拷锟�
            if (current_ramp_speed < SOFT_START_MIN && target_base_speed >= SOFT_START_MIN) {
                current_ramp_speed = SOFT_START_MIN;
            } else if (current_ramp_speed < target_base_speed) {
                current_ramp_speed += ACCEL_STEP; // (锟斤拷时锟矫碉拷锟斤拷 motor.c 锟斤拷锟� ACCEL_STEP)
                if (current_ramp_speed > target_base_speed) current_ramp_speed = target_base_speed;
            } else if (current_ramp_speed > target_base_speed) {
                current_ramp_speed = target_base_speed;
            }
        }
    }

    // 锟斤拷锟斤拷锟斤拷锟斤拷幕锟斤拷锟斤拷锟斤拷俣雀锟斤拷锟斤拷锟斤拷锟斤拷俣锟�
    base_speed = current_ramp_speed;
    // =========================================================================

    // === [频锟斤拷] 锟斤拷频锟叫讹拷锟竭硷拷锟斤拷3锟轿斤拷1锟斤拷 ===
    control_div_cnt++;
    if (control_div_cnt >= 2)
    {
        control_div_cnt = 0; // 锟斤拷锟斤拷锟斤拷锟斤拷位

        // --- 2. 模锟斤拷PD转锟斤拷锟斤拷锟� (锟斤拷频执锟叫ｏ拷锟斤拷省锟斤拷锟斤拷) ---
        float steer_speed = steering_fuzzy_pd_control(&turnPID, 0.0f, filtered_deviation);
        steer_speed = constrain(steer_speed, -STEER_OUTPUT_MAX, STEER_OUTPUT_MAX);

        // --- 4. 锟斤拷锟劫硷拷锟斤拷 ---
        if (steer_speed > 0)
        {
            right_speed = base_speed - (int) (DE_SPD_PARAM * steer_speed);
            left_speed  = base_speed + (int) (IN_SPD_PARAM * steer_speed);
        }
        else
        {
            right_speed = base_speed - (int)(IN_SPD_PARAM * steer_speed);
            left_speed  = base_speed + (int)(DE_SPD_PARAM * steer_speed);
        }

        // --- 锟斤拷止锟斤拷锟斤拷越锟斤拷 ---
        left_speed  = constrain(left_speed, -encoder_MAX, encoder_MAX);
        right_speed = constrain(right_speed, -encoder_MAX, encoder_MAX);
    }

    // --- 5. 锟劫讹拷锟斤拷锟� (每锟轿讹拷执锟斤拷) ---
    speed_control_L(left_speed);
    speed_control_R(right_speed);
    //sent_two_int((actual_right_speed+actual_left_speed)/2,current_ramp_speed);

    // --- 6. 每锟斤拷执锟斤拷锟斤拷锟剿拷鹿叩锟绞凳蔽伙拷锟� ---
    INS_Update(actual_left_speed, actual_right_speed, get_yaw());
}
/**************************************************************
 * 锟斤拷锟斤拷: 锟斤拷装锟矫碉拷锟斤拷锟竭达拷锟节凤拷锟酵猴拷锟斤拷
 * 锟斤拷式: 锟斤拷目锟斤拷,锟斤拷目锟斤拷,锟斤拷实锟斤拷,锟斤拷实锟斤拷\r\n
 **************************************************************/
void send_speed_to_pc(void)
{
    // 锟斤拷锟斤拷一锟斤拷锟街凤拷锟斤拷锟斤拷锟斤拷锟斤拷64锟街斤拷锟姐够装锟斤拷4锟斤拷锟斤拷锟斤拷锟斤拷
    char send_buf[64];

    // 使锟斤拷 sprintf 锟斤拷锟斤拷锟捷革拷式锟斤拷为锟街凤拷锟斤拷
    // %d 锟斤拷示锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟节分革拷锟斤拷\r\n 锟角伙拷锟叫凤拷(锟斤拷位锟斤拷锟斤拷图锟斤拷锟斤拷识锟斤拷锟叫的憋拷准)
    sprintf(send_buf, "%d,%d,%d,%d,%d\r\n",
            left_speed,       // 锟斤拷锟斤拷锟斤拷锟侥匡拷锟街�
            right_speed,      // 锟揭憋拷锟斤拷锟斤拷目锟斤拷值
            actual_left_speed,     // 锟斤拷锟斤拷锟斤拷锟绞碉拷锟街�
            actual_right_speed,
            return_error());   // 锟揭憋拷锟斤拷锟斤拷实锟斤拷值

    // 锟斤拷锟斤拷锟斤拷锟竭达拷锟节库发锟斤拷锟街凤拷锟斤拷锟斤拷锟斤拷
    wireless_uart_send_string(send_buf);
}

void car_stop(){
    stop_flag = 1;
    pit_disable(CCU60_CH0);
    L_set_PWM(0);
    R_set_PWM(0);
    system_delay_ms(500);
    negative_pressure_stop();
}
