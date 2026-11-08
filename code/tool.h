#ifndef CODE_TOOL_H_
#define CODE_TOOL_H_

#include "zf_common_headfile.h"
void show_point_red(int x,int y);
void show_point_blue(int x,int y);
void show_point_yellow(int x,int y);
void show_point(int x,int y);

extern float dt;

// ����ʽPID�ṹ�壨�ٶȻ���
typedef struct {
    float Kp;               // ����ϵ��
    float Ki;               // ����ϵ��
    float Kd;               // ΢��ϵ��
    int target;             // Ŀ��ֵ
    int error;              // ��ǰ��� e(k)
    int last_error;         // ��һ����� e(k-1)
    int prev_error;    // ���ϴ���� e(k-2)
    int last_output;        // ��һ������PWM���������PID���ģ�
} IncPID;



// ת��PID�ṹ�壨���򻷣�
typedef struct {
    float Kp;               // ����ϵ
    float Kd;               // ΢��ϵ��
    float GKD;              //����������
    float error;            // ��ǰƫ��
    float last_error;       // ��һ��ƫ��
    float prev_error;  // ���ϴ�ƫ��
    float last_actual;      // ��һ��ʵ��ֵ���ֱ����У�
} SteeringPID;

typedef struct {
    float K_nom;      // ���������� (��̬�ٶ� / PWM)
    float T_nom;      // ���ʱ�䳣�� (���ٵ�63%�����ʱ�䣬ͨ��0.05~0.1)
    float Q_alpha;    // Q�˲���ϵ�� (0.0~1.0��������������Ӧ�ٶȣ�����ADRC��W_o)
    float dt;         // �������� (��λ:��)

    int last_speed;   // ��һ�ε��ٶ� (������)
    float last_pwm;   // ��һ��ʵ��������������PWM
    float dob_comp;   // �۲�������Ĳ�����
} DOB_Controller;

// DOB ���㺯������
float DOB_Calculate(DOB_Controller *dob, int current_speed);

typedef struct {
    float alpha; // �˲���ϵ��
    float previous_output; // ��һ�ε����ֵ
} LowPassFilter;

// λ��ʽPD����
float steering_pd_control(SteeringPID *pid, float setpoint, float actual);
float steering_fuzzy_pd_control(SteeringPID *pid, float setpoint, float actual);
// ����ʽPID����
extern int IncPID_Calculate(IncPID *pid, int current_speed);

float angle_difference(float angle1, float angle2);

#define constrain(value, min, max) ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))
#define max(a,b) a>b?a:b
#define min(a,b) a>b?b:a

// ����ת�ַ���
const char* int_to_str(int value);

// ���߷�����������
void sent_two_int(int a, int b);

#endif /* CODE_TOOL_H_ */
