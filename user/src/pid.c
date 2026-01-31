#include "pid.h"
extern volatile uint8_t Car_Stop_Mod;
extern uint8_t zebra_mode_cnt ;     // 斑马线模式剩余帧数

// 电机PID初始化参数
//float speed_KP_l = 150, speed_KI_l = 0.1,speed_KD_l = 0.0, speed_IMAX = 5000.0, speed_OUTMAX = 5000.0;（最大速）
//float speed_KP_r = 100,speed_KI_r = 0.1,speed_KD_r = 0.0;(最大速)
//float speed_KP = 70, speed_KI = 0.1,speed_KD = 0.0, speed_IMAX = 900.0, speed_OUTMAX = 5000.0;
//float speed_KP = 200, speed_KI = 0.3,speed_KD = 0.3, speed_IMAX = 900.0, speed_OUTMAX = 5000.0;
//float speed_KP_l = 70, speed_KI_l = 0.1,speed_KD_l = 0.0, speed_IMAX = 900.0, speed_OUTMAX = 5000.0;
//float speed_KP_r = 70, speed_KI_r = 0.1,speed_KD_r = 0.0;
float speed_KP_l = 100, speed_KI_l = 0.505,speed_KD_l = 0, speed_IMAX = 2500.0, speed_OUTMAX = 5000.0;
                  //70                  0.1             0                   900
float speed_KP_r = 100, speed_KI_r = 0.57,speed_KD_r = 0;
                  //70                0.1                0
//float speed_KP_l = 150, speed_KI_l = 0.2,speed_KD_l = 0.03, speed_IMAX = 2500.0, speed_OUTMAX = 5000.0;
                  //70                  0.1             0                   900
//float speed_KP_r = 130, speed_KI_r = 0.08,speed_KD_r = 0.3;
//float speed_target = 100.0;
float speed_target = 150.0;

float speed_real = 0.0;
float speed_pwm_r = 0.0;
float speed_pwm_l = 0.0;
pid_param_t speed_pid_l;  // 电机PID
pid_param_t speed_pid_r;  // 电机PID

/*******************************************************************************
* 函 数 名         : My_Pid_Init
* 函数功能         : PID初始化	
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void My_Pid_Init(void)
{
    Pid_Param_Init(&speed_pid_l, speed_KP_l, speed_KI_l,speed_KD_l, speed_IMAX, speed_OUTMAX);
    Pid_Param_Init(&speed_pid_r, speed_KP_r, speed_KI_r,speed_KD_r, speed_IMAX, speed_OUTMAX);
}


/*******************************************************************************
* 函 数 名         : Pid_Param_Init
* 函数功能         : PID参数初始化	
* 输    入         : imax:积分项最大值
* 输    出         : 无
*******************************************************************************/
void Pid_Param_Init(pid_param_t * pid, float kp, float ki, float kd, float imax, float outmax)
{
    pid->kp        = kp;
    pid->ki        = ki;
    pid->kd        = kd;
    pid->imax      = imax;
    pid->out_p     = 0;
    pid->out_i     = 0;
    pid->out_d     = 0;
    pid->out       = 0;
    pid->outmax    = outmax;
    pid->integrator= 0;
    pid->last_error= 0;
    pid->last_derivative   = 0;
    pid->last_t    = 0;
}

/*******************************************************************************
* 函 数 名         : PidLocCtrl
* 函数功能	   : 位置式PID控制
* 输    入         : pid, error, t
* 输    出         : float
*******************************************************************************/
float PidLocCtrl(pid_param_t *pid, float error, float dt)
{
    // P
    pid->out_p = pid->kp * error;

    // I
    pid->integrator += error * dt;
    pid->integrator = constrain_float(pid->integrator,
                                      -pid->imax,
                                       pid->imax);
    pid->out_i = pid->ki * pid->integrator;

    // D（建议先关）
    pid->out_d = pid->kd * (error - pid->last_error) / dt;
    pid->last_error = error;

    // Sum
    pid->out = pid->out_p + pid->out_i + pid->out_d;
    pid->out = constrain_float(pid->out, 0, pid->outmax);

    return pid->out;
}


/*******************************************************************************
* 函 数 名         : PidIncCtrl
* 函数功能	   : 增量式PID控制
* 输    入         : pid, error, t
* 输    出         : float
*******************************************************************************/
float PidIncCtrl(pid_param_t * pid, float error, float t)
{

    pid->out_p = pid->kp * (error - pid->last_error);
    pid->out_i = pid->ki * error * t ;
    pid->out_d = pid->kd/t * ((error - pid->last_error) - pid->last_derivative);

    pid->last_derivative = error - pid->last_error;
    pid->last_error = error;

    pid->out += pid->out_p + pid->out_i + pid->out_d;

    pid->out = constrain_float(pid->out, -pid->outmax, pid->outmax);
    return pid->out;
}

/*******************************************************************************
* 函 数 名         : constrain_float
* 函数功能         : 浮点型数限幅
* 输    入         : amt,low,high
* 输    出         : float
*******************************************************************************/
float constrain_float(float amt, float low, float high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

/*******************************************************************************
* 函 数 名         : constrain_short
* 函数功能         : 短整型数限幅
* 输    入         : amt,low,high
* 输    出         : short
*******************************************************************************/
short constrain_short(short amt, short low, short high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}



/*******************************************************************************
* 舵机的PID调节
*******************************************************************************/

Servo_PID_t servo_pid = {0};

void Servo_PID_Init(void)
{
    //servo_pid.kp = 0.5f; // 需要调试 - 比例系数
    servo_pid.ki = 0.0f;          // 需要调试 - 积分系数  
    servo_pid.integral = 0.f;      // 积分项清零
    servo_pid.last_error = 0.f;    // 上次误差清零
    servo_pid.output = 0.f;        // 输出清零
    servo_pid.integral_limit = 50.f;  // 积分限幅，防止积分饱和
}

// 建议：在 pid.c 顶部加 extern（或 include 含 extern 的头文件）
extern volatile int Curve_Flag;
extern volatile int Left_Island_Flag;
extern volatile int Right_Island_Flag;
extern volatile int Cross_Time_Flag;
extern volatile int Zebra_Stripes_Flag;

void Servo_PID_Calculate(Servo_PID_t *pid, float error, float dt)
{
    // --------- 1) 模式判定（建议加优先级）---------
    enum { MODE_STRAIGHT=0, MODE_CURVE=1, MODE_SPECIAL=2, MODE_ZEBRA=3 };
    static uint8_t last_mode = MODE_STRAIGHT;

/*    // （可选）对 Curve_Flag 做简单防抖，避免一帧一变导致参数跳来跳去
    static uint8_t curve_hi = 0, curve_lo = 0;
    if (Curve_Flag) { curve_hi++; curve_lo = 0; }
    else            { curve_lo++; curve_hi = 0; }

    uint8_t curve_ok = (curve_hi >= 3);   // 连续3帧认为进入弯道
    uint8_t curve_off= (curve_lo >= 3);   // 连续3帧认为退出弯道

    static uint8_t curve_state = 0;
    if (!curve_state && curve_ok) curve_state = 1;
    if ( curve_state && curve_off) curve_state = 0;
*/
    uint8_t mode;

    // 斑马线（第一次/第二次）优先级最高（你也可以只对第一次处理）
    if (zebra_mode_cnt > 0) {
    mode = MODE_ZEBRA;
    zebra_mode_cnt--;
} 
    // 环岛/十字次高
    else if (Right_Island_Flag==1 || Left_Island_Flag==1|| Cross_Time_Flag==1) // || Cross_Time_Flag==1
    {
        mode = MODE_SPECIAL;
    }
    // 弯道
    else if (Curve_Flag) {
        mode = MODE_CURVE;
    }
    // 直线
    else {
        mode = MODE_STRAIGHT;
    }

    // --------- 2) 模式切换时：Reset，避免D项抽搐 ----------
    if (mode != last_mode) {
        pid->integral = 0.f;
        pid->last_error = error;   // 关键：让D项第一下为0，避免“跳变”
        pid->output = 0.f;
        last_mode = mode;
    }

    // --------- 3) 根据 mode 选参数（这里你按需要改数值）---------
    if (mode == MODE_CURVE) {
        pid->kp = 0.5f;
        pid->kd = 0.0005f;
        pid->output_limit = 900.f;
    }
    else if (mode == MODE_SPECIAL) {
        pid->kp = 0.3f;
        pid->kd = 0.0005f;
        pid->output_limit = 400.f;
    }
    else if (mode == MODE_ZEBRA) {
        // 斑马线第一次：你可以在这“固定舵机居中”也行，
        // 或者仅仅降低 kd（你说你想自己写 kd 变小函数也可以放这里调用）
        pid->kp = 0.0f;
        pid->kd = 0.0f;          // 这里只是示例，你要自己调就改/改成调用函数
        pid->output_limit = 0.0f;
    }
    else { // MODE_STRAIGHT
        pid->kp = 0.05f;
        pid->kd = 0.0005f;
        pid->output_limit = 400.f;
    }

    // --------- 4) 正常PID计算 ----------
    if (dt <= 0.000001f) dt = 0.001f;   // 防止除0

    float proportional = pid->kp * error;

    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float integral = pid->ki * pid->integral;

    float derivative = pid->kd * (error - pid->last_error) / dt;
    pid->last_error = error;

    pid->output = proportional + integral + derivative;

    if (pid->output > pid->output_limit) pid->output = pid->output_limit;
    else if (pid->output < -pid->output_limit) pid->output = -pid->output_limit;
}

void Servo_PID_Reset(Servo_PID_t *pid)
{
    pid->integral = 0.f;
    pid->last_error = 0.f;
    pid->output = 0.f;
}




