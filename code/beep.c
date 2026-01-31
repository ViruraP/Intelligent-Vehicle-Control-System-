#include "zf_driver_gpio.h"
#include "zf_common_headfile.h"
#include "beep.h"

// 蜂鸣器控制引脚 = B11 （来自原理图）
#define BEEP_PIN   B11

void Beep_Init(void)
{
    gpio_init(BEEP_PIN, GPO, 0, GPO_PUSH_PULL);   // 推挽输出，默认低电平
}

void Beep_On(void)
{
    gpio_set_level(BEEP_PIN, 1);
}

void Beep_Off(void)
{
    gpio_set_level(BEEP_PIN, 0);
}

void Beep_Beep(uint16 ms)
{
    Beep_On();
    system_delay_ms(ms);
    Beep_Off();
}
void Beep_Double(uint16 interval_ms)
{
    // 第一次
    Beep_On();
    system_delay_ms(80);   // 第一声持续时间
    Beep_Off();

    system_delay_ms(interval_ms);   // 两声之间的间隔

    // 第二次
    Beep_On();
    system_delay_ms(80);   // 第二声持续时间
    Beep_Off();
}

void Beep_Tri(uint16 interval_ms)
{
    // 第一次
    Beep_On();
    system_delay_ms(80);   // 第一声持续时间
    Beep_Off();

    system_delay_ms(interval_ms);   // 两声之间的间隔

    // 第二次
    Beep_On();
    system_delay_ms(80);   // 第二声持续时间
    Beep_Off();
    
    system_delay_ms(interval_ms);   // 两声之间的间隔

    // 第三次
    Beep_On();
    system_delay_ms(80);   // 第二声持续时间
    Beep_Off();
}


float gz_bias = 0;

void imu_calib_bias(void)
{

    long sum = 0;
    for(int i=0;i<300;i++)
    {
        imu660ra_get_gyro();
        sum += imu660ra_gyro_z;
        system_delay_ms(2);
    }
    float gz_raw = (float)sum / 300.0f;
    gz_bias = gz_raw / imu660ra_transition_factor[1]; // bias in °/s
}

float yaw_deg = 0.0f;

void imu_update_yaw(float dt) // dt：秒
{
    imu660ra_get_gyro();
    float gz_dps = imu660ra_gyro_z / imu660ra_transition_factor[1];
    gz_dps -= gz_bias;
//      tft180_show_float(20, 20, gz_dps, 3,2);
    // 小死区去噪（很重要）
    if(fabsf(gz_dps) < 1.5f) gz_dps = 0;

    yaw_deg += gz_dps * dt;   // 积分：deg/s * s = deg
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      陀螺仪计算航偏角角度
//  @param
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
/*void Get_Gyroscope_Angle(void)
{
    float K=0.7;
    FJ_gyro_z = imu660ra_gyro_z;
    FJ_LastAngleSpeed=FJ_AngleSpeed;
    FJ_AngleSpeed += ((FJ_gyro_z-ZeroDrift_gyro_z) * GYRO_SENS)*DT;
    FJ_Angle = FJ_AngleSpeed*K+FJ_LastAngleSpeed*(1-K);                //向左为正
    FJ_Angle = FJ_Angle > FJ_Angle_Max ? FJ_Angle_Max : FJ_Angle;
    FJ_Angle = FJ_Angle < FJ_Angle_Min ? FJ_Angle_Min : FJ_Angle;
    FJ_Angle = -FJ_Angle;
}*/