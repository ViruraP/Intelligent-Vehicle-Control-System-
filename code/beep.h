#ifndef _BEEP_H_
#define _BEEP_H_

#include "zf_common_headfile.h"
extern float gz_bias;
extern float yaw_deg;
void Beep_Init(void);
void Beep_On(void);
void Beep_Off(void);
void Beep_Beep(uint16_t ms);
void Beep_Double(uint16 interval_ms);
void Beep_Tri(uint16 interval_ms);
void imu_calib_bias(void);
void imu_update_yaw(float dt);
//void Get_Gyroscope_Angle(void);

#endif
