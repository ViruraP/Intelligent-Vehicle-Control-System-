#include "main.h" // 所有引脚信息更改在main.h里改宏
#include "zf_device_imu660ra.h"
#include "beep.h"
#include "zf_common_headfile.h"
#include "zf_common_debug.h"


//环岛控制
extern volatile int Island_State;     //环岛状态标志
extern volatile int Left_Island_Flag; //左右环岛标志
extern volatile int Right_Island_Flag;//左右环岛标志
extern volatile int Left_Up_Guai[2];    //四个拐点的坐标存储，[0]存y，第某行，{1}存x，第某列
extern volatile int Right_Up_Guai[2];   //四个拐点的坐标存储，[0]存y，第某行，{1}存x，第某列
extern volatile int Left_Down_Guai[2];    //四个拐点的坐标存储，[0]存y，第某行，{1}存x，第某列
extern volatile int Right_Down_Guai[2];   //四个拐点的坐标存储，[0]存y，第某行，{1}存x，第某列


extern volatile int Zebra_Stripes_Flag;
extern volatile int Cross_Flag;
uint8_t zebra_slowstop_flag = 0;      // 是否正在缓停
uint16_t zebra_slowstop_cnt = 0;      // 缓停计数器
volatile uint8_t Car_Stop_Mode = 0;
static int zebra_stop_delay = 0;   // 斑马线停车延迟计数（单位：帧）
#include "zf_common_headfile.h"
// 舵机动作角度中值
const float servo_motor_duty_middle = (SERVO_MOTOR_R_MAX + SERVO_MOTOR_L_MAX) / 2.f; 
extern volatile int Boundry_Start_Right;  //第一个非丢线点,常规边界起始点
void Init()
{
    // 初始化flash, 储存参数. 一个扇区有8页, 一页可以储存4096字节, 一个参数占4个字节, 因此一页最多只能存64个参数
    flash_init();
//    int aaa=imu660ra_init();
//    imu_calib_bias();

    tft180_init();

    // mt9v03x摄像头初始化
    while (1)
    {
//        tft180_show_int(20, 00, aaa, 3);
        if (mt9v03x_init())
            tft180_show_string(0, 16, "mt9v03x reinit.");
        else
            break;
        system_delay_ms(100);
    }
    tft180_show_string(0, 16, "init success.");

    // 编码器初始化
    encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B); // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B); // 初始化编码器模块与引脚 正交解码编码器模式

    // 电机初始化
    pwm_init(MOTOR1_PWM, 17000, 0); // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL); // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR2_PWM, 17000, 0); // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL); // GPIO 初始化为输出 默认上拉输出高

    // 舵机
    Servo_PID_Init();  // 新增舵机PID初始化
    pwm_init(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, 0);

       // 舵机测试：先转到左极限
    pwm_set_duty(SERVO_MOTOR_PWM, SERVO_MOTOR_DUTY(SERVO_MOTOR_L_MAX));
    tft180_show_string(0, 130, "Servo Left");
    system_delay_ms(500);
    
    // 转到右极限
    pwm_set_duty(SERVO_MOTOR_PWM, SERVO_MOTOR_DUTY(SERVO_MOTOR_R_MAX));
    tft180_show_string(0, 130, "Servo Right");
    system_delay_ms(500);
    
  
    // 回到中间
    pwm_set_duty(SERVO_MOTOR_PWM, SERVO_MOTOR_DUTY(servo_motor_duty_middle));
    tft180_show_string(0, 130, "Servo Middle");
    system_delay_ms(500);

    // PID初始化
    My_Pid_Init();

    // 定时器0初始化，10ms可调
    pit_ms_init(PIT_CH0, 10);

    Beep_Init();
}


extern int Cross_Frame_Interval;   // 两次“十字进入确认”之间的帧数


uint8_t image[MT9V03X_H][MT9V03X_W];
uint8_t left_line[MT9V03X_H];
uint8_t mid_line[MT9V03X_H];
uint8_t right_line[MT9V03X_H];
int Island1Count = 0;
int Island2Count = 0;
int Island3Count = 0;
int lock = 0;
int protect=0;
int Island5Count = 0;
int Island4Count = 0;
            
            
int monotonicity_change_line[2];//单调性改变点坐标，[0]寸某行，[1]寸某列
float k;
int zebra_forward_frames = 0;   // 控制短暂直行
uint8_t zebra_mode_cnt = 0;     // 斑马线模式剩余帧数
static uint8_t zebra_edge_lock = 0;    // 防止一直=1时重复触发
float abcde=0.1;  //abcde=0.1,左环yaw_deg=168.4200
            
            int aaaa=0;
            int bbbb=0;
            int cccc=0;
            int dddd=0;
            int eeee=0;

int x =0;

int main()
{
    clock_init(SYSTEM_CLOCK_600M); // 不可删除
    debug_init();                  // 调试端口初始化
    system_delay_ms(100);          // 等待主板其他外设上电完成
    
    Init();

    while (1)
    {
           //  mt9v03x摄像头
        if (mt9v03x_finish_flag)
        {
            lock++;
            // 另寻空间将图像保存下来，以免产生因读写冲突带来的未知后果
            memcpy((uint8_t *)image, (uint8_t *)mt9v03x_image, sizeof(uint8_t) * MT9V03X_H * MT9V03X_W);
            // 获取直方图
            
            // 二值化处理
            unsigned char threshold = FIXED_THRESHOLD;
            
            binaryzation_process((uint8_t *)image, MT9V03X_H, MT9V03X_W, threshold);
            // 边界线寻找
            auxiliary_process((uint8_t *)image, MT9V03X_H, MT9V03X_W, threshold, left_line, mid_line, right_line);
            
            Longest_White_Column((uint8_t*)right_line,(uint8_t*)left_line);
            
            Island_Detect((uint8_t*)right_line,(uint8_t*)left_line,(uint8_t*) mid_line);
            
            Cross_Detect((uint8_t*)right_line,(uint8_t*)left_line,(uint8_t*) mid_line);
            
            if(Cross_Time_Flag)
            {
                x++;
            }
            
            Zebra_Stripes_Detect((uint8_t*) right_line,(uint8_t*) left_line);
            


            //-----------------------------------------------------
            // 第一次斑马线：Flag==1 时，强制直行 60 帧
            /*-----------------------------------------------------*/
            // ===== 斑马线：只在“第一次变成1”时触发 5 帧 =====
            if (Zebra_Stripes_Flag == 1) 
            {
                if (zebra_edge_lock == 0) 
                {        // 只触发一次
                    zebra_edge_lock = 1;
                    zebra_mode_cnt = 12;          // 你要的“前5帧”
                    Servo_PID_Reset(&servo_pid);   // 可选：避免恢复时D项抽搐
                }
            } 
            else 
            {
                 zebra_edge_lock = 0;               // 只要flag回到0，就允许下次再触发
            }

            // ------------------------------
            // 处理“第二次经过斑马线”逻辑（Flag=2）
            // ------------------------------
            static uint8 zebra2_lock = 0;

            if (Zebra_Stripes_Flag == 2)
            {
                if (!zebra2_lock)
                {
                    zebra2_lock = 1;

                    // 立刻进入倒车刹停阶段（交给 ISR）
                    Car_Stop_Mode = 2;
                    speed_target  = 0;

                    // 清空速度环，防止积分顶死
                    speed_pid_l.integrator = 0;
                    speed_pid_l.last_error = 0;
                    speed_pid_r.integrator = 0;
                    speed_pid_r.last_error = 0;
                }
            }
            else
            {
                zebra2_lock = 0; // Flag回0后允许下次（如果你永远只停一次，也可以不写）
            }               

            if(Right_Island_Flag)
            {
                speed_target = 130.0;
              
//              if(yaw_deg<=168){Island_State=5;}
//              else if(yaw_deg<=-150){Island_State=4;}
                switch(Island_State)
                {
                    case 2:
//                        imu_update_yaw(abcde);
                        protect++;
                        if(protect>=1)
                        {
                            // 镜像左环岛逻辑：计算斜率并调用Left_Add_Line（对称替换为Right_Add_Line）
                            Left_Add_Line(left_line,right_line,mid_line,Right_Up_Guai[1],Right_Up_Guai[0],left_line[118],118);
                        }
                        break;
                    case 3:
//                        imu_update_yaw(abcde);
                        break;
                    case 4:
//                      imu_update_yaw(abcde);
                          // 镜像左环岛逻辑：对称替换left_line为right_line，偏移量方向反转
                        mid_line[3*MT9V03X_H/4-15]=right_line[3*MT9V03X_H/4-15]-55;
                        mid_line[3*MT9V03X_H/4-12]=right_line[3*MT9V03X_H/4-12]-50;
                        mid_line[3*MT9V03X_H/4-9]=right_line[3*MT9V03X_H/4-9]-50;
                        mid_line[3*MT9V03X_H/4-6]=right_line[3*MT9V03X_H/4-6]-45;
                        mid_line[3*MT9V03X_H/4-3]=right_line[3*MT9V03X_H/4-3]-45;
                        break;
                    case 5:
                        if (Right_Up_Guai[0]==0||Right_Up_Guai[0]>=40)
                        { 
                        mid_line[3*MT9V03X_H/4-15]=MT9V03X_H/2;
                        mid_line[3*MT9V03X_H/4-12]=MT9V03X_H/2;
                        mid_line[3*MT9V03X_H/4-9]=MT9V03X_H/2;
                        mid_line[3*MT9V03X_H/4-6]=MT9V03X_H/2;
                        mid_line[3*MT9V03X_H/4-3]=MT9V03X_H/2; 
                        }
                        else{}
//                yaw_deg=0.0f;
                        break;
                    default:
                        break;
                }          
            }
            else if(Left_Island_Flag)
            {
                speed_target = 130.0;
//              if(yaw_deg>=176){Island_State=5;}
//              else if(yaw_deg>=140){Island_State=4;}
                switch(Island_State)
                {
                    case 2:
//                    imu_update_yaw(abcde);
                        protect++;
                        if(protect>=1)
                        {
                            Right_Add_Line(right_line,left_line,mid_line,Left_Up_Guai[1],Left_Up_Guai[0],right_line[118],118);
                        }
                        break;
                    case 3:
//                      imu_update_yaw(abcde);
                        break;
                    case 4:
//                        imu_update_yaw(abcde);
                        mid_line[3*MT9V03X_H/4-15]=left_line[3*MT9V03X_H/4-15]+20;
                        mid_line[3*MT9V03X_H/4-12]=left_line[3*MT9V03X_H/4-12]+30;
                        mid_line[3*MT9V03X_H/4-9]=left_line[3*MT9V03X_H/4-9]+40;
                        mid_line[3*MT9V03X_H/4-6]=left_line[3*MT9V03X_H/4-6]+40;
                        mid_line[3*MT9V03X_H/4-3]=left_line[3*MT9V03X_H/4-3]+30;
                        break;
                    case 5:
                        if (Left_Up_Guai[0]==0||Left_Up_Guai[0]>=40)
                        { 
                        mid_line[3*MT9V03X_H/4-15]=MT9V03X_H/2;
                        mid_line[3*MT9V03X_H/4-12]=MT9V03X_H/2;
                        mid_line[3*MT9V03X_H/4-9]=MT9V03X_H/2;
                        mid_line[3*MT9V03X_H/4-6]=MT9V03X_H/2;
                        mid_line[3*MT9V03X_H/4-3]=MT9V03X_H/2; 
                        }
                        else{}
//                yaw_deg=0.0f;
                        break;
                    default:
                        break;
                }           
            }   
            else 
                speed_target = 130;
            
            if(Island_State==1) aaaa++;
            if(Island_State==2) bbbb++;
            if(Island_State==3) cccc++;
            if(Island_State==4) dddd++;
            if(Island_State==5) eeee++;
            
             tft180_show_int(20, 00, aaaa, 3);
              tft180_show_int(20, 20, bbbb, 3);
               tft180_show_int(20, 40, cccc, 3);
                tft180_show_int(20, 60, dddd, 3);
                 tft180_show_int(20, 80, eeee, 3);
            
            for (uint8_t _i = 0; _i < MT9V03X_H; ++_i)
            {
                // 将边界线也显示出来
                image[_i][left_line[_i]] = 0;
                image[_i][mid_line[_i]] = 0;
                image[_i][right_line[_i]] = 0;
            }
            // 显示图像
            tft180_displayimage03x((uint8_t *)image, 125, 100);
            tft180_show_int(40,0, x,3);

            tft180_show_int(0,20,Cross_Time_Flag,3);
//            tft180_show_float(20, 80, yaw_deg, 3,4);
            
            int row1 = MT9V03X_H * 3 / 4-15;  //25
            int row2 = MT9V03X_H * 3 / 4-12; 
            int row3 = MT9V03X_H * 3 / 4-9; 
            int row4 = MT9V03X_H * 3 / 4-6; 
            int row5 = MT9V03X_H * 3 / 4-3; 
            if(Island_State == 4)
            {
                row1 = MT9V03X_H * 3 / 4;  //25
                row2 = MT9V03X_H * 3 / 4; 
                row3 = MT9V03X_H * 3 / 4; 
                row4 = MT9V03X_H * 3 / 4; 
                row5 = MT9V03X_H * 3 / 4; 
            }
            float err1 = ((MT9V03X_W / 2) - mid_line[row1])*0.1+((MT9V03X_W / 2) - mid_line[row2])*0.1+((MT9V03X_W / 2) - mid_line[row3])*0.2+((MT9V03X_W / 2) - mid_line[row4])*0.3+((MT9V03X_W / 2) - mid_line[row5])*0.3;

            float mid_err = err1;
            // 舵机根据误差乘以系数打角
            if (Car_Stop_Mode == 2) mid_err = 0;   // ★倒车反号
            Servo_PID_Calculate(&servo_pid, mid_err, 0.01f); // dt=10ms
            float tmp_duty = servo_motor_duty_middle - servo_pid.output * SERVO_DIR;
            // 限幅
            tmp_duty = MAX(tmp_duty,MIN(SERVO_MOTOR_L_MAX,SERVO_MOTOR_R_MAX));
            tmp_duty = MIN(tmp_duty,MAX(SERVO_MOTOR_L_MAX,SERVO_MOTOR_R_MAX));

            pwm_set_duty(SERVO_MOTOR_PWM, SERVO_MOTOR_DUTY(tmp_duty));
            tft180_show_int(00, 00, Island_State, 3);
 
            // 处理完一帧图像后务必把该标志位清零！
            mt9v03x_finish_flag = 0;
        }
    }
}

