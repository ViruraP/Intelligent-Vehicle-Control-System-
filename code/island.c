#include "zf_common_headfile.h"
#include "Island.h"

// 原有全局变量定义完全保留（仅展示关键部分，实际代码中无需改动）
extern uint8 image_two_value[MT9V03X_H][MT9V03X_W];
extern int Left_Lost_Flag[MT9V03X_H] ;
extern int Right_Lost_Flag[MT9V03X_H];
extern volatile int Search_Stop_Line;
extern volatile int Boundry_Start_Left;
extern volatile int Boundry_Start_Right;
extern volatile int Left_Lost_Time;
extern volatile int Right_Lost_Time; 
extern volatile int Both_Lost_Time;
extern volatile float Err;
extern volatile int monotonicity_change_line[2];
extern float k;
extern int Island5Count;
extern int Island4Count;
extern int Island3Count;
extern int Island2Count;
extern int Island1Count;
extern volatile int Cross_Flag; 

volatile int Island_State=0;     
volatile int Left_Island_Flag=0; 
volatile int Right_Island_Flag=0;
volatile int Left_Up_Guai[2];    
volatile int Right_Up_Guai[2];   
volatile int Left_Down_Guai[2];    
volatile int Right_Down_Guai[2];   

extern volatile int Zebra_State;
extern int protect;

void Island_Detect(uint8_t* Right_Line,uint8_t* Left_Line,uint8_t* Mid_Line)
{ 
if(Cross_Time_Flag==1)
return;
    #define CHAR_WIDTH 8
    static float k=0;
    static int island_state_5_down[2]={0};
    static int island_state_3_up[2]={0};
    static int left_down_guai[2]={0};
    static int right_down_guai[2]={0};
    static int left_up_guai[2]={0};
    static int right_up_guai[2]={0};
    int monotonicity_change_left_flag=0;
    int monotonicity_change_right_flag=0;
    int continuity_change_right_flag=0; 
    int continuity_change_left_flag=0;  
    int lost_time=0;

    // 1. 基础判断（连续性/单调性）- 保留原有计算逻辑
    continuity_change_left_flag=Continuity_Change_Left(Left_Line,MT9V03X_H-1-5,10);
    continuity_change_right_flag=Continuity_Change_Right(Right_Line,MT9V03X_H-1-5,10);
    monotonicity_change_right_flag=Monotonicity_Change_Right(Right_Line,MT9V03X_H-1-10,10);
    monotonicity_change_left_flag=Monotonicity_Change_Left(Left_Line,MT9V03X_H-1-10,10);
    
    // 2. 拐点查找 - 保留原有遍历逻辑
    int valid_Down_guai_L = 0;
    int valid_UP_guai_L = 0;
    for(int y = MT9V03X_H-1; y >= 30; y -=5) 
    {
        valid_Down_guai_L = Find_Left_Down_Point(Left_Line, y, y-5);
        if(valid_Down_guai_L != 0) {
        break;
        }
    }
    left_down_guai[0] = valid_Down_guai_L;
    left_down_guai[1]=Left_Line[left_down_guai[0]];
    for(int y = MT9V03X_H/2+10; y >= 5; y -=5) 
    {
        valid_UP_guai_L = Find_Left_Up_Point(Left_Line, y, y-5);
        if(valid_UP_guai_L != 0) {
        break;
        }
    }
    left_up_guai[0]=valid_UP_guai_L;
    left_up_guai[1]=Left_Line[left_up_guai[0]];

    int valid_Down_guai_R = 0;
    int valid_UP_guai_R = 0;
    for(int y = MT9V03X_H-1; y >= 30; y -=5) 
    {
        valid_Down_guai_R = Find_Right_Down_Point(Right_Line, y, y-5);
        if(valid_Down_guai_R != 0) {
        break;
        }
    }
    right_down_guai[0] = valid_Down_guai_R;
    right_down_guai[1]=Right_Line[right_down_guai[0]];
    for(int y = MT9V03X_H/2+10; y >= 5; y -=5) 
    {
        valid_UP_guai_R = Find_Right_Up_Point(Right_Line, y, y-5);
        if(valid_UP_guai_R != 0) {
        break;
        }
    }
    right_up_guai[0]=valid_UP_guai_R;
    right_up_guai[1]=Right_Line[right_up_guai[0]];
    
    // 原有拐点赋值代码（保留）
    Left_Down_Guai[0] = left_down_guai[0];
    Left_Down_Guai[1] = Left_Line[left_down_guai[0]];
    Left_Up_Guai[0] = left_up_guai[0];
    Left_Up_Guai[1] = Left_Line[left_up_guai[0]];       
    Right_Down_Guai[0] = right_down_guai[0];
    Right_Down_Guai[1] = Right_Line[right_down_guai[0]];
    Right_Up_Guai[0] = right_up_guai[0];
    Right_Up_Guai[1] = Right_Line[right_up_guai[0]];

    // 新增：在指定行显示各拐点坐标（适配原有CHAR_WIDTH宏定义）
    #define CHAR_WIDTH 8  // 保持和原有代码一致的字符宽度宏
/*
    // 1. 0行显示左上拐点 Left_Up_Guai [y, x]
    tft180_show_string(0, 0, "L_Up:(");                   // 固定前缀
    tft180_show_int(CHAR_WIDTH*5, 0, Left_Up_Guai[0], 3); // 显示行号（y），占3位
    tft180_show_string(CHAR_WIDTH*8, 0, ",");             
    tft180_show_int(CHAR_WIDTH*9, 0, Left_Up_Guai[1], 3); // 显示列号（x），占3位
    tft180_show_string(CHAR_WIDTH*12, 0, ")");  

    // 2. 20行显示左下拐点 Left_Down_Guai [y, x]
    tft180_show_string(0, 20, "L_Down:(");                
    tft180_show_int(CHAR_WIDTH*7, 20, Left_Down_Guai[0], 3);
    tft180_show_string(CHAR_WIDTH*10, 20, ",");  
    tft180_show_int(CHAR_WIDTH*11, 20, Left_Down_Guai[1], 3);
    tft180_show_string(CHAR_WIDTH*14, 20, ")");  

    // 3. 40行显示右上拐点 Right_Up_Guai [y, x]
    tft180_show_string(0, 40, "R_Up:(");                 
    tft180_show_int(CHAR_WIDTH*5, 40, Right_Up_Guai[0], 3);
    tft180_show_string(CHAR_WIDTH*8, 40, ",");  
    tft180_show_int(CHAR_WIDTH*9, 40, Right_Up_Guai[1], 3);
    tft180_show_string(CHAR_WIDTH*12, 40, ")");  

    // 4. 60行显示右下拐点 Right_Down_Guai [y, x]
    tft180_show_string(0, 60, "R_Down:(");               
    tft180_show_int(CHAR_WIDTH*8, 60, Right_Down_Guai[0], 3);
    tft180_show_string(CHAR_WIDTH*11, 60, ",");  
    tft180_show_int(CHAR_WIDTH*12, 60, Right_Down_Guai[1], 3);
    tft180_show_string(CHAR_WIDTH*15, 60, ")");  

    tft180_show_int(0, 0, continuity_change_left_flag, 3); // 数值占3位，对齐显示
    tft180_show_int(CHAR_WIDTH*4, 0, continuity_change_right_flag, 3); // 数值占3位，对齐显示
    tft180_show_int(0, 20, monotonicity_change_left_flag, 3); // 数值占3位，对齐显示
    tft180_show_int(CHAR_WIDTH*4, 20, monotonicity_change_right_flag, 3); // 数值占3位，对齐显示
    tft180_show_int(0, 40, Left_Lost_Time, 3); // 数值占3位，对齐显示
    tft180_show_int(CHAR_WIDTH*4, 40, Right_Lost_Time, 3); // 数值占3位，对齐显示
    tft180_show_int(CHAR_WIDTH*8, 40, Both_Lost_Time, 3); // 数值占3位，对齐显示
    tft180_show_int(0, 40, Boundry_Start_Left, 3); // 数值占3位，对齐显示
    tft180_show_int(CHAR_WIDTH*4, 40, Boundry_Start_Right, 3); // 数值占3位，对齐显示
*/    
    
    if(Cross_Flag==0&&Island_State==0)
    {
        // ---------------------- 左环岛入口判断（完全保留原有逻辑） ----------------------
        if(Left_Island_Flag==0)
        {
            if(monotonicity_change_right_flag<=25&&       //15 
               continuity_change_right_flag<=15&&         
               continuity_change_left_flag>5&&          
               Left_Lost_Time>=20&&                      
               Left_Lost_Time<=70&&                      
               Right_Lost_Time<=15&&   //5                  
               Search_Stop_Line>=MT9V03X_H*0.95&&       
               Both_Lost_Time<=10&&//5
               Right_Line[1]>50)     
            {
                left_down_guai[0]=Find_Left_Down_Point(Left_Line, MT9V03X_H-1,20);
                if(left_down_guai[0]>=20)
                {
                    Island_State=1;
                    Left_Island_Flag=1;
                }
                else
                {
                    Island_State=0;
                    Left_Island_Flag=0;
                }
            }
        }

        // ---------------------- 右环岛入口判断（镜像左环岛逻辑） ----------------------
        if(Right_Island_Flag==0)
        {
            // 完全镜像左环岛入口条件：Left↔Right 数值对称反转
            if(monotonicity_change_left_flag<=25&&        // 右环岛左边连续（镜像左环岛右边）
               continuity_change_left_flag<=15&&         // 左边界连续（镜像右边界）
               continuity_change_right_flag>5&&         // 右边界不连续（镜像左边界）
               Right_Lost_Time>=20&&                     // 右丢线多（镜像左丢线）
               Right_Lost_Time<=70&&                     // 右丢线不超限
               Left_Lost_Time<=15&&                      // 左丢线少（镜像右丢线）
               Search_Stop_Line>=MT9V03X_H*0.95&&        // 搜索截止行足够远
               Both_Lost_Time<=10&&
               Left_Line[1]<120)     // 左边界列值合理（镜像右边界）
            {
                // 查找右环岛右下角拐点（镜像左环岛左下角）
                right_down_guai[0]=Find_Right_Down_Point(Right_Line, MT9V03X_H-1,20);
                if(right_down_guai[0]>=20)               // 拐点位置有效（镜像左环岛）
                {
                    Island_State=1;
                    Right_Island_Flag=1;
                }
                else
                {
                    Island_State=0;
                    Right_Island_Flag=0;
                }
            }
        }
    }

    // ---------------------- 左环岛状态机（完全保留原有逻辑） ----------------------
    if(Left_Island_Flag==1)
    {
        if(Island_State==1)
        {
            Island1Count++;
            lost_time=Count_Lost_In_Range(89,119, 1);//89,119,1
            
//            tft180_show_int(0, 80, lost_time, 3); // 数值占3位，对齐显示
            
            if(left_down_guai[0]!=0&&left_up_guai[0]!=0)
            {
                Left_Add_Line(Left_Line, Right_Line,Mid_Line,left_up_guai[1], left_up_guai[0], left_down_guai[1], left_down_guai[0]);
            }
            else
            {
                monotonicity_change_line[0]=Monotonicity_Change_Left(Left_Line, 30,5);
                monotonicity_change_line[1]=Left_Line[monotonicity_change_line[0]];
                Left_Add_Line(Left_Line, Right_Line,Mid_Line, monotonicity_change_line[1], monotonicity_change_line[0], 8, MT9V03X_H-1);
            }
            
            if(
//               Boundry_Start_Left>=95&&
//               Boundry_Start_Right>=95&&
               Left_Lost_Time>=35&&
               Left_Lost_Time<=95&&
               lost_time<=20&&//15
               Right_Lost_Time<=20&&
//               monotonicity_change_right_flag==0&&
               continuity_change_left_flag>5&&
               left_up_guai[0]>=15&&
               left_up_guai[0]<=65&&//45
               left_down_guai[0]==0&&
               right_down_guai[0]==0
               )
            {
                Island_State=2;
                Island1Count=0;
            }            
            else if(Island1Count>38){
                Island_State=0;
                Left_Island_Flag=0;
                Island1Count=0;
            }
        }
        else if(Island_State==2)
        {
            Island2Count++;
            monotonicity_change_line[0]=Monotonicity_Change_Left(Left_Line, 70,5);
            monotonicity_change_line[1]=Left_Line[monotonicity_change_line[0]];
            
            if((Island2Count>=30)||(//30
                                    left_up_guai[0]==0&&
                                    left_down_guai[0]==0
                                    ))
            {
                Island_State=3;
                Left_Island_Flag=1;
                Island2Count=0;
            }
        }
        else if(Island_State==3)
        {
            Island3Count++;
            monotonicity_change_line[0]=Monotonicity_Change_Right(Right_Line, 85,10);
            monotonicity_change_line[1]=Right_Line[monotonicity_change_line[0]];
            
            if((
               monotonicity_change_left_flag<=25&&       //25
               monotonicity_change_left_flag>=10&&
               40<=monotonicity_change_line[0]&&
               monotonicity_change_line[0]<=80&&
               Left_Lost_Time>=60&&         //60
               monotonicity_change_right_flag>=40&&
               monotonicity_change_right_flag<=90&&
               Island3Count>=10)||(Island3Count>=14)
               )
            {
                Island_State=4;
                Island3Count=0;
            }
              else if(Island3Count>=20){
                Island_State=0;
                Island3Count=0;
                Left_Island_Flag=0;
            }
        }
        else if(Island_State==4)
        {
            Right_Down_Guai[0]=Monotonicity_Change_Right(Right_Line, 85,10);
            Right_Down_Guai[1]=Right_Line[Right_Down_Guai[0]];
            Island4Count++;
            if(
               Left_Lost_Time>=65&&
               Left_Lost_Time<=95&&
               Right_Lost_Time<=20&&
               monotonicity_change_right_flag==0&&
               continuity_change_left_flag>=30&&
               left_up_guai[0]>=30&&
               left_up_guai[0]<=70&&
               left_down_guai[0]==0&&
               right_down_guai[0]==0&&
               Island4Count>=15
               )
            {
                Island_State=5;
                Island4Count=0;
            }
             else if(Island4Count>=25)//10
            {
                Island_State=5;
                Island4Count=0;
            } 
        }
        else if(Island_State==5)
        {
            Left_Up_Guai[0]=Find_Left_Up_Point(Left_Line, MT9V03X_H-1,10);
            Left_Up_Guai[1]=Left_Line[Left_Up_Guai[0]];
            Lengthen_Left_Boundry(Left_Line,Right_Line,Mid_Line, Left_Up_Guai[0]-1,MT9V03X_H-1);
            Island5Count++;
            if(((Left_Up_Guai[0]>=MT9V03X_H-20||(Left_Up_Guai[0]<10&&Boundry_Start_Right>=MT9V03X_H-10))||Boundry_Start_Left>=90&&Island5Count>=3)||Island5Count>=3)
            {
                Island_State=0;
                Left_Island_Flag=0;
                Island5Count=0;
            }
        }
    }
    // ---------------------- 右环岛状态机（完全镜像左环岛逻辑） ----------------------
    else if(Right_Island_Flag==1)
    {
        if(Island_State==1)// 1状态：拐点存在，未丢线（镜像左环岛State1）
        {
            Island1Count++;
            lost_time=Count_Lost_In_Range(89,119, 0);  // 0=统计右边界丢线（镜像左边界1）
            
//            tft180_show_int(0, 80, lost_time, 3); // 数值占3位，对齐显示
            
            //如果右边上下角点都能看到（镜像左环岛）
            if(right_down_guai[0]!=0&&right_up_guai[0]!=0)
            {
                Right_Add_Line(Right_Line, Left_Line,Mid_Line,right_up_guai[1], right_up_guai[0], right_down_guai[1], right_down_guai[0]);
            }
            else
            {
                // 右边界单调性突变检测（镜像左边界）
                monotonicity_change_line[0]=Monotonicity_Change_Right(Right_Line, 40,5);
                monotonicity_change_line[1]=Right_Line[monotonicity_change_line[0]];
                Right_Add_Line(Right_Line, Left_Line,Mid_Line, monotonicity_change_line[1], monotonicity_change_line[0], MT9V03X_W-8, MT9V03X_H-1); // 列镜像：8→MT9V03X_W-8
            }
            
            // 从1到2的条件（镜像左环岛）
            if(
//               Boundry_Start_Right>=95&&                // 右边界起始点靠下（镜像左边界）
//               Boundry_Start_Left>=95&&                 // 左边界起始点靠下（镜像右边界）
               Right_Lost_Time>=45&&                    // 右丢线多（镜像左丢线）
               Right_Lost_Time<=95&&                    // 右丢线范围（镜像左丢线）
               lost_time<=20&&                          // 丢线数阈值（镜像）
               Left_Lost_Time<=20&&                     // 左丢线少（镜像右丢线）
//               monotonicity_change_left_flag==0&&       // 左单调性无变化（镜像右）
               continuity_change_right_flag>5&&      // 右单调性有变化（镜像左）
               right_up_guai[0]>=15&&                   // 右上拐点行范围（镜像左上）
               right_up_guai[0]<=65&&                   // 右上拐点行范围（镜像左上）
               right_down_guai[0]==0&&                  // 右下拐点丢失（镜像左下）
               left_down_guai[0]==0)                    // 左下拐点丢失（镜像右下）
            {
                Island_State=2;
                Island1Count=0;
            }
            // 超时保护（镜像左环岛）
            else if(Island1Count>38){
                Island_State=0;
                Right_Island_Flag=0;
                Island1Count=0;
            }
        }
        else if(Island_State==2)// 2状态：右下方丢线，上方即将出现大弧线（镜像左环岛State2）
        {
            Island2Count++;

            // 右边界单调性检测（镜像左边界）
            monotonicity_change_line[0]=Monotonicity_Change_Right(Right_Line, 70,5);
            monotonicity_change_line[1]=Right_Line[monotonicity_change_line[0]];
            
            // 超时/丢线判定进入3状态（镜像左环岛）
            if((Island2Count>=30)||(
                                    right_up_guai[0]==0&&    // 右上拐点丢失（镜像左上）
                                    right_down_guai[0]==0    // 右下拐点丢失（镜像左下）
                                    ))
            {
                Island_State=3;// 进入环岛核心区
                Right_Island_Flag=1;
                Island2Count=0;
            }
        }
        else if(Island_State==3)// 3状态：完全进入右环岛（镜像左环岛State3）
        {
            Island3Count++;
            // 左边界单调性检测（镜像右边界）
            monotonicity_change_line[0]=Monotonicity_Change_Left(Left_Line, 85,10);
            monotonicity_change_line[1]=Left_Line[monotonicity_change_line[0]];
            
            // 镜像左环岛：数值范围对称
            if((
               monotonicity_change_right_flag<=25&&       // 右单调性范围（镜像左）//25
               monotonicity_change_right_flag>=10&&       // 右单调性范围（镜像左）
               30<=monotonicity_change_line[0]&&         // 单调点行范围（镜像）
               monotonicity_change_line[0]<=90&&         // 单调点行范围（镜像）
               Right_Lost_Time>=60&&                     // 右丢线多（镜像左丢线） //60
               monotonicity_change_left_flag>=40&&        // 左单调性范围（镜像右）
               monotonicity_change_left_flag<=90&&         // 左单调性范围（镜像右）
               Island3Count>=10)||(Island3Count>=12)
               )//单调点靠下，进入4
            {
                Island_State=4;
                Island3Count=0;
            }
              else if(Island3Count>=60){
                Island_State=0;
                Island3Count=0;
                Right_Island_Flag=0;
            }
        }
        else if(Island_State==4)//准备出右环岛（镜像左环岛State4）
        {
            Left_Down_Guai[0]=Monotonicity_Change_Left(Left_Line, 85,10);  // 左边界单调性（镜像右）
            Left_Down_Guai[1]=Left_Line[Left_Down_Guai[0]];
            Island4Count++;
            if(
               Right_Lost_Time>=40&&                     // 右丢线范围（镜像左）//60
               Right_Lost_Time<=95&&                     // 右丢线范围（镜像左）
               Left_Lost_Time<=20&&                      // 左丢线少（镜像右）
               monotonicity_change_left_flag<=15&&        // 左单调性无变化（镜像右）//==0
               continuity_change_right_flag>=30&&        // 右连续性（镜像左）
               right_up_guai[0]>=20&&                    // 右上拐点行范围（镜像左上）//30
               right_up_guai[0]<=80&&                    // 右上拐点行范围（镜像左上）//70
//               right_down_guai[0]==0&&                   // 右下拐点丢失（镜像左下）
               left_down_guai[0]==0&&
               Island4Count>=12
               )                     // 左下拐点丢失（镜像右下）
            {
                Island_State=5;
                Island4Count=0;
            }
            else if(Island4Count>=25)
            {
                Island_State=5;
                Island4Count=0;
            }  
        }
        else if(Island_State==5)//出右环岛阶段（镜像左环岛State5）
        {
            // 查找右上拐点（镜像左上拐点）
            Right_Up_Guai[0]=Find_Right_Up_Point(Right_Line, MT9V03X_H-1,10);
            Right_Up_Guai[1]=Right_Line[Right_Up_Guai[0]];
            Island5Count++;
            // 延长右边界（镜像左边界）
            Lengthen_Right_Boundry(Left_Line,Right_Line,Mid_Line, Right_Up_Guai[0]-1,MT9V03X_H-1);
            
            // 出环岛判定（镜像左环岛）
            if(((Right_Up_Guai[0]>=MT9V03X_H-20||(Right_Up_Guai[0]<10&&Boundry_Start_Left>=MT9V03X_H-10))||Boundry_Start_Right>=90&&Island5Count>=3)||Island5Count>=30)
            {
                Island_State=0;
                Right_Island_Flag=0;
                Island5Count=0;
            }
        }
    }
}

// 以下所有原有函数完全保留，无需修改
int Continuity_Change_Left(uint8_t* Left_Line,int start,int end)
{
    int i;
    int t;
    int continuity_change_flag=0;
    if(Left_Lost_Time>=0.9*MT9V03X_H)
       return 1;
    if(Search_Stop_Line<=5)
       return 1;
    if(start>=MT9V03X_H-1-5)
        start=MT9V03X_H-1-5;
    if(end<=5)
       end=5;
    if(start<end)
    {
       t=start;
       start=end;
       end=t;
    }
    
    for(i=start;i>=end;i--)
    {
       if(abs(Left_Line[i]-Left_Line[i-1])>=5)
       {
            continuity_change_flag=i;
            break;
       }
    }
    return continuity_change_flag;
}

int Continuity_Change_Right(uint8_t* Right_Line,int start,int end)
{
    int i;
    int t;
    int continuity_change_flag=0;
    if(Right_Lost_Time>=0.9*MT9V03X_H)
       return 1;
    if(start>=MT9V03X_H-5)
        start=MT9V03X_H-5;
    if(end<=5)
       end=5;
    if(start<end)
    {
       t=start;
       start=end;
       end=t;
    }

    for(i=start;i>=end;i--)
    {
        if(abs(Right_Line[i]-Right_Line[i-1])>=5)
       {
            continuity_change_flag=i;
//            break;
       }
    }
    return continuity_change_flag;
}

int Find_Left_Down_Point(uint8_t* Left_Line,int start,int end)
{
    int i,t;
    int left_down_line=0;
    if(Left_Lost_Time>=0.9*MT9V03X_H)
       return left_down_line;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-1-5)
        start=MT9V03X_H-1-5;
    if(end<=MT9V03X_H-Search_Stop_Line)
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)
       end=5;
    for(i=start;i>=end;i--)
    {
        if(left_down_line==0&&
           abs(Left_Line[i]-Left_Line[i+1])<=5&&
           abs(Left_Line[i+1]-Left_Line[i+2])<=5&&  
           abs(Left_Line[i+2]-Left_Line[i+3])<=5&&
              (Left_Line[i]-Left_Line[i-2])>=5&&
              (Left_Line[i]-Left_Line[i-3])>=10&&
              (Left_Line[i]-Left_Line[i-4])>=10)
        {
            left_down_line=i;
            break;
        }
    }
    return left_down_line;
}

int Find_Left_Up_Point(uint8_t* Left_Line,int start,int end)
{
    int i,t;
    int left_up_line=0;
    if(Left_Lost_Time>=0.9*MT9V03X_H)
       return left_up_line;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(end<=MT9V03X_H-Search_Stop_Line)
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)
        end=5;
    if(start>=MT9V03X_H-1-5)
        start=MT9V03X_H-1-5;
    for(i=start;i>=end;i--)
    {
        if(left_up_line==0&&
           abs(Left_Line[i]-Left_Line[i-1])<=5&&
           abs(Left_Line[i-1]-Left_Line[i-2])<=5&&  
           abs(Left_Line[i-2]-Left_Line[i-3])<=5&&
              (Left_Line[i]-Left_Line[i+2])>=8&& 
              (Left_Line[i]-Left_Line[i+3])>=15&&
              (Left_Line[i]-Left_Line[i+4])>=15)
        {
            left_up_line=i;
            break;
        }
    }
    return left_up_line;
}

int Find_Right_Down_Point(uint8_t* Right_Line,int start,int end)
{
    int i,t;
    int right_down_line=0;
    if(Right_Lost_Time>=0.9*MT9V03X_H)
        return right_down_line;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-1-5)
        start=MT9V03X_H-1-5;
    if(end<=MT9V03X_H-Search_Stop_Line)
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)
       end=5;
    for(i=start;i>=end;i--)
    {
        if(right_down_line==0&&
           abs(Right_Line[i]-Right_Line[i+1])<=5&&
           abs(Right_Line[i+1]-Right_Line[i+2])<=5&&  
           abs(Right_Line[i+2]-Right_Line[i+3])<=5&&
              (Right_Line[i]-Right_Line[i-2])<=-5&&
              (Right_Line[i]-Right_Line[i-3])<=-10&&
              (Right_Line[i]-Right_Line[i-4])<=-10)
        {
            right_down_line=i;
            break;
        }
    }
    return right_down_line;
}

int Find_Right_Up_Point(uint8_t* Right_Line, int start, int end)
{
    int i, t;
    int right_up_line = 0;

    if (Right_Lost_Time >= 0.9 * MT9V03X_H)
        return 0;

    if (start < end) { t = start; start = end; end = t; }

    if (end <= MT9V03X_H - Search_Stop_Line)
        end = MT9V03X_H - Search_Stop_Line;

    if (end <= 5)
        end = 5;

    if (start >= MT9V03X_H - 6)
        start = MT9V03X_H - 6;

    for (i = start; i >= end; i--)
    {
        if (i < 4) break;                 
        if (i + 4 >= MT9V03X_W) continue; 

        if (right_up_line == 0 &&
            abs(Right_Line[i]   - Right_Line[i-1]) <= 5 &&
            abs(Right_Line[i-1] - Right_Line[i-2]) <= 5 &&
            abs(Right_Line[i-2] - Right_Line[i-3]) <= 5 &&
            (Right_Line[i] - Right_Line[i+2]) <= -8 &&
            (Right_Line[i] - Right_Line[i+3]) <= -15 &&
            (Right_Line[i] - Right_Line[i+4]) <= -15)
        {
            right_up_line = i;
            break;
        }
    }

    return right_up_line;
}

int Monotonicity_Change_Left(uint8_t* Left_Line,int start,int end)
{
    int i;
    int monotonicity_change_line=0;
    if(Left_Lost_Time>=0.9*MT9V03X_H)
       return monotonicity_change_line;
    if(start>=MT9V03X_H-1-5)
       start=MT9V03X_H-1-5;
    if(end<=5)
        end=5;
    if(start<=end)
      return monotonicity_change_line;
    for(i=start;i>=end;i--)
    {
        if(Left_Line[i]==Left_Line[i+5]&&Left_Line[i]==Left_Line[i-5]&&
        Left_Line[i]==Left_Line[i+4]&&Left_Line[i]==Left_Line[i-4]&&
        Left_Line[i]==Left_Line[i+3]&&Left_Line[i]==Left_Line[i-3]&&
        Left_Line[i]==Left_Line[i+2]&&Left_Line[i]==Left_Line[i-2]&&
        Left_Line[i]==Left_Line[i+1]&&Left_Line[i]==Left_Line[i-1])
        {
            continue;
        }
        else if(Left_Line[i]>=Left_Line[i+5]&&Left_Line[i]>=Left_Line[i-5]&&
        Left_Line[i]>=Left_Line[i+4]&&Left_Line[i]>=Left_Line[i-4]&&
        Left_Line[i]>=Left_Line[i+3]&&Left_Line[i]>=Left_Line[i-3]&&
        Left_Line[i]>=Left_Line[i+2]&&Left_Line[i]>=Left_Line[i-2]&&
        Left_Line[i]>=Left_Line[i+1]&&Left_Line[i]>=Left_Line[i-1])
        {
            monotonicity_change_line=i;
            break;
        }
    }
    return monotonicity_change_line;
}

int Monotonicity_Change_Right(uint8_t* Right_Line,int start,int end)
{
    int i;
    int monotonicity_change_line=0;

    if(Right_Lost_Time>=0.9*MT9V03X_H)
        return monotonicity_change_line;
    if(start>=MT9V03X_H-1-5)
        start=MT9V03X_H-1-5;
     if(end<=5)
        end=5;
    if(start<=end)
        return monotonicity_change_line;
    for(i=start;i>=end;i--)
    {
        if(Right_Line[i]==Right_Line[i+5]&&Right_Line[i]==Right_Line[i-5]&&
        Right_Line[i]==Right_Line[i+4]&&Right_Line[i]==Right_Line[i-4]&&
        Right_Line[i]==Right_Line[i+3]&&Right_Line[i]==Right_Line[i-3]&&
        Right_Line[i]==Right_Line[i+2]&&Right_Line[i]==Right_Line[i-2]&&
        Right_Line[i]==Right_Line[i+1]&&Right_Line[i]==Right_Line[i-1])
        {
            continue;
        }
        else if(Right_Line[i]<=Right_Line[i+5]&&Right_Line[i]<=Right_Line[i-5]&&
        Right_Line[i]<=Right_Line[i+4]&&Right_Line[i]<=Right_Line[i-4]&&
        Right_Line[i]<=Right_Line[i+3]&&Right_Line[i]<=Right_Line[i-3]&&
        Right_Line[i]<=Right_Line[i+2]&&Right_Line[i]<=Right_Line[i-2]&&
        Right_Line[i]<=Right_Line[i+1]&&Right_Line[i]<=Right_Line[i-1])
        {
            monotonicity_change_line=i;
            break;
        }
    }
    return monotonicity_change_line;
}

#ifndef MT9V03X_W
#define MT9V03X_W 752
#endif

#ifndef MT9V03X_H
#define MT9V03X_H 480
#endif

static inline int bound_check(int val, int min, int max)
{
    if (val >= max) return max;
    if (val <= min) return min;
    return val;
}

void K_Add_Boundry_Left(uint8_t* Left_Line, uint8_t* Right_Line, uint8_t* Mid_Line, float k, int startX, int startY, int endY)
{
    int i, t;
    startY = bound_check(startY, 0, MT9V03X_H - 1);
    endY = bound_check(endY, 0, MT9V03X_H - 1);

    if (startY < endY)
    {
        t = startY;
        startY = endY;
        endY = t;
    }

    for (i = startY; i >= endY; i--)
    {
        int x_val = (int)((i - startY) / k + startX);
        Left_Line[i] = (uint8_t)bound_check(x_val, 0, MT9V03X_W - 1);
    }

    if (Mid_Line != NULL)
    {
        for (i = startY; i >= endY; i--)
        {
            if (Right_Line != NULL)
            {
                int mid_val = (Left_Line[i] + Right_Line[i]) / 2;
                Mid_Line[i] = (uint8_t)bound_check(mid_val, 0, MT9V03X_W - 1);
            }
        }
    }
}

void K_Add_Boundry_Right(uint8_t* Left_Line,uint8_t* Right_Line, uint8_t* Mid_Line, float k, int startX, int startY, int endY)
{
    int i, t;
    startY = bound_check(startY, 0, MT9V03X_H - 1);
    endY = bound_check(endY, 0, MT9V03X_H - 1);

    if (startY < endY)
    {
        t = startY;
        startY = endY;
        endY = t;
    }

    for (i = startY; i >= endY; i--)
    {
        int x_val = (int)(startX + (i - startY) * k);
        Right_Line[i] = (uint8_t)bound_check(x_val, 0, MT9V03X_W - 1);
    }

    if (Mid_Line != NULL)
    {
        for (i = startY; i >= endY; i--)
        {
            if (Left_Line != NULL)
            {
                int mid_val = (Left_Line[i] + Right_Line[i]) / 2;
                Mid_Line[i] = (uint8_t)bound_check(mid_val, 0, MT9V03X_W - 1);
            }
        }
    }
}

void K_Draw_Line(float k, int startX, int startY,int endY)
{
    int endX=0;

    if(startX>=MT9V03X_W-1)
        startX=MT9V03X_W-1;
    else if(startX<=0)
        startX=0;
    if(startY>=MT9V03X_H-1)
        startY=MT9V03X_H-1;
    else if(startY<=0)
        startY=0;
    if(endY>=MT9V03X_H-1)
        endY=MT9V03X_H-1;
    else if(endY<=0)
        endY=0;
    endX=(int)((endY-startY)/k+startX);
    Draw_Line(startX,startY,endX,endY);
}

int Get_Road_Wide(uint8_t* Right_Line,uint8_t* Left_Line,int start_line,int end_line)
{
    if(start_line>=MT9V03X_H-1)
        start_line=MT9V03X_H-1;
    else if(start_line<=0)
        start_line=0;
    if(end_line>=MT9V03X_H-1)
        end_line=MT9V03X_H-1;
    else if(end_line<=0)
        end_line=0;
    int i=0,t=0;
    int road_wide=0;
    if(start_line>end_line)
    {
        t=start_line;
        start_line=end_line;
        end_line=t;
    }
    for(i=start_line;i<=end_line;i++)
    {
        road_wide+=Right_Line[i]-Left_Line[i];
    }
    road_wide=road_wide/(end_line-start_line+1);
    return road_wide;
}

float Get_Left_K(uint8_t* Left_Line,int start_line,int end_line)
{
    if(start_line>=MT9V03X_H-1)
        start_line=MT9V03X_H-1;
    else if(start_line<=0)
        start_line=0;
    if(end_line>=MT9V03X_H-1)
        end_line=MT9V03X_H-1;
    else if(end_line<=0)
        end_line=0;
    float k=0;
    int t=0;
    if(start_line>end_line)
    {
        t=start_line;
        start_line=end_line;
        end_line=t;
    }
    k=(float)(((float)Left_Line[start_line]-(float)Left_Line[end_line])/(end_line-start_line+1));
    return k;
}

float Get_Right_K(uint8_t* Right_Line,int start_line,int end_line)
{
    if(start_line>=MT9V03X_H-1)
        start_line=MT9V03X_H-1;
    else if(start_line<=0)
        start_line=0;
    if(end_line>=MT9V03X_H-1)
        end_line=MT9V03X_H-1;
    else if(end_line<=0)
        end_line=0;
    float k=0;
    int t=0;
    if(start_line>end_line)
    {
        t=start_line;
        start_line=end_line;
        end_line=t;
    }
    k=(float)(((float)Right_Line[start_line]-(float)Right_Line[end_line])/(end_line-start_line+1));
    return k;
}

void DrawR3(uint8_t* Mid_Line, int start_row)
{
    if (start_row < 0) start_row = 0;
    if (start_row > 35) start_row = 35;

    float k_line = 0.0f;
    int row_diff = (MT9V03X_H - 1) - start_row;
    if (row_diff != 0) {
        k_line = (float)((2*MT9V03X_W / 5) - (5*MT9V03X_W/7+5)) / (float)row_diff;
    }
  
    for (int i = 0; i < MT9V03X_H; i++) {
        int new_mid_col = (int)((MT9V03X_W - 1) + k_line * (i - start_row));
        new_mid_col = new_mid_col < 0 ? 0 : (new_mid_col >= MT9V03X_W ? MT9V03X_W - 1 : new_mid_col);
        Mid_Line[i] = new_mid_col;
    }
}

void DrawL3(uint8_t* Mid_Line, int start_row)
{
    if (start_row < 0) start_row = 0;
    if (start_row > 35) start_row = 35;

    float k_line = 0.0f;
    int row_diff = (MT9V03X_H - 1) - start_row;
    if (row_diff != 0) {
        k_line = -((float)((2*MT9V03X_W / 5) - (5*MT9V03X_W/7+5)) / (float)row_diff);
    }
  
    for (int i = 0; i < MT9V03X_H; i++) {
        int r3_col = (int)((MT9V03X_W - 1) + (-k_line) * (i - start_row));
        int new_mid_col = (MT9V03X_W - 1) - r3_col;
        
        new_mid_col = new_mid_col < 0 ? 0 : (new_mid_col >= MT9V03X_W ? MT9V03X_W - 1 : new_mid_col);
        Mid_Line[i] = new_mid_col;
    }
}

void DrawR4(uint8_t* Mid_Line, int start_row)
{
    if (start_row < 0) start_row = 0;
    if (start_row > 35) start_row = 35;

    float k_line = 0.0f;
    int row_diff = (MT9V03X_H - 1) - start_row;
    if (row_diff != 0) {
        k_line = (float)((2*MT9V03X_W / 5) - (MT9V03X_W -58)) / (float)row_diff;
    }
  
    for (int i = 0; i < MT9V03X_H; i++) {
        int new_mid_col = (int)((MT9V03X_W - 1) + k_line * (i - start_row));
        new_mid_col = new_mid_col < 0 ? 0 : (new_mid_col >= MT9V03X_W ? MT9V03X_W - 1 : new_mid_col);
        Mid_Line[i] = new_mid_col;
    }
}

void DrawL4(uint8_t* Mid_Line, int start_row)
{
    if (start_row < 0) start_row = 0;
    if (start_row > 35) start_row = 35;

    float k_line = 0.0f;
    int row_diff = (MT9V03X_H - 1) - start_row;
    if (row_diff != 0) {
        k_line = -((float)((2*MT9V03X_W / 5) - (MT9V03X_W - 58)) / (float)row_diff);
    }

    for (int i = 0; i < MT9V03X_H; i++) {
        int new_mid_col = (int)(0 + k_line * (i - start_row));
        
        new_mid_col = new_mid_col < 0 ? 0 : (new_mid_col >= MT9V03X_W ? MT9V03X_W - 1 : new_mid_col);
        Mid_Line[i] = new_mid_col;
    }
}

void DrawR1(uint8_t* Left_Line, uint8_t* Mid_Line)
{
    const uint16_t FIXED_RIGHT_COL = 160;
    for (int i = 0; i < MT9V03X_H; i++)
    {
        uint16_t left_col = Left_Line[i];
        uint16_t mid_col = (left_col + FIXED_RIGHT_COL) / 2;
        uint16_t max_col = (MT9V03X_W > 0) ? (MT9V03X_W - 1) : 0;
        mid_col = (mid_col >= max_col) ? max_col : (mid_col <= 0 ? 0 : mid_col);
        Mid_Line[i] = (uint8_t)mid_col;
    }
}

int Count_Lost_In_Range(int start_row, int end_row, uint8_t is_left)
{
    int lost_count = 0;
    int i, t;

    if (start_row < end_row)
    {
        t = start_row;
        start_row = end_row;
        end_row = t;
    }
    if (start_row >= MT9V03X_H) start_row = MT9V03X_H - 1;
    if (end_row < 0) end_row = 0;

    for (i = start_row; i >= end_row; i--)
    {
        if (is_left)
        {
            if (Left_Lost_Flag[i] == 1) lost_count++;
        }
        else
        {
            if (Right_Lost_Flag[i] == 1) lost_count++;
        }
    }

    return lost_count;
}

// 补充左环岛绘制函数（原有逻辑保留）
void DrawL1(uint8_t* Left_Line, uint8_t* Mid_Line)
{
    const uint16_t FIXED_RIGHT_COL = 160;
    for (int i = 0; i < MT9V03X_H; i++)
    {
        uint16_t left_col = Left_Line[i];
        uint16_t mid_col = (left_col + FIXED_RIGHT_COL) / 2;
        uint16_t max_col = (MT9V03X_W > 0) ? (MT9V03X_W - 1) : 0;
        mid_col = (mid_col >= max_col) ? max_col : (mid_col <= 0 ? 0 : mid_col);
        Mid_Line[i] = (uint8_t)mid_col;
    }
}