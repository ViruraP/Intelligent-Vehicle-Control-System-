#ifndef __ISLAND_H__
#define __ISLAND_H__

#include "zf_common_headfile.h"
#include "img_process.h"






void Island_Detect(uint8_t* right_line,uint8_t* left_line,uint8_t* Mid_Line);
void Image_Flag_Show(uint8 InImg[][MT9V03X_W], uint8 image_flag);
// 含Right：新增第一个参数 uint8_t* Right_Line
int Monotonicity_Change_Right(uint8_t* Right_Line, int start,int end);
// 含Left：新增第一个参数 uint8_t* Left_Line
int Monotonicity_Change_Left(uint8_t* Left_Line, int start,int end);
// 含Right：新增第一个参数 uint8_t* Right_Line
int Continuity_Change_Right(uint8_t* Right_Line, int start,int end);
// 含Left：新增第一个参数 uint8_t* Left_Line
int Continuity_Change_Left(uint8_t* Left_Line, int start,int end);
// 含Left：新增第一个参数 uint8_t* Left_Line
int Find_Left_Down_Point(uint8_t* Left_Line, int start,int end);
// 含Left：新增第一个参数 uint8_t* Left_Line
int Find_Left_Up_Point(uint8_t* Left_Line, int start,int end);
// 含Right：新增第一个参数 uint8_t* Right_Line
int Find_Right_Down_Point(uint8_t* Right_Line, int start,int end);
// 含Right：新增第一个参数 uint8_t* Right_Line
int Find_Right_Up_Point(uint8_t* Right_Line, int start,int end);
int Get_Road_Wide(uint8_t* Right_Line,uint8_t* Left_Line,int start_line,int end_line);
// 含Left：新增第一个参数 uint8_t* Left_Line
void K_Add_Boundry_Left(uint8_t* Left_Line, uint8_t* Right_Line, uint8_t* Mid_Line, float k, int startX, int startY, int endY);
// 含Right：新增第一个参数 uint8_t* Right_Line
void K_Add_Boundry_Right(uint8_t* Right_Line, uint8_t* Left_Line, uint8_t* Mid_Line, float k, int startX, int startY, int endY);
void K_Draw_Line(float k, int startX, int startY,int endY);
// 含Right：新增第一个参数 uint8_t* Right_Line
float Get_Right_K(uint8_t* Right_Line, int start_line,int end_line);
// 含Left：新增第一个参数 uint8_t* Left_Line
float Get_Left_K(uint8_t* Left_Line, int start_line,int end_line);

int Count_Lost_In_Range(int start_row, int end_row, uint8_t is_left);

void Lengthen_Left_Boundry(uint8_t* Left_Line,uint8_t* Right_Line,uint8_t* Mid_Line,int start,int end);
void Lengthen_Right_Boundry(uint8_t* Left_Line,uint8_t* Right_Line,uint8_t* Mid_Line,int start,int end);
void DrawL1(uint8_t* Right_Line, uint8_t* Mid_Line);


void DrawR1(uint8_t* Left_Line, uint8_t* Mid_Line);
void DrawR3(uint8_t* Mid_Line, int start_row);
void DrawR4(uint8_t* Mid_Line, int start_row);
void DrawL1(uint8_t* Right_Line, uint8_t* Mid_Line);
void DrawL3(uint8_t* Mid_Line, int start_row);
void DrawL4(uint8_t* Mid_Line, int start_row);

#endif