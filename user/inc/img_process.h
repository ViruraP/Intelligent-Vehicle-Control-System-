#ifndef _IMG_PROCESS_H_
#define _IMG_PROCESS_H_
#include "zf_common_headfile.h"
#define IMG_BLACK     0X00      //0x00是黑
#define IMG_WHITE     0Xff      //0xff为白

extern volatile int Curve_Flag;
extern volatile int Island_State;     //环岛状态标志
extern volatile int Left_Island_Flag; //左右环岛标志
extern volatile int Right_Island_Flag;//左右环岛标志
extern volatile int Cross_Time_Flag;
unsigned char get_threshold_otsu(const short* _hist_gram);
void binaryzation_process(unsigned char* _img, 
						  const unsigned short _rows, 
						  const unsigned short _cols, 
						  const unsigned int _threshold_value);
void auxiliary_process(uint8_t* _src_pixel_mat, uint8_t _src_rows, uint8_t _src_cols, 
					   unsigned char _threshold_val, 
					   uint8_t* _left_line, uint8_t* _mid_line, uint8_t* _right_line);

void Longest_White_Column(uint8_t* Right_Line,uint8_t* Left_Line);
void Zebra_Stripes_Detect(uint8_t* right_line,uint8_t* left_line);
void Cross_Detect(uint8_t* Right_Line,uint8_t* Left_Line,uint8_t* Mid_Line);
void Find_Up_Point(int start,int end,uint8_t* Right_Line,uint8_t* Left_Line);
void Find_Down_Point(int start,int end,uint8_t* Right_Line,uint8_t* Left_Line);
void Left_Add_Line(uint8_t* Left_Line, uint8_t* Right_Line, uint8_t* Mid_Line, int x1, int y1, int x2, int y2);
void Right_Add_Line(uint8_t* Right_Line, uint8_t* Left_Line, uint8_t* Mid_Line, int x1, int y1, int x2, int y2);
void DrawCross(uint8_t* Mid_Line, int Left_Up_Find, int Right_Up_Find, 
            uint8_t* Left_Line, uint8_t* Right_Line);
float Err_Sum(uint8_t* Right_Line,uint8_t* Left_Line);





#endif
