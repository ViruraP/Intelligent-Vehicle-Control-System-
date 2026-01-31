

#include "img_process.h"
#include "string.h"
#include "Island.h"

//常用基本变量
extern const uint8 Image_Flags[][9][8];     //放在图上的数字标记
uint8 image_two_value[MT9V03X_H][MT9V03X_W];//二值化后的原数组
volatile int Search_Stop_Line;     //搜索截止行
volatile int Road_Wide[MT9V03X_H]; //赛宽数组
int White_Column[MT9V03X_W];    //每列白列长度
volatile int Boundry_Start_Left;   //左右边界起始点
volatile int Boundry_Start_Right;  //第一个非丢线点,常规边界起始点
volatile int Left_Lost_Time;       //边界丢线数
volatile int Right_Lost_Time;
volatile int Both_Lost_Time;//两边同时丢线数
int Longest_White_Column_Left[2]; //最长白列,[0]是最长白列的长度，也就是Search_Stop_Line搜索截止行，[1】是第某列
int Longest_White_Column_Right[2];//最长白列,[0]是最长白列的长度，也就是Search_Stop_Line搜索截止行，[1】是第某列
int Left_Lost_Flag[MT9V03X_H] ; //左丢线数组，丢线置1，没丢线置0
int Right_Lost_Flag[MT9V03X_H]; //右丢线数Road_Wide组，丢线置1，没丢线置0
//环岛


//十字
volatile int Cross_Flag=0;
volatile int Left_Down_Find=0; //十字使用，找到被置行数，没找到就是0
volatile int Left_Up_Find=0;   //四个拐点标志
volatile int Right_Down_Find=0;
volatile int Right_Up_Find=0;
volatile int Curve_Flag=0;
volatile int Cross_Time_Flag=0;
static uint8_t Cross_Lock = 0;      // 锁：防止同一次十字重复触发
static uint16_t Cross_HighFrame = 0; // 连续“检测到十字”的帧数
static uint16_t Cross_LowFrame  = 0; // 连续“没检测到十字”的帧数

// 全局变量
uint8_t Zebra_Counter = 0;    // 0: 未过；1: 过一次；2: 过两次（触发）
uint8_t Zebra_Lock = 0;       // 防止同一次斑马线连续触发（帧级锁）
uint16_t Zebra_LowFrameCount = 0;   // change_count <=30 的连续帧计数
static uint8_t Zebra_HighFrameCount = 0; 
//加权控制
const uint8 Weight[MT9V03X_H]=
{
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端00 ——09 行权重
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端10 ——19 行权重
        1, 1, 1, 1, 1, 1, 1, 3, 4, 5,              //图像最远端20 ——29 行权重
        6, 7, 9,11,13,15,17,19,20,20,              //图像最远端30 ——39 行权重
       19,17,15,13,11, 9, 7, 5, 3, 1,              //图像最远端40 ——49 行权重
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端50 ——59 行权重
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端60 ——69 行权重

};



//斑马线
volatile int Zebra_Stripes_Flag=0;//斑马线

//控制标
volatile uint8 Obstacle_Dir=0; //0右拐，1左拐
volatile uint8 Island_Switch=1;//环岛识别开启标志位
volatile uint8 Straight_Flag=0;//长直道识别标
const uint16 Standard_Road_Wide[120] =
{
    27,28,28,30,31,
    32,33,34,35,36,
    37,38,40,41,42,
    42,44,45,46,47,
    48,49,51,51,52,
    54,55,56,57,58,//30

58,59,60,61,62,
63,64,65,66,67,
68,69,70,71,72,
73,74,75,76,77,
78,79,80,81,82,
83,84,85,86,87,//60

89,91,93,95,97,
99,101,103,105,107,
109,111,113,115,117,
118,119,120,121,122,
123,124,124,126,126,
127,127,128,129,130,//90


    131,132,133,134,135,
    136,138,138,139,141,//100
    141,143,144,145,146,
    147,148,149,150,151,//110

    152,153,154,155,151,
    151,150,150,150,149
};//需要实测

void binaryzation_process(unsigned char* _img, 
                          const unsigned short _rows, 
                          const unsigned short _cols, 
                          const unsigned int _threshold_value)
{
    for (unsigned short i = 0; i < _rows; ++i)
    {
        for (unsigned short j = 0; j < _cols; ++j)
        {
            unsigned short idx = i * _cols + j;

            if (_img[idx] >= _threshold_value)
            {
                _img[idx] = 255;             // 原功能：图像二值化
                image_two_value[i][j] = IMG_WHITE; // 新增：给 LWC 使用
            }
            else
            {
                _img[idx] = 0;
                image_two_value[i][j] = IMG_BLACK;
            }
        }
    }
}


void auxiliary_process(uint8_t* _src_pixel_mat, uint8_t _src_rows, uint8_t _src_cols, 
					   unsigned char _threshold_val, 
					   uint8_t* _left_line, uint8_t* _mid_line, uint8_t* _right_line)
{
  Curve_Flag=0;
    uint8_t mid_point = _src_cols >> 1;
    if (_src_pixel_mat[(_src_rows - 1) * _src_cols + mid_point] < _threshold_val)
    {
        // 重新寻找扫线中点
        int16_t _tmp_left_mid = 0;
        int16_t _tmp_right_mid = _src_cols;
        //向右搜索赛道
		for (int16_t j = (_src_cols >> 1); j < _src_cols - 2; ++j)
        {
            if (_src_pixel_mat[(_src_rows - 1) * _src_cols + j] > _threshold_val && 
				_src_pixel_mat[(_src_rows - 1) * _src_cols + j + 1] > _threshold_val && 
				_src_pixel_mat[(_src_rows - 1) * _src_cols + j + 2] > _threshold_val)
            {
                _tmp_right_mid = j + 2;
            }
        }
		//向左搜索赛道
        for (int16_t j = (_src_cols >> 1); j > 2; --j)
        {
            if (_src_pixel_mat[(_src_rows - 1) * _src_cols + j] > _threshold_val && 
				_src_pixel_mat[(_src_rows - 1) * _src_cols + j - 1] > _threshold_val && 
				_src_pixel_mat[(_src_rows - 1) * _src_cols + j - 2] > _threshold_val)
            {
                _tmp_left_mid = j - 2;
            }
        }
        if (_tmp_right_mid - (_src_cols >> 1) < (_src_cols >> 1) - _tmp_left_mid)
        {
            mid_point = _tmp_right_mid;
        }
        else
        {
            mid_point = _tmp_left_mid;
        }
    }
    for (int i = _src_rows - 1; i >= 0; --i)
    {
        uint8_t cur_point = mid_point;
       // 扫描左线
        while (cur_point - 2 > 0)
        {
            _left_line[i] = 0;
            if (_src_pixel_mat[i * _src_cols + cur_point] < _threshold_val && 
				_src_pixel_mat[i * _src_cols + cur_point - 1] < _threshold_val && 
				_src_pixel_mat[i * _src_cols + cur_point - 2] < _threshold_val)
            {
                _left_line[i] = cur_point;
                break;
            }
            --cur_point;
        }
        // 扫描右线
        cur_point = mid_point;
        while (cur_point + 2 < _src_cols)
        {
            _right_line[i] = _src_cols - 1;
            if (_src_pixel_mat[i * _src_cols + cur_point] < _threshold_val &&
				_src_pixel_mat[i * _src_cols + cur_point + 1] < _threshold_val &&
				_src_pixel_mat[i * _src_cols + cur_point + 2] < _threshold_val)
            {
                _right_line[i] = cur_point;
                break;
            }
            ++cur_point;
        }

        _mid_line[i] = (_right_line[i] + _left_line[i]) >> 1;
        if (_left_line[i] != 0 || 
			_right_line[i] != _src_cols - 1)
        {
            mid_point = _mid_line[i];
        }
        else
        {
            mid_point = _src_cols >> 1;
        }
    }
int y = _src_rows - _src_rows / 4;   // 和你舵机用同一行
int dx = _mid_line[y] - _mid_line[y - 25];
//180_show_int(0, 60, dx, 3);

if (abs(dx) > 8&&Right_Island_Flag!=1&&Cross_Time_Flag!=1&&Left_Island_Flag!=1) {
    Curve_Flag = 1;
} else {
    Curve_Flag = 0;
}

}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     双最长白列巡线
  @param     null
  @return    null
  Sample     Longest_White_Column_Left();
  @note      最长白列巡线，寻找初始边界，丢线，最长白列等基础元素，后续读取这些变量来进行赛道识别
-------------------------------------------------------------------------------------------------------------------*/
void Longest_White_Column(uint8_t* Right_Line,uint8_t* Left_Line)//最长白列巡线
{
    int i, j;
    int start_column=20;//最长白列的搜索区间
    int end_column=MT9V03X_W-20;
    
    // 1. 初始化所有变量（避免野值）
    Longest_White_Column_Left[0] = 0;
    Longest_White_Column_Left[1] = 0;
    Longest_White_Column_Right[0] = 0;
    Longest_White_Column_Right[1] = 0;
    Right_Lost_Time = 0;    
    Left_Lost_Time  = 0;
    Boundry_Start_Left  = 0;
    Boundry_Start_Right = 0;
    Both_Lost_Time = 0;

    // 2. 清零丢线标志数组 + 白列数组（核心：补充White_Column清零，避免累计）
    memset(Left_Lost_Flag, 0, sizeof(Left_Lost_Flag));
    memset(Right_Lost_Flag, 0, sizeof(Right_Lost_Flag));
    memset(White_Column, 0, sizeof(White_Column));

    // 3. 环岛3状态调整最长白列搜索范围
    if(Right_Island_Flag==1 && Island_State==3)
    {
        start_column=40;
        end_column=MT9V03X_W-20;
    }
    else if(Left_Island_Flag==1 && Island_State==3)
    {
        start_column=20;
        end_column=MT9V03X_W-40;
    }

    // 4. 统计每列白像素数量（从下往上）
    for (j = start_column; j <= end_column; j++)
    {
        White_Column[j] = 0; // 每列单独清零，避免累计
        for (i = MT9V03X_H - 1; i >= 0; i--)
        {
            if(image_two_value[i][j] == IMG_BLACK)
                break;
            else
                White_Column[j]++;
        }
    }

    // 5. 找左侧最长白列（从左到右）
    for(i=start_column; i<=end_column; i++)
    {
        if (Longest_White_Column_Left[0] < White_Column[i])
        {
            Longest_White_Column_Left[0] = White_Column[i];
            Longest_White_Column_Left[1] = i;
        }
    }

    // 6. 找右侧最长白列（从右到左）
    for(i=end_column; i>=start_column; i--)
    {
        if (Longest_White_Column_Right[0] < White_Column[i])
        {
            Longest_White_Column_Right[0] = White_Column[i];
            Longest_White_Column_Right[1] = i;
        }
    }

    // 7. 确定搜索截止行
    Search_Stop_Line = Longest_White_Column_Left[0];
    if(Search_Stop_Line <= 0) Search_Stop_Line = 1; // 避免后续循环条件异常

    // 8. 扫描边界，标记丢线标志（外层循环：行遍历）
    for (i = MT9V03X_H - 1; i >= MT9V03X_H - Search_Stop_Line; i--)
    {
        // 8.1 扫描右边界
        Right_Lost_Flag[i] = 1; // 默认丢线
        for (j = Longest_White_Column_Right[1]; j <= MT9V03X_W - 3; j++)
        {
            if (image_two_value[i][j] == IMG_WHITE && 
                image_two_value[i][j + 1] == IMG_BLACK && 
                image_two_value[i][j + 2] == IMG_BLACK)
            {
                Right_Lost_Flag[i] = 0; // 找到边界，未丢线
                break;
            }
        }

        // 8.2 扫描左边界
        Left_Lost_Flag[i] = 1; // 默认丢线
        for (j = Longest_White_Column_Left[1]; j >= 2; j--)
        {
            if (image_two_value[i][j] == IMG_WHITE && 
                image_two_value[i][j - 1] == IMG_BLACK && 
                image_two_value[i][j - 2] == IMG_BLACK)
            {
                Left_Lost_Flag[i] = 0; // 找到边界，未丢线
                break;
            }
        }
    }

    // 9. 统计丢线总数（独立循环，避免变量复用冲突）
    for (i = MT9V03X_H - 1; i >= 0; i--)
    {
        if (Left_Lost_Flag[i] == 1)  Left_Lost_Time++;
        if (Right_Lost_Flag[i] == 1) Right_Lost_Time++;
        if (Left_Lost_Flag[i] == 1 && Right_Lost_Flag[i] == 1) Both_Lost_Time++;

        // 记录第一个非丢线点（边界起始点）
        if (Boundry_Start_Left == 0 && Left_Lost_Flag[i] != 1)
            Boundry_Start_Left = i;
        if (Boundry_Start_Right == 0 && Right_Lost_Flag[i] != 1)
            Boundry_Start_Right = i;

        // 计算赛宽
        Road_Wide[i] = Right_Line[i] - Left_Line[i];
    }

    // 10. 调试显示
//    tft180_show_int(0, 40, Left_Lost_Time, 3);
//    tft180_show_int(0, 60, Right_Lost_Time, 3);
}


/*-------------------------------------------------------------------------------------------------------------------
  @brief     左补线
  @param     补线的起点，终点
  @return    null
  Sample     Left_Add_Line(int x1,int y1,int x2,int y2);
  @note      补的直接是边界，点最好是可信度高的,不要乱补
-------------------------------------------------------------------------------------------------------------------*/
void Left_Add_Line(uint8_t* Left_Line, uint8_t* Right_Line, uint8_t* Mid_Line, int x1, int y1, int x2, int y2)
{
    if (y2 == y1) return;   // ★★★ 防止除 0 → 直接 return

    int i, max, a1, a2;
    int hx;

    // ========== 越界保护（坐标校正） ==========
    if(x1 >= MT9V03X_W - 1) x1 = MT9V03X_W - 1;
    else if(x1 <= 0) x1 = 0;

    if(y1 >= MT9V03X_H - 1) y1 = MT9V03X_H - 1;
    else if(y1 <= 0) y1 = 0;

    if(x2 >= MT9V03X_W - 1) x2 = MT9V03X_W - 1;
    else if(x2 <= 0) x2 = 0;

    if(y2 >= MT9V03X_H - 1) y2 = MT9V03X_H - 1;
    else if(y2 <= 0) y2 = 0;

    // ========== 确定补线行范围（a1≤i≤a2） ==========
    a1 = y1;
    a2 = y2;
    if(a1 > a2)
    {
        max = a1;
        a1 = a2;
        a2 = max;
    }

    // ========== 补左边界 + 同步更新中线 ==========
    for(i = a1; i <= a2; i++)
    {
        if (y2 == y1) break; // 二次防除0

        // 1. 计算补线后的左边界坐标
        hx = (i - y1) * (x2 - x1) / (y2 - y1) + x1;
        if(hx >= MT9V03X_W) hx = MT9V03X_W - 1; // 修正：原MT9V03X_W → MT9V03X_W-1（避免越界）
        else if(hx <= 0) hx = 0;
        Left_Line[i] = hx;

        // 2. 重新计算对应行的中线（核心新增逻辑）
        if(Mid_Line != NULL && Right_Line != NULL) // 判空保护，避免空指针
        {
            // 中线公式：Mid = (Left + Right) / 2
            int mid_col = (Left_Line[i] + Right_Line[i]) / 2;
            // 中线越界保护
            if(mid_col >= MT9V03X_W - 1) mid_col = MT9V03X_W - 1;
            else if(mid_col <= 0) mid_col = 0;
            Mid_Line[i] = mid_col;
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     右补线（补线后重新计算中线）
  @param     Right_Line: 右边界数组（输出，补线结果）
  @param     Left_Line: 左边界数组（输入，用于计算中线）
  @param     Mid_Line: 中线数组（输出，补线后重新计算）
  @param     x1/y1: 补线起点坐标
  @param     x2/y2: 补线终点坐标
  @return    null
  Sample     Right_Add_Line(Right_Line, Left_Line, Mid_Line, x1, y1, x2, y2);
  @note      补右边界后，按「中线=(左+右)/2」重新计算对应行的中线
-------------------------------------------------------------------------------------------------------------------*/
void Right_Add_Line(uint8_t* Right_Line, uint8_t* Left_Line, uint8_t* Mid_Line, int x1, int y1, int x2, int y2)
{
    if (y2 == y1) return;   // ★★★ 防止除 0 → 直接 return

    int i, max, a1, a2;
    int hx;

    // ========== 越界保护（坐标校正） ==========
    if(x1 >= MT9V03X_W - 1) x1 = MT9V03X_W - 1;
    else if(x1 <= 0) x1 = 0;

    if(y1 >= MT9V03X_H - 1) y1 = MT9V03X_H - 1;
    else if(y1 <= 0) y1 = 0;

    if(x2 >= MT9V03X_W - 1) x2 = MT9V03X_W - 1;
    else if(x2 <= 0) x2 = 0;

    if(y2 >= MT9V03X_H - 1) y2 = MT9V03X_H - 1;
    else if(y2 <= 0) y2 = 0;

    // ========== 确定补线行范围（a1≤i≤a2） ==========
    a1 = y1;
    a2 = y2;
    if(a1 > a2)
    {
        max = a1;
        a1 = a2;
        a2 = max;
    }

    // ========== 补右边界 + 同步更新中线 ==========
    for(i = a1; i <= a2; i++)
    {
        if (y2 == y1) break; // 二次防除0

        // 1. 计算补线后的右边界坐标
        hx = (i - y1) * (x2 - x1) / (y2 - y1) + x1;
        if(hx >= MT9V03X_W - 1) hx = MT9V03X_W - 1; // 修正越界逻辑
        else if(hx <= 0) hx = 0;
        Right_Line[i] = hx;

        // 2. 重新计算对应行的中线（核心新增逻辑）
        if(Mid_Line != NULL && Left_Line != NULL) // 判空保护
        {
            // 中线公式：Mid = (Left + Right) / 2
            int mid_col = (Left_Line[i] + Right_Line[i]) / 2;
            // 中线越界保护
            if(mid_col >= MT9V03X_W - 1) mid_col = MT9V03X_W - 1;
            else if(mid_col <= 0) mid_col = 0;
            Mid_Line[i] = mid_col;
        }
    }
}


/*-------------------------------------------------------------------------------------------------------------------
  @brief     左边界延长
  @param     延长起始行数，延长到某行
  @return    null
  Sample     Stop_Detect(void)
  @note      从起始点向上找5个点，算出斜率，向下延长，直至结束点
-------------------------------------------------------------------------------------------------------------------*/
void Lengthen_Left_Boundry(uint8_t* Left_Line,uint8_t* Right_Line,uint8_t* Mid_Line,int start,int end)
{
    int i,t;
    float k=0;
    if(start>=MT9V03X_H-1)//起始点位置校正，排除数组越界的可能
        start=MT9V03X_H-1;
    else if(start<=0)
        start=0;
    if(end>=MT9V03X_H-1)
        end=MT9V03X_H-1;
    else if(end<=0)
        end=0;
    if(end<start)//++访问，坐标互换
    {
        t=end;
        end=start;
        start=t;
    }

    if(start<=5)//因为需要在开始点向上找3个点，对于起始点过于靠上，不能做延长，只能直接连线
    {
         Left_Add_Line(Left_Line,Right_Line,Mid_Line,Left_Line[start],start,Left_Line[end],end);
    }

    else
    {
        k=(float)(Left_Line[start]-Left_Line[start-4])/5.0;//这里的k是1/斜率
        for(i=start;i<=end;i++)
        {
            Left_Line[i]=(int)(i-start)*k+Left_Line[start];//(x=(y-y1)*k+x1),点斜式变形
            if(Left_Line[i]>=MT9V03X_W-1)
            {
                Left_Line[i]=MT9V03X_W-1;
            }
            else if(Left_Line[i]<=0)
            {
                Left_Line[i]=0;
            }
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     延长右边界（支持向上/向下延长）
  @param     Left_Line: 左边界数组（输入，用于计算中线）
  @param     Right_Line: 右边界数组（输出，延长后的结果）
  @param     Mid_Line: 中线数组（输出，延长后重新计算）
  @param     start: 延长的起始行（Y坐标）
  @param     end: 延长的结束行（Y坐标）
  @return    null
  @note      1. start > end → 向上延长（行号减小，图像顶部）；
              2. start < end → 向下延长（行号增大，图像底部）；
              3. 自动适配斜率计算和遍历方向，保持边界趋势一致。
-------------------------------------------------------------------------------------------------------------------*/
void Lengthen_Right_Boundry(uint8_t* Left_Line,uint8_t* Right_Line,uint8_t* Mid_Line,int start,int end)
{
    int i,t;
    float k=0;
    int extend_direction = 0; // 延长方向：0=向下（start<end），1=向上（start>end）

    // ========== 1. 坐标校正 + 确定延长方向 ==========
    // 行号越界保护
    if(start>=MT9V03X_H-1) start=MT9V03X_H-1;
    else if(start<=0) start=0;
    if(end>=MT9V03X_H-1) end=MT9V03X_H-1;
    else if(end<=0) end=0;

    // 记录延长方向（不互换start/end，保留原始方向）
    if(start > end) extend_direction = 1; // 向上延长
    else if(start < end) extend_direction = 0; // 向下延长
    else return; // 起始=结束，无需延长

    // ========== 2. 向上延长逻辑（核心新增） ==========
    if(extend_direction == 1) 
    {
        // 斜率计算：基于start后4行（更靠近start的行，保证趋势准确）
        if (start + 4 < MT9V03X_H) // 防止越界（start+4不超过图像底部）
        {
            k = (float)(Right_Line[start] - Right_Line[start+4]) / (-5.0); 
            // 向上延长：行号减小，分母为-5（抵消行差的负数，保证斜率趋势一致）
        }
        else // start太靠近底部，取start前2行计算
        {
            k = (float)(Right_Line[start] - Right_Line[start-2]) / (-3.0);
        }

        // 逐行向上延长（i从start→end，行号递减）
        for(i = start; i >= end; i--)
        {
            // 点斜式：x = (y - y1)*k + x1 （y=i，y1=start，x1=Right_Line[start]）
            Right_Line[i] = (int)((i - start) * k + Right_Line[start]);
            // 越界保护
            if(Right_Line[i]>=MT9V03X_W-1) Right_Line[i]=MT9V03X_W-1;
            else if(Right_Line[i]<=0) Right_Line[i]=0;

            // 同步更新中线
            if(Mid_Line != NULL && Left_Line != NULL)
            {
                int mid_col = (Left_Line[i] + Right_Line[i]) / 2;
                mid_col = mid_col <0 ? 0 : (mid_col >= MT9V03X_W-1 ? MT9V03X_W-1 : mid_col);
                Mid_Line[i] = mid_col;
            }
        }
        return; // 向上延长逻辑结束
    }

    // ========== 3. 原有向下延长逻辑（保留，兼容旧调用） ==========
    if(extend_direction == 0)
    {
        if (start <= 5)
        {
            Right_Add_Line(Right_Line,Left_Line,Mid_Line,
                       Right_Line[start],
                       start,
                       Right_Line[end],
                       end);
            return;
        }
        // 向下延长的斜率计算（原有逻辑）
        k=(float)(Right_Line[start]-Right_Line[start-4])/5.0;
        for(i=start;i<=end;i++)
        {
            Right_Line[i]=(int)(i-start)*k+Right_Line[start];
            if(Right_Line[i]>=MT9V03X_W-1) Right_Line[i]=MT9V03X_W-1;
            else if(Right_Line[i]<=0) Right_Line[i]=0;

            // 同步更新中线
            if(Mid_Line != NULL && Left_Line != NULL)
            {
                int mid_col = (Left_Line[i] + Right_Line[i]) / 2;
                mid_col = mid_col <0 ? 0 : (mid_col >= MT9V03X_W-1 ? MT9V03X_W-1 : mid_col);
                Mid_Line[i] = mid_col;
            }
        }
    }
}


/*-------------------------------------------------------------------------------------------------------------------
  @brief     画线
  @param     输入起始点，终点坐标，补一条宽度为2的黑线
  @return    null
  Sample     Draw_Line(0, 0,MT9V03X_W-1,MT9V03X_H-1);
             Draw_Line(MT9V03X_W-1, 0,0,MT9V03X_H-1);
                                    画一个大×
  @note     补的就是一条线，需要重新扫线
-------------------------------------------------------------------------------------------------------------------*/
void Draw_Line(int startX, int startY, int endX, int endY)
{
    int i,x,y;
    int start=0,end=0;
    if(startX>=MT9V03X_W-1)//限幅处理
        startX=MT9V03X_W-1;
    else if(startX<=0)
        startX=0;
    if(startY>=MT9V03X_H-1)
        startY=MT9V03X_H-1;
    else if(startY<=0)
        startY=0;
    if(endX>=MT9V03X_W-1)
        endX=MT9V03X_W-1;
    else if(endX<=0)
        endX=0;
    if(endY>=MT9V03X_H-1)
        endY=MT9V03X_H-1;
    else if(endY<=0)
        endY=0;
    if(startX==endX)//一条竖线
    {
        if (startY > endY)//互换
        {
            start=endY;
            end=startY;
        }
        for (i = start; i <= end; i++)
        {
            if(i<=1)
                i=1;
            image_two_value[i][startX]=IMG_BLACK;
            image_two_value[i-1][startX]=IMG_BLACK;
        }
    }
    else if(startY == endY)//补一条横线
    {
        if (startX > endX)//互换
        {
            start=endX;
            end=startX;
        }
        for (i = start; i <= end; i++)
        {
            if(startY<=1)
                startY=1;
            image_two_value[startY][i]=IMG_BLACK;
            image_two_value[startY-1][i]=IMG_BLACK;
        }
    }
    else //上面两个是水平，竖直特殊情况，下面是常见情况
    {
        if(startY>endY)//起始点矫正
        {
            start=endY;
            end=startY;
        }
        else
        {
            start=startY;
            end=endY;
        }
        for (i = start; i <= end; i++)//纵向补线，保证每一行都有黑点
        {
            x =(int)(startX+(endX-startX)*(i-startY)/(endY-startY));//两点式变形
            if(x>=MT9V03X_W-1)
                x=MT9V03X_W-1;
            else if (x<=1)
                x=1;
            image_two_value[i][x] = IMG_BLACK;
            image_two_value[i][x-1] = IMG_BLACK;
        }
        if(startX>endX)
        {
            start=endX;
            end=startX;
        }
        else
        {
            start=startX;
            end=endX;
        }
        for (i = start; i <= end; i++)//横向补线，保证每一列都有黑点
        {

            y =(int)(startY+(endY-startY)*(i-startX)/(endX-startX));//两点式变形
            if(y>=MT9V03X_H-1)
                y=MT9V03X_H-1;
            else if (y<=0)
                y=0;
            image_two_value[y][i] = IMG_BLACK;
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     斑马线检测
  @param     null
  @return    null
  Sample     Zebra_Stripes_Detect(void)
  @note      边界起始靠下，最长白列较长，赛道宽度过窄，且附近大量跳变
-------------------------------------------------------------------------------------------------------------------*/
void Zebra_Stripes_Detect(uint8_t* Right_Line,uint8_t* Left_Line)
{
    int i=0,j=0;
    int change_count=0;//跳变计数
    int start_line=0;
    int endl_ine=0;
    int narrow_road_count=0;
    if(Left_Island_Flag==1||Right_Island_Flag==1)//元素互斥，不是十字，不是，不是坡道，不是停车
    {
        return;
    }

       ////赛宽变化判斑马线
    if(Search_Stop_Line>=60&&
       30<=Longest_White_Column_Left[1]&&Longest_White_Column_Left[1]<=MT9V03X_W-30&&
       30<=Longest_White_Column_Right[1]&&Longest_White_Column_Right[1]<=MT9V03X_W-30&&
       Boundry_Start_Left>=MT9V03X_H-15&&Boundry_Start_Right>=MT9V03X_H-15)
    {//截止行长，.最长白列的位置在中心附近，边界起始点靠下
        for(i=110;i>55;i--)//在靠下的区域进行寻找赛道宽度过窄的地方
        {
            if((Standard_Road_Wide[i]-Road_Wide[i])>10)
            {
                narrow_road_count++;//多组赛宽变窄，才认为是斑马线
                if(narrow_road_count>=5)
                {
                    start_line=i;//记录赛道宽度很窄的位置
                    break;
                }
            }
        }
    }
//     tft180_show_int(0, 20,narrow_road_count, 3);
    if(start_line!=0)//多组赛宽变窄，，以赛道过窄的位置为中心，划定一个范围，进行跳变计数
    {
        start_line=start_line+8;
        endl_ine=start_line-15;
        if(start_line>=MT9V03X_H-1)//限幅保护，防止数组越界
        {
            start_line=MT9V03X_H-1;
        }
        if(endl_ine<=0)//限幅保护，防止数组越界
        {
            endl_ine=0;
        }
        for(i=start_line;i>=endl_ine;i--)//区域内跳变计数
        {
            for(j=Left_Line[i];j<=Right_Line[i];j++)
            {
                if(image_two_value[i][j+1]-image_two_value[i][j]!=0)
                {
                    change_count++;

                }
            }
        }
//        ips200_show_uint(0*16,100,change_count,5);//debug使用，查看跳变数，便于适应赛道
    }

 //    tft180_show_int(0, 40,change_count, 3);
if(change_count > 30)
{
    Zebra_LowFrameCount = 0;       // 因为检测到跳变，离开计数归零
    Zebra_HighFrameCount++;        // 连续高跳变帧 +1

    if(Zebra_HighFrameCount >= 10)     // ★★★ 连续10帧才成立 ★★★
    {
        //------ 进入斑马线逻辑 ------
        if(Zebra_Lock == 0)         // 本次斑马线首次触发
        {
            Zebra_Lock = 1;
            Zebra_Counter++;

            if(Zebra_Counter == 1)
            {
                Beep_Double(100);   // 第一次经过
                Zebra_Stripes_Flag = 1;
            }
            else if(Zebra_Counter == 2)
            {
                Beep_Tri(100);      // 第二次经过
                Zebra_Stripes_Flag = 2;
            }
        }
    }
}
else  // change_count <= 30
{
    // 统计 change_count 小于等于 30 的连续帧数量
    Zebra_LowFrameCount++;

    // 连续 100 帧都 <=30，才认为完全离开斑马线，允许下一次计数
    if(Zebra_LowFrameCount >= 20)
    {
        Zebra_LowFrameCount = 0;
 //        Zebra_Stripes_Flag = 0;
        Zebra_Lock = 0;   // 解锁
 //       Beep_Double(80);
    }
}

//tft180_show_int(0, 20,Zebra_Counter, 3);
//tft180_show_int(0, 60,Zebra_Lock, 3);
}
/*
void Debug_Draw60_80_and_PrintWidth(uint8_t *Left_Line,
                                    uint8_t *Right_Line,
                                    uint8_t image[MT9V03X_H][MT9V03X_W])
{

    //-------------------------------
    // 1. 打印 60~80 行宽度
    //-------------------------------
    int row = 60; // 从 60 行开始

    for(int line = 0; line < 5; line++) // 显示 5 行
    {
        int y = line * 20;  // 行高 = 20 像素（你要求的）

        for(int k = 0; k < 4; k++)       // 每行打印 4 个 width
        {
            int width = Right_Line[row] - Left_Line[row];
            int x = k * 30;              // 每个数字占 30px 宽度

            tft180_show_int(x, y, width, 3);

            row++;  // 下一行数据
        }
    }
    //-------------------------------
    // 2. 在第 60 行画横线
    //-------------------------------
    int y = 110;
    if(y < MT9V03X_H)
    {
        for(int x = 0; x < MT9V03X_W; x++)
        {
            image[y][x] = 128;   // 128 用作“红线标记”
        }
    }

    //-------------------------------
    // 3. 在第 80 行画横线
    //-------------------------------
    y = 55;
    if(y < MT9V03X_H)
    {
        for(int x = 0; x < MT9V03X_W; x++)
        {
            image[y][x] = 128;   // 同样标记为灰色作为“红线”
        }
    }
}
*/


/*-------------------------------------------------------------------------------------------------------------------
  @brief     找下面的两个拐点，供十字使用
  @param     搜索的范围起点，终点
  @return    修改两个全局变量
             Right_Down_Find=0;
             Left_Down_Find=0;
  Sample     Find_Down_Point(int start,int end)
  @note      运行完之后查看对应的变量，注意，没找到时对应变量将是0
-------------------------------------------------------------------------------------------------------------------*/

void Find_Down_Point(int start,int end,uint8_t* Right_Line,uint8_t* Left_Line)
{
    int i,t;
    Right_Down_Find=0;
    Left_Down_Find=0;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-1-5)//下面5行数据不稳定，不能作为边界点来判断，舍弃
        start=MT9V03X_H-1-5;
    if(end<=MT9V03X_H-Search_Stop_Line)
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)
       end=5;
    for(i=start;i>=end;i--)
    {
        if(Left_Down_Find==0&&//只找第一个符合条件的点
           abs(Left_Line[i]-Left_Line[i+1])<=5&&//角点的阈值可以更改
           abs(Left_Line[i+1]-Left_Line[i+2])<=5&&
           abs(Left_Line[i+2]-Left_Line[i+3])<=5&&
              (Left_Line[i]-Left_Line[i-2])>=8&&
              (Left_Line[i]-Left_Line[i-3])>=15&&
              (Left_Line[i]-Left_Line[i-4])>=15)
        {
            Left_Down_Find=i;//获取行数即可
        }
        if(Right_Down_Find==0&&//只找第一个符合条件的点
           abs(Right_Line[i]-Right_Line[i+1])<=5&&//角点的阈值可以更改
           abs(Right_Line[i+1]-Right_Line[i+2])<=5&&
           abs(Right_Line[i+2]-Right_Line[i+3])<=5&&
              (Right_Line[i]-Right_Line[i-2])<=-8&&
              (Right_Line[i]-Right_Line[i-3])<=-15&&
              (Right_Line[i]-Right_Line[i-4])<=-15)
        {
            Right_Down_Find=i;
        }
        if(Left_Down_Find!=0&&Right_Down_Find!=0)//两个找到就退出
        {
            break;
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     找上面的两个拐点，供十字使用
  @param     搜索的范围起点，终点
  @return    修改两个全局变量
             Left_Up_Find=0;
             Right_Up_Find=0;
  Sample     Find_Up_Point(int start,int end)
  @note      运行完之后查看对应的变量，注意，没找到时对应变量将是0
-------------------------------------------------------------------------------------------------------------------*/

void Find_Up_Point(int start,int end,uint8_t* Right_Line,uint8_t* Left_Line)
{
    int i,t;
    Left_Up_Find=0;
    Right_Up_Find=0;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(end<=MT9V03X_H-Search_Stop_Line)
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)//及时最长白列非常长，也要舍弃部分点，防止数组越界
        end=5;
    if(start>=MT9V03X_H-1-5)//下面5行数据不稳定，不能作为边界点来判断，舍弃
        start=MT9V03X_H-1-5;
    for(i=start;i>=end;i--)
    {
        if(Left_Up_Find==0&&//只找第一个符合条件的点
           abs(Left_Line[i]-Left_Line[i-1])<=5&&
           abs(Left_Line[i-1]-Left_Line[i-2])<=5&&
           abs(Left_Line[i-2]-Left_Line[i-3])<=5&&
              (Left_Line[i]-Left_Line[i+2])>=8&&
              (Left_Line[i]-Left_Line[i+3])>=15&&
              (Left_Line[i]-Left_Line[i+4])>=15)
        {
            Left_Up_Find=i;//获取行数即可
        }
        if(Right_Up_Find==0&&//只找第一个符合条件的点
           abs(Right_Line[i]-Right_Line[i-1])<=5&&//下面两行位置差不多
           abs(Right_Line[i-1]-Right_Line[i-2])<=5&&
           abs(Right_Line[i-2]-Right_Line[i-3])<=5&&
              (Right_Line[i]-Right_Line[i+2])<=-8&&
              (Right_Line[i]-Right_Line[i+3])<=-15&&
              (Right_Line[i]-Right_Line[i+4])<=-15)
        {
            Right_Up_Find=i;//获取行数即可
        }
        if(Left_Up_Find!=0&&Right_Up_Find!=0)//下面两个找到就出去
        {
            break;
        }
    }
    if(abs(Right_Up_Find-Left_Up_Find)>=30)//纵向撕裂过大，视为误判
    {
        Right_Up_Find=0;
        Left_Up_Find=0;
    }
}


/*-------------------------------------------------------------------------------------------------------------------
  @brief     十字检测
  @param     null
  @return    null
  Sample     Cross_Detect(void);
  @note      利用四个拐点判别函数，查找四个角点，根据找到拐点的个数决定是否补线
-------------------------------------------------------------------------------------------------------------------*/

        // ====== 静态变量 ======
static uint8 Cross_CoolDown       = 0;   // 20 帧禁止识别
static uint8 Cross_SecondTimeout  = 0;   // 第二次十字 60 帧超时计数
// ===== 十字计时输出 =====
int Cross_Frame_Interval = 0;   // 两次“十字进入确认”之间的帧数
volatile uint8  Cross_Frame_Ready    = 0;   // =1 表示 Cross_Frame_Interval 已更新，可在主循环显示

static uint8  cross_timer_stage = 0;        // 0=未计时  1=计时中
static uint32 cross_timer_cnt   = 0;        // 帧计数

void Cross_Detect(uint8_t* Right_Line,uint8_t* Left_Line,uint8_t* Mid_Line)
{
    int down_search_start=0;//下点搜索开始行
    Cross_Flag=0;
        Left_Up_Find=0;
        Right_Up_Find=0;
        if(Both_Lost_Time>=10)//十字必定有双边丢线，在有双边丢线的情况下再开始找角点
        {
            Find_Up_Point( MT9V03X_H-1, 0 ,Right_Line,Left_Line);
            if(Left_Up_Find==0&&Right_Up_Find==0)//只要没有同时找到两个上点，直接结束
            {
                return;
            }
        }
        uint8_t cross_detected = 0;
        if(Left_Up_Find!=0&&Right_Up_Find!=0)//找到两个上点，就找到十字了
        {
          cross_detected = 1;
            Cross_Flag=1;//对应标志位，便于各元素互斥掉
            down_search_start=Left_Up_Find>Right_Up_Find?Left_Up_Find:Right_Up_Find;//用两个上拐点坐标靠下者作为下点的搜索上限
            Find_Down_Point(MT9V03X_H-5,down_search_start+2,Right_Line,Left_Line);//在上拐点下2行作为下点的截止行
            if(Left_Down_Find<=Left_Up_Find)
            {
                Left_Down_Find=0;//下点不可能比上点还靠上
            }
            if(Right_Down_Find<=Right_Up_Find)
            {
                Right_Down_Find=0;//下点不可能比上点还靠上
            }
            if(Left_Down_Find!=0&&Right_Down_Find!=0)
            {//四个点都在，无脑连线，这种情况显然很少
                Left_Add_Line (Left_Line,Right_Line,Mid_Line,Left_Line [Left_Up_Find ],Left_Up_Find ,Left_Line [Left_Down_Find ] ,Left_Down_Find);
                Right_Add_Line(Right_Line,Left_Line,Mid_Line,Right_Line[Right_Up_Find],Right_Up_Find,Right_Line[Right_Down_Find],Right_Down_Find);
            }
            else if(Left_Down_Find==0&&Right_Down_Find!=0)//11//这里使用的都是斜率补线
            {//三个点                                     //01
                Lengthen_Left_Boundry(Left_Line,Right_Line,Mid_Line,Left_Up_Find-1,MT9V03X_H-1);
                Right_Add_Line(Right_Line,Left_Line,Mid_Line,Right_Line[Right_Up_Find],Right_Up_Find,Right_Line[Right_Down_Find],Right_Down_Find);
            }
            else if(Left_Down_Find!=0&&Right_Down_Find==0)//11
            {//三个点                                     //10
              Left_Add_Line (Left_Line,Right_Line,Mid_Line,Left_Line [Left_Up_Find ],Left_Up_Find ,Left_Line [Left_Down_Find ] ,Left_Down_Find);
                Lengthen_Right_Boundry(Left_Line,Right_Line,Mid_Line,Right_Up_Find-1,MT9V03X_H-1);
            }
            else if(Left_Down_Find==0&&Right_Down_Find==0)//11
            {//就俩上点                                   //00
      //          Lengthen_Left_Boundry (Left_Line,Left_Up_Find-1,MT9V03X_H-10);
      //          Lengthen_Right_Boundry(Right_Line,Right_Up_Find-1,MT9V03X_H-10);
              DrawCross( Mid_Line, Left_Up_Find, Right_Up_Find, 
             Left_Line, Right_Line);
            }
        }
        else
        {
            Cross_Flag=0;
        }

        // ================= 十字判决状态机 =================


// ====== 超时逻辑（优先级最高） ======
if (Cross_Time_Flag == 1)
{
    Cross_SecondTimeout++;

    if (Cross_SecondTimeout >= 80)
    {
        Cross_Time_Flag = 0;          // 强制回 0
        Cross_SecondTimeout = 0;      // 清零
        Cross_HighFrame = 0;
        Cross_CoolDown = 0;           // 可选：是否解除冷却（建议清）
    }
}
else
{
    Cross_SecondTimeout = 0;          // 还没第一次，不计时
}

// ====== 十字识别逻辑 ======
if (Cross_CoolDown > 0)
{
    Cross_CoolDown--;
    Cross_HighFrame = 0;
}
else
{
    if (cross_detected)
    {
        Cross_HighFrame++;

        if (Cross_HighFrame >= 3)     // 连续 2 帧确认
        {
            Cross_Time_Flag ^= 1;     // 0->1 或 1->0

            Cross_CoolDown = 20;      // 冷却 20 帧
            Cross_HighFrame = 0;

            // ★ 如果是第二次成功识别，取消超时 ★
            if (Cross_Time_Flag == 0)
            {
                Cross_SecondTimeout = 0;
            }
        }
    }
    else
    {
        Cross_HighFrame = 0;
    }
}
    }







void DrawCross(uint8_t* Mid_Line, int Left_Up_Find, int Right_Up_Find, 
            uint8_t* Left_Line, uint8_t* Right_Line)
{
    // ------------------------
    // 1. 获取两个上点的坐标
    // ------------------------
    int xL = Left_Line[Left_Up_Find];
    int yL = Left_Up_Find;

    int xR = Right_Line[Right_Up_Find];
    int yR = Right_Up_Find;

    // ------------------------
    // 2. 计算十字上两个点的中点
    // ------------------------
    int mid_top_x = (xL + xR) / 2;
    int mid_top_y = (yL < yR) ? yL : yR;   // 用更靠下（y 值更大）的作为起点避免穿洞

    // ------------------------
    // 3. 屏幕底部中点
    // ------------------------
    int mid_bottom_x = MT9V03X_W / 2;
    int mid_bottom_y = MT9V03X_H - 1;

    // ------------------------
    // 4. 计算斜率 k
    // ------------------------
    float denom = (float)(mid_bottom_y - mid_top_y);
    float k = 0.0f;
    if (denom != 0)
        k = (float)(mid_bottom_x - mid_top_x) / denom;

    // ------------------------
    // 5. 生成整条新中线
    // ------------------------
    for (int i = 0; i < MT9V03X_H; i++)
    {
        float x = mid_top_x + k * (i - mid_top_y);
        int xi = (int)x;

        if (xi < 0) xi = 0;
        if (xi >= MT9V03X_W) xi = MT9V03X_W - 1;

        Mid_Line[i] = xi;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     摄像头误差获取
  @param     null
  @return    获取到的误差
  Sample     err=Err_Sum();
  @note      加权取平均
-------------------------------------------------------------------------------------------------------------------*/
float Err_Sum(uint8_t* Right_Line,uint8_t* Left_Line)
{
    int i;
    float err=0;
    float weight_count=0;
    //常规误差
    for(i=MT9V03X_H-1;i>=MT9V03X_H-Search_Stop_Line-1;i--)//常规误差计算
    {
        err+=(MT9V03X_W/2-((Left_Line[i]+Right_Line[i])>>1))*Weight[i];//右移1位，等效除2
        weight_count+=Weight[i];
    }
    err=err/weight_count;
    
    return err;
}
