//@filename:Vision.h
//@author:  陈志伟
//@date:    2021-1-10
//@brief:   视觉通信

#ifndef _VISION_H_
#define _VISION_H_

#include "rcs.h"

#define VISION_BAUD				9600  //115200
void Vision_Init(RCS_PIN_USART Pick_USARTx_MAP,RCS_PIN_USART Cup_USARTx_MAP,RCS_PIN_USART Dist_USARTx_MAP);

void Vision_Front_Init(RCS_PIN_USART USARTx_MAP);//车前摄像头与视觉通信的串口初始化
void Vision_Back_Init(RCS_PIN_USART USARTx_MAP);//车后摄像头与视觉通信的串口初始化


#define VISION_UART_START_CHAR 0x2B
#define VISION_UART_END_CHAR   0x5B
#define VISION_PACAGE_LEN      3

#define VISION_UART_START_CHAR_FRONT 0x2B
#define VISION_UART_END_CHAR_FRONT   0x5B
#define VISION_PACAGE_LEN_FRONT      6

#define VISION_UART_START_CHAR_BACK 0x2B
#define VISION_UART_END_CHAR_BACK   0x5B
#define VISION_PACAGE_LEN_BACK      3

#define BALL_NONE 0
#define BALL_BLUE 1
#define BALL_RED  2

typedef struct scattered_pos{
	int scattered_x;
	int scattered_y;
}SCATTERED_POS;//散球坐标

typedef struct scara_angle SCARA_AXIS;
typedef struct descartes DESCARTES_AXIS;
int Vision_Get_Ball_Count(void);
DESCARTES_AXIS Vision_Get_Ball_Pos(void);
DESCARTES_AXIS vision_judge(int side_color);
int Vision_Get_Basket_Num(void);
int Vision_Get_Basket_Ball(void);
SCATTERED_POS Vision_Get_Scattered_Ball_Pos(void);
int Vision_Get_Scattered_Ball_Or_Not(void);

int Vision_Cup_Get_Ball_Pos_X(void);
int Vision_Cup_Get_Ball_Pos_Y(void);
int Vision_Cup_Get_Ball_Dist(void);

#endif //_VISION_H_

