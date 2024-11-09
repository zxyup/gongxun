/*@filename: Joystick_Ctrl.h
 *@author 胡兴�?
 *@brief:  手柄控制
 *@date:2023-7-27
*/
#ifndef _JOYSTICK_CTRL_H_
#define _JOYSTICK_CTRL_H_

#include "rcs.h"

#define BLUETOOTH_BAUD 9600
   
#define INIT_KEY                128             //手柄初始返回128
#define KEY_ON                  1               //按键按下
#define KEY_OFF                 0               //按键松开
#define AMPLIFY                 10              //放大信号
//无限手柄按键对照�?
#define UP_KEY									0x0001					//方向�?
#define DOWN_KEY								0x0002
#define LEFT_KEY								0x0004
#define RIGHT_KEY								0x0008
#define KEY1									0x0010					//普通按�?
#define KEY2									0x0020
#define KEY3									0x0040
#define KEY4									0x0080
#define KEY5									0x0100
#define KEY6									0x0200
#define LSWITCH_KEY							    0x0400					//左右自锁开�?
#define RSWITCH_KEY							    0x0800	
#define LSTICK_KEY							    0x1000					//左右遥感按键
#define RSTICK_KEY							    0x2000



void BlueTooth_Init(RCS_PIN_USART USARTx_MAP);
void Joystick_Ctrl(void);    //简单遥控控制by小胡
float Get_Current2(void);
void Joystick_Absolute_Ctrl(void);
void Joystick_Helm_Ctrl(void); //舵轮遥控
void Joystick_Direction_Ctrl(void);
void Joystick_Rocker_Ctrl(void);
void Joystick_Direction_Ctrl_Slow(void);

extern int key_flag[10];       //ʮ�����ܰ��������ߣ����������ܰ��������ߣ�
extern int stop_key;             //��ͣ�� �����ߴ��ɫ�����������ҿ��أ����ߣ�
extern int switch_key;            //�Զ�/�ֶ���ת���������ߴ��ɫ�ڶ��������󿪹أ����ߣ�
extern int stick_key[2];							//ң�а�����0�ң�1�����ߣ�
extern int direction_key[4];    //���򰴼�,0�ϣ�1�£�2��3��
extern int rocker_lx,rocker_ly;  //��ҡ�˿����˶�
extern int rocker_rx,rocker_ry;  //��ң�п�����ת

#endif
