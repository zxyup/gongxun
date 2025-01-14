/*@filename: Joystick_Ctrl.h
 *@author 鑳″叴鍥?
 *@brief:  鎵嬫焺鎺у埗
 *@date:2023-7-27
*/
#ifndef _JOYSTICK_CTRL_H_
#define _JOYSTICK_CTRL_H_

#include "rcs.h"

#define BLUETOOTH_BAUD 9600
   
#define INIT_KEY                128             //鎵嬫焺鍒濆杩斿洖128
#define KEY_ON                  1               //鎸夐敭鎸変笅
#define KEY_OFF                 0               //鎸夐敭鏉惧紑
#define AMPLIFY                 10              //鏀惧ぇ淇″彿
//鏃犻檺鎵嬫焺鎸夐敭瀵圭収琛?
#define UP_KEY									0x0001					//鏂瑰悜閿?
#define DOWN_KEY								0x0002
#define LEFT_KEY								0x0004
#define RIGHT_KEY								0x0008
#define KEY1									0x0010					//鏅�氭寜閿?
#define KEY2									0x0020
#define KEY3									0x0040
#define KEY4									0x0080
#define KEY5									0x0100
#define KEY6									0x0200
#define LSWITCH_KEY							    0x0400					//宸﹀彸鑷攣寮�鍏?
#define RSWITCH_KEY							    0x0800	
#define LSTICK_KEY							    0x1000					//宸﹀彸閬ユ劅鎸夐敭
#define RSTICK_KEY							    0x2000



void BlueTooth_Init(RCS_PIN_USART USARTx_MAP);
void Joystick_Ctrl(void);    //绠�鍗曢仴鎺ф帶鍒禸y灏忚儭
float Get_Current2(void);
void Joystick_Absolute_Ctrl(void);
void Joystick_Helm_Ctrl(void); //鑸佃疆閬ユ帶
void Joystick_Direction_Ctrl(void);
void Joystick_Rocker_Ctrl(void);
void Joystick_Direction_Ctrl_Slow(void);

extern int key_flag[10];       //十个功能按键（有线），六个功能按键（无线）
extern int stop_key;             //急停键 （有线大红色第三个），右开关（无线）
extern int switch_key;            //自动/手动挡转换键（有线大红色第二个），左开关（无线）
extern int stick_key[2];							//遥感按键（0右，1左无线）
extern int direction_key[4];    //方向按键,0上，1下，2左，3右
extern int rocker_lx,rocker_ly;  //左摇杆控制运动
extern int rocker_rx,rocker_ry;  //右遥感控制旋转

#endif
