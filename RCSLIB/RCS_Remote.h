/*@filename: RCS_Remoute.h
 *@author     ��־ΰ       
 *@brief:     RM�ֱ����ܶ�����
 *@date: 2020-7-17
*/
#ifndef _RCS_REMOTE_H_
#define _RCS_REMOTE_H_

#include "rcs.h"	
/* ----------------------- ң��ʾ��ͼ---------------------------- 
					!S1															      					!S2
								 ��ch3										    	��ch1
								 |		ch2								  			|		ch0
						 ��--+--��										  ��--+--��
								 |															|
                 ��   													��

-----------------------------------------------------------------*/

/* -------------------------�ֱ�����---------------------------------*/
#define JOYSTICK_USART      USART3
#define JOYSTICK_GPIO       GPIOD
#define JOYSTICK_TXD        GPIO_Pin_5
#define JOYSTICK_RXD        GPIO_Pin_6
#define JOYSTICK_BAUD       115200
/* ----------------------- ң��ģ��ҡ��ֵ---------------------------- */
#define RC_CH_VALUE_MIN ((uint16_t)364 )				
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)
/* ----------------------- ң�ز�������ֵ----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
/* ----------------------- ���Լ������ֵ-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)0x01<<0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)0x01<<1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)0x01<<2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)0x01<<3)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)0x01<<4)
#define KEY_PRESSED_OFFSET_E ((uint16_t)0x01<<5)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)0x01<<6)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)0x01<<7)
#define RC_FRAME_LENGTH 18u
/* ----------------------- �������ܰ��ṹ�� ------------------------------------- */
typedef struct
 {
	uint16_t ch0;			//right Joystick left-right
	uint16_t ch1;			//right Joystick ahead-back
	uint16_t ch2;			//left Joystick left-right
	uint16_t ch3;			//left Joystick ahead-back
	uint8_t s1;				//left switch
	uint8_t s2;				//right switch
 }rc_ctrl;							//ң��
typedef struct
 {
	int16_t x;				//mouse left-right
	int16_t y;				//mouse ahead-back
	int16_t z;				//
	uint8_t press_l;	//mouse left button
	uint8_t press_r;	//mouse right button
 }mouse_ctrl;					//���
typedef struct
 {
	uint16_t v;				//keyboard value
 }key_ctrl;						//����
 
 void Get_RC_Value(rc_ctrl *rc_value);				//��ȡң��ֵ
 void Get_Key_Value(key_ctrl *key_value);			//��ȡ����ֵ
 void Get_Mouse_Value(mouse_ctrl *mouse_value);//��ȡ���ֵ
 void RC_Init(void);													//��ʼ��ң�ؽ�����
 
#endif
