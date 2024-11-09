/*@filename: RCS_BaseMove.h
 *@author     ���˹�       
 *@brief:     �����ٶȽ���
 *@date: 2023-7-28
*/
#ifndef _RCS_BASEMOVE_H_
#define _RCS_BASEMOVE_H_

#include "rcs.h"		
/***********���̹��(mm)************/
#define BASE_LENGTH     350.0f  //���̰볤
#define BASE_WIDTH			350.0f	//���
#define RAD_WHEEL				76.2f	//���Ӱ뾶
#define SLOWDOWN_RATE	  19.0f	  //������ٱ�
#define SLOWDOWN_RATE_2006  36.0f //2006������ٱ�
#define MM_S2R_MIN			0.1772276212907689f	// ������ٶ�(mm/s)ת��������ת��(r/min)�����㷽����60/(2*RAD_WHEEL*pi*cos(45��))
#define MAX_MOTOR_SPEED 9000.0f	//3508������ת��
#define MAX_2006_SPEED  20000.0f	//2006������ת��

void M_BaseMove_Init(void);			//���̳�ʼ��
void BaseMove_P2P(float move_speed_x,float move_speed_y,float rotate_speed);	
void BaseMove_Polar(float direction_angle,float run_speed,float rotate_speed);
void BaseMove_Helm_Around(float Speed1,float Speed2,float Speed3);
void BaseMove_Vision(float move_speed,float rotate_speed);																		
#endif
