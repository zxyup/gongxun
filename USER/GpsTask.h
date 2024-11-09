/*@filename: GPS.h
 *@author     ���˹�      
 *@brief:    ȫ����λ���������ư�
 *@date: 2023-7-27
*/
/* --------.h head------------------------------*/
#ifndef _GPS_H_
#define _GPS_H_

/* --------�����ļ�------------------------------*/
#include "rcs.h"	
#include "GPS_DSP.h"
/* --------���ú�--------------------------------*/
//#define RCS16_GPS
#define RCS13_GPS

/* --------˽�к�------------------------------*/
#ifdef RCS16_GPS


#endif













#ifdef RCS13_GPS
#define ENCODER_X_DIR	 1					//��������������
#define ENCODER_Y_DIR	 -1			

#define X_ROTATE_ARM		61.3			//����X����ת�۳�mm
#define Y_ROTATE_ARM		61.3			//����Y����ת�۳�mm

#define	CYCLE_LINES			1024			//c,����������(!!!!!!!!!!���Ĺ���ԭ����1024)
#define	FREQUENCY_DIV		4					//q,��Ƶ��
#define	GPS_WHEEL_RADIUS	25.4f		//R,���Ӱ뾶mm
#define TRANSFER_CONST		0.038963112f//(1024)				//C,������λ��/mmת������,C=2*pi*R/(q*c)

#define TRANSFORM_DZ   0.785398163f//0.785398163f//-2.35619449019f//-3.926990816987f          //���̶�λ�Ƕ�ת��

#define TRANSFORM_DX	  0.0f                 //��λ�����복���ĵ�X�������
#define TRANSFORM_DY		0.0f                 //��λ�����복���ĵ�Y�������

#define ANGLE_DX	  2.35619449019f            //���1��2����
#define ANGLE_DY    2.35619449019f         
#define TRANSFORM_AX	  0//4050.0f                 //���1��2����
#define TRANSFORM_AY	  3600.0f//3600.0f
#define TRANSFORM_X	       0.68288f             //x�������궨����ֵ
#define TRANSFORM_Y	       0.68288f             //y�������궨����ֵ

void GPS_Init(void);
int32_t gps_cycle_test_x(void);
int32_t gps_cycle_test_y(void);
float Get_GPS_X(void);
float Get_GPS_Y(void);
float Get_GPS_Z(void);
float Get_GPS_Ori_X(void);
float Get_GPS_Ori_Y(void);
float Get_GPS_Raw_X(void);
float Get_GPS_Raw_Y(void);
void X_Cycle(uint8_t flag);
void Y_Cycle(uint8_t flag);

void Set_GPS_X(float laser_x);
void Set_GPS_Y(float laser_y);


#endif


#endif