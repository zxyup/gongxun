/*@filename: Laser_Positioning.h
 *@author     ���˹�       
 *@brief:     ���ⶨλ
 *@date: 2023-7-27
*/

#ifndef _LASER_POSITION_H_
#define _LASER_POSITION_H_

#include "rcs.h"

#define LASER_EQUIVALENT                                0.8544921875		//(3.6m)         0.2197265625f(1m)		// ���̲�/4096.0
#define LASER_MIN_DIS																		100.0f					//��С�궨����			
#define LASER_MAX_DIS																		10000.0f				//���궨����

#define LASER3X4																				160.0f					//����3������4�İ�װ�ĺ������
#define MIDPOSX																					5500.0f					//�г�X����

#define BUFFERLEN																				5							//�˲����г���
#define BUFFERLEN_VISION																20						//�Ӿ������˲�����
#define TIMES                                           100            //һ����λ���˲��������

void Laser_Init(void);																									//�����ʼ��
void Laser_Task(void);
float Get_Laser_GPS_X(void);
float Get_Laser_GPS_Y(void);
float Get_Laser_GPS_Z(void);
float Get_Laser_Distance_X(void);
float Get_Laser_Distance_Y(void);
float Get_Laser_Z(void);																								//��ȡ����ƫ����
void Laser_Get_Pos(float *now_pos_x,float *now_pos_y,float now_pos_z);	//���ⶨλ
#endif
