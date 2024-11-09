/**
 * @filename:RCS_Ctrl_BaseMove
 * @brief:�����ۺϿ���
 * @changlog: 2023-7-28 ���˹� �޸�����
 * @changlog: 2024-2-20 ���Ͽ� ������ֵ���
*/

/* --------.h header--------------------------------------------------*/
#ifndef _RCS_CTRL_BASEMOVE_H_
#define _RCS_CTRL_BASEMOVE_H_

/* --------ͷ�ļ�-----------------------------------------------------*/
#include "RCS_PIDctrl.h"
#include "RCS_MOTOR.h"	

/* --------������غ�--------------------------------------------------*/

//��������
//#define USE_OMNI_CHASSIS    //ʹ��ȫ���ֵ���,ȫ���ֵ��̲�����RCS_BaseMove.h
#define USE_AGV_CHASSIS     //ʹ�ö��ֵ���,���ֵ��̲�����RCS_AGV_BaseMove.h

//�Զ����Ʋ���
#define POINT_NUM       200		//���������ߵ���
#define LEAD_POINT_NUM  4.0f	//����������ǣ���㳬ǰ��
#define SLOW_POINT      150.0f	//���ٵ��ٽ��

#define BIG_ANGLE           5.0  //��Ƕ�
#define START_SPEED         500  //�����ٶ�
#define GET_HELM_ANGLE      0.2f //���ֽǶȵ����ж�


/* ---------�������ݽṹ-----------------------------------------------*/
typedef struct Point{
	float x;
	float y;
	float z;
}Coord_Point;//�����ṹ��

typedef struct Param{
	float ACCE;//���ٶ�
	float DECE;//���ٶ�
	float L_ANGLE;//�Ƕȵ����ݲ�_��
	float R_ANGLE;//�Ƕȵ����ݲ�_��
	float run_speed;//�ܵ��ٶ�
	PID_Struct rotate_angle_pid;
	
}Trace_Param;//���˹��ܵ��㷨����

typedef struct Speed_Vector{
	float value;									
	float direction;							
}Route_BridgeSpeed;


typedef struct{
	struct Speed_Vector bridge;			//�Ž��ٶ�
	struct Speed_Vector speed[POINT_NUM];					//��������·���е��г��ٶ�
	struct Point ctrl_point[2];			//0Ϊ���㡢1ΪԶ��			//���ױ�����������Ҫ�������Ƶ�
	struct Point point[POINT_NUM];	//·�ߵ�
	char  direction;								//������Ϊ'x'��'y'(������ǣ�����ֵ��45��~135��֮���Ϊ'x',����Ϊ'y')
	uint8_t begin_speed_ctrl;				//�������Ƿ���Ҫ���٣�1�ǣ�0����
	uint8_t end_speed_ctrl;					//�����յ��Ƿ���Ҫ���٣�1�ǣ�0����
}Route_Point;//�����������ϸ�����̬�ṹ��


/* ---------��������------------------------------------------------------*/

//��ʼ��
void Ctrl_BaseMove_Init(void); 

//�����ٶȿ���
void Chassis_Move(float spd_x,float spd_y,float spd_z);//ֱ�ӿ��Ƶ����ٶ�
void BaseMove_Joystick(void);                          //ң�ؿ��Ƶ����ٶ�

//���̾������
void Point2Point_ER(Coord_Point* cp,Trace_Param* tp,int *ER_flag);
void Point2Point(float target_x,float target_y,float target_z,PID_Struct distance_x_pid,PID_Struct distance_y_pid,PID_Struct rotate_angle_pid);
void Bessel_P2P(Route_Point *rpoint,int *index,int AUTO_SPEED,float target_z,PID_Struct distance_x_pid,PID_Struct distance_y_pid,PID_Struct rotate_angle_pid,PID_Struct angle_pid);

//����λ���ж�
int Judge_Pos(float target_x,float target_y,float target_z);		//����ﵽ�ж�
int Judge_Pos_B(int index);																			//������ߵ�λ�����ж�

#endif