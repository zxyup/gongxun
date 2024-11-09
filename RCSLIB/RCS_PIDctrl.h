/*@filename: RCS_PIDctrl.h
 *@author     ���˹�     
 *@brief:     PID�㷨
 *@date: 2023-7-27
*/
#ifndef _RCS_PIDCTRL_H_
#define _RCS_PIDCTRL_H_

#include <stdint.h>
//#include "rcs.h" ����Ҫ��ͷ�ļ����õĻ�������include��ͷ�ļ�		 



#define LIMIT_ERR						10000				//��������޷�
#define	LIMIT_SPEED						16000				//��������޷�

//����PID�ṹ��
typedef struct {
float P;
float I;
float D;
float Last_Error;			//�ϴ����
float Limit_Output;  	//�������
float Limit_Integral; //����������
float Integral; 			//������,�洢�������	
}PID_Struct;


typedef struct{
    float Cur_Spd;
    float Cur_Dis;
    float Target_Spd;
    float Target_Dis;
    PID_Struct Spd_Pid;
    PID_Struct Dis_Pid;
}PID_F32_CTRL_Struct;


void PID_Init(void);		//PID��ʼ��
PID_Struct PID_Get_RM3508_Speed_Pid(void);
PID_Struct PID_Get_RM3508_Angle_Pid(void);
PID_Struct PID_Get_RM2006_Speed_Pid(void);
PID_Struct Get_Valve_Speed_Pid(void);
PID_Struct Get_Valve_Angle_Pid(void);

//����PID
float PID_Normal_Ctrl(float target,float current,PID_Struct *_pid);
float PID_Angle_Ctrl(float target,float current,PID_Struct *_pid);
int32_t PID_Motor_Ctrl(float target,float current,PID_Struct *_pid);
float DoubleRing_Float_Ctrl(float target_angle,float now_angle,float now_speed,PID_Struct* DR_Angle_pid,PID_Struct* DR_Speed_pid);


#endif