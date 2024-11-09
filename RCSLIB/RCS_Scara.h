/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _RCS_SCARA_H_
#define _RCS_SCARA_H_

/* Includes ------------------------------------------------------------------*/
#include "RCS_MOTOR.h"

/* Scara Configuration -------------------------------------------------------*/
typedef double SCARA_FLOAT_T;      //ʹ��˫���ȸ������(15λ��Ч����)
#define SCARA_COS cos              //��Ӧ���ȵ���ѧ����
#define SCARA_SIN sin

#define SCARA_PINV_COMMON 0.001

/* Exported Define & types----------------------------------------------------*/
typedef struct scara_angle{
	SCARA_FLOAT_T joint_angle_main;
	SCARA_FLOAT_T joint_angle_end;
}SCARA_AXIS;//�ؽڽǶ�����

typedef struct descartes{
	SCARA_FLOAT_T x;
	SCARA_FLOAT_T y;
}DESCARTES_AXIS;//�ѿ�����������

typedef struct scara_param{
	SCARA_FLOAT_T Main_Len;   //��۳���
	SCARA_FLOAT_T End_Len;    //С�۳���
	SCARA_FLOAT_T Pinv_Epsilon;//�����������ϵ��
	
	SCARA_FLOAT_T Main_SlDn;  //��ۼ��ٱ�(��۽Ƕ�/��������Ƕ�)(must >0)
	SCARA_FLOAT_T End_SlDn;   //С�ۼ��ٱ�(С�۽Ƕ�/��������Ƕ�)(must >0)
	
	SCARA_AXIS    Start_Pos;  //rad
}SCARA_PARAM;//��е�۲���

/* Exported functions --------------------------------------------------------*/

SCARA_PARAM Scara_Param_Init(SCARA_FLOAT_T Main_Len,         SCARA_FLOAT_T End_Len,
                             SCARA_FLOAT_T Main_Slowdown,    SCARA_FLOAT_T End_Slowdown,
														 SCARA_FLOAT_T Main_StartPos_Deg,SCARA_FLOAT_T End_StartPos_Deg);

SCARA_AXIS Scara_Spd_Ctrl(SCARA_FLOAT_T spd_x,            SCARA_FLOAT_T spd_y,
                          SCARA_FLOAT_T Main_MotorPos_Rad,SCARA_FLOAT_T End_MotorPos_Rad,
                          SCARA_PARAM* scara);

SCARA_AXIS Scara_Pos_Ctrl(SCARA_FLOAT_T desc_pos_x,SCARA_FLOAT_T desc_pos_y,
                          SCARA_FLOAT_T Main_MotorPos_Rad,SCARA_FLOAT_T End_MotorPos_Rad,
                          SCARA_PARAM* scara);

SCARA_AXIS Scara_Pos_Ctrl_Direct(SCARA_FLOAT_T Main_Pos_Deg,SCARA_FLOAT_T End_Pos_Deg,SCARA_PARAM* scara);

SCARA_AXIS Scara_Get_Scara_Pos(SCARA_FLOAT_T Main_MotorPos_Rad,SCARA_FLOAT_T End_MotorPos_Rad,SCARA_PARAM* scara);
DESCARTES_AXIS Scara_Get_Desc_Pos(SCARA_FLOAT_T Main_MotorPos_Rad,SCARA_FLOAT_T End_MotorPos_Rad,SCARA_PARAM* scara);

//�����˶�ѧ
DESCARTES_AXIS Scara_Kin(SCARA_AXIS* jonit_vector,SCARA_PARAM* scara);
SCARA_AXIS Scara_InvKin(DESCARTES_AXIS* target_vector,SCARA_AXIS* now_axis,SCARA_PARAM* scara);
//����΢���˶�ѧ
DESCARTES_AXIS Scara_Jcb_Kin(SCARA_AXIS* jonit_vector,SCARA_AXIS* now_axis,SCARA_PARAM* scara);
SCARA_AXIS Scara_Jcb_InvKin(DESCARTES_AXIS* target_speed,SCARA_AXIS* now_axis,SCARA_PARAM* scara);
SCARA_AXIS Scara_Jcb_PInvKin(DESCARTES_AXIS* target_speed,SCARA_AXIS* now_axis,SCARA_PARAM* scara);
//���ٱȻ���
SCARA_AXIS Scara_Trans_Motor_Pos(SCARA_AXIS* jonit_vector,SCARA_PARAM* scara);
SCARA_AXIS Motor_Trans_Scara_Pos(SCARA_AXIS* motor_vector,SCARA_PARAM* scara);
SCARA_AXIS Scara_Trans_Motor_Spd(SCARA_AXIS* jonit_vector,SCARA_PARAM* scara);
SCARA_AXIS Motor_Trans_Scara_Spd(SCARA_AXIS* motor_vector,SCARA_PARAM* scara);
//�����λ�ж�
SCARA_FLOAT_T Judge_Scara_Jcb_InvKin(DESCARTES_AXIS* target_speed,SCARA_AXIS* now_axis,SCARA_PARAM* scara);
uint8_t Judge_Scara_InvKin(DESCARTES_AXIS* target_pos,SCARA_AXIS* now_axis,SCARA_PARAM* scara);



#endif
