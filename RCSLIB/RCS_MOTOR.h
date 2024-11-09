/*@filename: RCS_MOTOR.h
 *@author     ���˹������Ͽ�       
 *@brief:     CAN����������
 *@date: 2023-7-28
 *@data: 2023-11-15
*/
/* ----------.h head-----------------------------------*/
#ifndef _RCS_MOTOR_H_
#define _RCS_MOTOR_H_

/* -----------ͷ�ļ�------------------------------------*/
//C���Թ�����
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

//BSP���м��
#include "matrix_functions.h" //ARM�ṩ�ľ��������(Ӳ)
#include "stm32f4xx.h"        //STM32�Ĵ�������
#include "stm32f4xx_conf.h"   //STM32��׼��ͷ�ļ�
#include "bsp.h"              //uCOS�ײ����
#include "includes.h"         //uCOS�ر����
#include "delay.h"            //��ʱ����
#include "matrix.h"           //Matrix_Hub���������(��)

//RCSLIBӲ�������
#include "RCS_HAL.h"           //�Ĵ�������
#include "RCSLIB_inc.h"        //����Ӳ������

//RCSLIBģ��������(�㷨��)
#include "RCS_PIDctrl.h"       //PID�㷨
#include "RCS_DataStructure.h" //�������ݽṹ

//RCSLIBģ��������(CAN���繦��)
#include "RCS_CAN_Interface.h" //CAN���߽����ӿ�		

/* ----------���ú�-------------------------------------*/
#define RCS_MOTOR_DEBUG        //���õ������
#define MAX_RMESC_ON_CAN  13   //RM����ش����Ĳ��ᳬ��12��
#define BATTERY_VOLTAGE   24

#define M2006_SLD_RATE    36.0f
#define M3508_SLD_RATE    19.0f

/* ------------˽�к�--------------------------------------*/
#define ENCODER2ANGLE	  0.0439453125 //360/8192

typedef enum {
	RM_FRONT=0x200,          //ID1~4 ����
	RM_BEHIND=0x1FF,         //ID5~8 ����
	RM_GM6020_V_FRONT=0x1FF, //ID1~4 (6020��ѹ)
	RM_GM6020_V_BEHIND=0x2FF,//ID5~7 (6020��ѹ)
	RM_GM6020_FRONT=0x1FE,   //ID1~4 (6020����)
	RM_GM6020_BEHIND=0x2FE,  //ID5~8 (6020����)
} RM_PACKET_ID;              //RM���CAN���ݰ�ID

/* -----------��������(�½ӿ�)-------------------------------------*/
//��ʼ��
void RMMotor_CAN_Init(CAN_TypeDef* CANx);

//�������ֵ
void RM3508_Update_Current(CAN_TypeDef* CANx,uint8_t id,int16_t current);
void GM6020_Update_Current(CAN_TypeDef* CANx,uint8_t id,int16_t current);
void GM6020_Update_Voltage(CAN_TypeDef* CANx,uint8_t id,int16_t volt);

//ִ�����ֵ
void RM3508_Excute_Front_Current(CAN_TypeDef* CANx);
void RM3508_Excute_Behind_Current(CAN_TypeDef* CANx);
void GM6020_Excute_Front_Current(CAN_TypeDef* CANx);
void GM6020_Excute_Behind_Current(CAN_TypeDef* CANx);
void GM6020_Excute_Front_Voltage(CAN_TypeDef* CANx);
void GM6020_Excute_Behind_Voltage(CAN_TypeDef* CANx);

//��ȡ����״̬(ĩ��ת��)
float RM3508_Get_Angle_Deg(CAN_TypeDef* CANx,uint8_t id);
float GM6020_Get_Angle_Deg(CAN_TypeDef* CANx,uint8_t id);
int16_t RM3508_Get_Speed_Rpm(CAN_TypeDef* CANx,uint8_t id);
int16_t GM6020_Get_Speed_Rpm(CAN_TypeDef* CANx,uint8_t id);

//��ȡ����״̬(�����)
float RM3508_Get_Angle_Deg_Sld(CAN_TypeDef* CANx,uint8_t id);//�Ƕ������
float RM2006_Get_Angle_Deg_Sld(CAN_TypeDef* CANx,uint8_t id);
float RM3508_Get_Angle_Rad_Sld(CAN_TypeDef* CANx,uint8_t id);//���������
float RM2006_Get_Angle_Rad_Sld(CAN_TypeDef* CANx,uint8_t id);
float RM3508_Get_Speed_Rpm_Sld(CAN_TypeDef* CANx,uint8_t id);//ת�������
float RM2006_Get_Speed_Rpm_Sld(CAN_TypeDef* CANx,uint8_t id);

/* -----------��������(�Ͻӿ�)-------------------------------------*/

//���������ʼ��
void Motor_Init(void);		
void Motor_Init2(void);		

//RM�����������
void Motor_Send2(int16_t ID1_Current, int16_t ID2_Current, int16_t ID3_Current, int16_t ID4_Current);	
void Motor_Send(int16_t ID1_Current, int16_t ID2_Current, int16_t ID3_Current, int16_t ID4_Current);
void Motor_Send_ADD(int16_t ID5_Current, int16_t ID6_Current, int16_t ID7_Current, int16_t ID8_Current);	
void Motor_Send2_ADD(int16_t ID5_Current, int16_t ID6_Current, int16_t ID7_Current, int16_t ID8_Current);
void GM6020_Send(int16_t ID1_Current,int16_t ID2_Current,int16_t ID3_Current,int16_t ID4_Current);
void GM6020_Send_ADD(int16_t ID5_Current,int16_t ID6_Current,int16_t ID7_Current);
void GM6020_Send2(int16_t ID1_Current,int16_t ID2_Current,int16_t ID3_Current,int16_t ID4_Current);
void GM6020_Send2_ADD(int16_t ID5_Current,int16_t ID6_Current,int16_t ID7_Current);
void Motor_Send_6020(int16_t ID1_Voltage, int16_t ID2_Voltage, int16_t ID3_Voltage, int16_t ID4_Voltage);
void Motor_Send_6020_ADD(int16_t ID5_Voltage, int16_t ID6_Voltage, int16_t ID7_Voltage);
void Motor_Send2_6020(int16_t ID1_Voltage, int16_t ID2_Voltage, int16_t ID3_Voltage, int16_t ID4_Voltage);
void Motor_Send2_6020_ADD(int16_t ID5_Voltage, int16_t ID6_Voltage, int16_t ID7_Voltage);

//RM�����ȡ����״̬
float 	Get_Motor_Float_Angle(uint8_t Motor_ID);//CAN1��������е�Ƕ�,��λdeg
int32_t Get_Motor_Integer_Angle(uint8_t Motor_ID);//����,��λdeg
int16_t Get_Motor_Speed(uint8_t Motor_ID);//CAN1���ת��,��λRPM
float 	Get_Motor_Float_Angle2(uint8_t Motor_ID);//CAN2�����е�Ƕȣ���λdeg
int32_t Get_Motor_Integer_Angle2(uint8_t Motor_ID);
int16_t Get_Motor_Speed2(uint8_t Motor_ID);	//CAN2���ת��,��λRPM
int16_t Get_Motor_Power(uint8_t Motor_ID);
int16_t Get_Motor_Power2(uint8_t Motor_ID);
float Get_GM6020_Float_Angle(uint8_t Motor_ID);
float Get_GM6020_Float_Angle2(uint8_t Motor_ID);
int16_t Get_GM6020_Speed(uint8_t Motor_ID);
int16_t Get_GM6020_Speed2(uint8_t Motor_ID);
float Get_Motor_Rad_Angle_M3508(uint8_t Motor_ID);
float Get_Motor_Rad_Angle_M2006(uint8_t Motor_ID);
float Get_Motor_Rad_Angle2_M3508(uint8_t Motor_ID);
float Get_Motor_Rad_Angle2_M2006(uint8_t Motor_ID);

//����ӿ�
void RM_Motor_Data_Filter1(RCS_CAN2B_DATA_FM_RX rx_message);
void RM_Motor_Data_Filter2(RCS_CAN2B_DATA_FM_RX rx_message);

#endif



//#define _RCS_MOTOR_H_

//#include "rcs.h"		

//void Motor_Init(void);		//�����ʼ��
//void Motor_Init2(void);			
//void Motor_Send2(int16_t ID1_Current, int16_t ID2_Current, int16_t ID3_Current, int16_t ID4_Current);	//��3508��������
//void Motor_Send(int16_t ID1_Current, int16_t ID2_Current, int16_t ID3_Current, int16_t ID4_Current);
//void Motor_Send_ADD(int16_t ID5_Current, int16_t ID6_Current, int16_t ID7_Current, int16_t ID8_Current);	//��2006��������
//void Motor_Send2_ADD(int16_t ID5_Current, int16_t ID6_Current, int16_t ID7_Current, int16_t ID8_Current);
//void Motor_Send_6020(int16_t ID1_Current, int16_t ID2_Current, int16_t ID3_Current, int16_t ID4_Current);
//void Motor_Send2_6020(int16_t ID1_Current, int16_t ID2_Current, int16_t ID3_Current, int16_t ID4_Current);
//float 	Get_Motor_Float_Angle(uint8_t Motor_ID);//CAN1��������е�Ƕ�,��λdeg
//int32_t Get_Motor_Integer_Angle(uint8_t Motor_ID);//����,��λdeg
//int16_t Get_Motor_Speed(uint8_t Motor_ID);//CAN1���ת��,��λRPM
//float 	Get_Motor_Float_Angle2(uint8_t Motor_ID);//CAN2�����е�Ƕȣ���λdeg
//int32_t Get_Motor_Integer_Angle2(uint8_t Motor_ID);
//int16_t Get_Motor_Speed2(uint8_t Motor_ID);	//CAN2���ת��,��λRPM

////void Current_Data_Takepart(int16_t Current_Data, uint8_t *High_Data, uint8_t *Low_Data);
////���ݲ��Ϊ�߰�λ�͵Ͱ�λ
//__inline void Current_Data_Takepart(int16_t Current_Data, uint8_t *High_Data, uint8_t *Low_Data)
//{
//	*High_Data = (Current_Data & 0xff00) >> 8;
//	*Low_Data = (Current_Data & 0x00ff);
//}

//#endif
