/**
 * @filename: rcs.h
 * @brief:ģ����빫�ô�ͷ�ļ�,RCSers ��д�ĳ��ù��ܷ�װ
 * @date:     2012-08-17          �¹���
 * @changlog: 2012-08-19  21:54   ������   �����"RCS_pwm.h"�İ���.
 * @changlog: 2019-07-12	09:03   ����	   �����"RCS_OLED.h"��"RCS_OLED_ASCII.h"��"RCS_ADC.h"�İ������������������
 * @changlog: 2020-11-21  20:38		��־ΰ	 ��������
 * @changlog: 2023-7-28   00:59   ���˹�   ��������
 * @changlog: 2024-2-20   15:54   ���Ͽ�   �淶����ʹ�÷�ʽ
*/

/* ------------h header-------------------------------------*/
#ifndef _RCS_H_
#define _RCS_H_

/* -------------���ó�������---------------------------------*/
#define pi 				3.14159265357f        //��
#define DEG2RAD         1.74532925199433e-2f  //�Ƕȱ任�ɻ���     
#define RAD2DEG         5.72957795130823e1f		//���ȱ任�ɽǶ�
#define FLOAT_ZERO      1e-10f                //��������ȵ��ݲ�
#define G               9.8f                  //�������ٶ�

/* --------------�������-------------------------------------*/
//��ʱ������
#define HARD_STAMPER_TIMER   TIM9
#define CAN1_BUFFER_TIMER    TIM7  //����CAN1���ζ��еĶ�ʱ��   
#define CAN2_BUFFER_TIMER    TIM6  //����CAN2���ζ��еĶ�ʱ��  
#define GPS_X_TIM			 TIM1
#define GPS_Y_TIM			 TIM4

//�ⲿ�ж�

/* -------------�ж����ȼ�����--------------------------------*/
#define RCS_CAN1_BUFFER_PRI 0x12
#define RCS_CAN2_BUFFER_PRI 0x13
#define LASER_PRI           0x02
#define GYRO_PRI            0x10
#define ACTION_PRI          0x12
#define VISION_PRI          0x02
#define BLUETOOTH_PRI       0x01	
#define STAMP_PRI           0x01

#define R1_CALLBACK_PRI     0x02

/* --------------�����ʱ��λ�Ź���----------------------------*/
#define STIM_SCARA_PICK     1
#define STIM_UPCTRL_R1      2

/* -----------------ͷ�ļ�------------------------------------*/
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
#include "RCS_Filter.h"        //�˲��㷨
#include "DoubleRing.h"        //˫��PID�㷨
#include "FSM_Lite.h"          //����״̬��
#include "RCS_DataStructure.h" //�������ݽṹ
#include "RCS_AcceCtrl.h"

//RCSLIBģ��������(CAN���繦��)
#include "RCS_CAN_Interface.h" //CAN���߽����ӿ�
#include "RCS_MOTOR.h"         //CAN����RM�������
#include "RCS_VESC_MOTOR.h"    //CAN����VESC�������

//RCSLIBģ��������(OLED��Ļ)
#include "RCS_OLED.h"          //OLED����
#include "RCS_ASCII.h"         //OLED�ֿ�
#include "RCS_Interface.h"     //OLED�ӿ�

//RCSLIBģ��������(����ģ��)
#include "RCS_Pwm.h"           //PWM�����
#include "RCS_Remote.h"        //RM�ֱ�����(��ͣ��)
#include "Joystick_Ctrl.h"     //�����ֱ�����
#include "RCS_Encoder.h"       //����������
#include "Laser_Ranging.h"     //Ħ�켤��������
#include "Gyro.h"              //ά����������������
#include "GPS_Action.h"        //����ȫ����λ����
#include "G431_GPS.h"

//RCSLIBģ��������(���̿���)
#include "RCS_BaseMove.h"    //������ȫ���ֵ�������
#include "RCS_AGV_BaseMove.h"//�����˶��ֵ�������

//RCSLIB�����˹��ܲ�(�����˶�λ)
#include "Laser_Position.h"     //�ں϶�λ�㷨

//RCSLIB�����˹��ܲ�(�����˿���)
#include "RCS_Ctrl_BaseMove.h"  //�����˵��̵������
#include "RCS_Motor_Upctrl.h"   //�������ϲ�������
#include "RCS_Actor_Upctrl.h"   //�������ϲ�����/����ȿ���
#include "RCS_Debug.h"          //���õ��Թ���
#include "FSM.h"                //����״̬��
#include "RCS_RTOS.h"           //����RTOS����
#include "RCS_Ptl.h"            //���ͨ��Э��

//ҵ���߼���(������ҵ��ӿ�)
#include "Vision.h"   //�Ӿ�����
#include "Ch_Ctrl.h"    //������ȫ�Զ�����·��
#include "Up_Ctrl.h"  //�������ϲ��������ʵ��
#include "R2_Upctrl.h"//R2�ϲ��������ʵ��

//ҵ���߼���(��������)
#include "app_cfg.h"  //���ø�����Ķ�ջ��С�����ȼ�
#include "MainTask.h" //�����˳�ʼ��
#include "GpsTask.h"      //�����˶�λ(����)

#include "Core407_RCS12_PinMapping.h"
#include "Core407_RCS12_Debug.h"

/* ---------end of .h header-------------------------------*/
#endif
