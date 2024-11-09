/* --------.h head------------------------------------*/
#ifndef RCS_CAN_INTERFACE_H_
#define RCS_CAN_INTERFACE_H_

/* --------ͷ�ļ�--------------------------------------*/
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
#include "RCS_DataStructure.h" //�������ݽṹ

/* --------������--------------------------------------*/
typedef void (*CAN_HANDLER_VOID)(RCS_CAN2B_DATA_FM_RX);//CAN������ָ������

/* --------���ú�--------------------------------------*/
#define CAN_BUFFER_ALL_FUNCTION   //������CAN���ͺ������ó����buffer����
#define CAN_DEBUG_PRINT           //�����������    
#define CAN_HANDLER_MAX_LEN 5     //����ж��ٸ�CAN������Ա�ע��    
#define CAN_PERI_COUNT      2     //�м�·CAN����

#define CAN_SOFT_BUFFER_LEN  60        //�����г���
#define CAN_BUFFER_TIMER_FRQ 1000000   //���������еĶ�ʱ����Ƶ��(1us)
#define CAN_BUFFER_SEND_FRQ  150       //��ʱ����������count֮�󽫶�ͷ�������� (50us)(<150usʱ������6020���п���)


/* ---------��������-----------------------------------*/

//����CAN�����Լ�CAN�������
void RCS_CANx_Config_With_Buffer(RCS_PIN_CAN CANx_MAP, uint32_t _baudRate);
//ΪCAN������ӽ��շ�����
void RCS_CANx_Add_Handler(RCS_CAN_T *CANx,CAN_HANDLER_VOID Handler);

//���ͱ�׼֡
void RCS_CANx_Send_STDID(RCS_CAN_T *CANx,uint32_t Id, uint8_t Length, uint8_t* sendData);
//������չ֡
void RCS_CANx_Send_EXTID(RCS_CAN_T *CANx,uint32_t Id, uint8_t Length, uint8_t* sendData);
//���ͱ�׼֡/��չ֡
void RCS_CANx_Send(RCS_CAN_T *CANx,uint32_t Id, uint8_t Length, uint8_t* sendData);

//��ȡ��ǰ�ķ�������
uint8_t RCS_Get_CAN_Service_Len(RCS_CAN_T *CANx);
//��ȡCANx�ϵ�ID��������ķ���ID��0��ʼ
CAN_HANDLER_VOID RCS_Get_CAN_Service_Handler(RCS_CAN_T *CANx,uint8_t ID);
//�жϷ����Ƿ��Ѿ��ڶ�����
uint8_t RCS_Judge_CAN_Service(RCS_CAN_T *CANx,CAN_HANDLER_VOID Handler);

void CAN1_Rx_Service_Handler(void);
void CAN2_Rx_Service_Handler(void);


/* --------.h head------------------------------------*/
#endif