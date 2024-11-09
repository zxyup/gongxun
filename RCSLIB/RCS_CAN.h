/* ----C++֧��----------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif
/* -----.h Head----------------------------------------------*/
#ifndef _RCS_CAN_H_
#define _RCS_CAN_H_

/* -----����ͷ�ļ�---------------------------------------------*/
//C���Թ�����
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

//BSP���м��
#include "stm32f4xx.h"        //STM32�Ĵ�������
#include "stm32f4xx_conf.h"   //STM32��׼��ͷ�ļ�
#include "bsp.h"              //uCOS�ײ����
#include "includes.h"         //uCOS�ر����
#include "delay.h"            //��ʱ����

//RCSLIBӲ�������
#include "RCS_Types.h"       //��ȡMCU������-ʱ�����ӹ�ϵ
#include "RCS_HAL.h"         //��ȡMCU�ļĴ�������
#include "RCS_Pin_Mapping.h" //���ذ������ӳ�������ӳ��


/* ------���ú�---------------------------------------------------*/
#define CAN1_BAUD     1000000
#define CAN2_BAUD     1000000

/* ------���������ӿ�----------------------------------------------*/

//���ú���
void RCS_CAN1_Config(RCS_PIN_CAN CANx_MAP,FNCT_VOID _isr, uint32_t _baudRate);
void RCS_CAN2_Config(RCS_PIN_CAN CANx_MAP,FNCT_VOID _isr, uint32_t _baudRate);

//���պ���
void RCS_CANx_Recieve(RCS_CAN_T * CANx,RCS_CAN2B_DATA_FM_RX* RxMessage);
//���ͺ���
void RCS_CANx_Transmit(RCS_CAN_T * CANx,RCS_CAN2B_DATA_FM_TX* TxMessage);

/* ------˽�к���ṹ��--------------------------------------------*/
#define CAN_DATAFM_MAX_DLC 8 //STM32F4��֧��CAN2.0B,�����8���ֽ�

typedef struct 
{
    //todo
}RCS_CAN2B_FLR;//CAN�˲�������

/* ----C++֧��----------------------------------------------*/
#endif
#ifdef __cplusplus
}
#endif
