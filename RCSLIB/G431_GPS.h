#ifndef G431_GPS_H_
#define G431_GPS_H_

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
#include "RCS_Ptl.h"           //����ͨ��Э��




//=========================���ú�=======================================

#define G431GPS_START_BYTE 0xEF
#define G431GPS_PKG_LEN    12
#define G431_GPS_BAUD      115200

typedef enum{
	CMD_ID_LOGIN = 0x01,
	CMD_ID_LOGOUT= 0x02,
	CMD_ID_SET_TX= 0x03,
	CMD_ID_SET_TY= 0x04,
	CMD_ID_SET_DX= 0x05,
	CMD_ID_SET_DY= 0x06,
	CMD_ID_SET_AX= 0x07,
	CMD_ID_SET_AY= 0x08,
	CMD_ID_SET_CF= 0x09,
	CMD_ID_SET_CR= 0x0A,
}Uart_Cmd;

//========================��������======================================
void G431GPS_Init(RCS_PIN_USART USARTx_MAP,uint8_t _pri);
float G431GPS_Get_X(void);
float G431GPS_Get_Y(void);
float G431GPS_Get_Z(void);
void G431GPS_Set_Parameter(Uart_Cmd CMD_ID_xx,float param);

#endif
