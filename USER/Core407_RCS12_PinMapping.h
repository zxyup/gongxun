#ifndef CORE407_RCS12_PINMAPPING_H_
#define CORE407_RCS12_PINMAPPING_H_

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
//RCSLIBӲ�������
#include "RCS_Pin_Mapping.h"

/* ------����������------------------------------*/
extern RCS_PIN_USART    USART1_MAP,USART2_MAP,USART3_MAP,USART4_MAP,USART5_MAP,USART6_MAP;
extern RCS_PIN_CYLINDER CY1_MAP,CY2_MAP,CY3_MAP;
extern RCS_PIN_IO       IO1_MAP,IO2_MAP,IO3_MAP,IO4_MAP;
extern RCS_PIN_CAN      CAN1_MAP,CAN2_MAP;
extern RCS_PIN_TIM      TIMER1_MAP,TIMER2_MAP,TIMER3_MAP,TIMER4_MAP;
extern RCS_PIN_IO       IO_DEBUG_MAP;

/* -------��������-------------------------------*/
void RCS_Core407_PinMap_Init(void);
RCS_MCU_CLK RCS_Get_SYSCLK(void);

#endif