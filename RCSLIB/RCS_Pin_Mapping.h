/**
 * @filename:RCS_Pin_Mapping.h
 * @brief:MCU������ܽ�->���ذ�ܽŵ�ӳ��
*/
#ifndef RCS_PIN_MAPPING_H_
#define RCS_PIN_MAPPING_H_

/* --------ͷ�ļ� ------------------------------*/
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
#include "RCS_HAL.h"

/* --------˽�нṹ��---------------------------*/
typedef struct io_map
{
    GPIO_TypeDef* GPIOx;
    uint32_t      GPIO_Pin_x;
}RCS_MAP_IO;

/* ---------�����ṹ�� ------------------------*/
typedef struct can_map
{
    RCS_GPIO_T* GPIOx;
    uint32_t      GPIO_Pin_Tx;
    uint32_t      GPIO_Pin_Rx;
    RCS_CAN_T*    CANx;
}RCS_PIN_CAN;//�����ϵ�4pinCAN�ӿ�

typedef struct usart_map
{
    RCS_GPIO_T*    GPIOx;
    uint32_t       GPIO_Pin_Tx;
    uint32_t       GPIO_Pin_Rx;
    RCS_UART_T*    USARTx;
}RCS_PIN_USART;//�����ϵ�4pin����

typedef struct gh6pin_io_map
{
    RCS_MAP_IO IO[4];
}RCS_PIN_IO;//�����ϵ�6pinIO�ӿ�

typedef struct gh6pin_tim_map
{
    TIM_TypeDef* TIMx[4];
    uint8_t      Channel[4];
    RCS_MAP_IO   IO[4];
}RCS_PIN_TIM;//�����ϵ�6pin��ʱ���ӿ�

typedef RCS_PIN_IO RCS_PIN_CYLINDER;//�����ϵ�6pinQG�ӿ�
typedef RCC_ClocksTypeDef RCS_MCU_CLK;//��Ƭ����ʱ��





#endif