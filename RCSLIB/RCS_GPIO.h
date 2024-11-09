//@filename: RCS_GPIO.h
//@date: 2020-7-31
//@author: ��־ΰ
//@brief: ����GPIO����
#ifndef  _RCS_GPIO_H_
#define  _RCS_GPIO_H_

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

void RCS_GPIO_Output_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void RCS_GPIO_Input_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void RCS_GPIO_Set(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void RCS_GPIO_Reset(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void RCS_GPIO_Toggle(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint8_t RCS_GPIO_Read(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

#endif
