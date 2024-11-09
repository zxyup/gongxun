//@filename: ADC.h
//@date: 2019-07-22
//@author: 闫锐
//@brief: 模拟量转换数字量
#ifndef _RCS_ADC_H_
#define _RCS_ADC_H_

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

#define LASER_ADC_ONE                   ADC1
#define LASER_GPIO_ONE                  GPIOA
#define LASER_PIN_ONE                   GPIO_Pin_4
#define LASER_CHANNEL_ONE               ADC_Channel_4

#define LASER_ADC_TWO                   ADC1
#define LASER_GPIO_TWO                  GPIOA
#define LASER_PIN_TWO                   GPIO_Pin_5
#define LASER_CHANNEL_TWO               ADC_Channel_5

#define LASER_ADC_THREE                 ADC1
#define LASER_GPIO_THREE                GPIOA
#define LASER_PIN_THREE                 GPIO_Pin_6
#define LASER_CHANNEL_THREE             ADC_Channel_6

#define LASER_ADC_FOUR                  ADC1
#define LASER_GPIO_FOUR                 GPIOA
#define LASER_PIN_FOUR                  GPIO_Pin_7
#define LASER_CHANNEL_FOUR              ADC_Channel_7

#define VOLTAGE_REFERANCE	3.3f//参考电�?
#define	MAX_RANGE			4096//最大量�?

//@name: RCS_ADC_Init
//@brief: 初始化ADC
//@param:ADC_TypeDef *_ADCx ADC�?
//@param:GPIO_TypeDef *_GPIOx GPIO�?
//@param:uint32_t _pin 管脚
//@note:测量电压不得超过3.3V
void RCS_ADC_Init(ADC_TypeDef *_ADCx, GPIO_TypeDef *_GPIOx, uint32_t _pin);

//@name: RCS_Get_ADC
//@brief: 获取ADC的�?
//@param:ADC_TypeDef *_ADCx ADC�?
//@param:ADC_Channel_x 管脚通道�?
uint16_t RCS_Get_ADC(ADC_TypeDef *_ADCx, uint8_t ADC_Channel_x);

//@name: RCS_Get_Voltage        
//@brief: 获取ADC的电压�?
//@param:ADC_TypeDef *_ADCx ADC�?
//@param:_channel 管脚通道�?
float RCS_Get_Voltage(ADC_TypeDef *_ADCx, uint8_t ADC_Channel_x);

#endif
