#ifndef _RCS_DT35_H_
#define _RCS_DT35_H_

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

#include "stm32f4xx_adc.h"
#include "RCS_ADC.h"
#include "RCS_Stastic.h"

typedef struct 
{
    float Last_P;//�ϴι���Э����
    float Now_P;//��ǰ����Э����
    float out;//�������˲������
    float K;//����������
    float Q;//��������Э����
    float R;//�۲�����Э����
		float Predit;
}Kalman_Para;

typedef struct {
	float min_dist_mm;
	float max_dist_mm;
	ADC_TypeDef* _ADCx;
	GPIO_TypeDef* _GPIOx;
	uint32_t _pin;
	uint8_t  channel;
	
	Comm_WindowData_t filter;
	Kalman_Para       KF_Param;
}DT35_Handler_t;

void DT35_Init(DT35_Handler_t* hdl,float min_mm,float max_mm,ADC_TypeDef* ADCx,uint8_t ADC_Channel_x,GPIO_TypeDef* GPIOx,uint32_t GPIO_Pin_x);
void DT35_Setup_Filter(DT35_Handler_t* hdl,uint16_t window_size);
void DT35_Setup_KalmanFilter(DT35_Handler_t* hdl,float p,float q,float r,float gain);
uint16_t DT35_Get_Raw(DT35_Handler_t* hdl);
float DT35_Get_dB(DT35_Handler_t* hdl);
float DT35_Get_MM(DT35_Handler_t* hdl);
float DT35_Get_Kalman(DT35_Handler_t* hdl);

#endif