//@filename: Timer.c
//@date: 2012-08-16
//@author: 李万�?
//@brief: 时钟中断

#ifndef _TIMER_H_
#define _TIMER_H_

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


//@name: InitTimerInt
//@brief: 启动时钟中断
//@param:TIM_TypeDef * _TIM 定时器号
//@param:uint32_t _TIM_period 计数周期
//@param:uint32_t  _div 将一秒分割为_div个单�?,必须大于3000，否则分频溢�?
//@param:FNCT_VOID _TIM_ISR: ISR指针
//@param:uint8_t _priority ：前4位为抢占优先级，�?4位为响应优先�?
//@notes：ISR函数最后需�? 清中�?
//TIM_ClearITPendingBit(TIMx,TIM_IT_Update);
//周期计算�? _TIM_period/_div �?
void InitTimerInt(TIM_TypeDef *_TIM, uint32_t _TIM_period, uint32_t  _div,
				  FNCT_VOID _TIM_ISR, uint8_t _priority);

//@name: StopTimer
//@brief: 停止时钟中断
//@param:TIM_TypeDef * _TIM 定时器号
void StopTimer(TIM_TypeDef *_TIM );
void StartTimer(TIM_TypeDef *_TIM);
#endif
