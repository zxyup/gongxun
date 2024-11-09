//@filename: Timer.c
//@date: 2012-08-16
//@author: 李万雪
//@brief: 时钟中断

#include "RCS_Timer.h"

/**
  @name: InitTimerInt
  @brief: 启动时钟中断
  @param:TIM_TypeDef * _TIM 定时器号
  @param:uint32_t _TIM_period 计数周期
  @param:uint32_t  _div 将一秒分割为_div个单位,必须大于3000，否则分频溢出
  @param:FNCT_VOID _TIM_ISR: ISR指针
  @param:uint8_t _priority ：前4位为抢占优先级，后4位为响应优先级
  @notes:ISR函数最后需要 清中断
  @notes:STM32的定时器的CNT最大65535,若_div过高导致无法实现，则该函数会通过assert_param引发报错


  TIM_ClearITPendingBit(TIMx,TIM_IT_Update);
  周期计算为 _TIM_period/_div 秒
*/
void InitTimerInt(TIM_TypeDef *_TIM, uint32_t _TIM_period, uint32_t  _div,
				  FNCT_VOID _TIM_ISR, uint8_t _priority)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_ClocksTypeDef rcc_clocks;
	RCC_GetClocksFreq(&rcc_clocks);

	uint32_t SysClk;

	//TIM_X clock enable
	if (_TIM == TIM1 ||  _TIM == TIM8 || _TIM == TIM9 || _TIM == TIM10 || _TIM == TIM11 )
	{
		RCC_APB2PeriphClockCmd(GetRCS_RCC_APB2Periph_TIM(_TIM), ENABLE);
		SysClk = (uint32_t ) (rcc_clocks.SYSCLK_Frequency );
	}
	else
	{
		RCC_APB1PeriphClockCmd(GetRCS_RCC_APB1Periph_TIM(_TIM), ENABLE);
		SysClk = (uint32_t ) (rcc_clocks.SYSCLK_Frequency / 2);
	}
	assert_param( (uint32_t)(SysClk / (float)_div) <= 65536 );
	//enable the TIM_X global interrupt
	NVIC_InitStructure.NVIC_IRQChannel = GetRCS_TIM_IRQn(_TIM);
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = (_priority >> 4) & 0x0f;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = _priority & 0x0f;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//set interrupt time _TIM_period
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = _TIM_period - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)(SysClk / (float)_div) - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(_TIM, &TIM_TimeBaseStructure);
	BSP_IntVectSet(GetBSP_INT_ID_TIM(_TIM), (_TIM_ISR));
	TIM_ITConfig(_TIM, TIM_IT_Update, ENABLE);

	//TIM enable counter
	TIM_Cmd(_TIM, ENABLE);
}
//@name: StopTimer
//@brief: 停止时钟中断
//@param:TIM_TypeDef * _TIM 定时器号
void StopTimer(TIM_TypeDef *_TIM )
{
	TIM_Cmd(_TIM, DISABLE);
}

void StartTimer(TIM_TypeDef *_TIM)
{
	TIM_Cmd(_TIM,ENABLE);
}