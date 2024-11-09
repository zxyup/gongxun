/*@filename: RCS_Encoder.c
 *@author     ��־ΰ       
 *@brief:    ������
 *@date: 2021-12-19
*/

#include "RCS_Encoder.h"
/*------------------ȫ�ֱ���------------------------*/

/*------------------��̬����-----------------------*/
static void Encoder_Init(TIM_TypeDef *TIMx, GPIO_TypeDef *GPIOx,
                             uint32_t GPIO_Pin_A, uint32_t GPIO_Pin_B,FNCT_VOID _intFuc, uint8_t PRIO);
static void Encoder_X_Isr(void);
static void Encoder_Y_Isr(void);
/*-------------------------------------------------*/

/**
	@name: GPS_Encoder_Init
	@brief: ��λ�Ǳ�������ʼ��
**/
void GPS_Encoder_Init(void)
{
	Encoder_Init(GPS_X_TIM,GPS_X_GPIO,GPS_X_A_PIN,GPS_X_B_PIN,Encoder_X_Isr,0x01);
	Encoder_Init(GPS_Y_TIM,GPS_Y_GPIO,GPS_Y_A_PIN,GPS_Y_B_PIN,Encoder_Y_Isr,0x02);
}

/**
	@name: Encoder_Init
	@brief: ��������ʼ��
	@param:TIM_TypeDef *TIMx			��ʱ����
	@param:GPIO_TypeDef *GPIOx 		GPIO��
	@param:uint32_t   GPIO_Pin_A 	A������
	@param:uint32_t   GPIO_Pin_B 	B������
	@param:FNCT_VOID  _intFuc			�жϺ���ָ��
	@param:uint8_t	PRIO					���ȼ�
**/
static void Encoder_Init(TIM_TypeDef *TIMx, GPIO_TypeDef *GPIOx,
                             uint32_t GPIO_Pin_A, uint32_t GPIO_Pin_B,FNCT_VOID _intFuc, uint8_t PRIO)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	if (TIMx == TIM1 || TIMx == TIM8 || TIMx == TIM9 || TIMx == TIM10 || TIMx == TIM11)
		RCC_APB2PeriphClockCmd(GetRCS_RCC_APB2Periph_TIM(TIMx), ENABLE);//����TIMʱ��
	else
		RCC_APB1PeriphClockCmd(GetRCS_RCC_APB1Periph_TIM(TIMx), ENABLE);//����TIMʱ��
	RCC_AHB1PeriphClockCmd(GetRCS_RCC_AHB1Periph_GPIO(GPIOx), ENABLE);//����GPIOʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_A | GPIO_Pin_B;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;						//����ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; 			//��©���
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;				//����
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOx, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOx, GetRCS_GPIO_PinSource(GPIO_Pin_A), GetRCS_GPIO_AF_TIM(TIMx));
	GPIO_PinAFConfig(GPIOx, GetRCS_GPIO_PinSource(GPIO_Pin_B), GetRCS_GPIO_AF_TIM(TIMx));
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);  
	TIM_TimeBaseStructure.TIM_Prescaler = 0;//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period = 65535;//������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

	//���ö�ʱ��Ϊ������ģʽ���ķ�Ƶ
	TIM_EncoderInterfaceConfig(TIMx, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = GetRCS_TIM_Filter(TIMx);  //�����˲���
	TIM_ICInit(TIMx, &TIM_ICInitStructure);
	
	BSP_IntVectSet(GetBSP_INT_ID_TIM(TIMx), _intFuc);
	//����ж�����
	NVIC_InitStructure.NVIC_IRQChannel = GetRCS_TIM_IRQn(TIMx);
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = (PRIO >> 4) & 0x0f;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = PRIO & 0x0f;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ClearFlag(TIMx, TIM_FLAG_Update);  //������б�־λ
	TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE); //�����жϸ���

	//Reset counter
	TIM_SetCounter(TIMx, 0); //
	TIM_Cmd(TIMx, ENABLE);  //ʹ��TIM
}

/**
	@name: Encoder_X_Isr
	@brief: X�������������Ȧ������
**/
static void Encoder_X_Isr(void)
{
	if ( TIM_GetITStatus(GPS_X_TIM , TIM_IT_Update) != RESET )
	{
			if((GPS_X_TIM->CR1 & TIM_CR1_DIR) == TIM_CR1_DIR)//���¼���
			{
					X_Cycle(0);
			}
			else if((GPS_X_TIM->CR1 & TIM_CR1_DIR) == 0x00)//���ϼ���
			{
					X_Cycle(1);
			}
	}
	TIM_ClearITPendingBit(GPS_X_TIM, TIM_FLAG_Update);
}

/**
	@name: Encoder_Y_Isr
	@brief: Y�������������Ȧ������
**/
static void Encoder_Y_Isr(void)
{
	if ( TIM_GetITStatus(GPS_Y_TIM , TIM_IT_Update) != RESET )
	{
			if((GPS_Y_TIM->CR1 & TIM_CR1_DIR) == TIM_CR1_DIR)//���¼���
			{
					Y_Cycle(0);
			}
			else if((GPS_Y_TIM->CR1 & TIM_CR1_DIR) == 0x00)//���ϼ���
			{
					Y_Cycle(1);
			}
	}
	TIM_ClearITPendingBit(GPS_Y_TIM, TIM_FLAG_Update);
}
