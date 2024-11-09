//@filename: RCS_GPIO.c
//@date: 2020-7-31
//@author: ��־ΰ
//@brief: ����GPIO����
#include "RCS_GPIO.h"


/**
	@name: RCS_GPIO_Output_Init
	@brief: �ܽ������ʼ��
	@param:GPIO_TypeDef *GPIOx GPIO��
	@param:uint32_t   GPIO_Pin ����ܽź�
**/
void RCS_GPIO_Output_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
	assert_param(IS_GPIO_PIN(GPIO_Pin));
	//����GPIOʱ��
	RCC_AHB1PeriphClockCmd(GetRCS_RCC_AHB1Periph_GPIO(GPIOx), ENABLE);
	//�������ģʽ���ٶ�100MHZ
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;			//���ģʽ
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin;						//����
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			//����
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;				//����
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHZ
	GPIO_Init(GPIOx, &GPIO_InitStruct);
}



/**
	@name: RCS_GPIO_Input_Init
	@brief: �ܽ������ʼ��
	@param:GPIO_TypeDef *GPIOx GPIO��
	@param:uint32_t   GPIO_Pin ����ܽź�
**/
void RCS_GPIO_Input_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct;				//������ʼ���ṹ��
	
	assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
	assert_param(IS_GPIO_PIN(GPIO_Pin));

	RCC_AHB1PeriphClockCmd(GetRCS_RCC_AHB1Periph_GPIO(GPIOx), ENABLE);
	//��������ģʽ
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;				//����ģʽ
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin;						//����
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;				//����
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;	//100MHZ
	
	GPIO_Init(GPIOx, &GPIO_InitStruct);
}



/**
	@name: RCS_GPIO_Set
	@brief: �ܽ��ø�
	@param:GPIO_TypeDef *GPIOx GPIO��
	@param:uint32_t   GPIO_Pin ����ܽź�
**/
void RCS_GPIO_Set(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
		GPIO_SetBits(GPIOx, GPIO_Pin);
}


/**
	@name: RCS_GPIO_Reset
	@brief: �ܽ��õ�
	@param:GPIO_TypeDef *GPIOx GPIO��
	@param:uint32_t   GPIO_Pin ����ܽź�
**/
void RCS_GPIO_Reset(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GPIO_ResetBits(GPIOx, GPIO_Pin);
}



/**
	@name: RCS_GPIO_Toggle
	@brief: ���ùܽ�
	@param:GPIO_TypeDef *GPIOx GPIO��
	@param:uint32_t   GPIO_Pin ����ܽź�
**/
void RCS_GPIO_Toggle(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	if (GPIO_ReadOutputDataBit(GPIOx, GPIO_Pin) == Bit_SET)
		RCS_GPIO_Reset(GPIOx, GPIO_Pin);
	else 
		RCS_GPIO_Set(GPIOx, GPIO_Pin);
}


/**
	@name: RCS_GPIO_Read
	@brief: ��ȡ�ܽŵ�ƽ�ߵ�(����ֵ ��1����0)
	@param:GPIO_TypeDef *GPIOx GPIO��
	@param:uint32_t   GPIO_Pin ����ܽź�
**/
uint8_t RCS_GPIO_Read(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	return GPIO_ReadInputDataBit(GPIOx, GPIO_Pin);
}
