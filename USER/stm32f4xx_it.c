/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    04-November-2016
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
//#include "main.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}


/**
 * @name:HardFault_Callback
 * @brief:��Ӳ������ʱ,���øûص������Ե�����м�ɲ
 * @usage:Ϊ�˱��⳵�����ˣ���weak���������ں��ʵĵط������ض��壬���ں�����ʵ��ȫ����ͣ
 **/
#include "cmsis_armclang.h"
#include "RCS_MOTOR.h"
__WEAK void HardFault_Callback(void)
{
  //Ĭ�ϵ���ͣ����
  printf("ERROR:Hard Fault Occured!!\n\r");
  Motor_Send(0,0,0,0);
  Motor_Send_ADD(0,0,0,0);
  Motor_Send2(0,0,0,0);
  Motor_Send2_ADD(0,0,0,0);
  while(1);
}
/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  { 
    HardFault_Callback();                
		                 
    //2023-11-20����Inf��������/��NaN�������� ���½���HardFault---------����ǳ���������/������,����C/0=Inf,tan(0.5pi)=NaN
    //                                                                ע�⸡����һ�㲻ʹ��==�������жϣ�����ʹ�����֮�����ֵС��һ����С�����������ж�
    //                                                                �����ж�x�Ƿ��п��ܵ���0.5pi����������if (fabsf(x-0.5*pi)<=FLOATING_ZERO)
    
    //2024-2-23:  ��ͼ����һ���յĺ���ָ�뵼�½���HardFault     ---------Ϊ�˱���Ұָ�룬�ڶ�һ��ָ��ptrȡ*����ʱ����Ҫд��if (ptr==NULL)

    //2024-6-13�� ����sprintf�ı����ǷǾ�̬�ֲ�����,��ջ�ռ䱬�� --------�����ڷǱ�Ҫʱ���ֲ�����ȫ������Ϊ��̬
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void)
//{
//}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
//void SysTick_Handler(void)
//{
//  //TimingDelay_Decrement();
//}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
