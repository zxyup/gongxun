#ifndef RCS_HAL_H_
#define RCS_HAL_H_

#include "stm32f4xx.h"        //STM32�Ĵ�������

/* -------�����Ĵ�����---------------------------*/

//CAN2.0B����֡��������HAL����
typedef CanTxMsg  RCS_CAN2B_DATA_FM_TX; 
typedef CanRxMsg  RCS_CAN2B_DATA_FM_RX;

//CAN�����жϵ�HAL����
#define RCS_CAN_RXIT_FLAG(CANx) CAN_GetITStatus(CANx, CAN_IT_FMP0) 

/*--------����������------------------------------*/
typedef CAN_TypeDef   RCS_CAN_T;
typedef GPIO_TypeDef  RCS_GPIO_T;
typedef USART_TypeDef RCS_UART_T;




#endif