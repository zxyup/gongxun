/*@filename: Laser_Ranging.h
 *@author     ���˹�       
 *@brief:     ��ɫ��ǰ漤����ģ��
 *@date: 2023-8-29
*/
#ifndef _LASER_RANGING_H_
#define _LASER_RANGING_H_

#include "rcs.h"	


#define LASER_BAUD				  9600			

void Laser_Ranging_Init(RCS_PIN_USART USARTx_MAP);
float Get_Laser_Data(void);
float Get_Ranging_Y(void);

#endif
