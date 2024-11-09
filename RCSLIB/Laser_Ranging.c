/*@filename: Laser_Ranging.c
 *@author     ���˹�       
 *@brief:     ��ɫ��ǰ�Ħ�켤����ģ�飬UsartͨѶ
 *@brief:     ��෶Χ0.05m~20m,֡��20HZ����������1mm
 *@date: 2023-8-30
*/

#include "Laser_Ranging.h"
/*---------------ȫ�ֱ���-------------*/
static float data = 0;
static float laser_data = 0;
static uint8_t check = 0;   //У��
RCS_PIN_USART Laser_USART_MAP;
/*---------------��̬����-------------*/
static void LASER_Interrupt(void);
/*------------------------------------*/


/**
	@name: Laser_Ranging_Init
	@brief: �������ʼ��
**/
void Laser_Ranging_Init(RCS_PIN_USART USARTx_MAP)
{
		Laser_USART_MAP=USARTx_MAP;
    RCS_USART_Config(USARTx_MAP.USARTx,USARTx_MAP.GPIOx,USARTx_MAP.GPIO_Pin_Tx,USARTx_MAP.GPIO_Pin_Rx,LASER_Interrupt,LASER_BAUD,LASER_PRI);
	  delay_ms(100);
}

/**
	@name: LASER_Interrupt
	@brief: �жϽ��ܺ���(ʹ��ʱӳ��ɶ�ӦUSART���жϺ���)
**/
static void LASER_Interrupt(void)
{
	uint8_t receive_char;
	static uint8_t step = 0;
	
	if(USART_GetITStatus(Laser_USART_MAP.USARTx, USART_IT_RXNE) == SET)
	{
		receive_char = RCS_USART_Accept_Char(Laser_USART_MAP.USARTx);
		
		switch(step)
		{
			case 0: if(receive_char==0xB4)
							{
								data = 0;
								check = 0;
								check ^= receive_char;
								step ++;
							}
							else
							{
								step = 0;
							}
							break;
							
			case 1: if(receive_char==0x69)
							{
								check ^= receive_char;
								step ++;
							}
							else
							{
								step = 0;
							}
							break;				
							
			case 2: if(receive_char==0x04)
							{
								check ^= receive_char;
								step ++;
							}
							else
							{
								step = 0;
							}
							break;			

			case 3: data += (receive_char<<24)&0xFF000000;
							check ^= receive_char;
							step ++;
							break; 				
							
			case 4: data += (receive_char<<16)&0xFF0000;
							check ^= receive_char;
							step ++;
							break; 		
							
			case 5: data += (receive_char<<8)&0xFF00;
							check ^= receive_char;
							step ++;
							break; 		

      case 6: data += receive_char;
							check ^= receive_char;
							step ++;
							break; 		

      case 7: if(receive_char == check)  //У��
			        {
								laser_data = data;
			        }
				      step = 0;
							break; 					
							
			default:step = 0;
							break;				
		}
	}
	USART_ClearITPendingBit(Laser_USART_MAP.USARTx, USART_IT_RXNE);
}

/**
	@name: Get_Laser_Data
	@brief:��ȡ������ֵ
**/
float Get_Laser_Data(void)
{
	return laser_data/10.0f;
}

/**
	@name: Get_Ranging_Y
	@brief:������Y����
**/
float Get_Ranging_Y(void)
{
	return (laser_data/10.0f - 2354.0f) * cos(Get_GPS_Z() * DEG2RAD);
}	