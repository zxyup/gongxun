/**
 * @filename:G431_GPS.c
 * @brief:������G431ȫ����λ����н���(����ģʽ)
 * @contribute:CYK-Dot @2024-6-1
 --------------------------------------------------
 * @todo:�����ع��ڵ���ģʽ��,�����Uart�����ж��д�ϵ�,G431�ͻ������ػָ�����֮����
**/

/*=================ͷ�ļ�===========================================*/
#include "G431_GPS.h"


/*=================ȫ�ֱ���=========================================*/

RCS_PIN_USART        G431_GPS_USART;                //�ܽ�
RCS_LenVerify_Ptl    G431_GPS_Ptl;                  //Э��

uint8_t              G431_GPS_Pkg_Rcv_Char;         //�������յ����ֽ�
uint8_t              G431_GPS_Pkg_Rcv_State;        //Э��״̬�����ص���־
uint8_t              G431_GPS_Pkg_RcvBufr_Arr[20];  //���Ļ�����
uint8_t              G431_GPS_Pkg_RcvCplt_Arr[20];  //��ɽ��յı����ֽ���
uint8_t              G431_GPS_Pkg_Txn[32];


float                G431_GPS_Pkg_X;                //��ȡ����XYZ����
float                G431_GPS_Pkg_Y;
float                G431_GPS_Pkg_Z;

/*===============��̬������������====================================*/

static void G431GPS_Usart_Isr_Callback(void);
__WEAK void G431GPS_RcvCplt_Callback(void);
__WEAK void G431GPS_RcvFail_Callback(void);

/*=================������������=====================================*/

/**
 * @name: G431GPS_Init
 * @brief:��ʼ����ش��ڣ���ָ����ʼ���Ĺܽź��ж����ȼ�
**/
void G431GPS_Init(RCS_PIN_USART USARTx_MAP,uint8_t _pri)
{
	G431_GPS_USART=USARTx_MAP;
	RCS_USART_Config(USARTx_MAP.USARTx,
	                 USARTx_MAP.GPIOx,USARTx_MAP.GPIO_Pin_Tx,USARTx_MAP.GPIO_Pin_Rx,
	                 G431GPS_Usart_Isr_Callback,
	                 G431_GPS_BAUD,_pri);

	Protocol_Init_LenVerify(&G431_GPS_Ptl,G431GPS_START_BYTE,G431GPS_PKG_LEN,G431_GPS_Pkg_RcvBufr_Arr);
}

/**
 * @name:G431GPS_Get_X
 * @brief:��ȡG431ȫ����λ���ص�X����
**/
float G431GPS_Get_X(void)
{
	return G431_GPS_Pkg_X;
}

/**
 * @name:G431GPS_Get_Y
 * @brief:��ȡG431ȫ����λ���ص�Y����
**/
float G431GPS_Get_Y(void)
{
	return G431_GPS_Pkg_Y;
}

/**
 * @name:G431GPS_Get_Z
 * @brief:��ȡG431ȫ����λ���ص�Z����(�Ƕ���)
**/
float G431GPS_Get_Z(void)
{
	return G431_GPS_Pkg_Z;
}

/**
 * @name:G431GPS_Set_Parameter
 * @brief:����ȫ����λ�Ĳ���
 * @param:CMD_ID_xx ������,����CMD_ID_��keil���Զ�����   @ref:Uart_Cmd@G431_GPS.h
 * @param:param     ����
**/
void G431GPS_Set_Parameter(Uart_Cmd CMD_ID_xx,float param)
{
	uint8_t i;
	G431_GPS_Pkg_Txn[0]=CMD_ID_xx;
	sprintf(&G431_GPS_Pkg_Txn[1],"%.6f\n\r",param);
	while(G431_GPS_Pkg_Txn[i]!='\r')
	{
		RCS_USART_Send_Char(G431_GPS_USART.USARTx,G431_GPS_Pkg_Txn[i]);
	}
	RCS_USART_Send_Char(G431_GPS_USART.USARTx,'\r');
}


/*=================��̬��������=====================================*/

/**
 * @name:G431GPS_Usart_Isr_Callback
 * @brief:Uart�����ж�
**/
static void G431GPS_Usart_Isr_Callback(void)
{
	//--------------�ֲ�����---------------------------------------------------
	uint32_t temp_u32_x;
	uint32_t temp_u32_y;
	uint32_t temp_u32_z;

	//---------�ж��Ƿ�ΪRXNE�жϱ�־λ---------------------------------------
	if (RCS_USART_Judge_RcvCplt_ITPendingBit(G431_GPS_USART.USARTx)==1)
	{
		//�����յ����ֽ�����Э��״̬��
		G431_GPS_Pkg_Rcv_Char =(uint8_t)RCS_USART_Accept_Char(G431_GPS_USART.USARTx);
		G431_GPS_Pkg_Rcv_State=Protocol_Rcv_LenVerify_IT((void*)&G431_GPS_Ptl,G431_GPS_Pkg_Rcv_Char,G431_GPS_Pkg_RcvCplt_Arr);

		//״̬�����ؽ��ճɹ�
		if (G431_GPS_Pkg_Rcv_State==RCV_CPLT)
		{
			//����ֱ�Ӹ�ֵΪfloat���ᱻ��������Ϊǿ������ת��������ʹ�����������memcpy
			temp_u32_x =((uint32_t)G431_GPS_Pkg_RcvCplt_Arr[0] << 24) |
                  ((uint32_t)G431_GPS_Pkg_RcvCplt_Arr[1] << 16) |
                  ((uint32_t)G431_GPS_Pkg_RcvCplt_Arr[2] << 8)  |
                  ((uint32_t)G431_GPS_Pkg_RcvCplt_Arr[3]);
			temp_u32_y =((uint32_t)G431_GPS_Pkg_RcvCplt_Arr[4] << 24) |
                  ((uint32_t)G431_GPS_Pkg_RcvCplt_Arr[5] << 16) |
                  ((uint32_t)G431_GPS_Pkg_RcvCplt_Arr[6] << 8)  |
                  ((uint32_t)G431_GPS_Pkg_RcvCplt_Arr[7]);
			temp_u32_z =((uint32_t)G431_GPS_Pkg_RcvCplt_Arr[8] << 24) |
                  ((uint32_t)G431_GPS_Pkg_RcvCplt_Arr[9] << 16) |
                  ((uint32_t)G431_GPS_Pkg_RcvCplt_Arr[10] << 8) |
                  ((uint32_t)G431_GPS_Pkg_RcvCplt_Arr[11]);

			memcpy(&G431_GPS_Pkg_X,&temp_u32_x,4);
			memcpy(&G431_GPS_Pkg_Y,&temp_u32_y,4);
			memcpy(&G431_GPS_Pkg_Z,&temp_u32_z,4);

			//�ص�����
			G431GPS_RcvCplt_Callback();
		}
		else if (G431_GPS_Pkg_Rcv_State==RCV_FAIL)
		{
			//�ص�����
			G431GPS_RcvFail_Callback();
		}
	}
	//-------------------����жϱ�־λ---------------------------------------
	RCS_USART_Clear_RcvCplt_ITPendingBit(G431_GPS_USART.USARTx);
}

/*=================����������=======================================*/

/**
 * @name:G431GPS_RcvCplt_Callback
 * @brief:�����һ�α��Ľ���ʱ�ᱻ���õĻص�����
**/
__WEAK void G431GPS_RcvCplt_Callback(void)
{

}
/**
 * @name:G431GPS_RcvCplt_Callback
 * @brief:������У��ʧ��ʱ�ᱻ���õĻص�����
**/
__WEAK void G431GPS_RcvFail_Callback(void)
{

}
