/**
 @name:Vision.c
 @brief:�Ӿ����ڵ�Ӧ�ò�����
**/

/*==============ͷ�ļ�====================================*/
#include "Vision.h"

/*=============˽��ȫ�ֱ���===============================*/

//���򴮿�
RCS_PIN_USART VISION_PICK_USART_MAP;
RCS_Easy_Ptl  Vision_Pick_Protocol;
#define       PICK_USART_ISR_PRI   0x02
#define       PICK_USART_PKG_LEN   4

//�±��й�����ͷ����
RCS_PIN_USART VISION_CUP_USART_MAP;
RCS_Easy_Ptl  Vision_Cup_Protocol;
#define       CUP_USART_ISR_PRI   0x03
#define       CUP_USART_PKG_LEN   10

//���ض�����������
RCS_PIN_USART VISION_DIST_USART_MAP;
RCS_Easy_Ptl  Vision_Dist_Protocol;
#define       DIST_USART_ISR_PRI   0x01
#define       DIST_USART_PKG_LEN   10






/*=============�û�ȫ�ֱ���===============================*/
DESCARTES_AXIS Vision_Ball_DescPos;
int            Vision_Ball_Count;
DESCARTES_AXIS Vision_Cup_Target_DescPos;
int            Vision_Cup_Target_Distance;

/*=============�ص���������===============================*/

//��������ص�
static void Vision_Pick_Isr(void);
void Vision_Pick_Get(uint8_t* rcv_cplt_arr);

//�±��й�ص�
static void Vision_Cup_Isr(void);
void Vision_Cup_Get(uint8_t* rcv_cplt_arr);

//���ض��ص�
static void Vision_Dist_Isr(void);
void Vision_Dist_Get(uint8_t* rcv_cplt_arr);

/*=============Ӧ�ò�ӿں���=============================*/

/**
 @addtogroup: ���Ӿ�ͨ�ų�ʼ��
**/
void Vision_Init(RCS_PIN_USART Pick_USARTx_MAP,RCS_PIN_USART Cup_USARTx_MAP,RCS_PIN_USART Dist_USARTx_MAP)
{
	//========================USB Camera======================================================
	//������·��Э���ʼ��
	Vision_Pick_Protocol.Start_Byte=0xBB;
	Vision_Pick_Protocol.End_Byte=0xBC;
	Vision_Pick_Protocol.Len=4;
	Vision_Pick_Protocol.Finish_Callback=Vision_Pick_Get;
	//������ʼ��
	VISION_PICK_USART_MAP=Pick_USARTx_MAP;                  //ָ������
	RCS_USART_Config(Pick_USARTx_MAP.USARTx, Pick_USARTx_MAP.GPIOx,Pick_USARTx_MAP.GPIO_Pin_Tx,Pick_USARTx_MAP.GPIO_Pin_Rx,
		             Vision_Pick_Isr ,                 //�����жϻص�
		             VISION_BAUD,                      //������
		             PICK_USART_ISR_PRI);              //�����ж����ȼ�

	//========================307 Camera========================================================
	//������·��Э���ʼ��
	Vision_Cup_Protocol.Start_Byte=0xFA;
	Vision_Cup_Protocol.End_Byte=0xFF;
	Vision_Cup_Protocol.Len=6;
	Vision_Cup_Protocol.Finish_Callback=Vision_Cup_Get;
	//������ʼ��
	VISION_CUP_USART_MAP=Cup_USARTx_MAP;                  //ָ������
	RCS_USART_Config(Cup_USARTx_MAP.USARTx, Cup_USARTx_MAP.GPIOx,Cup_USARTx_MAP.GPIO_Pin_Tx,Cup_USARTx_MAP.GPIO_Pin_Rx,
		             Vision_Cup_Isr ,                 //�����жϻص�
		             VISION_BAUD,                      //������
		             CUP_USART_ISR_PRI);              //�����ж����ȼ�
	
	//=======================Intel D455 Camera==================================================
	//������·��Э���ʼ��
	
	//������ʼ��
	VISION_DIST_USART_MAP=Dist_USARTx_MAP;                  //ָ������
	RCS_USART_Config(Dist_USARTx_MAP.USARTx, Dist_USARTx_MAP.GPIOx,Dist_USARTx_MAP.GPIO_Pin_Tx,Dist_USARTx_MAP.GPIO_Pin_Rx,
		             Vision_Dist_Isr ,                 //�����жϻص�
		             VISION_BAUD,                      //������
		             PICK_USART_ISR_PRI);              //�����ж����ȼ�

	//===================�ȴ���������׼������===================================================
	delay_ms(50);
}

/**
 @name: Vision_Get_Ball_Count
 @brief: ��ȡ��Ұ���������
**/
int Vision_Get_Ball_Count(void)
{
	return Vision_Ball_Count;
}
/**
 @name: Vision_Get_Ball_Pos
 @brief: ��ȡ��Ұ�����һ���������
**/
DESCARTES_AXIS Vision_Get_Ball_Pos(void)
{
	return Vision_Ball_DescPos;
}
/**
 @name: Vision_Get_Ball_Pos
 @brief: ��ȡ�±��й������Ŀ�����Xλ��
**/
int Vision_Cup_Get_Ball_Pos_X(void)
{
	return Vision_Cup_Target_DescPos.x;
}
/**
 @name: Vision_Get_Ball_Pos
 @brief: ��ȡ�±��й������Ŀ�����Yλ��
**/
int Vision_Cup_Get_Ball_Pos_Y(void)
{
	return Vision_Cup_Target_DescPos.y;
}
/**
 @name: Vision_Get_Ball_Pos
 @brief: ��ȡ�±��й������Ŀ����ľ��룬������Χ��0
**/
int Vision_Cup_Get_Ball_Dist(void)
{
	return Vision_Cup_Target_Distance;
}




/*=============Ӧ�ò�ص�����=============================*/

uint8_t test_pick_char;
volatile uint8_t pick_rcv_data[12];
volatile uint8_t pick_rcv_buffer[12];
volatile uint8_t pick_rcv_flag=0;
volatile uint8_t pick_rcv_char;
static void Vision_Pick_Isr(void)
{
	//�ж��Ƿ�Ϊ�����ֽ��ж�
	if(RCS_USART_Judge_RcvCplt_ITPendingBit(VISION_PICK_USART_MAP.USARTx))
	{
		//��ȡ�����ֽ�
		pick_rcv_char = (uint8_t)USART_ReceiveData(VISION_PICK_USART_MAP.USARTx);
		test_pick_char=pick_rcv_char;
		//����������·�㴦����
		Protocol_Rcv_Easy(pick_rcv_char,pick_rcv_buffer,&pick_rcv_flag,pick_rcv_data,&Vision_Pick_Protocol);
		//����жϱ�־λ
		RCS_USART_Clear_RcvCplt_ITPendingBit(VISION_PICK_USART_MAP.USARTx);
	}
}

uint8_t test_cup_char;
volatile uint8_t cup_rcv_data[12];
volatile uint8_t cup_rcv_buffer[12];
volatile uint8_t cup_rcv_flag=0;
volatile uint8_t cup_rcv_char;
static void Vision_Cup_Isr(void)
{
	//�ж��Ƿ�Ϊ�����ֽ��ж�
	if(RCS_USART_Judge_RcvCplt_ITPendingBit(VISION_CUP_USART_MAP.USARTx))
	{
		//��ȡ�����ֽ�
		cup_rcv_char = (uint8_t)USART_ReceiveData(VISION_CUP_USART_MAP.USARTx);
		test_cup_char=cup_rcv_char;
		//����������·�㴦����
		Protocol_Rcv_Easy(cup_rcv_char,cup_rcv_buffer,&cup_rcv_flag,cup_rcv_data,&Vision_Cup_Protocol);
		//����жϱ�־λ
		RCS_USART_Clear_RcvCplt_ITPendingBit(VISION_CUP_USART_MAP.USARTx);
	}
}

static void Vision_Dist_Isr(void)
{
	static volatile uint8_t rcv_data[12];
	static volatile uint8_t rcv_buffer[12];
	static volatile uint8_t rcv_flag=0;
	static volatile uint8_t rcv_char;
	
	//�ж��Ƿ�Ϊ�����ֽ��ж�
	if(RCS_USART_Judge_RcvCplt_ITPendingBit(VISION_DIST_USART_MAP.USARTx))
	{
		//��ȡ�����ֽ�
		rcv_char = (uint8_t)USART_ReceiveData(VISION_DIST_USART_MAP.USARTx);
		//����������·�㴦����
		Protocol_Rcv_Easy(rcv_char,rcv_buffer,&rcv_flag,rcv_data,&Vision_Dist_Protocol);
		//����жϱ�־λ
		RCS_USART_Clear_RcvCplt_ITPendingBit(VISION_DIST_USART_MAP.USARTx);
	}
}

/*=============Ӧ�ò�ص�����=============================*/

/**
 @name:Vision_Get
 @brief:��������·����ɱ��ĵĽ��պ��Զ����ô˻ص�������������
 @tips:�ú�����ͨѶЭ���ʼ��ʱ������Ϊ����ָ�����ý���������·�㣬��˻��Զ�����
 @param:rcv_cplt_arr ������ɺ������ָ��
**/
void Vision_Pick_Get(uint8_t* rcv_cplt_arr)
{
	Vision_Ball_DescPos.x=((uint16_t)rcv_cplt_arr[0]<<8)|((uint16_t)rcv_cplt_arr[1]);
	Vision_Ball_DescPos.y=((uint16_t)rcv_cplt_arr[2]<<8)|((uint16_t)rcv_cplt_arr[3]);
}

void Vision_Cup_Get(uint8_t* rcv_cplt_arr)
{
	Vision_Cup_Target_DescPos.x=((uint16_t)rcv_cplt_arr[0]<<8)|((uint16_t)rcv_cplt_arr[1]);
	Vision_Cup_Target_DescPos.y=((uint16_t)rcv_cplt_arr[2]<<8)|((uint16_t)rcv_cplt_arr[3]);
	Vision_Cup_Target_Distance=((uint16_t)rcv_cplt_arr[4]<<8)|((uint16_t)rcv_cplt_arr[5]);
}

void Vision_Dist_Get(uint8_t* rcv_cplt_arr)
{
	
}


