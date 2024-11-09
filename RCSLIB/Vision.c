/**
 @name:Vision.c
 @brief:视觉串口的应用层驱动
**/

/*==============头文件====================================*/
#include "Vision.h"

/*=============私有全局变量===============================*/

//捡球串口
RCS_PIN_USART VISION_PICK_USART_MAP;
RCS_Easy_Ptl  Vision_Pick_Protocol;
#define       PICK_USART_ISR_PRI   0x02
#define       PICK_USART_PKG_LEN   4

//奥比中光摄像头串口
RCS_PIN_USART VISION_CUP_USART_MAP;
RCS_Easy_Ptl  Vision_Cup_Protocol;
#define       CUP_USART_ISR_PRI   0x03
#define       CUP_USART_PKG_LEN   10

//因特尔深度相机串口
RCS_PIN_USART VISION_DIST_USART_MAP;
RCS_Easy_Ptl  Vision_Dist_Protocol;
#define       DIST_USART_ISR_PRI   0x01
#define       DIST_USART_PKG_LEN   10






/*=============用户全局变量===============================*/
DESCARTES_AXIS Vision_Ball_DescPos;
int            Vision_Ball_Count;
DESCARTES_AXIS Vision_Cup_Target_DescPos;
int            Vision_Cup_Target_Distance;

/*=============回调函数声明===============================*/

//捡球相机回调
static void Vision_Pick_Isr(void);
void Vision_Pick_Get(uint8_t* rcv_cplt_arr);

//奥比中光回调
static void Vision_Cup_Isr(void);
void Vision_Cup_Get(uint8_t* rcv_cplt_arr);

//因特尔回调
static void Vision_Dist_Isr(void);
void Vision_Dist_Get(uint8_t* rcv_cplt_arr);

/*=============应用层接口函数=============================*/

/**
 @addtogroup: 与视觉通信初始化
**/
void Vision_Init(RCS_PIN_USART Pick_USARTx_MAP,RCS_PIN_USART Cup_USARTx_MAP,RCS_PIN_USART Dist_USARTx_MAP)
{
	//========================USB Camera======================================================
	//数据链路层协议初始化
	Vision_Pick_Protocol.Start_Byte=0xBB;
	Vision_Pick_Protocol.End_Byte=0xBC;
	Vision_Pick_Protocol.Len=4;
	Vision_Pick_Protocol.Finish_Callback=Vision_Pick_Get;
	//物理层初始化
	VISION_PICK_USART_MAP=Pick_USARTx_MAP;                  //指定串口
	RCS_USART_Config(Pick_USARTx_MAP.USARTx, Pick_USARTx_MAP.GPIOx,Pick_USARTx_MAP.GPIO_Pin_Tx,Pick_USARTx_MAP.GPIO_Pin_Rx,
		             Vision_Pick_Isr ,                 //接收中断回调
		             VISION_BAUD,                      //波特率
		             PICK_USART_ISR_PRI);              //接收中断优先级

	//========================307 Camera========================================================
	//数据链路层协议初始化
	Vision_Cup_Protocol.Start_Byte=0xFA;
	Vision_Cup_Protocol.End_Byte=0xFF;
	Vision_Cup_Protocol.Len=6;
	Vision_Cup_Protocol.Finish_Callback=Vision_Cup_Get;
	//物理层初始化
	VISION_CUP_USART_MAP=Cup_USARTx_MAP;                  //指定串口
	RCS_USART_Config(Cup_USARTx_MAP.USARTx, Cup_USARTx_MAP.GPIOx,Cup_USARTx_MAP.GPIO_Pin_Tx,Cup_USARTx_MAP.GPIO_Pin_Rx,
		             Vision_Cup_Isr ,                 //接收中断回调
		             VISION_BAUD,                      //波特率
		             CUP_USART_ISR_PRI);              //接收中断优先级
	
	//=======================Intel D455 Camera==================================================
	//数据链路层协议初始化
	
	//物理层初始化
	VISION_DIST_USART_MAP=Dist_USARTx_MAP;                  //指定串口
	RCS_USART_Config(Dist_USARTx_MAP.USARTx, Dist_USARTx_MAP.GPIOx,Dist_USARTx_MAP.GPIO_Pin_Tx,Dist_USARTx_MAP.GPIO_Pin_Rx,
		             Vision_Dist_Isr ,                 //接收中断回调
		             VISION_BAUD,                      //波特率
		             PICK_USART_ISR_PRI);              //接收中断优先级

	//===================等待串口外设准备就绪===================================================
	delay_ms(50);
}

/**
 @name: Vision_Get_Ball_Count
 @brief: 获取视野中球的数量
**/
int Vision_Get_Ball_Count(void)
{
	return Vision_Ball_Count;
}
/**
 @name: Vision_Get_Ball_Pos
 @brief: 获取视野中最近一颗球的坐标
**/
DESCARTES_AXIS Vision_Get_Ball_Pos(void)
{
	return Vision_Ball_DescPos;
}
/**
 @name: Vision_Get_Ball_Pos
 @brief: 获取奥比中光相机中目标球的X位置
**/
int Vision_Cup_Get_Ball_Pos_X(void)
{
	return Vision_Cup_Target_DescPos.x;
}
/**
 @name: Vision_Get_Ball_Pos
 @brief: 获取奥比中光相机中目标球的Y位置
**/
int Vision_Cup_Get_Ball_Pos_Y(void)
{
	return Vision_Cup_Target_DescPos.y;
}
/**
 @name: Vision_Get_Ball_Pos
 @brief: 获取奥比中光相机中目标球的距离，超过范围是0
**/
int Vision_Cup_Get_Ball_Dist(void)
{
	return Vision_Cup_Target_Distance;
}




/*=============应用层回调函数=============================*/

uint8_t test_pick_char;
volatile uint8_t pick_rcv_data[12];
volatile uint8_t pick_rcv_buffer[12];
volatile uint8_t pick_rcv_flag=0;
volatile uint8_t pick_rcv_char;
static void Vision_Pick_Isr(void)
{
	//判断是否为接收字节中断
	if(RCS_USART_Judge_RcvCplt_ITPendingBit(VISION_PICK_USART_MAP.USARTx))
	{
		//获取接收字节
		pick_rcv_char = (uint8_t)USART_ReceiveData(VISION_PICK_USART_MAP.USARTx);
		test_pick_char=pick_rcv_char;
		//交由数据链路层处理报文
		Protocol_Rcv_Easy(pick_rcv_char,pick_rcv_buffer,&pick_rcv_flag,pick_rcv_data,&Vision_Pick_Protocol);
		//清除中断标志位
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
	//判断是否为接收字节中断
	if(RCS_USART_Judge_RcvCplt_ITPendingBit(VISION_CUP_USART_MAP.USARTx))
	{
		//获取接收字节
		cup_rcv_char = (uint8_t)USART_ReceiveData(VISION_CUP_USART_MAP.USARTx);
		test_cup_char=cup_rcv_char;
		//交由数据链路层处理报文
		Protocol_Rcv_Easy(cup_rcv_char,cup_rcv_buffer,&cup_rcv_flag,cup_rcv_data,&Vision_Cup_Protocol);
		//清除中断标志位
		RCS_USART_Clear_RcvCplt_ITPendingBit(VISION_CUP_USART_MAP.USARTx);
	}
}

static void Vision_Dist_Isr(void)
{
	static volatile uint8_t rcv_data[12];
	static volatile uint8_t rcv_buffer[12];
	static volatile uint8_t rcv_flag=0;
	static volatile uint8_t rcv_char;
	
	//判断是否为接收字节中断
	if(RCS_USART_Judge_RcvCplt_ITPendingBit(VISION_DIST_USART_MAP.USARTx))
	{
		//获取接收字节
		rcv_char = (uint8_t)USART_ReceiveData(VISION_DIST_USART_MAP.USARTx);
		//交由数据链路层处理报文
		Protocol_Rcv_Easy(rcv_char,rcv_buffer,&rcv_flag,rcv_data,&Vision_Dist_Protocol);
		//清除中断标志位
		RCS_USART_Clear_RcvCplt_ITPendingBit(VISION_DIST_USART_MAP.USARTx);
	}
}

/*=============应用层回调函数=============================*/

/**
 @name:Vision_Get
 @brief:当数据链路层完成报文的接收后，自动调用此回调函数解析报文
 @tips:该函数在通讯协议初始化时，被作为函数指针配置进了数据链路层，因此会自动调用
 @param:rcv_cplt_arr 接收完成后的数组指针
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


