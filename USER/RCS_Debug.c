/**
 * @filename:RCS_Debug.c
 * @brief:���ն���λ��(MobaXterm)�����ڲ�����λ��(VOFA)����
 * @tips:ֻ����BleTask��ʹ��Debug��غ���,�����Ӱ�����������ʵʱ��
 * @contribute:���Ͽ� @2024-2-2
 * @changlog:  ���Ͽ� @2024-2-20 ����˴�log�汾�ĵ��Կ���̨ 
*/
#include "RCS_Debug.h"
#include "RCS_ANSI.h"

/* ==== ������ȫ�ֱ��� ==================================*/
const char RCS_SHELL_ASCII_DRAW[8][50]={
" _____   _____  _____    _____ _          _ _", 
"|  __ \\ / ____|/ ____|  / ____| |        | | |",
"| |__) | |    | (___   | (___ | |__   ___| | |",
"|  _  /| |     \\___ \\   \\___ \\| '_ \\ / _ \\ | |",
"| | \\ \\| |____ ____) |  ____) | | | |  __/ | |",
"|_|  \\_\\\\_____|_____/  |_____/|_| |_|\\___|_|_|"
};//https://patorjk.com/software/taag/#p=display&v=3&f=Big&t=RCS%20Shell
RCS_Shell_Outs RCSLIB_Debug_View;
USART_TypeDef* shell_usart;
USART_TypeDef* vofa_usart;

/* ==== ��̬�������� ====================================*/
static void RCS_Shell_Init_USART(RCS_PIN_USART USARTx_MAP);
static void RCS_JustFloat_Init_USART(RCS_PIN_USART USARTx_MAP);
static void RCS_Shell_Process_Cmd(void);
static void RCS_Shell_Print_Logo(void);

/* ==== ��������  =======================================*/

/**
 * @name:RCS_Shell_Init
 * @brief:ָ��һ������Ϊ�����ն������,��ʼ����ɺ�չʾLogo
 * @param:USARTx_MAP ���ذ��ϵĵڼ���USART��
 * @param:Shell_Type �ն˷��,SHELL_TYPE_LOG_WATCH������,SHELL_TYPE_LOG����
 * @addtogroup:�����ն�
*/
void RCS_Shell_Init(RCS_Shell_Outs* debug_view,RCS_PIN_USART USARTx_MAP,uint8_t Shell_Type)
{
	//��ʼ��USART����
	RCS_Shell_Init_USART(USARTx_MAP);
	//����printf�ض����ĸ�����
	shell_usart=USARTx_MAP.USARTx;
	//Shell��ʼ��
	debug_view->logs_len=0;
	debug_view->points_len=0;
	debug_view->shell_style=Shell_Type;
	ANSI_CLEAR_ALL();
	ANSI_MOVE_CURSOR(0,0);
	RCS_Shell_Print_Logo();
}

/**
 * @name:RCS_JustFloat_Init
 * @brief:ָ��һ������Ϊ���ڲ��������
 * @param:USARTx_MAP ���ذ��ϵĵڼ���USART��
 * @addtogroup:���ڲ���
*/
void RCS_JustFloat_Init(RCS_PIN_USART USARTx_MAP)
{
	vofa_usart=USARTx_MAP.USARTx;
	RCS_JustFloat_Init_USART(USARTx_MAP);
	ANSI_Set_OutPort(USARTx_MAP.USARTx);
}

/**
 * @name:RCS_Shell_Add_Watch_Points
 * @brief:����ն˹۲����ֵ
 * @param:Watch_Name ������۲�ֵȡһ������
 * @param:value �۲�ֵ���ڵĵ�ַ,��keil��������,Ҫע��ʹ����ȷ����ַ�ı������˴���֧��int����
 * @addtogroup:�����ն�
*/
void RCS_Shell_Add_Watch_Points(RCS_Shell_Outs* debug_view,char* Watch_Name,int* value)
{
	debug_view->points_len++;
	if (debug_view->points_len>=RCS_SHELL_WATCH_MAX_LEN) debug_view->points_len=0;
	
	debug_view->points_name[debug_view->points_len]=Watch_Name;
	debug_view->watch_points[debug_view->points_len]=value;
}

/**
 * @name:RCS_Shell_Logs
 * @brief:���ն����һ����־,���ڹ۲��������״̬
 * @param:Log_String,��־������
 * @addtogroup:�����ն�
*/
void RCS_Shell_Logs(RCS_Shell_Outs* debug_view,char* Log_String)
{
	//���۴�ӡ������־,���ǰ���־�������
	debug_view->logs_len++;
	if (debug_view->logs_len>=RCS_SHELL_LOGS_MAX_LEN) debug_view->logs_len=0;
	debug_view->watch_logs[debug_view->logs_len]=Log_String;
	//������Shell,ֱ�Ӵ�ӡ
	if (debug_view->shell_style == SHELL_TYPE_LOG)
	{
		printf(Log_String);
		printf("\n\r");
	}
}

/**
 * @name:RCS_Shell_Main
 * @brief:�ն�������,�����BleTask��
 * @addtogroup:�����ն�
*/
void RCS_Shell_Main(RCS_Shell_Outs* debug_view)
{
	//---------------------WATCH-----------------------------------------
	if ((debug_view->shell_style == SHELL_TYPE_LOG_WATCH)||(debug_view->shell_style == SHELL_TYPE_WATCH))
	{
		//�Ű�
		ANSI_MOVE_CURSOR(8,1);
		ANSI_CLEAR_CURSOR();   
		ANSI_CHANGE_COLOR(ANSI_SHADE_NONE,ANSI_COLOR_GREEN);
		printf("Watch Points\n\r");
		ANSI_CHANGE_COLOR(ANSI_SHADE_NONE,ANSI_COLOR_WHITE);
		//�۲���ӡ
		for(int i=1;i<=debug_view->points_len;i++)
		{
			printf("%s",debug_view->points_name[i]);
			ANSI_MOVE_CURSOR(8+i,25);
			printf("%d\n\r",*(debug_view->watch_points[i]));
		}
	}
	//---------------------LOG-----------------------------------------
	if (debug_view->shell_style == SHELL_TYPE_LOG_WATCH)
	{
		//�Ű�
		ANSI_MOVE_CURSOR(8+RCS_SHELL_WATCH_MAX_LEN+4,1);
		ANSI_CHANGE_COLOR(ANSI_SHADE_NONE,ANSI_COLOR_RED);
		printf("Logs\n\r");
		ANSI_CHANGE_COLOR(ANSI_SHADE_NONE,ANSI_COLOR_WHITE);	
		//��־��ӡ	
		for(int i=1;i<=debug_view->logs_len;i++)
		{
			printf("%s",debug_view->watch_logs[i]);
			printf("\n\r");
		}
	}
	else if (debug_view->shell_style == SHELL_TYPE_LOG)
	{
		return;//�����������ƹ��λ��
	}
	else if (debug_view->shell_style == SHELL_TYPE_WATCH)
	{
		return;
	}
}

/**
 * @name:JustFloat_Printf
 * @brief:��JustFloat��ʽ��VOFA��λ�����Ͳ�������
 * @param:fdata һ����������,���ŵ�ɸ�ͨ���Ĳ���
 * @param:channel_count ͨ������
 * @param:USARTx_MAP ���ذ��ϵĵڼ���USART��
 * @addtogroup:���ڲ���
*/
void JustFloat_Printf(float* fdata,uint16_t channel_count,RCS_PIN_USART USARTx_MAP)
{
	ANSI_Set_OutPort(USARTx_MAP.USARTx);
	DataType_32Bit output;
	uint8_t c_data[4];
	for(int i=0;i<channel_count;i++)
	{
		output.fdata=fdata[i];
		Bit32_Split_BigEndian(output,c_data);
		RCS_USART_Send_Char(USARTx_MAP.USARTx,c_data[3]);
		RCS_USART_Send_Char(USARTx_MAP.USARTx,c_data[2]);
		RCS_USART_Send_Char(USARTx_MAP.USARTx,c_data[1]);
		RCS_USART_Send_Char(USARTx_MAP.USARTx,c_data[0]);
	}
	RCS_USART_Send_Char(USARTx_MAP.USARTx,0x00);
	RCS_USART_Send_Char(USARTx_MAP.USARTx,0x00);
	RCS_USART_Send_Char(USARTx_MAP.USARTx,0x80);
	RCS_USART_Send_Char(USARTx_MAP.USARTx,0x7f);
}

/* === ��̬����ʵ�� ==================================*/

//��ʼ����������
static void RCS_Shell_Init_USART(RCS_PIN_USART USARTx_MAP)
{
	RCS_USART_With_NoIsr_Config(USARTx_MAP.USARTx,USARTx_MAP.GPIOx,USARTx_MAP.GPIO_Pin_Tx,USARTx_MAP.GPIO_Pin_Rx,DEBUG_BAUD);
}
static void RCS_JustFloat_Init_USART(RCS_PIN_USART USARTx_MAP)
{
	RCS_USART_With_NoIsr_Config(USARTx_MAP.USARTx,USARTx_MAP.GPIOx,USARTx_MAP.GPIO_Pin_Tx,USARTx_MAP.GPIO_Pin_Rx,DEBUG_BAUD);
}

static void RCS_Shell_Print_Logo(void)
{
	printf("\033[34m");//blue
	printf("\033[1m\n\r");//width
	printf("%s\n\r",RCS_SHELL_ASCII_DRAW[0]);
	printf("%s\n\r",RCS_SHELL_ASCII_DRAW[1]);
	printf("%s\n\r",RCS_SHELL_ASCII_DRAW[2]);
	printf("%s\n\r",RCS_SHELL_ASCII_DRAW[3]);
	printf("%s\n\r",RCS_SHELL_ASCII_DRAW[4]);
	printf("%s\n\r",RCS_SHELL_ASCII_DRAW[5]);
	ANSI_CHANGE_COLOR(ANSI_SHADE_DARK,ANSI_COLOR_WHITE);
}


//todo:�����ն�����
static void RCS_Shell_Process_Cmd(void)
{
	
}



//�����ն���Ӧ�󣬶�scanf���ض���
#ifdef RCS_SHELL_PROCESS_CMD
	int fgetc(FILE *stream)
	{
		while(!(shell_usart->SR & (1 << 5))){};//�ȴ����ݽ������
		return shell_usart->DR;
	}
#endif 

