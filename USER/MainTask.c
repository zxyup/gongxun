/* ===========ͷ�ļ�=============================================*/
#include "MainTask.h"
#include "rcs.h"
#include "R2_CombCtrl.h"
#include "Ch_Ctrl.h"
#include "Valve_2006.h"

/* ============����������������===================================*/
void MainTask(void *p_arg)
{
	/*------��������ǰ��׼��--------------------*/
	#if(OS_TASK_STAT_EN > 0)
    OSStatInit(); //��������CPUʹ����
	#endif
	
	/*------ȫ�ֳ�ʼ��--------------------------*/
	MainTask_AllInit();
	
	/*------������������------------------------*/
	UpCtrlTask_Init();    //��Ҫ����
	ChassisTask_Init();   //��Ҫ����
	GPS_Init();           //��λ��������
	BleTask_Init();       //��������(���ȼ����)
	
	RCS_Shell_Logs(&Core407_RCSLIB_Debug,"MainTask.c:All Task Created");
	
	/*------����ǰ����,��RTOS������������------*/
	OSTaskSuspend(MainTask_PRIO);
	while(1)
	{
		OSTimeDly(1);
	}
}
/* ============�������������ʼ��===================================*/
void MainTask_AllInit(void) 
{
	/*-------ͨ�ù��ܳ�ʼ��-------------------------*/
	RCS_Core407_PinMap_Init();        //Ϊ���ذ�Ĺܽ�ӳ�丳ֵ(����Ķ�)
	RCS_Hard_Stamper_Init(HARD_STAMPER_TIMER);
	
	/*-------�����������ʼ��-----------------------*/
//	RCS_GPIO_Output_Init(GPIOE,2);
//	RCS_GPIO_Output_Init(GPIOE,3);
//	RCS_GPIO_Output_Init(GPIOE,4);
//	RCS_GPIO_Output_Init(GPIOE,5);
//	
//	RCS_GPIO_Output_Init(GPIOE,12);
//	RCS_GPIO_Output_Init(GPIOE,10);
//	RCS_GPIO_Output_Init(GPIOE,8);
//	RCS_GPIO_Output_Init(GPIOE,7);
//	
//	RCS_GPIO_Output_Init(GPIOC,4);
//	RCS_GPIO_Output_Init(GPIOC,5);
//	RCS_GPIO_Output_Init(GPIOB,0);
//	RCS_GPIO_Output_Init(GPIOB,1);
//    
//	RCS_GPIO_Output_Init(GPIOD,GPIO_Pin_3);
	
	R2_Upctrl_Init(USART4_MAP);
	CombCtrl_Param_Init(USART4_MAP);
//	AGV_Init();
	//ChCtrl_Init_DT35();
	/*-------�����������ʼ��-----------------------*/
	//Vision_Init(USART2_MAP,USART1_MAP,USART4_MAP);
	//Gyro_Init(USART3_MAP);
	//G431GPS_Init(USART6_MAP,0x12);
	myServo_Init();
	//ChCtrl_Init_DT35();
	//BlueTooth_Init(USART5_MAP);
	//RCS_JustFloat_Init(USART1_MAP);
	
//	Set_GPS_X(1000.0f);
//	Set_GPS_Y(8900.0f);
	/*-------�ȴ���������--------------------------*/
	delay_ms(200);
	//RCS_Shell_Logs(&Core407_RCSLIB_Debug,"MainTask.c: Config Finished");
	RCS_Hard_Stamper_enable();
}
