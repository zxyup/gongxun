/* ===========头文件=============================================*/
#include "MainTask.h"
#include "rcs.h"
#include "R2_CombCtrl.h"
#include "Ch_Ctrl.h"
#include "Valve_2006.h"

/* ============三级引导程序任务===================================*/
void MainTask(void *p_arg)
{
	/*------创建任务前的准备--------------------*/
	#if(OS_TASK_STAT_EN > 0)
    OSStatInit(); //重新评估CPU使用率
	#endif
	
	/*------全局初始化--------------------------*/
	MainTask_AllInit();
	
	/*------创建各个任务------------------------*/
	UpCtrlTask_Init();    //主要任务
	ChassisTask_Init();   //次要任务
	GPS_Init();           //定位解算任务
	BleTask_Init();       //调试任务(优先级最低)
	
	RCS_Shell_Logs(&Core407_RCSLIB_Debug,"MainTask.c:All Task Created");
	
	/*------挂起当前任务,由RTOS调度其余任务------*/
	OSTaskSuspend(MainTask_PRIO);
	while(1)
	{
		OSTimeDly(1);
	}
}
/* ============三级引导程序初始化===================================*/
void MainTask_AllInit(void) 
{
	/*-------通用功能初始化-------------------------*/
	RCS_Core407_PinMap_Init();        //为主控板的管脚映射赋值(无需改动)
	RCS_Hard_Stamper_Init(HARD_STAMPER_TIMER);
	
	/*-------机器人输出初始化-----------------------*/
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
	/*-------机器人输入初始化-----------------------*/
	//Vision_Init(USART2_MAP,USART1_MAP,USART4_MAP);
	//Gyro_Init(USART3_MAP);
	//G431GPS_Init(USART6_MAP,0x12);
	myServo_Init();
	//ChCtrl_Init_DT35();
	//BlueTooth_Init(USART5_MAP);
	//RCS_JustFloat_Init(USART1_MAP);
	
//	Set_GPS_X(1000.0f);
//	Set_GPS_Y(8900.0f);
	/*-------等待蓝牙连接--------------------------*/
	delay_ms(200);
	//RCS_Shell_Logs(&Core407_RCSLIB_Debug,"MainTask.c: Config Finished");
	RCS_Hard_Stamper_enable();
}
