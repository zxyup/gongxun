/* ===========ͷ�ļ�==============================================*/
#include "rcs.h"

/* ============˽��ȫ�ֱ���========================================*/
static OS_STK startup_task_stk[STARTUP_TASK_STK_SIZE];
static OS_STK MainTask_stk[NORMAL_TASK_STK_SIZE];

/* ============��̬����===========================================*/
static void startup_task(void *p_arg);//�������������


/* ============һ����������=======================================*/
int main(void)
{
	/*--------����Ӳ����������------------------------------*/
	//�ں�����
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//Cortex-M4�жϳ�ʼ��
	delay_init(168);  	                           //����ʱ��Ϊ168MHz
	//RTOS����
	BSP_Init();                                    //�Խ�Ӳ�����RTOS��
	OSInit();                                      //RTOS��ʼ��

	/*--------���������������------------------------------*/
	OSTaskCreate(startup_task,//������������
	             (void *)0,
	             &startup_task_stk[STARTUP_TASK_STK_SIZE - 1],
	             STARTUP_TASK_PRIO);
	OSStart();//����RTOS����
	return 0;
}

/* ============������������=======================================*/
static void startup_task(void *p_arg)
{
	/*--------����������������----------------------------*/
	#if(OS_TASK_STAT_EN > 0)
    OSStatInit(); //��������CPUʹ����,׼����������
	#endif
	
	OSTaskCreate(MainTask,//����MainTask������Ϊ������������
	           (void *)0,
	           &MainTask_stk[NORMAL_TASK_STK_SIZE - 1],
	           MainTask_PRIO);

	/*------����ǰ����,��RTOS����������������------------*/
	OSTaskSuspend(STARTUP_TASK_PRIO);
	while (1)
	{
		OSTimeDly(1000);
	}
	
}
