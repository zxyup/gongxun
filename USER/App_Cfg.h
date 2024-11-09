/**
 @brief:��������Ķ�ջ��С�Լ����ȼ�
 @tips:uC/OS-II��֧����ͬ���ȼ����������ȼ�����ԽСԽ���ȡ�
**/
#ifndef  _APP_CFG_H_
#define  _APP_CFG_H_

#include "UpctrlTask.h"
#include "BleTask.h"
#include "ChassisTask.h"
/*******************�����������ȼ�*******************/
#define STARTUP_TASK_PRIO		10
#define MainTask_PRIO			  11
#define chassis_task_PRIO		5//4
#define bluetooth_task_PRIO 8
#define upctrl_task_PRIO		4//5
#define my_gps_task_PRIO		7
/************����ջ��С����λΪ OS_STK ��************/
#define STARTUP_TASK_STK_SIZE	256   
#define NORMAL_TASK_STK_SIZE	512
/************��������Ƶ��****************************/
#define UPCTRL_TASK_CYCLE_MS    5
#define BLE_TASK_CYCLE_MS       5
#endif

