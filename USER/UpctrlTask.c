/* ===========头文件==============================================*/
#include "UpctrlTask.h"
#include "rcs.h"

#include "Tester.h"
#include "RCS_dsp.h"
#include "RCS_Scara.h"

#include "R1_UpCtrl_Motion.h"
#include "R2_CombCtrl.h"
#include "Ch_Ctrl.h"

#include "Valve_2006.h"
/* ============私有全局变量========================================*/
extern Motor_Ctrl_Node Tray;
extern Motor_Ctrl_Node Arm;
extern Motor_Ctrl_Node Claw;
extern Motor_Ctrl_Node Yuntai;
extern PWM_Device_t servo270;
OS_STK   upctrl_task_stk[NORMAL_TASK_STK_SIZE];
volatile uint16_t ctrl_time_us;
volatile float    motor_monitor[15];
volatile float    robot_monitor[15];
volatile uint16_t robot_bmonitor[15];
float    test_spd;
int pick_test;
int act_test = 0;
int Armangle = 0;
int Clawangle = 0;
int Yuntaiangle = 0;
int Trayangle = 0;
/* ============用户全局变量========================================*/
R2_FSM_Var_t glb_var;
volatile uint8_t blink_var;
uint8_t window_pan=VISION_WINDOW_SIZE;

uint8_t agv_test_state;
uint8_t finish_flag;


/* ============机器人主任务========================================*/
void upctrl_task(void *p_arg)
{
	p_arg = p_arg;
	while(1)
	{
		/*----------更新调试变量----------------------------------*/
		RCS_Hard_Stamper_Reset();
		RCS_Hard_Stamper_enable();
//		motor_monitor[1]=Get_Motor_Float_Angle2(1);
//		motor_monitor[2]=Get_Motor_Float_Angle2(2);
//		motor_monitor[3]=R2_Scara_Get_SPos().joint_angle_main*RAD2DEG;//Get_Motor_Rad_Angle2_M3508(3);
//		motor_monitor[4]=R2_Scara_Get_SPos().joint_angle_end*RAD2DEG;//Get_Motor_Rad_Angle2_M3508(4);
//		motor_monitor[5]=Get_Motor_Float_Angle2(5);
//		motor_monitor[6]=Get_Motor_Float_Angle2(6);
//		motor_monitor[11]=Get_Motor_Float_Angle(1);
//		motor_monitor[12]=Get_Motor_Float_Angle(2);
//		motor_monitor[13]=Get_Motor_Float_Angle(3);
//		motor_monitor[14]=Vision_Cup_Get_Ball_Pos_X();
//		
//		robot_bmonitor[0]=RCS_GPIO_Read(GPIOC,GPIO_Pin_4);
//		robot_bmonitor[1]=RCS_GPIO_Read(GPIOC,GPIO_Pin_5);
//		robot_bmonitor[2]=RCS_GPIO_Read(GPIOB,GPIO_Pin_0);
//		robot_bmonitor[3]=RCS_GPIO_Read(GPIOB,GPIO_Pin_1);
//		robot_bmonitor[4]=RCS_GPIO_Read(GPIOC,GPIO_Pin_0);
//		robot_bmonitor[5]=RCS_GPIO_Read(GPIOC,GPIO_Pin_1);
//		robot_bmonitor[6]=RCS_GPIO_Read(GPIOC,GPIO_Pin_2);
//		robot_bmonitor[7]=RCS_GPIO_Read(GPIOC,GPIO_Pin_3);
		ChCtrl_Laser_Main();
		Trayangle = MotorNode_Get_Angle(&Tray);
		Armangle = MotorNode_Get_Angle(&Arm);
		Clawangle = MotorNode_Get_Angle(&Claw);
		Yuntaiangle = MotorNode_Get_Angle(&Yuntai);
		
		/*----------更新机器人控制指令----------------------------*/

//		if (stop_key)
//		{
//				
//			if (switch_key)
//			{
//				CombCtrl_JoyStick();
//				
//				agv_test_state=0;
//				if(key_flag[0]) AGV_Init_Angle();
//				//if(key_flag[1]){Set_GPS_X(-1000.0f);Set_GPS_Y(8900.0f);}
//			}
//			else
//			{
//				//R2_Comm_Action(7500.0f,-200.0f,0,0);
//				//HAL_Chassis_Ctrl(0.0f,0.0f,-45.0f,AGV_CTRL_TYPE_POS);
//				Block_Pick_Test(ZONE_RED);
//				//SZ_to_Block4(ZONE_RED, 2);
//				//UpCh_Pick_BlockBall(2);
//			}
//		}
		
//		
//		switch(0)
//		{
//			case 0:
//			{
				//Claw_open(&Claw);
				//Arm_up(&Arm);
				//Yuntai_back(&Yuntai);
//				if(Count_Delay(200,3))
//				{
//					act_test++;
//				}
////				if( fabs(Get_UpCtrl_angle(CLAW_CAN_ID)-1280.0f) < 5.0f)
////				{
////					act_test++;
////				}
//			}break;
////			case 1:
////			{
//				Servo_Output(&servo270,0.05);
//				//Stop(&Arm);
////				Arm_up(&Arm);
////				Claw_close(&Claw);
//				if(Count_Delay(200,3))
//				{
//					act_test--;
//				}
//			}break;
			//gongxun();
		//	Motor_Send_ADD(0, 1000, 0, 0);
//			if(Tray_Reset(&Tray))
//				act_test++;
			
//			break;
//			case 1:
//				if(Count_Delay(200,90))
//						act_test++;
//			break;	
//			case 2:
//				//if(Tray_Set(&Tray))
//				//act_test ++;
//			break;	
//			case 3:
//				if(Count_Delay(200,90))
//						act_test=0;
//			break;
		
		test2();
		//AGV_Init_Angle();
		//SZ_to_Block1(ZONE_BLUE,1);
		//Pick_BlockBall(1);
		//R2_Scara_Action(0.0f,-90.0f,2);
		/*----------下发机器人控制指令----------------------------*/
		//Motor_Send2(0,0,600,0);
		MotorList_Excute(CAN_GROUP_1);//将电机控制指令下发给电调
		MotorList_Excute(CAN_GROUP_2);

		/*-----------200Hz控制帧率-------------------------------*/
		RCS_Hard_Stamper_Disable();
		if (blink_var==0){GPIO_SetBits(GPIOD,GPIO_Pin_3);}
		if (blink_var==1){GPIO_ResetBits(GPIOD,GPIO_Pin_3);}
		if (Count_Delay(200,1)) 
		{
			if (blink_var==0){blink_var=1;}
			else if (blink_var==1){blink_var=0;}
		}
		ctrl_time_us=Hard_Get_RealTime_Stamp();
		delay_ms(UPCTRL_TASK_CYCLE_MS);//如无必要不应改动
		
	}
}

/* =============主任务初始化======================================*/
void UpCtrlTask_Init()
{
	/*-----自定义初始化部分--------------------------------------*/
	
	
	M_BaseMove_Init();
	robot_monitor[0]=350.0f;
	//Tester_Add_RM3508(CAN_GROUP_1,3);

	/*-----RTOS任务初始化部分-------------------------------------*/
	OSTaskCreate(upctrl_task,
	           (void *)0,
	           &upctrl_task_stk[NORMAL_TASK_STK_SIZE - 1],
	           upctrl_task_PRIO);
}



