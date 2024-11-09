/*************对象从属关系与命名规则***********************
 @object:R2
   @object:Gantry   龙门架
     @motor:P       向上动角度变大的电机
     @motor:N       向上动角度变小的电机

   @object:Scara    机械臂
     @motor:Main    大臂电机
     @motor:End     小臂电机

   @object:Comm     末端执行机构
     @motor:Actor   吸盘自转电机
     @switch:Punk   吸盘真空泵
     @switch:Swiv   吸盘泄气电磁阀

   @object:Chassis  底盘
*********************************************************/


/* ==============头文件=====================================*/
#include "Up_Ctrl.h"
#include "RCS_Stastic.h"
#include "Valve_2006.h"

/* ==============全局变量===================================*/
Motor_Ctrl_Node R2_Scara_Main;
Motor_Ctrl_Node R2_Scara_End;
Motor_Ctrl_Node R2_Gantry_P;
Motor_Ctrl_Node R2_Gantry_N;
Motor_Ctrl_Node R2_Comm_Actor;
Motor_Ctrl_Node Tray;
Motor_Ctrl_Node Arm;
Motor_Ctrl_Node Yuntai;
Motor_Ctrl_Node Claw;


PID_Struct      R2_Scara_Main_Spd_Pid;
PID_Struct      R2_Scara_End_Spd_Pid;
PID_Struct      R2_Scara_Main_Pos_Pid;
PID_Struct      R2_Scara_End_Pos_Pid;
DacePID_Struct  R2_Scara_Main_Pos_DacePid;
DacePID_Struct  R2_Scara_End_Pos_DacePid;

PID_Struct      R2_Comm_Actor_Spd_Pid;
PID_Struct      R2_Comm_Actor_Ang_Pid;
DacePID_Struct  R2_Comm_Actor_Ang_DacePid;

PID_Struct      R2_Gantry_P_Spd_Pid;
PID_Struct      R2_Gantry_N_Spd_Pid;
DacePID_Struct  R2_Gantry_P_Pos_Pid;
DacePID_Struct  R2_Gantry_N_Pos_Pid; 

PID_Struct      Arm_speed_pid;  //can 2 id3
DacePID_Struct  Arm_angle_pid;

PID_Struct      Yuntai_speed_pid;  // can 1 id 5
DacePID_Struct  Yuntai_angle_pid;

PID_Struct      Claw_speed_pid;  // can 2  id 1
DacePID_Struct  Claw_angle_pid;

PID_Struct      Tray_speed_pid;  //can 1 id 6
DacePID_Struct  Tray_angle_pid;

PWM_Device_t servo270;

SCARA_PARAM     R2_Scara_Param;

float           R2_Actor_Start_Pos=R2_COMM_ACTOR_START_POS;

extern valve_para Valve_data_1;


//日志系统
LogServer_t     Scara_Log;  
LogServer_t     Comm_Log;
LogServer_t     FSM_Log;  

//滑动窗口
Comm_WindowData_t Vision_DataWindow_X;
Comm_WindowData_t Vision_DataWindow_Y;

float R2_Comm_Get_Actor_Pos(uint8_t id);

/*********************************************************************************
                
                @addtogroup:全功能接口

*********************************************************************************/




void myServo_Init()
{
	PWMInit(TIM4,0,GPIOD,GPIO_Pin_13,84000,50);  //GPIO D ,GPIO_Pin_13
	PWMInit(TIM4,2,GPIOD,GPIO_Pin_15,84000,50);
}
void test2()
{
	PWMOutput(TIM4,0,0.05);
	PWMOutput(TIM4,2,0.05);
}

/**

 * @name:R2_Upctrl_Init
 * @brief:上层机构完全初始化
 * @param:USARTx_MAP 调试信息从哪个串口送出
 **/
void R2_Upctrl_Init(RCS_PIN_USART USARTx_MAP)
{
	//------------------软初始化：SCARA参数------------------
//	//R2_Scara_Param=Scara_Param_Init(R2_SCARA_MAIN_LEN,R2_SCARA_END_LEN,
//		                            R2_SCARA_MAIN_SLOWDOWN_RATE,R2_SCARA_END_SLOWDOWN_RATE,
//		                            45.0f,-158.0f);

	//------------------软初始化：日志系统-------------------
	//LogServer_Init(&Scara_Log,USARTx_MAP.USARTx,"Scara\0"); //机械臂相关的日志将通过USARTx_MAP送出，冠以SCARA的名称
	//LogServer_Init(&Comm_Log,USARTx_MAP.USARTx,"Actor\0");  //末端机构相关的日志将通过USARTx_MAP送出，冠以Actor的名称
	//LogServer_Init(&FSM_Log,USARTx_MAP.USARTx,"FSM\0");     //运行逻辑相关的日志将通过USARTx_MAP送出，冠以FSM的名称
	
	//-----------------软初始化：视觉回传数据窗口--------------
	WindowFloat_Init_Slide(&Vision_DataWindow_X,VISION_WINDOW_SIZE);//10次数据为一组(50ms)
	WindowFloat_Init_Slide(&Vision_DataWindow_Y,VISION_WINDOW_SIZE);//10次数据为一组(50ms)

	//------------------软初始化：电机节点------------------

	//Step1(必选):配置电调和电机
	MotorNode_Init_C620(R2_UPCTRL_CAN_GROUP,R2_SCARA_MAIN_CAN_ID,&R2_Scara_Main);
	MotorNode_Init_C620(R2_UPCTRL_CAN_GROUP,R2_SCARA_END_CAN_ID,&R2_Scara_End);
	MotorNode_Init_C620(R2_UPCTRL_CAN_GROUP,R2_SCARA_ACTOR_CAN_ID,&R2_Comm_Actor);
	MotorNode_Init_C620(R2_UPCTRL_CAN_GROUP,R2_GANTRY_P_CAN_ID,&R2_Gantry_P);
	MotorNode_Init_C620(R2_UPCTRL_CAN_GROUP,R2_GANTRY_N_CAN_ID,&R2_Gantry_N);

	//Step2(必选):获取闭环控制参数
	PID_Init();
	R2_Scara_Main_Spd_Pid=PID_Get_RM3508_Speed_Pid();
	R2_Scara_End_Spd_Pid=PID_Get_RM3508_Speed_Pid();
	R2_Comm_Actor_Spd_Pid=PID_Get_RM3508_Speed_Pid();
	R2_Gantry_P_Spd_Pid=PID_Get_RM3508_Speed_Pid();
	R2_Gantry_N_Spd_Pid=PID_Get_RM3508_Speed_Pid();
	
	R2_Comm_Actor_Ang_Pid.P=100.0f;
	R2_Comm_Actor_Ang_Pid.Limit_Output=5000.0f;//1200.0f;
	R2_Comm_Actor_Ang_DacePid.pid.P=3.0f;
	R2_Comm_Actor_Ang_DacePid.pid.Limit_Output=5000.0f;//1200.0f;
	R2_Comm_Actor_Ang_DacePid.dead_zone=60.0f;
	R2_Comm_Actor_Ang_DacePid.max_acce=400.0f;
	R2_Comm_Actor_Ang_DacePid.max_dcce=400.0f;
	

	
	R2_Scara_Main_Pos_Pid.P=200.0f;//400
	R2_Scara_Main_Pos_Pid.Limit_Output=5000.0f;
	R2_Scara_Main_Pos_DacePid.pid.P=400.0f;//600
	R2_Scara_Main_Pos_DacePid.pid.Limit_Output=5000.0f;
	R2_Scara_Main_Pos_DacePid.dead_zone=10.0f*DEG2RAD;
	R2_Scara_Main_Pos_DacePid.max_acce=40.0f;
	R2_Scara_Main_Pos_DacePid.max_dcce=800.0f;
	
	R2_Scara_End_Pos_Pid.P=200.0f;//400
	R2_Scara_End_Pos_Pid.Limit_Output=5000.0f;
	R2_Scara_End_Pos_DacePid.pid.P=400.0f;//600
	R2_Scara_End_Pos_DacePid.pid.Limit_Output=5000.0f;
	R2_Scara_End_Pos_DacePid.dead_zone=10.0f*DEG2RAD;
	R2_Scara_End_Pos_DacePid.max_acce=40.0f;
	R2_Scara_End_Pos_DacePid.max_dcce=800.0f;
	
	
	
	R2_Gantry_P_Pos_Pid.pid.P=30.0f;
	R2_Gantry_P_Pos_Pid.pid.Limit_Output=20000.0f;
	R2_Gantry_P_Pos_Pid.dead_zone=100.0f;
	R2_Gantry_P_Pos_Pid.max_acce=50.0f;
	R2_Gantry_P_Pos_Pid.max_dcce=60.0f;
	
	R2_Gantry_N_Pos_Pid=R2_Gantry_P_Pos_Pid;

	Tray_speed_pid=PID_Get_RM3508_Speed_Pid();
	Tray_angle_pid.pid.P=2.0f;
	Tray_angle_pid.pid.Limit_Output=5000.0f;
	Tray_angle_pid.dead_zone=0.01f;
	Tray_angle_pid.max_acce=4.0f;
	Tray_angle_pid.max_dcce=80.0f;
	
	Yuntai_speed_pid=PID_Get_RM2006_Speed_Pid();
	Yuntai_angle_pid.pid.P=2.0f;
	Yuntai_angle_pid.pid.Limit_Output=5000.0f;
	Yuntai_angle_pid.dead_zone=0.01f;
	Yuntai_angle_pid.max_acce=5.0f;	
	Yuntai_angle_pid.max_dcce=5.0f;
	
	Claw_speed_pid=PID_Get_RM3508_Speed_Pid();
	Claw_angle_pid.pid.P=2.0f;
	Claw_angle_pid.pid.Limit_Output=5000.0f;
	Claw_angle_pid.dead_zone=0.01f;
	Claw_angle_pid.max_acce=4.0f;
	Claw_angle_pid.max_dcce=80.0f;
	
	Arm_speed_pid=PID_Get_RM3508_Speed_Pid();
	Arm_angle_pid.pid.P=65.0f;
	Arm_angle_pid.pid.D=4.8f;
	Arm_angle_pid.pid.Limit_Output=5000.0f;
	Arm_angle_pid.dead_zone=0.01f;
	Arm_angle_pid.max_acce=40.0f;
	Arm_angle_pid.max_dcce=80.0f;
	
	//step3(必选):配置闭环控制参数
	MotorNode_Add_SpeedPid(&R2_Scara_Main,&R2_Scara_Main_Spd_Pid);
	MotorNode_Add_SpeedPid(&R2_Scara_End,&R2_Scara_End_Spd_Pid);
	MotorNode_Add_AnglePid(&R2_Scara_Main,&R2_Scara_Main_Pos_Pid);
	MotorNode_Add_AnglePid(&R2_Scara_End,&R2_Scara_End_Pos_Pid);
	MotorNode_Add_DaceAnglePid(&R2_Scara_Main,&R2_Scara_Main_Pos_DacePid);
	MotorNode_Add_DaceAnglePid(&R2_Scara_End,&R2_Scara_End_Pos_DacePid);
	
	MotorNode_Add_SpeedPid(&R2_Comm_Actor,&R2_Comm_Actor_Spd_Pid);
	MotorNode_Add_SpeedPid(&R2_Gantry_P,&R2_Gantry_P_Spd_Pid);
	MotorNode_Add_SpeedPid(&R2_Gantry_N,&R2_Gantry_N_Spd_Pid);
	
	MotorNode_Add_AnglePid(&R2_Comm_Actor,&R2_Comm_Actor_Ang_Pid);
	MotorNode_Add_DaceAnglePid(&R2_Comm_Actor,&R2_Comm_Actor_Ang_DacePid);
	MotorNode_Add_DaceAnglePid(&R2_Gantry_P,&R2_Gantry_P_Pos_Pid);
	MotorNode_Add_DaceAnglePid(&R2_Gantry_N,&R2_Gantry_N_Pos_Pid);
	//step4(非必选):配置电机保护
	//MotorNode_Add_PosProtect(&R2_Scara_Main,R2_SCARA_END_MAX_LPOS_RAD,R2_SCARA_END_MAX_UPOS_RAD);

	//step5(非必选):修改电机节点的接口
	if (R2_UPCTRL_CAN_GROUP==CAN_GROUP_1)
	{
		R2_Scara_Main.Get_Node_Angle=Get_Motor_Rad_Angle_M3508;//使用输出轴rad角度作为电机节点的角度,方便scara解算
		R2_Scara_End.Get_Node_Angle=Get_Motor_Rad_Angle_M3508;
	}
	else
	{
		R2_Scara_Main.Get_Node_Angle=Get_Motor_Rad_Angle2_M3508;//使用输出轴rad角度作为电机节点的角度,方便scara解算
		R2_Scara_End.Get_Node_Angle=Get_Motor_Rad_Angle2_M3508;
		//R2_Comm_Actor.Get_Node_Angle=R2_Comm_Get_Actor_Pos;
	}

	//step6(必选):将电机节点送入链表
	MotorNode_Add(R2_UPCTRL_CAN_GROUP,&R2_Scara_Main);
	MotorNode_Add(R2_UPCTRL_CAN_GROUP,&R2_Scara_End);
	MotorNode_Add(R2_UPCTRL_CAN_GROUP,&R2_Comm_Actor);
	MotorNode_Add(R2_UPCTRL_CAN_GROUP,&R2_Gantry_P);
	MotorNode_Add(R2_UPCTRL_CAN_GROUP,&R2_Gantry_N);
	Tray_Setup(&Tray,Tray_CAN_GROUP,TRAY_CAN_ID,&Tray_speed_pid,&Tray_angle_pid);
	Arm_Setup(&Arm,CAN_GROUP_2,ARM_CAN_ID,&Arm_speed_pid,&Arm_angle_pid);
	Claw_Setup(&Claw,CAN_GROUP_2,CLAW_CAN_ID,&Claw_speed_pid,&Claw_angle_pid);
	Yuntai_Setup(&Yuntai,CAN_GROUP_1,YUNTAI_CAN_ID,&Yuntai_speed_pid,&Yuntai_angle_pid);
//	Claw_Setup
//	Yuntai_Setup
	//------------------硬初始化：CAN外设------------------
	//Motor_Init();
	Motor_Init2();

	//------------------硬初始化：UART外设------------------
	//日志串口
	RCS_USART_With_NoIsr_Config(USARTx_MAP.USARTx,USARTx_MAP.GPIOx,USARTx_MAP.GPIO_Pin_Tx,USARTx_MAP.GPIO_Pin_Rx,DEBUG_BAUD);
	ANSI_Set_OutPort(USARTx_MAP.USARTx);
	ANSI_CLEAR_ALL();

	//------------------硬初始化：GPIO外设------------------
	RCS_GPIO_Output_Init(R2_SCARA_ACTOR_PUNK_GPIO,R2_SCARA_ACTOR_PUNK_PIN);
	RCS_GPIO_Output_Init(R2_SCARA_ACTOR_SWIV_GPIO,R2_SCARA_ACTOR_SWIV_PIN);
	

	//-----------------发出配置完成的日志--------------------
	LogServer_Collect(&Scara_Log,"E","Upctrl Initialed","NULL");
	LogServer_Collect(&FSM_Log,"E","Upctrl Initialed","NULL");
	LogServer_Collect(&Comm_Log,"E","Upctrl Initialed","NULL");
}

void Shenghsai_Upctrl_Init(RCS_PIN_USART USARTx_MAP)
{
	//------------------软初始化：SCARA参数------------------
	R2_Scara_Param=Scara_Param_Init(R2_SCARA_MAIN_LEN,R2_SCARA_END_LEN,
		                            R2_SCARA_MAIN_SLOWDOWN_RATE,R2_SCARA_END_SLOWDOWN_RATE,
		                            45.0f,-158.0f);

	//------------------软初始化：日志系统-------------------
	LogServer_Init(&Scara_Log,USARTx_MAP.USARTx,"Scara\0"); //机械臂相关的日志将通过USARTx_MAP送出，冠以SCARA的名称
	LogServer_Init(&Comm_Log,USARTx_MAP.USARTx,"Actor\0");  //末端机构相关的日志将通过USARTx_MAP送出，冠以Actor的名称
	LogServer_Init(&FSM_Log,USARTx_MAP.USARTx,"FSM\0");     //运行逻辑相关的日志将通过USARTx_MAP送出，冠以FSM的名称
	
	//-----------------软初始化：视觉回传数据窗口--------------
	WindowFloat_Init_Slide(&Vision_DataWindow_X,VISION_WINDOW_SIZE);//10次数据为一组(50ms)
	WindowFloat_Init_Slide(&Vision_DataWindow_Y,VISION_WINDOW_SIZE);//10次数据为一组(50ms)

	//------------------软初始化：电机节点------------------

	//Step1(必选):配置电调和电机
	MotorNode_Init_C620(R2_UPCTRL_CAN_GROUP,R2_SCARA_MAIN_CAN_ID,&R2_Scara_Main);
	MotorNode_Init_C620(R2_UPCTRL_CAN_GROUP,R2_SCARA_END_CAN_ID,&R2_Scara_End);
	MotorNode_Init_C620(R2_UPCTRL_CAN_GROUP,R2_SCARA_ACTOR_CAN_ID,&R2_Comm_Actor);
	MotorNode_Init_C620(R2_UPCTRL_CAN_GROUP,R2_GANTRY_P_CAN_ID,&R2_Gantry_P);
	MotorNode_Init_C620(R2_UPCTRL_CAN_GROUP,R2_GANTRY_N_CAN_ID,&R2_Gantry_N);

	//Step2(必选):获取闭环控制参数
	PID_Init();
	R2_Scara_Main_Spd_Pid=PID_Get_RM3508_Speed_Pid();
	R2_Scara_End_Spd_Pid=PID_Get_RM3508_Speed_Pid();
	R2_Comm_Actor_Spd_Pid=PID_Get_RM3508_Speed_Pid();
	R2_Gantry_P_Spd_Pid=PID_Get_RM3508_Speed_Pid();
	R2_Gantry_N_Spd_Pid=PID_Get_RM3508_Speed_Pid();
	
	R2_Comm_Actor_Ang_Pid.P=100.0f;
	R2_Comm_Actor_Ang_Pid.Limit_Output=5000.0f;//1200.0f;
	R2_Comm_Actor_Ang_DacePid.pid.P=3.0f;
	R2_Comm_Actor_Ang_DacePid.pid.Limit_Output=5000.0f;//1200.0f;
	R2_Comm_Actor_Ang_DacePid.dead_zone=60.0f;
	R2_Comm_Actor_Ang_DacePid.max_acce=400.0f;
	R2_Comm_Actor_Ang_DacePid.max_dcce=400.0f;
	

	
	R2_Scara_Main_Pos_Pid.P=200.0f;//400
	R2_Scara_Main_Pos_Pid.Limit_Output=5000.0f;
	R2_Scara_Main_Pos_DacePid.pid.P=400.0f;//600
	R2_Scara_Main_Pos_DacePid.pid.Limit_Output=5000.0f;
	R2_Scara_Main_Pos_DacePid.dead_zone=10.0f*DEG2RAD;
	R2_Scara_Main_Pos_DacePid.max_acce=40.0f;
	R2_Scara_Main_Pos_DacePid.max_dcce=800.0f;
	
	R2_Scara_End_Pos_Pid.P=200.0f;//400
	R2_Scara_End_Pos_Pid.Limit_Output=5000.0f;
	R2_Scara_End_Pos_DacePid.pid.P=400.0f;//600
	R2_Scara_End_Pos_DacePid.pid.Limit_Output=5000.0f;
	R2_Scara_End_Pos_DacePid.dead_zone=10.0f*DEG2RAD;
	R2_Scara_End_Pos_DacePid.max_acce=40.0f;
	R2_Scara_End_Pos_DacePid.max_dcce=800.0f;
	
	
	
	R2_Gantry_P_Pos_Pid.pid.P=30.0f;
	R2_Gantry_P_Pos_Pid.pid.Limit_Output=20000.0f;
	R2_Gantry_P_Pos_Pid.dead_zone=100.0f;
	R2_Gantry_P_Pos_Pid.max_acce=50.0f;
	R2_Gantry_P_Pos_Pid.max_dcce=60.0f;
	
	R2_Gantry_N_Pos_Pid=R2_Gantry_P_Pos_Pid;

	Tray_speed_pid=PID_Get_RM3508_Speed_Pid();
	Tray_angle_pid.pid.P=2.0f;
	Tray_angle_pid.pid.Limit_Output=5000.0f;
	Tray_angle_pid.dead_zone=0.01f;
	Tray_angle_pid.max_acce=4.0f;
	Tray_angle_pid.max_dcce=80.0f;
	
	//step3(必选):配置闭环控制参数
	MotorNode_Add_SpeedPid(&R2_Scara_Main,&R2_Scara_Main_Spd_Pid);
	MotorNode_Add_SpeedPid(&R2_Scara_End,&R2_Scara_End_Spd_Pid);
	MotorNode_Add_AnglePid(&R2_Scara_Main,&R2_Scara_Main_Pos_Pid);
	MotorNode_Add_AnglePid(&R2_Scara_End,&R2_Scara_End_Pos_Pid);
	MotorNode_Add_DaceAnglePid(&R2_Scara_Main,&R2_Scara_Main_Pos_DacePid);
	MotorNode_Add_DaceAnglePid(&R2_Scara_End,&R2_Scara_End_Pos_DacePid);
	
	MotorNode_Add_SpeedPid(&R2_Comm_Actor,&R2_Comm_Actor_Spd_Pid);
	MotorNode_Add_SpeedPid(&R2_Gantry_P,&R2_Gantry_P_Spd_Pid);
	MotorNode_Add_SpeedPid(&R2_Gantry_N,&R2_Gantry_N_Spd_Pid);
	
	MotorNode_Add_AnglePid(&R2_Comm_Actor,&R2_Comm_Actor_Ang_Pid);
	MotorNode_Add_DaceAnglePid(&R2_Comm_Actor,&R2_Comm_Actor_Ang_DacePid);
	MotorNode_Add_DaceAnglePid(&R2_Gantry_P,&R2_Gantry_P_Pos_Pid);
	MotorNode_Add_DaceAnglePid(&R2_Gantry_N,&R2_Gantry_N_Pos_Pid);
	//step4(非必选):配置电机保护
	//MotorNode_Add_PosProtect(&R2_Scara_Main,R2_SCARA_END_MAX_LPOS_RAD,R2_SCARA_END_MAX_UPOS_RAD);

	//step5(非必选):修改电机节点的接口
	if (R2_UPCTRL_CAN_GROUP==CAN_GROUP_1)
	{
		R2_Scara_Main.Get_Node_Angle=Get_Motor_Rad_Angle_M3508;//使用输出轴rad角度作为电机节点的角度,方便scara解算
		R2_Scara_End.Get_Node_Angle=Get_Motor_Rad_Angle_M3508;
	}
	else
	{
		R2_Scara_Main.Get_Node_Angle=Get_Motor_Rad_Angle2_M3508;//使用输出轴rad角度作为电机节点的角度,方便scara解算
		R2_Scara_End.Get_Node_Angle=Get_Motor_Rad_Angle2_M3508;
		//R2_Comm_Actor.Get_Node_Angle=R2_Comm_Get_Actor_Pos;
	}

	//step6(必选):将电机节点送入链表
	MotorNode_Add(R2_UPCTRL_CAN_GROUP,&R2_Scara_Main);
	MotorNode_Add(R2_UPCTRL_CAN_GROUP,&R2_Scara_End);
	MotorNode_Add(R2_UPCTRL_CAN_GROUP,&R2_Comm_Actor);
	MotorNode_Add(R2_UPCTRL_CAN_GROUP,&R2_Gantry_P);
	MotorNode_Add(R2_UPCTRL_CAN_GROUP,&R2_Gantry_N);
	Tray_Setup(&Tray,Tray_CAN_GROUP,TRAY_CAN_ID,&Tray_speed_pid,&Tray_angle_pid);

	//------------------硬初始化：CAN外设------------------
	//Motor_Init();
	Motor_Init2();

	//------------------硬初始化：UART外设------------------
	//日志串口
	RCS_USART_With_NoIsr_Config(USARTx_MAP.USARTx,USARTx_MAP.GPIOx,USARTx_MAP.GPIO_Pin_Tx,USARTx_MAP.GPIO_Pin_Rx,DEBUG_BAUD);
	ANSI_Set_OutPort(USARTx_MAP.USARTx);
	ANSI_CLEAR_ALL();

	//------------------硬初始化：GPIO外设------------------
	RCS_GPIO_Output_Init(R2_SCARA_ACTOR_PUNK_GPIO,R2_SCARA_ACTOR_PUNK_PIN);
	RCS_GPIO_Output_Init(R2_SCARA_ACTOR_SWIV_GPIO,R2_SCARA_ACTOR_SWIV_PIN);

	//-----------------发出配置完成的日志--------------------
	LogServer_Collect(&Scara_Log,"E","Upctrl Initialed","NULL");
	LogServer_Collect(&FSM_Log,"E","Upctrl Initialed","NULL");
	LogServer_Collect(&Comm_Log,"E","Upctrl Initialed","NULL");
}
/**
 * @name:R2_Log_Outout
 * @brief:上层机构日志全部输出,串口输出花时间,请将本函数放在BleTask或ChassisTask
 **/
void R2_Log_Outout(void)
{
	uint64_t TimeStamp=RTOS_Get_RealTime_Us();
	//LogServer_Output_TimeStamp(&Scara_Log,TimeStamp);
	//LogServer_Output_TimeStamp(&Comm_Log,TimeStamp);
	//LogServer_Output_TimeStamp(&FSM_Log,TimeStamp);
	LogServer_Output(&Scara_Log);
	LogServer_Output(&Comm_Log);
	LogServer_Output(&FSM_Log);
}

/*********************************************************************************

               @addtogroup:机械臂相关的API接口
                 @subgroup:保护接口            -@func_intro:保护接口函数对控制做出限制。控制接口调用保护接口实现某种特殊的保护功能。
                 @subgroup:日志接口            -@func_intro:日志接口函数获取控制过程中可能出现的各类突发情况,将其记录后以函数返回值返回。控制接口可以根据返回值对突发情况做出响应
                 @descrip:以上两类接口由本层的控制和反馈接口调用

                 @subgroup:控制接口            -@func_intro:控制一个机构实现某种特定功能的函数。入参可以描述该动作的所有参数。队里俗称Upctrl大函数
                 @subgroup:反馈接口            -@func_intro:告知状态机当前机构的工作情况。状态机调用反馈接口，基于反馈进行状态跳转。
                 @descrip:以上两类接口由基本动作状态机层的函数调用

                 @subgroup:调试接口            -@func_intro:这类函数用完就注释掉了，不对可维护性做出要求。唯一的要求是赛场集训前写好并这些函数，提高赛前调参效率。
                 @descrip:以上一类接口可以独立在主任务中被调用,用于调参或是性能检测

*********************************************************************************/

/**
 * @name: R2_Scara_Get_MotorNode_Err
 * @brief:获取Scara在运行过程中出现的错误,将其记录在日志之后,通过函数值返回,由控制接口对错误进行处理
 * @reval:[0]大臂正角度抵达限幅 [1]大臂负角度抵达限幅 [2]大臂正速度抵达限幅 [3]大臂负速度抵达限幅 [4~7]小臂同理
 * @subgroup:日志接口
 **/
uint8_t R2_Scara_Get_MotorNode_Err(void)//会进hardfault，没找到原因
{
	//局部变量
	uint8_t reval=0;
	static char    addl_log[30];

	//收集错误
	if (R2_Scara_Main.Now_CtrlStatus.IsrBit_PosProtect==1)
	{
		reval |= 1 << 0;
	}
	if (R2_Scara_Main.Now_CtrlStatus.IsrBit_PosProtect==-1)
	{
		reval |= 1 << 1;
	}
	if (R2_Scara_Main.Now_CtrlStatus.IsrBit_SpdProtect==1)
	{
		reval |= 1 << 2;
	}
	if (R2_Scara_Main.Now_CtrlStatus.IsrBit_SpdProtect==-1)
	{
		reval |= 1 << 3;
	}

	if (R2_Scara_End.Now_CtrlStatus.IsrBit_PosProtect==1)
	{
		reval |= 1 << 4;
	}
	if (R2_Scara_End.Now_CtrlStatus.IsrBit_PosProtect==-1)
	{
		reval |= 1 << 5;
	}
	if (R2_Scara_End.Now_CtrlStatus.IsrBit_SpdProtect==1)
	{
		reval |= 1 << 6;
	}
	if (R2_Scara_End.Now_CtrlStatus.IsrBit_SpdProtect==-1)
	{
		reval |= 1 << 7;
	}

	//日志记录
	if (reval!=0)
	{
		//sprintf(addl_log,"Err Code:%d-%d-%d-%d",(int)(reval&0x80),(int)(reval&0x40),(int)(reval&0x08),(int)(reval&0x04));
		//LogServer_Collect(&Scara_Log,"ES","Scara Motor Overload!",NULL);
	}
	//返回值记录
	return reval;
}

/**
 @name:R2_Scara_Action
 @brief:控制SCARA机械臂
 @param:ctrl_type 0控制笛卡尔坐标 1控制笛卡尔速度 2直接控制电机角度 3
 @debug:Scara_Output和Scara_ctrl_type可用于随时观察scara工作状态
 @reval:是否完成
 @subgroup:控制接口
**/
SCARA_AXIS Scara_Output;//预期输出值
SCARA_AXIS save_output; //安全输出值
uint8_t    Scara_ctrl_type;

uint8_t R2_Scara_Action(double x_or_main,double y_or_end,uint8_t ctrl_type)
{
	float overflow;
	uint8_t reval_flag[2];
	R2_ERROR_ENUM err;      //错误报告
	static char   addl_log[60];  //日志字符串
	
	//ctrl
	switch(ctrl_type)
	{
		//-------------------------笛卡尔坐标系位置控制-------------------------
		case 0:
			//逆运动学解算
			Scara_Output=Scara_Pos_Ctrl(x_or_main,y_or_end,MotorNode_Get_Angle(&R2_Scara_Main),MotorNode_Get_Angle(&R2_Scara_End),&R2_Scara_Param);
		
			//发起控制
			MotorNode_Update_AngleFull(Scara_Output.joint_angle_main,&R2_Scara_Main);
			MotorNode_Update_AngleFull(Scara_Output.joint_angle_end,&R2_Scara_End);

			//错误捕获
			err=R2_Scara_Get_MotorNode_Err();

			//日志反馈
			if (err!=0)
			{
//				sprintf(addl_log,"Target_X=%.1f,Target_Y=%.1f",Scara_Output.joint_angle_main,Scara_Output.joint_angle_end);
//				LogServer_Collect(&Scara_Log,"ES","Overload@R2_Scara_Action@DPos",addl_log);
			}
			if ((MotorNode_Get_TargetSpeed(&R2_Scara_Main)==0))
			{
				reval_flag[0]=1;
			}
			else
			{
				reval_flag[0]=0;
			}
			if (MotorNode_Get_TargetSpeed(&R2_Scara_End)==0)
			{
				reval_flag[1]=1;
			}
			else
			{
				reval_flag[1]=0;
			}
		break;

		//-------------------------笛卡尔坐标系速度控制-------------------------
		case 1:
			//微分逆运动学解算
			Scara_Output=Scara_Spd_Ctrl(x_or_main,y_or_end,MotorNode_Get_Angle(&R2_Scara_Main),MotorNode_Get_Angle(&R2_Scara_End),&R2_Scara_Param);
			if ((fabs(Scara_Output.joint_angle_main)>=2000.0)||(fabs(Scara_Output.joint_angle_end)>=2000.0))
			{
				overflow=fabs(Scara_Output.joint_angle_main)/500.0;

				Scara_Output.joint_angle_main=Scara_Output.joint_angle_main/overflow;
				Scara_Output.joint_angle_end=Scara_Output.joint_angle_end/overflow;
			}
			//发起控制
			MotorNode_Update_Spd(Scara_Output.joint_angle_main,&R2_Scara_Main);
			MotorNode_Update_Spd(Scara_Output.joint_angle_end,&R2_Scara_End);
			reval_flag[0]=1;
			reval_flag[1]=1;

			//错误捕获
			err=R2_Scara_Get_MotorNode_Err();
			//日志反馈
			if (err!=0)
			{
//				sprintf(addl_log,"Target_Vx=%.1f,Target_Vy=%.1f",Scara_Output.joint_angle_main,Scara_Output.joint_angle_end);
//				LogServer_Collect(&Scara_Log,"ES","Overload@R2_Scara_Action@DSpd",addl_log);
			}

		break;

		//-------------------------SCARA坐标系位置控制-------------------------
		case 2:
			//度量变换
			Scara_Output=Scara_Pos_Ctrl_Direct(x_or_main*DEG2RAD,y_or_end*DEG2RAD,&R2_Scara_Param);
			//发起控制
			MotorNode_Update_AngleFull(Scara_Output.joint_angle_main,&R2_Scara_Main);
			MotorNode_Update_AngleFull(Scara_Output.joint_angle_end,&R2_Scara_End);
			
			//错误捕获
			err=R2_Scara_Get_MotorNode_Err();
			//日志反馈
			if (err!=0)
			{
//				sprintf(addl_log,"Target_M=%.1f,Target_E=%.1f",Scara_Output.joint_angle_main,Scara_Output.joint_angle_end);
//				LogServer_Collect(&Scara_Log,"ES","Overload@R2_Scara_Action@SPos",addl_log);
			}
			if ((MotorNode_Get_TargetSpeed(&R2_Scara_Main)==0))
			{
				reval_flag[0]=1;
			}
			else
			{
				reval_flag[0]=0;
			}
			if (MotorNode_Get_TargetSpeed(&R2_Scara_End)==0)
			{
				reval_flag[1]=1;
			}
			else
			{
				reval_flag[1]=0;
			}
		break;

		//-------------------------SCARA坐标系速度控制-------------------------
	    case 3:
				//度量变换
				MotorNode_Update_Spd(x_or_main,&R2_Scara_Main);
				MotorNode_Update_Spd(y_or_end,&R2_Scara_End);
				reval_flag[0]=1;
				reval_flag[1]=1;
	    break;
	}


	
	//reval
	if (reval_flag[0]&&reval_flag[1])   return 1;
	else                                return 0;
}


/**
 @name:R2_Scara_Get_Pos
 @brief:获取SCARA机械臂的位置(笛卡尔坐标系)
 @subgroup:反馈接口
**/
DESCARTES_AXIS R2_Scara_Get_Pos(void)
{
	DESCARTES_AXIS reval;
	reval=Scara_Get_Desc_Pos(MotorNode_Get_Angle(&R2_Scara_Main),MotorNode_Get_Angle(&R2_Scara_End),&R2_Scara_Param);
	return reval;
}

/**
 @name:R2_Scara_Get_Pos
 @brief:获取SCARA机械臂的位置(SCARA坐标系下的机械臂角度)(rad)
 @subgroup:反馈接口
**/
SCARA_AXIS    R2_Scara_Get_SPos(void)
{
	SCARA_AXIS reval;
	reval.joint_angle_main=MotorNode_Get_Angle(&R2_Scara_Main);
	reval.joint_angle_end=MotorNode_Get_Angle(&R2_Scara_End);
	SCARA_AXIS tmp;
	tmp=Scara_Get_Scara_Pos(reval.joint_angle_main,reval.joint_angle_end,&R2_Scara_Param);
	return tmp;
}

/**
 @name:R2_Scara_Get_MPos
 @brief:获取SCARA机械臂的位置(SCARA坐标系下的电机角度)(rad)
 @subgroup:反馈接口
**/
SCARA_AXIS  R2_Scara_Get_MPos(void)
{
	SCARA_AXIS reval;
	reval.joint_angle_main=MotorNode_Get_Angle(&R2_Scara_Main);
	reval.joint_angle_end=MotorNode_Get_Angle(&R2_Scara_End);
	return reval;
}

/**
 @name:R2_Scara_Get_Pos
 @brief:获取SCARA机械臂的当前的预期输出
 @subgroup:反馈接口
**/
SCARA_AXIS R2_Scara_Get_Target_xPos(void)
{
	return Scara_Output;
}

/**
 @name:R2_Scara_Get_Pos
 @brief:获取SCARA机械臂的当前控制类型
 @subgroup:反馈接口
**/
uint8_t R2_Scara_Get_Target_Type(void)
{
	return Scara_ctrl_type;
}


/**
 @name:R2_Scara_Debug_Joystick
 @brief:遥控移动机械臂,方便采集视觉识别数据
 @subgroup:测试接口
**/
extern int rocker_lx;
extern int rocker_ly;
extern int rocker_ry;  //左摇杆控制运动
float prop=10.0f;
void R2_Scara_Debug_Joystick(void)
{
	float scara_spd[3];

	int lx,ly;
	if(abs(rocker_lx-127)<10) lx=127;else lx=rocker_lx;
	if(abs(rocker_ly-127)<10) ly=127;else ly=rocker_ly;
		
	scara_spd[0]=((float)(lx-127))*prop;
	scara_spd[1]=((float)(ly-127))*prop;

	if (stop_key)
	{
		R2_Scara_Action(scara_spd[0],scara_spd[1],1);
	}
	else
	{
		R2_Scara_Action(0,0,1);
	}
}

/**
 @name:R2_Scara_Debug_Forward_Back
 @brief:自动前后移动机械臂,方便通过日志观察视觉识别数据
 @subgroup:测试接口
**/
void R2_Scara_Debug_Forward_Back(void)
{
	static PID_Struct scara_pos_pid[2];
	static uint8_t status;
	static DESCARTES_AXIS scara_current_desc;
	static float   desc_output[2];
	
	scara_current_desc=R2_Scara_Get_Pos();
	scara_pos_pid[0].P=5.5f;
	scara_pos_pid[0].Limit_Output=200.0f;
	scara_pos_pid[1].P=5.5f;
	scara_pos_pid[1].Limit_Output=200.0f;
	
	if (stop_key)
	{
		switch(status)
		{
			case 0:
				desc_output[0]=PID_Normal_Ctrl(0.9f*R2_SCARA_MAIN_LEN,scara_current_desc.x,&scara_pos_pid[0]);
				desc_output[1]=PID_Normal_Ctrl(0.f                   ,scara_current_desc.y,&scara_pos_pid[1]);
			
				R2_Scara_Action(desc_output[0],desc_output[1],1);
				if (Count_Delay(900,15)) status=1;
			break;
			
			case 1:
				desc_output[0]=PID_Normal_Ctrl(2.1f*R2_SCARA_MAIN_LEN,scara_current_desc.x,&scara_pos_pid[0]);
				desc_output[1]=PID_Normal_Ctrl(0.f                   ,scara_current_desc.y,&scara_pos_pid[1]);
			
				R2_Scara_Action(desc_output[0],desc_output[1],1);
				if (Count_Delay(900,15)) status=0;
			break;
		}
	}
	else
	{
		R2_Scara_Action(0.0f,0.0f,1);
	}
}


/**
 @name:R2_Scara_Debug_Easy_Trace
 @brief:用stop_key控制追球的开始和关闭+视觉数据日志返回，方便赛前视觉和电控联调参数
 @tips:没有处理机械臂够不到和机械臂撞限位这两种情况，以及当准确率高而方差低时的情况(可能需要移植一个聚类算法)
 @subgroup:测试接口
**/
volatile DESCARTES_AXIS Valid_Spd_Output;
volatile int8_t valid_state=0;
volatile DacePID_Struct Vision_Pid[2];
volatile DESCARTES_AXIS Valid_Pos_Output; //ball pos of camera(transfered to chassis)
volatile DESCARTES_AXIS Valid_Pos_Vision;
void R2_Scara_Debug_Easy_Trace(void)
{
	
	static volatile float valid_confidence;
	static volatile float valid_variance;
	static volatile float valid_mean_x;
	static volatile float valid_mean_y;
	static volatile float output_x;
	static volatile float output_y;
	
	
	//get filtered data
	R2_Actor_Vision_ValidCheck_Filter(VISION_WINDOW_SIZE,&valid_confidence,&valid_variance,&valid_mean_x,&valid_mean_y);
	
	//setup pid
	Vision_Pid[0].pid.P=-6.0f;
	Vision_Pid[0].pid.Limit_Output=1000.0f;
	Vision_Pid[0].dead_zone=30.0f;
	Vision_Pid[0].max_acce=15.0f;
	Vision_Pid[0].max_dcce=500.0f;
	
	Vision_Pid[1].pid.P=-6.0f;
	Vision_Pid[1].pid.Limit_Output=1000.0f;
	Vision_Pid[1].dead_zone=30.0f;
	Vision_Pid[1].max_acce=15.0f;
	Vision_Pid[1].max_dcce=500.0f;
	
	//ctrl fsm
	switch(valid_state)
	{
		//data valid
		case 0:	
			if ((valid_confidence>=VISION_MIN_VALID_PERCENT)&&(valid_variance<=VISION_MAX_VALID_VAR))
			{
				Valid_Pos_Vision.x=valid_mean_x;
				Valid_Pos_Vision.y=valid_mean_y;
				Valid_Pos_Output=R2_Actor_Axis_Convert(Valid_Pos_Vision); 
				Valid_Spd_Output.x=DacePID_Normal_Ctrl(0.0f,Valid_Pos_Output.x,&Vision_Pid[0]);
				Valid_Spd_Output.y=DacePID_Normal_Ctrl(0.0f,Valid_Pos_Output.y,&Vision_Pid[1]);
				if (stop_key)
					R2_Scara_Action(Valid_Spd_Output.x,Valid_Spd_Output.y,1);
				else
					R2_Scara_Action(0,0,1);
			}
			else
			{
				Valid_Spd_Output.x=Valid_Spd_Output.x*0.95f;
				Valid_Spd_Output.y=Valid_Spd_Output.y*0.95f;
				if (stop_key)
					R2_Scara_Action(Valid_Spd_Output.x,Valid_Spd_Output.y,1);
				else
					R2_Scara_Action(0,0,1);
				
				if (Count_Delay(15,15)) valid_state=1;
			}
		break;
		
		//data disvalid
		case 1:
			MotorNode_Update_Current(0,&R2_Scara_Main);
			MotorNode_Update_Current(0,&R2_Scara_End);
		
			if ((valid_confidence>=VISION_MIN_VALID_PERCENT)&&(valid_variance<=VISION_MAX_VALID_VAR))
				valid_state=0;
		break;
	}
}
/*********************************************************************************

               @addtogroup:除机械臂外的机构控制接口
                 @subgroup:保护接口            -@func_intro:保护接口函数对控制做出限制。控制接口调用保护接口实现某种特殊的保护功能。
                 @subgroup:日志接口            -@func_intro:日志接口函数获取控制过程中可能出现的各类突发情况,将其记录后以函数返回值返回。控制接口可以根据返回值对突发情况做出响应
                 @descrip:以上两类接口由本层的控制和反馈接口调用

                 @subgroup:控制接口            -@func_intro:控制一个机构实现某种特定功能的函数。入参可以描述该动作的所有参数。队里俗称Upctrl大函数
                 @subgroup:反馈接口            -@func_intro:告知状态机当前机构的工作情况。状态机调用反馈接口，基于反馈进行状态跳转。
                 @descrip:以上两类接口由基本动作状态机层的函数调用

                 @subgroup:调试接口            -@func_intro:这类函数用完就注释掉了，不对可维护性做出要求。唯一的要求是赛场集训前写好并这些函数，提高赛前调参效率。
                 @descrip:以上一类接口可以独立在主任务中被调用,用于调参或是性能检测

*********************************************************************************/


/**
 @name:R2_Actor_Axis_Convert
 @brief:末端相机坐标系坐标转底盘坐标系坐标
 @usage:注释到函数声明之前，定义的全局变量为运行过程的中间变量，可以加入全局观测
 @addtogroup:反馈接口
**/
SCARA_AXIS     Scara_SPos;
float offset_angle,offset_angle_deg;
DESCARTES_AXIS Actor_Axis_Convert;
DESCARTES_AXIS R2_Actor_Axis_Convert(DESCARTES_AXIS Actor_Camera_Axis)
{
    //============局部变量======================================
	DESCARTES_AXIS Regular_Camera_Axis;

    //============相机坐标系正则化+转为末端坐标系===============
	Regular_Camera_Axis=Actor_Camera_Axis;
	Regular_Camera_Axis.x=Regular_Camera_Axis.x-CAMERA_AXIS_CENTER_X;
	Regular_Camera_Axis.y=Regular_Camera_Axis.y-CAMERA_AXIS_CENTER_Y;
	//Regular_Camera_Axis.x=Regular_Camera_Axis.x*(-1.0f);
	Regular_Camera_Axis.y=Regular_Camera_Axis.y*(-1.0f);
    //============末端坐标系转为底盘坐标系======================
	Scara_SPos=Scara_Get_Scara_Pos(MotorNode_Get_Angle(&R2_Scara_Main),MotorNode_Get_Angle(&R2_Scara_End),&R2_Scara_Param);
	offset_angle=R2_SCARA_ACTOR_OFFSET_RAD+Scara_SPos.joint_angle_main+Scara_SPos.joint_angle_end;
	offset_angle_deg=offset_angle*RAD2DEG;
	Actor_Axis_Convert.x=Regular_Camera_Axis.x*cosf(offset_angle)-Regular_Camera_Axis.y*sinf(offset_angle);//x=Xc+Ys
	Actor_Axis_Convert.y=Regular_Camera_Axis.y*cosf(offset_angle)+Regular_Camera_Axis.x*sinf(offset_angle);//y=Yc-Xs

	//reval.x-=R2_SCARA_ACTOR_OFFSET_X;
	//reval.y-=R2_SCARA_ACTOR_OFFSET_Y;
	return Actor_Axis_Convert;
}

/**
 @name:R2_Comm_Action
 @brief:控制SCARA机械臂之外的其他机构
 @reval:是否完成
 @addtogroup:控制接口
**/
uint8_t klklklk[3];
uint8_t R2_Comm_Action(float gantry_pos,float actor_pos,uint8_t punk_status,uint8_t valve_status)
{
	uint8_t finish_flag[3];

	//龙门架
	finish_flag[0]=MotorNode_Update_AngleFull(gantry_pos,&R2_Gantry_P);
	MotorNode_Update_AngleFull(-1*gantry_pos,&R2_Gantry_N);
	
	//吸盘爪
	finish_flag[1]=MotorNode_Update_AngleFull(actor_pos,&R2_Comm_Actor);

	//真空泵
	if (punk_status) RCS_GPIO_Set(R2_SCARA_ACTOR_PUNK_GPIO,R2_SCARA_ACTOR_PUNK_PIN);
	else             RCS_GPIO_Reset(R2_SCARA_ACTOR_PUNK_GPIO,R2_SCARA_ACTOR_PUNK_PIN);

	//泄压阀
	//if (valve_status)
		//Tray_Set(&Tray);
	//else
		//Tray_Reset(&Tray);
	
	
	klklklk[0] = finish_flag[0];
	klklklk[1] = finish_flag[1];
	klklklk[2] = finish_flag[2];
	//返回值
	if (finish_flag[0]&&finish_flag[1]) return 1;
	else                                return 0;
}


float R2_Comm_Get_Gantry_Pos(void)
{
	return MotorNode_Get_Angle(&R2_Gantry_P);
}
float R2_Comm_Get_Actor_Pos(uint8_t id)
{
	return Get_Motor_Float_Angle2(id)+R2_Actor_Start_Pos;
}



/**
 @name:R2_Actor_Vision_ValidCheck_Filter(滤波,用于捡球性能评估)
 @brief:检查视觉数据是否异常
 @addtogroup:安全接口/反馈接口
**/
uint32_t R2_Actor_Vision_ValidCheck_Filter(uint16_t window_size,float* valid_percent,float* valid_var,float* mean_x,float* mean_y)
{
	uint32_t reval;
	uint16_t i;
	static char addl_logs[60];
	static volatile uint8_t  valid_vision_count=0;
	static volatile float valid_vision_xy[3][100];
	static volatile float temp_float_x;
	static volatile float temp_float_y;
	static volatile float temp_input_x;
	static volatile float temp_input_y;
	
	static volatile float    valid_variance;
	static volatile float    valid_mean_x;
	volatile float    valid_mean_y;
	volatile float    valid_percent_count;
	volatile DESCARTES_AXIS current_ball_pos;
	volatile uint8_t cuurent_ball_status;
	
	//获取数据
	current_ball_pos=Vision_Get_Ball_Pos();
	cuurent_ball_status=Vision_Get_Ball_Count();
	WindowFloat_Update_Size(&Vision_DataWindow_X,window_size);
	WindowFloat_Update_Size(&Vision_DataWindow_Y,window_size);
	
	//将数据放入滑动窗口
	temp_input_x=(float)(current_ball_pos.x);
	temp_input_y=(float)(current_ball_pos.y);
	WindowFloat_Update_Member(&Vision_DataWindow_X,temp_input_x);
	WindowFloat_Update_Member(&Vision_DataWindow_Y,temp_input_y);
	//WindowFloat_Test(&Vision_DataWindow_X);
	
	//滑动统计相关参数
	if (WindowFloat_Get_Redy_Flag(&Vision_DataWindow_X))
	{
		valid_vision_count=0;
		//遍历整个窗口，收集有效数据
		for(i=0;i<Vision_DataWindow_X.window_size;i++)
		{
			//拷贝浮点数
			memcpy(&temp_float_x,WindowFloat_Get_Member_Ptr(&Vision_DataWindow_X,i),sizeof(float));
			memcpy(&temp_float_y,WindowFloat_Get_Member_Ptr(&Vision_DataWindow_Y,i),sizeof(float));
			//判断是否有效	
			if ((temp_float_x==0.0f)&&(temp_float_y==0.0f))
			{
				//无效数据不做处理
				__NOP();
			}
			else
			{
				//有效数据加入数组中
				valid_vision_xy[0][valid_vision_count]=temp_float_x;
				valid_vision_xy[1][valid_vision_count]=temp_float_y;
				valid_vision_xy[2][valid_vision_count]=temp_float_x+temp_float_y;
				valid_vision_count++;
			}		
		}

		//求有效数据的均值和方差
		valid_variance=ArrayFloat_Get_Var(valid_vision_xy[2],valid_vision_count);
		valid_mean_x=ArrayFloat_Get_Mean(valid_vision_xy[0],valid_vision_count);
		valid_mean_y=ArrayFloat_Get_Mean(valid_vision_xy[1],valid_vision_count);
		
		//整合数据分析的结果并返回
		if (window_size!=0) valid_percent_count=(1.0f*valid_vision_count)/(1.0f*window_size);else valid_percent_count=1.0f;
		*valid_percent=valid_percent_count;
		*valid_var=valid_variance;
		*mean_x=valid_mean_x;
		*mean_y=valid_mean_y;

		//如果数据超过预期，传回日志
		if ((valid_percent_count<VISION_MIN_VALID_PERCENT)||(valid_variance>VISION_MAX_VALID_VAR))
		{
			sprintf(addl_logs,"Valid_Prp(100x)=%d,Var(100x)=%d",(int)(valid_percent_count*100.0f),(int)(valid_variance*100.0f));
			LogServer_Collect(&Comm_Log,"ES","Vision Data Invalid!@Check_Filter",addl_logs);
		}
	}
}

/*********************************************************************************

               @addtogroup:基本动作状态机

*********************************************************************************/


/**
 @name:R2_Upctrl_PickBall_State/Quit
 @brief:指挥上层机构捡球,并以状态机的格式返回事件ID
 @addtogroup:可以被业务逻辑层调用的函数/被状态机调度的状态函数
 @debug:scara_pickball_status可用于随时观察scara工作状态
**/
int8_t  scara_pickball_status;
uint32_t ball_centered_tick_count;
uint32_t ball_losted_tick_count;
uint32_t ball_get_tick_count;
int test;


DESCARTES_AXIS ball_pos_chassis;
DESCARTES_AXIS ball_pos_camera;
int16_t R2_Upctrl_PickBall_State(void* global_var)
{
	//========局部变量=========================
    uint8_t reval_flag[3];
	R2_FSM_Var_t* glb_var;
    //========状态机全局变量===================
	glb_var=(R2_FSM_Var_t*)global_var;

    //========状态函数本体=====================
    switch(scara_pickball_status)
    {
        //----------丢球第一阶段：惯性移动----------------------------------------------
        //------丢球超过1s，认为球完全丢失。视野中再次连续出现100ms的球，再次开始取球
        case -1:
        {
            //状态函数
            MotorNode_Update_Current(0,&R2_Scara_Main);
            MotorNode_Update_Current(0,&R2_Scara_End);
            R2_Comm_Action(14000,0,0,0);
            ball_losted_tick_count++;
            //跳转逻辑
            if (Vision_Get_Ball_Count()!=0)
            {
                ball_get_tick_count++;
                ball_losted_tick_count=0;
                if(ball_get_tick_count>=2)
                {
                    scara_pickball_status=2;  //视野中重新出现球大于100ms，则重新开始找球
                }
            }
            else
            {
                ball_get_tick_count=0;

            }
            //事件触发
            return SCARA_EVENT_LOST_RADM;
        }
        break;

        //----------丢球第二阶段：底盘补偿----------------
        case -2:
        {

        }
        break;

        //----------捡球预备阶段：抬臂--------------------
        case 0:
        {
            //状态函数
            reval_flag[0]=R2_Scara_Action(525.0,0.0,0);
            reval_flag[1]=R2_Comm_Action(14000,0,0,0);
            //跳转逻辑
            if (reval_flag[0]&&reval_flag[1])
                scara_pickball_status=2;
            //事件触发
            return R2_EVENT_NONE;
        }
        break;

        //----------捡球第一阶段：找球--------------------
        case 1:
        {
            //todo
            scara_pickball_status=2;
        }
        break;

        //----------捡球第二阶段：对球--------------------
        case 2:
        {

            //--------------状态函数----------------------------------
            ball_pos_camera=Vision_Get_Ball_Pos();                    //当前视觉返回的坐标
            ball_pos_chassis=R2_Actor_Axis_Convert(ball_pos_camera);  //解算到的相对底盘的坐标


            //--------------跳转逻辑----------------------------------

            //视野中没球
            if (Vision_Get_Ball_Count()==0)
            {
								//printf("Ball Losted from case 2 to case -1\n\r");
                ball_centered_tick_count--; //丢一次球，重新开始计数
								if(ball_centered_tick_count<0) ball_centered_tick_count=0;
                scara_pickball_status=-1;   //丢球，视为随机误差
            }

            //视野中有球,而且位于视野重心
            else if ((fabsf(ball_pos_chassis.x)<R2_SCARA_CTRL_TOR_PIXEL)&&(fabsf(ball_pos_chassis.y)<R2_SCARA_CTRL_TOR_PIXEL))
            {
								//printf("Ball Arrived\n\r");
                reval_flag[0]=R2_Scara_Action(0.0f,0.0f,1);//scara沿着坐标移动
                reval_flag[2]=R2_Comm_Action(14000,0,0,0);  //龙门架不变
                ball_centered_tick_count++;
                if (ball_centered_tick_count>=50)
                {
                    scara_pickball_status=3;//收敛完成，开始捡球
                }
            }

            //视野中有球而且不位于视野中心
            else
            {
								//printf("Finding for ball in case 2\n\r");
                reval_flag[0]=R2_Scara_Action(ball_pos_chassis.x*(10.0),ball_pos_chassis.y*(10.0),1);//scara沿着坐标移动
                reval_flag[2]=R2_Comm_Action(14000,0,0,0);                                           //龙门架不变
                ball_centered_tick_count=0;//还未收敛
            }


            //--------------事件触发---------------------------------
            return R2_EVENT_NONE;
        }
        break;
        //----------捡球第三阶段：降臂--------------------
        case 3:
        {
						ball_centered_tick_count=0;
            //状态函数
            MotorNode_Update_Spd(0,&R2_Scara_Main);
            MotorNode_Update_Spd(0,&R2_Scara_Main);
            reval_flag[0]=R2_Comm_Action(-100,-2500,1,1);
            //跳转逻辑
            if (reval_flag[0])
            {
                scara_pickball_status=4;
            }
            //事件触发
            return R2_EVENT_NONE;
        }
        break;

        //----------捡球第四阶段：二审--------------------
        case 4:
        {
            scara_pickball_status=5;
        }
        break;

        //----------捡球第五阶段：抬球--------------------
        case 5:
        {
            //状态函数
            reval_flag[0]=R2_Scara_Action(0.0f,0.0f,1);
            reval_flag[1]=R2_Comm_Action(14000,-100,0,0);
            //跳转逻辑
            if (Count_Delay(600,10)) scara_pickball_status=0;
            //事件触发
            return R2_EVENT_NONE;
        }
        break;
        //----------逻辑出错-------------------------------
        default:

        break;
    }
    return R2_EVENT_NONE;
}
int16_t R2_Upctrl_PickBall_Quit(void* global_var)
{
	scara_pickball_status=0;
	ball_centered_tick_count=0;
	ball_losted_tick_count=0;
}



/**
 @name:R2_Aim
 @brief:车后摄像头视野中有球时底盘对准前往
 @addtogroup:可以被业务逻辑层调用的函数/被状态机调度的状态函数
 @debug:aim可用于随时观察工作状态
**/
int look_for;
int aim;
SCATTERED_POS scattered_pos;
int16_t R2_Aim(void)
{
//	uint8_t reval_flag[3];
//
//	scattered_pos.scattered_x=Vision_Get_Scattered_Ball_Pos().scattered_x;
//    scattered_pos.scattered_y=Vision_Get_Scattered_Ball_Pos().scattered_y;
//	switch (aim)
//				{
//					case 0://横向对准
//						if(scattered_pos.scattered_x>80)//球偏右
//			            {
//					      //向右跑对准
//							reval_flag[0]=;
//							if(reval_flag[0])
//							{
//								aim=1;
//							}
//
//
//			            }
//			            else if(scattered_pos.scattered_x<70)//球偏左
//			            {
//			              //向左跑对准
//							reval_flag[0]=;
//							if(reval_flag[0])
//							{
//								aim=1;
//							}
//
//			            }
//			            else//球在前方
//			            {
//							aim=1;
//			            }
//						break;
//					case 1://纵向前往
//						reval_flag[0]=;
//					    if(reval_flag[0])
//						{
//				      		return 1;
//						}
//
//						break;
//				}

}


/**
 @name:R2_Aim
 @brief:车后摄像头视野中有球时底盘对准前往
 @addtogroup:可以被业务逻辑层调用的函数/被状态机调度的状态函数
 @debug:aim可用于随时观察工作状态
**/
int scattered_or_not;

int16_t R2_Look_For_Scan(void)
{
//	uint8_t reval_flag[3];
//	scattered_or_not=Vision_Get_Scattered_Ball_Or_Not();
//
//	switch (look_for)
//				{
//					case 0://向左跑扫描
//						reval_flag[0]=;
//						if(scattered_or_not)
//						{
//							return 1;
//						}
//						else if(reval_flag[0])
//						{
//                           look_for=1;
//						}
//
//						break;
//					case 1://向右跑扫描
//						reval_flag[0]=;
//					    if(scattered_or_not)
//						{
//							return 1;
//						}
//					    else if(reval_flag[0])
//						{
//				      		look_for=0;
//						}
//
//						break;
//				}

}



/**
 @name:R2_Upctrl_PickBall_State_Scan/Quit_Scan
 @brief:指挥上层机构捡球,并以状态机的格式返回事件ID
 @addtogroup:可以被业务逻辑层调用的函数/被状态机调度的状态函数
 @debug:scara_pickball_status_scan可用于随时观察scara工作状态
**/
int8_t  scara_pickball_status_scan;
uint32_t ball_centered_tick_count_scan;
uint32_t ball_losted_tick_count_scan;
uint32_t ball_get_tick_count_scan;
int scan;
int scan_flag;
int look_for_scan;

DESCARTES_AXIS ball_pos_chassis_scan;
DESCARTES_AXIS ball_pos_camera_scan;

int16_t R2_Upctrl_PickBall_State_Scan(void* global_var)
{
        //========局部变量=========================

    uint8_t reval_flag[3];
		R2_FSM_Var_t* glb_var;
    //========状态机全局变量===================
		glb_var=(R2_FSM_Var_t*)global_var;

    //========状态函数本体=====================
    switch(scara_pickball_status_scan)
    {
        //----------丢球第一阶段：惯性移动----------------------------------------------
        //------丢球超过1s，认为球完全丢失。视野中再次连续出现100ms的球，再次开始取球
        case -1:
        {
            //状态函数
            MotorNode_Update_Current(0,&R2_Scara_Main);
            MotorNode_Update_Current(0,&R2_Scara_End);
            R2_Comm_Action(14000,0,0,0);
            ball_losted_tick_count_scan++;
            //跳转逻辑
            if (Vision_Get_Ball_Count()!=0)
            {
                ball_get_tick_count_scan++;
                ball_losted_tick_count_scan=0;
                if(ball_get_tick_count_scan>=2)
                {
                    scara_pickball_status_scan=2;  //视野中重新出现球大于100ms，则重新开始找球
                }
            }
            else
            {
                ball_get_tick_count_scan=0;//-2

            }
            //事件触发
            return SCARA_EVENT_LOST_RADM;
        }
        break;

        //----------丢球第二阶段：底盘补偿----------------
        case -2:
        {
            //scattered_or_not=Vision_Get_Scattered_Ball_Or_Not();
			if(scattered_or_not)//视野中有球
			{
				look_for_scan=0;

			}

			else//视野中没球
			{
				look_for_scan=1;

			}

			switch(look_for_scan)
			{
				case 0:
					//底盘对准
					reval_flag[0]=R2_Aim();
				    if(reval_flag[0])//已对准
				    {
					  ball_get_tick_count_scan=0;
				    }
					break;
				case 1:
					//底盘水平扫描找散球
				    reval_flag[0]=R2_Look_For_Scan();
				    if(reval_flag[0])//已找到
				    {
					  look_for_scan=0;
				    }
					break;

			}


        }
        break;

        //----------捡球预备阶段：抬臂--------------------
        case 0:
        {
            //状态函数
            reval_flag[0]=R2_Scara_Action(525.0,0.0,0);
            reval_flag[1]=R2_Comm_Action(14000,0,0,0);
            //跳转逻辑
            if (reval_flag[0]&&reval_flag[1])
                scara_pickball_status_scan=1;
            //事件触发
            return R2_EVENT_NONE;
        }
        break;

        //----------捡球第一阶段：找球--------------------
        case 1:
        {
            //todo
            //scara_pickball_status_scan=2;
			 //状态函数
			switch(scan_flag)
			{
				case 0:
				reval_flag[0]=R2_Comm_Action(14000,0,0,0);
				MotorNode_Update_AngleEasy(0,1000,100,&R2_Scara_Main);
				MotorNode_Update_AngleEasy(0,1000,100,&R2_Scara_End);
				if(reval_flag[0])
				{
					scan_flag=1;
				}
					break;

				case 1:
					reval_flag[0]=R2_Comm_Action(14000,0,0,0);
					scan=MotorNode_Update_AngleEasy(4.2f,1000,1,&R2_Scara_End);
				    MotorNode_Update_AngleEasy(0,1000,100,&R2_Scara_Main);

				    if ((Vision_Get_Ball_Count()!=0)&&(reval_flag[0]))//视野中有球
					{
						MotorNode_Update_Current(0,&R2_Scara_Main);
                        MotorNode_Update_Current(0,&R2_Scara_End);
						scara_pickball_status_scan=2;
					}

					if(scan)
					{
						MotorNode_Update_Current(0,&R2_Scara_Main);
                        MotorNode_Update_Current(0,&R2_Scara_End);
						scan_flag=2;
					}

					break;
				case 2:
					reval_flag[0]=R2_Comm_Action(14000,0,0,0);
					scan=MotorNode_Update_AngleEasy(-4.2f,1000,2,&R2_Scara_End);
				    MotorNode_Update_AngleEasy(0,1000,100,&R2_Scara_Main);
				    if ((Vision_Get_Ball_Count()!=0)&&(reval_flag[0]))//视野中有球
                    {
						MotorNode_Update_Current(0,&R2_Scara_Main);
                        MotorNode_Update_Current(0,&R2_Scara_End);
						scara_pickball_status_scan=2;
					}
					if(scan)
					{
						MotorNode_Update_Current(0,&R2_Scara_Main);
                        MotorNode_Update_Current(0,&R2_Scara_End);
						scara_pickball_status_scan=-1;
					}

					break;
			}

			//if((chan_x>=100)||(chan_y>=100))
			//{
			//	break;
			//}
            //事件触发
            return R2_EVENT_NONE;
        }
        break;

        //----------捡球第二阶段：对球--------------------
        case 2:
        {

            //--------------状态函数----------------------------------
            ball_pos_camera_scan=Vision_Get_Ball_Pos();                    //当前视觉返回的坐标
            ball_pos_chassis_scan=R2_Actor_Axis_Convert(ball_pos_camera_scan);  //解算到的相对底盘的坐标


            //--------------跳转逻辑----------------------------------

            //视野中没球
            if (Vision_Get_Ball_Count()==0)
            {
								//printf("Ball Losted from case 2 to case -1\n\r");
                ball_centered_tick_count_scan--; //丢一次球，重新开始计数
								if(ball_centered_tick_count_scan<0) ball_centered_tick_count_scan=0;
                scara_pickball_status_scan=-1;   //丢球，视为随机误差
            }

            //视野中有球,而且位于视野重心
            else if ((fabsf(ball_pos_chassis_scan.x)<R2_SCARA_CTRL_TOR_PIXEL)&&(fabsf(ball_pos_chassis_scan.y)<R2_SCARA_CTRL_TOR_PIXEL))
            {
								//printf("Ball Arrived\n\r");
                reval_flag[0]=R2_Scara_Action(0.0f,0.0f,1);//scara沿着坐标移动
                reval_flag[2]=R2_Comm_Action(14000,0,0,0);  //龙门架不变
                ball_centered_tick_count_scan++;
                if (ball_centered_tick_count_scan>=50)
                {
                    scara_pickball_status_scan=3;//收敛完成，开始捡球
                }
            }

            //视野中有球而且不位于视野中心
            else
            {
								//printf("Finding for ball in case 2\n\r");
                reval_flag[0]=R2_Scara_Action(ball_pos_chassis_scan.x*(10.0),ball_pos_chassis_scan.y*(10.0),1);//scara沿着坐标移动
                reval_flag[2]=R2_Comm_Action(14000,0,0,0);                                           //龙门架不变
                ball_centered_tick_count_scan=0;//还未收敛
            }


            //--------------事件触发---------------------------------
            return R2_EVENT_NONE;
        }
        break;
        //----------捡球第三阶段：降臂--------------------
        case 3:
        {
						ball_centered_tick_count_scan=0;
            //状态函数
            MotorNode_Update_Spd(0,&R2_Scara_Main);
            MotorNode_Update_Spd(0,&R2_Scara_End);
            reval_flag[0]=R2_Comm_Action(-100,-2500,1,1);
            //跳转逻辑
            if (reval_flag[0])
            {
                scara_pickball_status_scan=4;
            }
            //事件触发
            return R2_EVENT_NONE;
        }
        break;

        //----------捡球第四阶段：二审--------------------
        case 4:
        {
            scara_pickball_status_scan=5;
        }
        break;

        //----------捡球第五阶段：抬球--------------------
        case 5:
        {
            //状态函数
            reval_flag[0]=R2_Scara_Action(0.0f,0.0f,1);
            reval_flag[1]=R2_Comm_Action(14000,-100,1,1);
            //跳转逻辑
            if (Count_Delay(600,10)) return 1;
            //事件触发
            return R2_EVENT_NONE;
        }
        break;
        //----------逻辑出错-------------------------------
        default:

        break;
    }
    return R2_EVENT_NONE;
}
int16_t R2_Upctrl_PickBall_Quit_Scan(void* global_var)
{
	scara_pickball_status_scan=0;
	ball_centered_tick_count_scan=0;
	ball_losted_tick_count_scan=0;
}








/**
 @name:Bottom_Num()
 @brief:判断筐中最底层己方球个数
**/
int Basket_Num;
int Basket_Ball[5];
int num=0;
int bottom_num=0;

int Bottom_Num(int side_color)
{
	//Basket_Num=Vision_Get_Basket_Num();
	//*Basket_Ball=Vision_Get_Basket_Ball();
	if(Basket_Num==5)
		{
			if(side_color==0)//红方
			{
				for(num==0;num<=4;num++)
				{
					if((Basket_Ball[num]==2)||(Basket_Ball[num]==4)||(Basket_Ball[num]==5)||(Basket_Ball[num]==8)||(Basket_Ball[num]==9)||(Basket_Ball[num]==10)||(Basket_Ball[num]==11))
					{
						bottom_num++;
					}
					else
					{
						bottom_num=bottom_num;
					}
				}

			}
			else if(side_color==1)//蓝方
			{
				for(num==0;num<=4;num++)
				{
					if((Basket_Ball[num]==3)||(Basket_Ball[num]==6)||(Basket_Ball[num]==7)||(Basket_Ball[num]==12)||(Basket_Ball[num]==13)||(Basket_Ball[num]==14)||(Basket_Ball[num]==15))
					{
						bottom_num++;
					}
					else
					{
						bottom_num=bottom_num;
					}
				}
			}

		}
	else
	{
		bottom_num=6;
	}
	return bottom_num;
}


/**
 @name:Basket_State_Judge()
 @brief:判断筐中情况,用数字1-8表示各种情况并赋给basket_state_num[5]（从下至上看球）
**/
int serial=0;
int basket_state_num[5];

int Basket_State_Judge(int side_color)
{
//	Basket_Num=Vision_Get_Basket_Num();
//	*Basket_Ball=Vision_Get_Basket_Ball();
	if(Basket_Num==5)
	{
		if(side_color==0)//红方
		{
			for(serial==0;serial<=4;serial++)
			{
				if(Basket_Ball[serial]==16)//空
			   {
				basket_state_num[serial]=1;
			   }
			   else if((Basket_Ball[serial]==6)||(Basket_Ball[serial]==5))//对己，己对
			   {
				basket_state_num[serial]=2;
			   }
			   else if(Basket_Ball[serial]==4)//己己
			   {
				basket_state_num[serial]=3;
			   }
			   else if(Basket_Ball[serial]==7)//对对
			   {
				basket_state_num[serial]=4;
			   }
			   else if(Basket_Ball[serial]==3)//对
			   {
				basket_state_num[serial]=5;
			   }
			   else if(Basket_Ball[serial]==2)//己
			   {
				basket_state_num[serial]=6;
			   }
			   else//满
			   {
				basket_state_num[serial]=7;
			   }
			}
		}


		else if(side_color==1)//蓝方
		{
			for(serial==0;serial<=4;serial++)
			{
				if(Basket_Ball[serial]==16)//空
			   {
				basket_state_num[serial]=1;
			   }
			   else if((Basket_Ball[serial]==5)||(Basket_Ball[serial]==6))//对己，己对
			   {
				basket_state_num[serial]=2;
			   }
			   else if(Basket_Ball[serial]==7)//己己
			   {
				basket_state_num[serial]=3;
			   }
			   else if(Basket_Ball[serial]==4)//对对
			   {
				basket_state_num[serial]=4;
			   }
			   else if(Basket_Ball[serial]==2)//对
			   {
				basket_state_num[serial]=5;
			   }
			   else if(Basket_Ball[serial]==3)//己
			   {
				basket_state_num[serial]=6;
			   }
			   else//满
			   {
				basket_state_num[serial]=7;
			   }
			}
		}

	}
	else//没看全5个筐
	{
		for(serial==0;serial<=4;serial++)
		{
			basket_state_num[serial]=8;
		}
	}
	return *basket_state_num;
}



/**
 @name:Basket_Decide(int side_color)
 @brief:R2决定将球投入哪个筐中
 @return:basket_decision从左至右1-5
**/
int bt_num;
int bas_sta_num[5];
int basket_decision=0;
int ergodic=0;
int rank=0;
int Basket_Decide(int side_color)
{
	bt_num=Bottom_Num(side_color);
	*bas_sta_num=Basket_State_Judge(side_color);

		if(bt_num<3)
		{
			switch(rank)
			{
				case 0:

				for(ergodic==0;ergodic<=4;ergodic++)
				{
					if((ergodic<4)&&(bas_sta_num[ergodic]==1))//空
					{
						ergodic=0;
						rank=7;
					}
					else if((ergodic==4)&&(bas_sta_num[ergodic]!=1))
					{
						ergodic=0;
						rank=1;
					}
				}

					break;
				case 1:

				for(ergodic==0;ergodic<=4;ergodic++)
				{
					if((ergodic<4)&&(bas_sta_num[ergodic]==2))//对己/己对
					{
						rank=7;
					}
					else if((ergodic==4)&&(bas_sta_num[ergodic]!=2))
					{
						ergodic=0;
						rank=2;
					}
				}

					break;
				case 2:

				for(ergodic==0;ergodic<=4;ergodic++)
				{
					if((ergodic<4)&&(bas_sta_num[ergodic]==3))//己己
					{
						rank=7;
					}
					else if((ergodic==4)&&(bas_sta_num[ergodic]!=3))
					{
						ergodic=0;
						rank=3;
					}
				}

					break;
				case 3:

				for(ergodic==0;ergodic<=4;ergodic++)
				{
					if((ergodic<4)&&(bas_sta_num[ergodic]==4))//对对
					{
						rank=7;
					}
					else if((ergodic==4)&&(bas_sta_num[ergodic]!=4))
					{
						ergodic=0;
						rank=4;
					}
				}

					break;
				case 4:

				for(ergodic==0;ergodic<=4;ergodic++)
				{
					if((ergodic<4)&&(bas_sta_num[ergodic]==5))//对
					{
						rank=7;
					}
					else if((ergodic==4)&&(bas_sta_num[ergodic]!=5))
					{
						ergodic=0;
						rank=5;
					}
				}

					break;
				case 5:

				for(ergodic==0;ergodic<=4;ergodic++)
				{
					if((ergodic<4)&&(bas_sta_num[ergodic]==6))//己
					{
						rank=7;
					}
					else if((ergodic==4)&&(bas_sta_num[ergodic]!=6))
					{
						ergodic=0;
						rank=6;
					}
				}

					break;
				case 6:

				for(ergodic==0;ergodic<=4;ergodic++)
				{
					if((ergodic<4)&&(bas_sta_num[ergodic]==7))//满
					{
						rank=8;
					}
					else if((ergodic==4)&&(bas_sta_num[ergodic]!=7))
					{
						ergodic=0;
						rank=8;
					}
				}

					break;
				case 7://做出决定
					basket_decision=ergodic+1;
				    ergodic=0;
					break;

				case 8://无结果
					basket_decision=0;
				    ergodic=0;
					break;
			}

			rank=0;
		}
		else if(bt_num>=3)
		{
			switch(rank)
			{
				case 0:

				for(ergodic==0;ergodic<=4;ergodic++)
				{
					if((ergodic<4)&&(bas_sta_num[ergodic]==2))//对己/己对
					{
						ergodic=0;
						rank=7;
					}
					else if((ergodic==4)&&(bas_sta_num[ergodic]!=2))
					{
						ergodic=0;
						rank=1;
					}
				}

					break;
				case 1:

				for(ergodic==0;ergodic<=4;ergodic++)
				{
					if((ergodic<4)&&(bas_sta_num[ergodic]==3))//己己
					{
						rank=7;
					}
					else if((ergodic==4)&&(bas_sta_num[ergodic]!=3))
					{
						ergodic=0;
						rank=2;
					}
				}

					break;
				case 2:

				for(ergodic==0;ergodic<=4;ergodic++)
				{
					if((ergodic<4)&&(bas_sta_num[ergodic]==4))//对对
					{
						rank=7;
					}
					else if((ergodic==4)&&(bas_sta_num[ergodic]!=4))
					{
						ergodic=0;
						rank=3;
					}
				}

					break;
				case 3:

				for(ergodic==0;ergodic<=4;ergodic++)
				{
					if((ergodic<4)&&(bas_sta_num[ergodic]==1))//空
					{
						rank=7;
					}
					else if((ergodic==4)&&(bas_sta_num[ergodic]!=1))
					{
						ergodic=0;
						rank=4;
					}
				}

					break;
				case 4:

				for(ergodic==0;ergodic<=4;ergodic++)
				{
					if((ergodic<4)&&(bas_sta_num[ergodic]==6))//己
					{
						rank=7;
					}
					else if((ergodic==4)&&(bas_sta_num[ergodic]!=6))
					{
						ergodic=0;
						rank=5;
					}
				}

					break;
				case 5:

				for(ergodic==0;ergodic<=4;ergodic++)
				{
					if((ergodic<4)&&(bas_sta_num[ergodic]==5))//对
					{
						rank=7;
					}
					else if((ergodic==4)&&(bas_sta_num[ergodic]!=5))
					{
						ergodic=0;
						rank=6;
					}
				}

					break;
				case 6:

				for(ergodic==0;ergodic<=4;ergodic++)
				{
					if((ergodic<4)&&(bas_sta_num[ergodic]==7))//满
					{
						rank=8;
					}
					else if((ergodic==4)&&(bas_sta_num[ergodic]!=7))
					{
						ergodic=0;
						rank=8;
					}
				}

					break;
				case 7://做出决定
					basket_decision=ergodic+1;
				    ergodic=0;
					break;

				case 8://无结果
					basket_decision=0;
				    ergodic=0;
					break;
			}

			rank=0;
		}


		else if(bt_num==6)//没看全五个筐
		{
			basket_decision=0;
		}


		return basket_decision;
}


/**
 @name:R2_Upctrl_DropBall_State/Quit
 @brief:R2拾起一个球后到筐处放下球,并以状态机的格式返回事件ID
 @addtogroup:可以被业务逻辑层调用的函数/被状态机调度的状态函数
 @debug:scara_dropball_status可用于随时观察scara工作状态
**/
int8_t  scara_dropball_status;
//uint8_t ball_centered_tick_count;
//uint8_t ball_losted_tick_count;
//DESCARTES_AXIS ball_pos_chassis;

int16_t R2_Upctrl_DropBall_State(int decision)
{
//	uint8_t reval_flag[3];
//	switch(scara_dropball_status)
//	{
//		case 0://跑到筐处
//			reval_flag[0]=R2_Comm_Action(14000,0,1,1);
//			if()
//			{
//
//			}
//			else if()
//			{
//
//			}
//			else if()
//			{
//
//			}
//			else if()
//			{
//
//			}
//			if((reval_flag[0])&&())
//			{
//				scara_dropball_status=1;
//			}
//			break;
//		case 1://机械臂旋转
//			reval_flag[2]=R2_Comm_Action(14000,0,1,1);
//			if((decision==1)||(decision==2)||(decision==3))
//			{
//				reval_flag[0]=MotorNode_Update_AngleEasy(-2000,-1000,100,&R2_Scara_Main);
//				reval_flag[1]=MotorNode_Update_AngleEasy(0,1000,100,&R2_Scara_End);
//			}
//			else if((decision==4)||(decision==5))
//			{
//				reval_flag[0]=MotorNode_Update_AngleEasy(-2000,-1000,100,&R2_Scara_Main);
//				reval_flag[1]=MotorNode_Update_AngleEasy(0,1000,100,&R2_Scara_End);
//			}
//			if((reval_flag[0])&&(reval_flag[1])&&(reval_flag[2]))
//			{
//				scara_dropball_status=2;
//			}
//			break;
//		case 2://吸盘松手
//			reval_flag[0]=R2_Comm_Action(14000,0,0,0);
//		    if(reval_flag[0])
//			{
//				return 1;
//			}
//			break;
//
//	}
}


/**
 @name:camera_debug
 @brief:R2机械臂上摄像头校准拾球坐标
 @addtogroup:可以被业务逻辑层调用的函数/被状态机调度的状态函数
**/
uint8_t camera_debug(void)
{
	uint8_t finish_flag[2];
	finish_flag[0]=MotorNode_Update_AngleEasy(14000,1000,100,&R2_Gantry_P);
	finish_flag[1]=MotorNode_Update_AngleEasy(-14000,1000,100,&R2_Gantry_N);


	if (finish_flag[0]&&finish_flag[1]) return 1;
	else                                return 0;
}


/**
 @name:punk_pick_test
 @brief:R2真空泵吸球检测
 @addtogroup:可以被业务逻辑层调用的函数/被状态机调度的状态函数
**/
void punk_pick_test(int high_or_low)
{
	if(high_or_low==1)
	{
		RCS_GPIO_Set(GPIOC,4);
	RCS_GPIO_Set(GPIOE,2);

	RCS_GPIO_Set(GPIOC,5);
	RCS_GPIO_Set(GPIOE,3);

	RCS_GPIO_Set(GPIOB,0);
	RCS_GPIO_Set(GPIOE,4);

	RCS_GPIO_Set(GPIOB,1);
	RCS_GPIO_Set(GPIOE,5);
	}

	else if(high_or_low==0)
	{
	RCS_GPIO_Reset(GPIOC,4);
	RCS_GPIO_Reset(GPIOE,2);

	RCS_GPIO_Reset(GPIOC,5);
	RCS_GPIO_Reset(GPIOE,3);

	RCS_GPIO_Reset(GPIOB,0);
	RCS_GPIO_Reset(GPIOE,4);

	RCS_GPIO_Reset(GPIOB,1);
	RCS_GPIO_Reset(GPIOE,5);
	}

}

float count[5];

/**
 @name:count_test
 @brief:串口传送数据
 @addtogroup:可以被业务逻辑层调用的函数/被状态机调度的状态函数
**/
void count_test(void)
{
	count[0]=ball_centered_tick_count;
	count[1]=ball_losted_tick_count;
	count[2]=scara_pickball_status;
	//count[3]=chan_x;
	//count[4]=chan_y;
	JustFloat_Printf(&count,5,USART3_MAP);
}

int actor_test_flag;
int actor_finish_flag;
/**
 @name:actor_test
 @brief:吸盘自转调试
 @addtogroup:可以被业务逻辑层调用的函数/被状态机调度的状态函数
**/
void actor_test(int method)
{
	if(method==0)
	{
		switch(actor_test_flag)
		{
			case 0:
				actor_finish_flag=MotorNode_Update_AngleEasy(-2500,800,20,&R2_Comm_Actor);
			if(actor_finish_flag)
			{
				actor_test_flag=1;
			}

				break;
			case 1:
				MotorNode_Update_AngleEasy(-100,800,20,&R2_Comm_Actor);
				break;
		}

	}
	else if(method==1)
	{

		DoubleRing_Float_Ctrl(2400.0,R2_Comm_Actor.Now_Angle,R2_Comm_Actor.Now_Speed,&R2_Comm_Actor.Motor_Angle_PID,&R2_Comm_Actor.Motor_Speed_PID);
	}

}

/**
 @name:scan_test
 @brief:扫描调试
 @addtogroup:可以被业务逻辑层调用的函数/被状态机调度的状态函数
**/
void scan_test(void)
{
	uint8_t reval_flag[3];
	switch(scan_flag)
			{
				case 0:
				reval_flag[0]=R2_Comm_Action(14000,0,0,0);
				MotorNode_Update_AngleEasy(0,1000,100,&R2_Scara_Main);
				MotorNode_Update_AngleEasy(0,1000,100,&R2_Scara_End);
				if(reval_flag[0])
				{
					scan_flag=1;
				}
					break;

				case 1:
					reval_flag[0]=R2_Comm_Action(14000,0,0,0);
					scan=MotorNode_Update_AngleEasy(4.2f,1000,1,&R2_Scara_End);
				    MotorNode_Update_AngleEasy(0,1000,100,&R2_Scara_Main);

				    if ((Vision_Get_Ball_Count()!=0)&&(reval_flag[0]))//视野中有球
					{
						MotorNode_Update_Current(0,&R2_Scara_Main);
                        MotorNode_Update_Current(0,&R2_Scara_End);
						scara_pickball_status_scan=2;
					}

					if(scan)
					{
						MotorNode_Update_Current(0,&R2_Scara_Main);
                        MotorNode_Update_Current(0,&R2_Scara_End);
						scan_flag=2;
					}

					break;
				case 2:
					reval_flag[0]=R2_Comm_Action(14000,0,0,0);
					scan=MotorNode_Update_AngleEasy(-4.2f,1000,2,&R2_Scara_End);
				    MotorNode_Update_AngleEasy(0,1000,100,&R2_Scara_Main);
				    if ((Vision_Get_Ball_Count()!=0)&&(reval_flag[0]))//视野中有球
                    {
						MotorNode_Update_Current(0,&R2_Scara_Main);
                        MotorNode_Update_Current(0,&R2_Scara_End);
						scara_pickball_status_scan=2;
					}
					if(scan)
					{
						MotorNode_Update_Current(0,&R2_Scara_Main);
                        MotorNode_Update_Current(0,&R2_Scara_End);
						scara_pickball_status_scan=-1;
					}

					break;
			}
}


float main_angle;
float end_angle;
void Get_Scara_Angle(void)
{
	main_angle=MotorNode_Get_Angle(&R2_Scara_Main);
	end_angle=MotorNode_Get_Angle(&R2_Scara_End);
}

int main_flag;
void main_test(void)
{
	main_flag=MotorNode_Update_AngleEasy(-4.2,1000,1,&R2_Scara_End);
	if(main_flag)
	{
		MotorNode_Update_Current(0,&R2_Scara_End);

	}
}

/**
 @name:Judge_Rival
 @brief:判断对方放球情况返回数组[零数，对数，满数，其他数]

**/
int rival_basket[5];
int rival;
int rival_num[4];//[零数，对数，满数，其他数]
int single[5];//存对筐编号
int single_rival;
uint16_t Judge_Rival(int color)//己方颜色
{
	*rival_basket=Basket_State_Judge(color);
		for(rival==0;rival<5;rival++)
		{
			if(rival_basket[rival]==1)
			{
				rival_num[0]++;
			}
			else if(rival_basket[rival]==3)
			{
				single[single_rival]=rival+1;//对筐编号，范围1-5
				single_rival++;
				rival_num[1]++;
			}
			else if(rival_basket[rival]==7)
			{
				rival_num[2]++;
			}
			else
			{
				rival_num[3]++;
			}

		}
	return *rival_num;

}
/**
 @name:Store
 @brief:放地上存球

**/
int store_state;
uint16_t Store(void)
{
	uint16_t finish_flag[3];
	switch(store_state)
	{
		case 0:
//			finish_flag[0]=;//跑到下一个存球点
		    if(finish_flag[0])
		    {
			   store_state=1;
		    }
			break;

		case 1:
			finish_flag[0]=R2_Comm_Action(-100,-2500,1,1);//降臂
		    if(finish_flag[0])
			{
				store_state=2;
			}
			break;
		case 2:
			finish_flag[0]=R2_Comm_Action(-100,-2500,0,0);//松手
		    if(finish_flag[0])
			{
				return 1;
			}
			break;
	}

}
/**
 @name:Pick_Store
 @brief:拾起存的球

**/
int pick_store_state;
uint16_t Pick_Store(void* global_var)
{
	uint16_t finish_flag[3];
	switch(pick_store_state)
	{
		case 0:
//			finish_flag[0]=;//跑到上一个存球点
		    if(finish_flag[0])
		    {
			   store_state=1;
		    }
			break;
		case 1:
			finish_flag[0]=R2_Upctrl_PickBall_State_Scan(global_var);
		    if(finish_flag[0])
			{
				return 1;
			}
			break;
	}
}
/**
 @name:Strategy
 @brief:取球，存球，投球策略
 @addtogroup:可以被业务逻辑层调用的函数/被状态机调度的状态函数
**/
int strategy;
int rival_decision;
int store_member;
int storage_num;
int planB_decision;
uint16_t Strategy(void* global_var,int color)
{
	uint16_t finish_flag[3];
	switch(strategy)
	{
		case 0://取散球
			finish_flag[0]=R2_Upctrl_PickBall_State_Scan(global_var);
		    if(finish_flag[0])
			{
				strategy=1;
			}
			break;
		case 1://中间看筐
//			finish_flag[0]=;//走到能看全五个框的位置
		  if(finish_flag[0])
		  {
			  if(rival_num[0]==5)//对方没放
			{
				strategy=2;
			}
			else //对方放了
			{
				strategy=3;//planB决定筐编号
			}
		  }

			break;
		case 2://放地上存球
			finish_flag[0]=Store();

		    if(finish_flag[0])
			{
				storage_num++;
				strategy=4;
			}
			break;

		case 4://再取散球
			finish_flag[0]=R2_Upctrl_PickBall_State_Scan(global_var);
		    if(finish_flag[0])
			{
				strategy=5;
			}
			break;
		case 5://中间看筐
//			finish_flag[0]=;//走到能看全五个框的位置
		  if(finish_flag[0])
		  {
			  if((rival_num[1]==0)&&(rival_num[3]==0))//通过判断全是满或空，判断对方没放
			{
				strategy=2;//planA,放地上存球
			}
			else if((rival_num[1]==1)&&(rival_num[3]==0)) //对方放了一个
			{
				rival_decision=single[0];//对筐编号，范围1-5
				strategy=6;//planA,投入此筐
			}
			else//对方放了多个
			{
				strategy=3;//planB决定筐编号
			}
		  }
			break;
		case 6:
			finish_flag[0]=R2_Upctrl_DropBall_State(rival_decision);//压入第一个球
		    store_member=storage_num;//存球数
		    if(finish_flag[0])
			{
				if(store_member>0)//地上有存球
				{
					strategy=7;//取地上存球
				}
				else if(store_member==0)//地上没存球
				{
					strategy=8;//取散球
				}
			}
			break;
		case 7://取地上存球
			finish_flag[0]=Pick_Store(global_var);
		    if(finish_flag[0])
			{
				storage_num--;
				strategy=9;
			}
			break;
		case 8://取散球
			finish_flag[0]=R2_Upctrl_PickBall_State_Scan(global_var);
		    if(finish_flag[0])
			{
				strategy=9;
			}
			break;
		case 9://压上第二个球
			finish_flag[0]=R2_Upctrl_DropBall_State(rival_decision);//投入此筐
		    if(finish_flag[0])
			{
				strategy=4;//取散球
			}
			break;

		case 3://planB决定筐编号
			planB_decision=Basket_Decide(color);
		    finish_flag[0]=R2_Upctrl_DropBall_State(planB_decision);//投入此筐
		    store_member=storage_num;//存球数
		    if(finish_flag[0])
			{
				if(store_member>0)//地上有存球
				{
					strategy=10;//拾起存球
				}
				else if(store_member==0)//地上没有存球
				{
					strategy=11;//取散球
				}
			}
			break;
		case 10://拾起存球
			finish_flag[0]=Pick_Store(global_var);
		    if(finish_flag[0])
			{
				storage_num--;
				strategy=12;
			}
			break;
		case 11://取散球
			finish_flag[0]=R2_Upctrl_PickBall_State_Scan(global_var);
		    if(finish_flag[0])
			{
				strategy=12;
			}
			break;
		case 12://中间看筐
//			finish_flag[0]=;//走到能看全五个框的位置
		    if(finish_flag[0])
			{
				strategy=3;//planB决策
			}
			break;
	}
}

/**
 @name:R2_Actor_Vision_ValidCheck(未滤波,用于识别性能评估)
 @brief:检查视觉数据是否异常,并进行修正
 @param:jump_limit   最大允许跳变
 @param:count_limit  最大连续允许多少帧看不到球
 @param:window_count 以多少轮5ms作为采样评估窗口,窗口会随着采样的进行而滑动
 @reval:[0]当前帧的跳变是否超过限制 [1]当前帧的丢帧是否超过限制 [8~15]滑动窗口内的球的平均跳变率 [16~31]滑动窗口内的球的平均丢帧计数
 @addtogroup:日志接口
**/
// uint32_t R2_Actor_Vision_ValidCheck(uint16_t jump_limit,uint16_t count_limit,uint8_t window_count)
// {
// 	//------------局部与静态变量----------------------------------------
// 	static DESCARTES_AXIS last_ball_pos;    //上次球的位置
// 	static DESCARTES_AXIS last_ball_valid;  //上次球的有效位置
// 	static DESCARTES_AXIS current_ball_pos; //当前球的位置
// 	static uint8_t        validcheck_count; //统计阶段
// 	static uint8_t        validdata_count;  //统计时段内有效数据的个数
// 	uint8_t reval=0;                        //错误代码
// 	static char    log[30];                        //日志字符串
	
// 	//-------------获取实时数据--------------------------------------
// 	current_ball_pos=Vision_Get_Ball_Pos();
// 	validcheck_count++;

// 	//-----------单帧数据统计分析-------------------------------------

	
// 	//有球+跳变误差足够小可以接受,则该时刻的数据有效
// 	if (Vision_Get_Ball_Count()!=0)        
// 	if (fabsf(current_ball_pos.x-last_ball_pos.x)+fabsf(current_ball_pos.y-last_ball_pos.y)<jump_limit)
// 	{
// 		validdata_count++;//有效数据计数+1
// 	}

// 	//正常来说，视野中有球时跳变不该发生，因此这件事是需要记录的
// 	else if (fabsf(current_ball_pos.x-last_ball_pos.x)+fabsf(current_ball_pos.y-last_ball_pos.y)>=jump_limit)
// 	     if (Vision_Get_Ball_Count()!=0)    
// 	{
// 		//日志记录
// 		sprintf(log,"Delta_X+Delta_Y=%d",abs(current_ball_pos.x-last_ball_pos.x)+abs(current_ball_pos.y-last_ball_pos.y));
// 		LogServer_Collect(&Comm_Log,"ES","Ball Jump Lost!",log);
// 		//返回值赋值
// 		reval|=1<<0;
// 	}

// 	//突然没球是可以接受的识别误差
// 	else
// 	{		
		
// 	}

// 	//-----------------多帧数据统计分析--------------------------------

// 	if (validcheck_count>=window_count)
// 	{
// 		//在统计范围内，丢球的占比过高，这是不应该发生的，因此需要记录
// 		if (validcheck_count-validdata_count >= count_limit)
// 		{
// 			sprintf(log,"Valid_per=%.1f",(float)validdata_count/(float)validcheck_count);
// 			LogServer_Collect(&Comm_Log,"ES","Ball Count Lost!",log);
// 			reval|=1<<1;
// 		}
// 		//完成多帧数据的分析后，再次发起一个多帧数据分析
// 		validcheck_count=0;
// 		validdata_count=0;
// 	}

// 	//------------------解析数据返回----------------------------------
// 	last_ball_pos=current_ball_pos;
// 	return reval;
// }
