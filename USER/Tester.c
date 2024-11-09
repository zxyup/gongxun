#include "Tester.h"
PID_Struct RM_speed_pid[2][9];
Motor_Ctrl_Node RM_Node[2][9];
Motor_Ctrl_Node MBVESC_Node[2][9];

extern int rocker_lx,rocker_ly;  //左摇杆控制运动
extern int rocker_rx,rocker_ry;  //右遥感控制旋转
extern int direction_key[4];
static void Tester_pid_init(void);


void Tester_MBVESC_Init(void)
{
	MotorNode_Init_MBVESC(CAN_GROUP_1,11,CTRL_TYPE_CUR,&MBVESC_Node[0][1]);
	MotorNode_Add_SpeedPid(&MBVESC_Node[0][1],&RM_speed_pid[0][1]);
	MotorNode_Add(CAN_GROUP_1,&MBVESC_Node[0][1]);
}

void Tester_MBVESC_Run(float* current_ptr,float target)
{
	MotorNode_Update_Current(target,&MBVESC_Node[0][1]);
	//MotorNode_Update_Spd(target,&MBVESC_Node[0][1]);
	*current_ptr=MotorNode_Get_Speed(&MBVESC_Node[0][1]);
}



void Alien_Chassis_Init(void)
{
	Motor_Init();
	Tester_pid_init();
	
	Tester_Add_RM3508(CAN_GROUP_1,1);
	Tester_Add_RM3508(CAN_GROUP_1,2);
	Tester_Add_RM3508(CAN_GROUP_1,3);
	Tester_Add_RM3508(CAN_GROUP_1,4);
}


#define X1 109.5f
#define X2 312.5f
#define Y1 89.0f
void Alien_Move(float spd_x,float spd_y,float spd_z)
{
	float spd[4];
	float theta;
	float x_speed,y_speed;
	
	theta=Get_Gyro_Z();
	x_speed=spd_x*cosf(theta*DEG2RAD)+spd_y*sinf(theta*DEG2RAD);
	y_speed=spd_y*cosf(theta*DEG2RAD)-spd_x*sinf(theta*DEG2RAD);

	spd[0]= y_speed  - spd_z;
	spd[1]= x_speed  - spd_z*  (Y1/(sqrtf(X1*X1+Y1*Y1)));
	spd[2]=-y_speed  - spd_z;
	spd[3]=-x_speed  - spd_z*  (Y1/(sqrtf(X1*X1+Y1*Y1)));

	MotorNode_Update_Spd(spd[0],&RM_Node[0][1]);
	MotorNode_Update_Spd(spd[1],&RM_Node[0][2]);
	MotorNode_Update_Spd(spd[2],&RM_Node[0][3]);
	MotorNode_Update_Spd(spd[3],&RM_Node[0][4]);
}


void Alien_Joystick(void)
{
	float thetas;
	int16_t speeds[3];
	float x_speed,y_speed;
	int16_t output[2],speed[2];
	thetas=Get_Gyro_Z();
		
	  if(abs(rocker_lx-125)<10) rocker_lx=125;//中值
	  if(abs(rocker_ly-126)<10) rocker_ly=126;
	  if(abs(rocker_rx-130)<10) rocker_rx=130;
		
		if (switch_key)
	  {
			speeds[0]=((float)(rocker_lx-125))*20;
			speeds[1]=((float)(rocker_ly-126))*20;
			speeds[2]=((float)(rocker_rx-130))*-5;
		}
		else
		{
			speeds[0]=((float)(rocker_lx-125))*10;
			speeds[1]=((float)(rocker_ly-126))*10;
			speeds[2]=((float)(rocker_rx-130))*-2.5;
		}
		
		if (stop_key)
			Alien_Move(speeds[0],speeds[1],speeds[2]);
		else
			Alien_Move(0,0,0);
}


void Alien_P2P(float dis_x,float dis_y,float dis_z)
{
	
}



void Tester_Add_RM3508(uint8_t CAN_Group,uint8_t id)
{
	Tester_pid_init();
	if (CAN_Group==CAN_GROUP_1)
	{
		MotorNode_Init_C620(CAN_Group,id,&RM_Node[0][id]);
		MotorNode_Add_SpeedPid(&RM_Node[0][id],&RM_speed_pid[0][id]);
		MotorNode_Add_BldcProtect(&RM_Node[0][id],RMESC_M3508);
		
		MotorNode_Add(CAN_Group,&RM_Node[0][id]);
	}
	else if (CAN_Group==CAN_GROUP_2)
	{
		MotorNode_Init_C620(CAN_Group,id,&RM_Node[1][id]);
		MotorNode_Add_SpeedPid(&RM_Node[1][id],&RM_speed_pid[1][id]);
		MotorNode_Add_BldcProtect(&RM_Node[1][id],RMESC_M3508);
		
		MotorNode_Add(CAN_Group,&RM_Node[1][id]);
	}
}

void Tester_Test_RM3508(uint8_t CAN_Group,uint8_t id,uint8_t type,int32_t output)
{
	if (type==1)
	{
		if (CAN_Group==CAN_GROUP_1)
			MotorNode_Update_Current(output,&RM_Node[0][id]);
		else
			MotorNode_Update_Current(output,&RM_Node[1][id]);
	}
	else if (type==2)
	{
		if (CAN_Group==CAN_GROUP_1)
			MotorNode_Update_Spd(output,&RM_Node[0][id]);
		else
			MotorNode_Update_Spd(output,&RM_Node[1][id]);
	}
	else if (type==3)
	{
		if (CAN_Group==CAN_GROUP_1)
			MotorNode_Update_AngleEasy(output,600,20,&RM_Node[0][id]);
		else
			MotorNode_Update_AngleEasy(output,600,20,&RM_Node[1][id]);
	}
}

static void Tester_pid_init(void)
{
	for (int i=0;i<2;i++)
	for (int j=0;j<9;j++)
	{
		RM_speed_pid[i][j]=PID_Get_RM3508_Speed_Pid();
	}
}
