/**
 @filename:RCS_AGV_BaseMove.c
 @brief:舵轮底盘速度环控制
 @date:2023/11/2
 @author:陈煜楷
**/

/***************************************************************
长方形四舵轮底盘,默认CAN1通讯
电机编号、相对坐标系及电机正转时轮子的有效方向如下
        ↑Y
        |
 1 ↑----+---↑2
   |    |   |
   |    +---+----→X
   |        |长
 4 ↑--------↑3
       宽

绝对坐标系定义如下
       ↑Y
       |
       |
       |
     Z +---------→ X      
********************************************************************
三角形三舵轮底盘,默认CAN1通讯
电机编号、相对坐标系及电机正转时轮子的有效方向如下
       ↑Y
       |
       ↑ 1
      / \
     /   \    -------→ X 
    /     \
  2↑-------↑3 
绝对坐标系定义如下
       ↑Y
       |
       |
       |
     Z +---------→ X      
********************************************************************/
#include "RCS_AGV_BaseMove.h"
#include "Cov_AcceLimit_Ctrl.h"
#include "RCS_dsp.h"

/* =================全局变量=========================================*/
PID_Struct      M2006_pid_speed[4],M2006_pid_angle[4];
PID_Struct      M3508_pid_speed[4];
Motor_Ctrl_Node agv_drv[4],agv_srv[4];

float tester;
float watch_angle[4];
float watch_speed[4];
int speed_reverse_flag[4] = {0,0,0,0};
DacePID_Struct AGV_3508[3];

CovDace_Handler_t AGV_CovDace;
StaySample_Handler_t AGV_VESC_Sampler[4];

/* =================中间层函数声明=======================================*/
static __INLINE AGV_Speed AGV_Calculate(Dct_Speed* input);
static __INLINE float AngleLoop(float target_angle, int ID);
void AGV_Execute(AGV_Speed* input);
static __INLINE void AGV_Execute_VESC(AGV_Speed* input);
static __INLINE float AGV_Read_X(void);
static __INLINE float AGV_Read_Y(void);
static __INLINE float AGV_Read_Angle(void);
static void AGV_PID_Init(void);
float Get_IrZero_Pos(uint8_t id);
static void IrZero_Init(void);
void AGV_Silumate_Move(float speed_x, float speed_y, float speed_z);

/* =================接口函数定义=========================================*/
/**
 * @name:AGV_PID_Init
 * @brief:舵轮PID
*/
static void AGV_PID_Init(void)
{
	PID_Struct Srv_Com_Pid[4];
	
	for(int i = 0; i < 4; i++)
	{
		Srv_Com_Pid[i].P=4;		//10;//18;
		Srv_Com_Pid[i].I=0;		//1;
		Srv_Com_Pid[i].D=9;		//8;//7;
		Srv_Com_Pid[i].Integral=0;
		Srv_Com_Pid[i].Last_Error=0;
		Srv_Com_Pid[i].Limit_Integral=0;
		Srv_Com_Pid[i].Limit_Output=5000;//8000;
	}
	
	
	float U8_VESC_Dace=240.0f;
	AGV_CovDace.Acce_Max[0]=U8_VESC_Dace;
	AGV_CovDace.Acce_Max[1]=U8_VESC_Dace;
	AGV_CovDace.Acce_Max[2]=U8_VESC_Dace;
	AGV_CovDace.Acce_Max[3]=U8_VESC_Dace;
	AGV_CovDace.Dcce_Max[0]=-U8_VESC_Dace;
	AGV_CovDace.Dcce_Max[1]=-U8_VESC_Dace;
	AGV_CovDace.Dcce_Max[2]=-U8_VESC_Dace;
	AGV_CovDace.Dcce_Max[3]=-U8_VESC_Dace;
	
	AGV_VESC_Sampler[0].rate=3;
	AGV_VESC_Sampler[1].rate=3;
	AGV_VESC_Sampler[2].rate=3;
	AGV_VESC_Sampler[3].rate=3;
	
	M2006_pid_speed[0]=PID_Get_RM3508_Speed_Pid();
	M2006_pid_speed[0].Limit_Output=10000;
	M2006_pid_speed[1]=M2006_pid_speed[0];
	M2006_pid_speed[2]=M2006_pid_speed[0];
	M2006_pid_speed[3]=M2006_pid_speed[0];
	M2006_pid_angle[0]=Srv_Com_Pid[0];
	M2006_pid_angle[1]=Srv_Com_Pid[1];
	M2006_pid_angle[2]=Srv_Com_Pid[2];
	M2006_pid_angle[3]=Srv_Com_Pid[3];
	
	M3508_pid_speed[0]=PID_Get_RM3508_Speed_Pid();
	M3508_pid_speed[1]=M3508_pid_speed[0];
	M3508_pid_speed[2]=M3508_pid_speed[0];
	M3508_pid_speed[3]=M3508_pid_speed[0];
	
	for(int i = 0; i<2;i++)
	{
		AGV_3508[i].pid.P = 0.5;
		AGV_3508[i].pid.I = 0;
		AGV_3508[i].pid.D = 0;
		(AGV_3508[i].pid).Limit_Output=50.0f;
		AGV_3508[i].max_acce = 1;
		AGV_3508[i].max_dcce = 50;
		AGV_3508[i].dead_zone = 10;
	}
		AGV_3508[2].pid.P = 7.0f;
		AGV_3508[2].pid.I = 0;
		AGV_3508[2].pid.D = 0;
	(AGV_3508[2].pid).Limit_Output=360.0f;		//一秒车体转一整圈为最大速度
		AGV_3508[2].max_acce = 10;
		AGV_3508[2].max_dcce = 50;
		AGV_3508[2].dead_zone = 0.5;
}

float Get_AGV_Max_Speed_X()
{
	return (AGV_3508[0].pid).Limit_Output;
}

float Get_AGV_Max_Speed_Y()
{
	return (AGV_3508[1].pid).Limit_Output;
}

/**
 * @name: AGV_Init
 * @brief:初始化舵轮底盘
*/
void AGV_Init(void)
{
	//PID初始化
	AGV_PID_Init();
	
	//底盘电机节点初始化
	for(int i=0;i<4;i++)
	{
		MotorNode_Init_C620(CAN_GROUP_1,5+i,&agv_drv[i]);
		#if Srv_Type == RMESC_M2006
			MotorNode_Init_C610(CAN_GROUP_1,1+i, &agv_srv[i]);
		#elif Srv_Type == RMESC_M3508
			Motor_Ctrl_Init_C620_Motor(CAN_GROUP_1,1+i, &agv_srv[i]);
		#endif
		
		MotorNode_Add_SpeedPid(&agv_drv[i],&M3508_pid_speed[i]);
		
		agv_srv[i].Motor_Speed_PID=&M2006_pid_speed[i];
		agv_srv[i].Motor_Angle_PID=&M2006_pid_angle[i];
		agv_srv[i].Get_Node_Angle=&Get_IrZero_Pos;
		
		MotorNode_Add(CAN_GROUP_1,&(agv_drv[i]));
		MotorNode_Add(CAN_GROUP_1,&(agv_srv[i]));
		
		agv_srv[i].Angle = 0;
		
	}

	//底盘电机CAN外设初始化
	Motor_Init();
	IrZero_Init();
}





/**
 @name:AGV_Move
 @brief:三、四舵轮驱动舵向电机移动
 @param:speed_x,舵轮在x方向的线速度(以场地为参考系),单位为m/min
 @param:speed_y,舵轮在y方向的线速度(以场地为参考系),单位为m/min
 @param:rotate_speed,舵轮的自转线速度,单位为m/min
**/


volatile AGV_Speed AGV_Opt_Trace;
volatile float target_speed[4];
void AGV_Move(float move_speed_x, float move_speed_y, float rotate_speed)
{
	Dct_Speed ipt_spd;
	AGV_Speed opt_sta;

	ipt_spd.Dct_vw=rotate_speed*Drv_Wheel_Radius_Rec;//半径一般是小数,和speed数量级相差较大,容易放大误差
	ipt_spd.Dct_vx=move_speed_x*Drv_Wheel_Radius_Rec;
	ipt_spd.Dct_vy=move_speed_y*Drv_Wheel_Radius_Rec;

	//opt_sta=AGV_Calculate(&ipt_spd);
	AGV_Opt_Trace=opt_sta;
	AGV_Execute(&opt_sta);
}



AGV_Speed AGV_Get_Last_Spd(void)
{
	return AGV_Opt_Trace;
}

/**
 @name:AGV_BaseMove_P2P
 @brief:三、四舵轮驱动舵向电机移动，自动校准舵向角度
 @param:speed_x,舵轮在x方向的线速度(以场地为参考系),单位为m/min
 @param:speed_y,舵轮在y方向的线速度(以场地为参考系),单位为m/min
 @param:rotate_speed,舵轮的自转线速度,单位为m/min
**/
int state_Angle_P2P;
int flag_arrive_Angle_P2P;
void AGV_BaseMove_P2P(float move_speed_x, float move_speed_y, float rotate_speed, float allow_angle_err, float target_speed_max)
{
	switch(state_Angle_P2P)
		{
			case 0:
				AGV_Angle_First(move_speed_x, move_speed_y, rotate_speed, allow_angle_err, &flag_arrive_Angle_P2P);
				if(flag_arrive_Angle_P2P)
				{
					state_Angle_P2P++;
				}
				break;
				
			case 1:
				AGV_Re_Center_Move_v3_3(move_speed_x, move_speed_y, rotate_speed);
				break;
			}
}

void AGV_Angle_First(float move_speed_x, float move_speed_y, float rotate_speed, float allow_angle_err, int8_t* flag_arrive)
{
	AGV_Speed re_center_move;
	
	float Rotate_Angle[4], velocity_x[4], velocity_y[4],R_W2RC[4], a_F, a_B, b_L, b_R	//R_W2RC: R_Wheel_to_Re_Center，轮子到指定中心的距离
		,current_angle, target_angle, composite_speed, wheel_angle[4], current_wheel_angle[4],wheel_angle2[4];	//target_angle是xy速度和与车身x轴的夹角	
	
	current_angle = AGV_Read_Angle() * DEG2RAD;
	target_angle = atan2(move_speed_y, move_speed_x);
	composite_speed = sqrt(move_speed_x * move_speed_x + move_speed_y * move_speed_y);
	
	a_F = - RC_Y + Chassis_Lenth/2.0f;
	a_B = - RC_Y - Chassis_Lenth/2.0f;
	b_L = - RC_X + Chassis_Width/2.0f;
	b_R = - RC_X - Chassis_Width/2.0f;
	
	R_W2RC[0] = sqrt(a_F * a_F + b_L * b_L)*6.0f/100.0f;
	R_W2RC[1] = sqrt(a_F * a_F + b_R * b_R)*6.0f/100.0f;
	R_W2RC[2] = sqrt(a_B * a_B + b_R * b_R)*6.0f/100.0f;
	R_W2RC[3] = sqrt(a_B * a_B + b_L * b_L)*6.0f/100.0f;
	
	Rotate_Angle[0] = atan2(-Chassis_Lenth/2.0f - RC_Y, Chassis_Width/2.0f - RC_X) - pi/2.0f;		//以指定中心展开坐标系的角度
	Rotate_Angle[1] = atan2(-Chassis_Lenth/2.0f - RC_Y, -Chassis_Width/2.0f - RC_X) - pi/2.0f;
	Rotate_Angle[2] = atan2(Chassis_Lenth/2.0f - RC_Y, -Chassis_Width/2.0f - RC_X) - pi/2.0f;
	Rotate_Angle[3] = atan2(Chassis_Lenth/2.0f - RC_Y, Chassis_Width/2.0f - RC_X) - pi/2.0f;
	
	for(int i = 0; i < 4; i++)
	{
		current_wheel_angle[i] = MotorNode_Get_Angle(&agv_srv[i])/Srv_Slowdown_Rate/36.0f;
		velocity_x[i] = composite_speed * cosf(target_angle - current_angle) + rotate_speed *DEG2RAD * R_W2RC[i] * cosf(Rotate_Angle[i]);
		velocity_y[i] = composite_speed * sinf(target_angle - current_angle) + rotate_speed *DEG2RAD * R_W2RC[i] * sinf(Rotate_Angle[i]);
		wheel_angle[i] = RAD2DEG * (atan2(velocity_y[i], velocity_x[i])-pi/2);
		{
			re_center_move.angle[i] = wheel_angle[i];
		}
		
		re_center_move.speed[i] = 0;
	}
	AGV_Execute(&re_center_move);
	
	if(re_center_move.angle[0] - allow_angle_err < MotorNode_Get_Angle(&agv_srv[0]) /Srv_Slowdown_Rate/36.0 && re_center_move.angle[0] + allow_angle_err > MotorNode_Get_Angle(&agv_srv[0]) /Srv_Slowdown_Rate/36.0
		&&re_center_move.angle[1] - allow_angle_err < MotorNode_Get_Angle(&agv_srv[1]) /Srv_Slowdown_Rate/36.0 && re_center_move.angle[1] + allow_angle_err > MotorNode_Get_Angle(&agv_srv[1]) /Srv_Slowdown_Rate/36.0
		&&re_center_move.angle[2] - allow_angle_err < MotorNode_Get_Angle(&agv_srv[2]) /Srv_Slowdown_Rate/36.0 && re_center_move.angle[2] + allow_angle_err > MotorNode_Get_Angle(&agv_srv[2]) /Srv_Slowdown_Rate/36.0
		&&re_center_move.angle[3] - allow_angle_err < MotorNode_Get_Angle(&agv_srv[3]) /Srv_Slowdown_Rate/36.0 && re_center_move.angle[3] + allow_angle_err > MotorNode_Get_Angle(&agv_srv[3]) /Srv_Slowdown_Rate/36.0)
	{
		*flag_arrive = 1;
	}
	else
	{
		*flag_arrive = 0;
	}
}

/**
 @name:AGV_Re_Center_Move_v3_3（指定旋转中心的移动，旋转中心由define的RC_X和RC_Y决定）（角速度相同版）（让电调控制速度）
 @param:speed_x,舵轮在x方向的线速度(以场地为参考系),单位为m/min
 @param:speed_y,舵轮在y方向的线速度(以场地为参考系),单位为m/min
@param:rotate_speed,舵轮的自转角速度,单位为deg/s			//原本是deg/min，现在是deg/s，需要注意不要再给很大的自转速度！！
 @note:这里是认为各个旋转角速度相同的情况下投影的。
		和上一版不同之处在于加入了当前角度的读取
		*						
		|\						LF----------RF
		| \						|			 |
		a  \					|			 |
		|   R					LB----------RB
		|    \
		*--b--*
**/
AGV_Speed re_center_move;
AGV_Speed covdace_center_move;
AGV_Speed last_covdace_center_move;
void AGV_Re_Center_Move_v3_3(float move_speed_x, float move_speed_y, float rotate_speed)
{
	
	
	float Rotate_Angle[4], velocity_x[4], velocity_y[4],R_W2RC[4], a_F, a_B, b_L, b_R	//R_W2RC: R_Wheel_to_Re_Center，轮子到指定中心的距离
		,current_angle, target_angle, composite_speed, wheel_angle[4], current_wheel_angle[4],wheel_angle2[4];	//target_angle是xy速度和与车身x轴的夹角	
	
	current_angle = AGV_Read_Angle() * DEG2RAD;
	target_angle = atan2(move_speed_y, move_speed_x);
	composite_speed = sqrt(move_speed_x * move_speed_x + move_speed_y * move_speed_y);
	
	a_F = - RC_Y + Chassis_Lenth/2.0f;
	a_B = - RC_Y - Chassis_Lenth/2.0f;
	b_L = - RC_X + Chassis_Width/2.0f;
	b_R = - RC_X - Chassis_Width/2.0f;
	
	R_W2RC[0] = sqrt(a_F * a_F + b_L * b_L)*6.0f/100.0f;
	R_W2RC[1] = sqrt(a_F * a_F + b_R * b_R)*6.0f/100.0f;
	R_W2RC[2] = sqrt(a_B * a_B + b_R * b_R)*6.0f/100.0f;
	R_W2RC[3] = sqrt(a_B * a_B + b_L * b_L)*6.0f/100.0f;
	
	Rotate_Angle[0] = atan2(-Chassis_Lenth/2.0f - RC_Y, Chassis_Width/2.0f - RC_X) - pi/2.0f;		//以指定中心展开坐标系的角度
	Rotate_Angle[1] = atan2(-Chassis_Lenth/2.0f - RC_Y, -Chassis_Width/2.0f - RC_X) - pi/2.0f;
	Rotate_Angle[2] = atan2(Chassis_Lenth/2.0f - RC_Y, -Chassis_Width/2.0f - RC_X) - pi/2.0f;
	Rotate_Angle[3] = atan2(Chassis_Lenth/2.0f - RC_Y, Chassis_Width/2.0f - RC_X) - pi/2.0f;
	
	
	if(move_speed_x <= FLOAT_ZERO && move_speed_x >= - FLOAT_ZERO
		&& move_speed_y <= FLOAT_ZERO && move_speed_y >= - FLOAT_ZERO
		&& rotate_speed <= FLOAT_ZERO && rotate_speed >= - FLOAT_ZERO)
	{
		re_center_move.speed[0] = 0;
		re_center_move.speed[1] = 0;
		re_center_move.speed[2] = 0;
		re_center_move.speed[3] = 0;
	}
	else
	{
		for(int i = 0; i < 4; i++)
		{
			current_wheel_angle[i] = MotorNode_Get_Angle(&agv_srv[i])/Srv_Slowdown_Rate/36.0f;
			velocity_x[i] = composite_speed * cosf(target_angle - current_angle) + rotate_speed *DEG2RAD * R_W2RC[i] * cosf(Rotate_Angle[i]);
			velocity_y[i] = composite_speed * sinf(target_angle - current_angle) + rotate_speed *DEG2RAD * R_W2RC[i] * sinf(Rotate_Angle[i]);
			wheel_angle[i] = RAD2DEG * (atan2(velocity_y[i], velocity_x[i])-pi/2);
			re_center_move.speed[i] = sqrt(velocity_x[i] * velocity_x[i] + velocity_y[i] * velocity_y[i]);
			//角度回环&最短路径（注意此处re_center_move.angle为上次更新的轮子角度，在此近似为轮子当前角度，可能导致结果不准确）
			if(wheel_angle[i] - current_wheel_angle[i] > 90)
			{
				while(wheel_angle[i] - current_wheel_angle[i] > 90)
				{
					wheel_angle[i] -= 180;
					re_center_move.speed[i] = - re_center_move.speed[i];
				}
				re_center_move.angle[i] = wheel_angle[i];
			}
			else if(wheel_angle[i] - current_wheel_angle[i] < -90)
			{
				while(wheel_angle[i] - current_wheel_angle[i] < -90)
				{
					wheel_angle[i] += 180;
					re_center_move.speed[i] = - re_center_move.speed[i];
				}
				re_center_move.angle[i] = wheel_angle[i];
			}
			else
			{
				re_center_move.angle[i] = wheel_angle[i];
			}
		}
	}
	//执行电机指令
	//AGV_Execute_VESC(&last_covdace_center_move);//输出本杰明
	//AGV_Execute_VESC(&re_center_move);
	AGV_Opt_Trace=re_center_move;
	AGV_Execute(&re_center_move);
}

void AGV_Re_Center_Move_v3_4(float move_speed_x, float move_speed_y, float rotate_speed)
{
	float allow_angle_err = 5;
	
	float Rotate_Angle[4], velocity_x[4], velocity_y[4],R_W2RC[4], a_F, a_B, b_L, b_R	//R_W2RC: R_Wheel_to_Re_Center，轮子到指定中心的距离
		,current_angle, target_angle, composite_speed, wheel_angle[4], current_wheel_angle[4],wheel_angle2[4];	//target_angle是xy速度和与车身x轴的夹角	
	
	current_angle = AGV_Read_Angle() * DEG2RAD;
	target_angle = atan2(move_speed_y, move_speed_x);
	composite_speed = sqrt(move_speed_x * move_speed_x + move_speed_y * move_speed_y);
	
	a_F = - RC_Y + Chassis_Lenth/2.0f;
	a_B = - RC_Y - Chassis_Lenth/2.0f;
	b_L = - RC_X + Chassis_Width/2.0f;
	b_R = - RC_X - Chassis_Width/2.0f;
	
	R_W2RC[0] = sqrt(a_F * a_F + b_L * b_L)*6.0f/100.0f;
	R_W2RC[1] = sqrt(a_F * a_F + b_R * b_R)*6.0f/100.0f;
	R_W2RC[2] = sqrt(a_B * a_B + b_R * b_R)*6.0f/100.0f;
	R_W2RC[3] = sqrt(a_B * a_B + b_L * b_L)*6.0f/100.0f;
	
	Rotate_Angle[0] = atan2(-Chassis_Lenth/2.0f - RC_Y, Chassis_Width/2.0f - RC_X) - pi/2.0f;		//以指定中心展开坐标系的角度
	Rotate_Angle[1] = atan2(-Chassis_Lenth/2.0f - RC_Y, -Chassis_Width/2.0f - RC_X) - pi/2.0f;
	Rotate_Angle[2] = atan2(Chassis_Lenth/2.0f - RC_Y, -Chassis_Width/2.0f - RC_X) - pi/2.0f;
	Rotate_Angle[3] = atan2(Chassis_Lenth/2.0f - RC_Y, Chassis_Width/2.0f - RC_X) - pi/2.0f;
	
	if(move_speed_x <= FLOAT_ZERO && move_speed_x >= - FLOAT_ZERO
		&& move_speed_y <= FLOAT_ZERO && move_speed_y >= - FLOAT_ZERO
		&& rotate_speed <= FLOAT_ZERO && rotate_speed >= - FLOAT_ZERO)
	{
		re_center_move.speed[0] = 0;
		re_center_move.speed[1] = 0;
		re_center_move.speed[2] = 0;
		re_center_move.speed[3] = 0;
	}
	else
	{
		for(int i = 0; i < 4; i++)
		{
			current_wheel_angle[i] = MotorNode_Get_Angle(&agv_srv[i])/Srv_Slowdown_Rate/36.0f;
			velocity_x[i] = composite_speed * cosf(target_angle - current_angle) + rotate_speed *DEG2RAD * R_W2RC[i] * cosf(Rotate_Angle[i]);
			velocity_y[i] = composite_speed * sinf(target_angle - current_angle) + rotate_speed *DEG2RAD * R_W2RC[i] * sinf(Rotate_Angle[i]);
			wheel_angle[i] = RAD2DEG * (atan2(velocity_y[i], velocity_x[i])-pi/2);
			re_center_move.speed[i] = sqrt(velocity_x[i] * velocity_x[i] + velocity_y[i] * velocity_y[i]);
			//角度回环&最短路径（注意此处re_center_move.angle为上次更新的轮子角度，在此近似为轮子当前角度，可能导致结果不准确）
			if(wheel_angle[i] - current_wheel_angle[i] > 90)
			{
				while(wheel_angle[i] - current_wheel_angle[i] > 90)
				{
					wheel_angle[i] -= 180;
					re_center_move.speed[i] = - re_center_move.speed[i];
				}
				re_center_move.angle[i] = wheel_angle[i];
			}
			else if(wheel_angle[i] - current_wheel_angle[i] < -90)
			{
				while(wheel_angle[i] - current_wheel_angle[i] < -90)
				{
					wheel_angle[i] += 180;
					re_center_move.speed[i] = - re_center_move.speed[i];
				}
				re_center_move.angle[i] = wheel_angle[i];
			}
			else
			{
				re_center_move.angle[i] = wheel_angle[i];
			}
		}
	}
	
	//在调2006pid时暂时注释，取消注释即可拿来跑
	if(re_center_move.angle[0] - allow_angle_err < MotorNode_Get_Angle(&agv_srv[0]) /Srv_Slowdown_Rate/36.0 && re_center_move.angle[0] + allow_angle_err > MotorNode_Get_Angle(&agv_srv[0]) /Srv_Slowdown_Rate/36.0
		&&re_center_move.angle[1] - allow_angle_err < MotorNode_Get_Angle(&agv_srv[1]) /Srv_Slowdown_Rate/36.0 && re_center_move.angle[1] + allow_angle_err > MotorNode_Get_Angle(&agv_srv[1]) /Srv_Slowdown_Rate/36.0
		&&re_center_move.angle[2] - allow_angle_err < MotorNode_Get_Angle(&agv_srv[2]) /Srv_Slowdown_Rate/36.0 && re_center_move.angle[2] + allow_angle_err > MotorNode_Get_Angle(&agv_srv[2]) /Srv_Slowdown_Rate/36.0
		&&re_center_move.angle[3] - allow_angle_err < MotorNode_Get_Angle(&agv_srv[3]) /Srv_Slowdown_Rate/36.0 && re_center_move.angle[3] + allow_angle_err > MotorNode_Get_Angle(&agv_srv[3]) /Srv_Slowdown_Rate/36.0);
	else
	{
		re_center_move.speed[0] = 0;
		re_center_move.speed[1] = 0;
		re_center_move.speed[2] = 0;
		re_center_move.speed[3] = 0;
	}
	
	AGV_Opt_Trace=re_center_move;
	AGV_Execute(&re_center_move);
}

///////////////////
float Read_AGV_Target_Angle(int num)
{
	return watch_angle[num];
}
float Read_AGV_Target_Speed(int num)
{
	return watch_speed[num];
}


void AGV_Drv_Debug(int16_t rpm1,int16_t rpm2,int16_t rpm3,int16_t rpm4)
{
	MotorNode_Update_Spd(rpm1,&agv_drv[0]);
	MotorNode_Update_Spd(rpm2,&agv_drv[1]);
	MotorNode_Update_Spd(rpm3,&agv_drv[2]);
	MotorNode_Update_Spd(rpm4,&agv_drv[3]);
	
}

void AGV_Srv_Debug(int16_t cur1,int16_t cur2,int16_t cur3,int16_t cur4)
{
	MotorNode_Update_Angle(cur1,&agv_srv[0]);
	MotorNode_Update_Angle(cur2,&agv_srv[1]);
	MotorNode_Update_Angle(cur3,&agv_srv[2]);
	MotorNode_Update_Angle(cur4,&agv_srv[3]);
}


/* =================舵轮控制-硬件抽象层=========================================*/
/**
 @name:AGV_Execute
 @brief:将解算结果下发给舵轮底盘
**/
void AGV_Execute(AGV_Speed* input)
{
		MotorNode_Update_Spd((float)input->speed[0]*19.0f/2.0f/pi*Drv_Wheel_Radius_Rec*(-1.0f),&agv_drv[0]);
		MotorNode_Update_Spd((float)input->speed[1]*19.0f/2.0f/pi*Drv_Wheel_Radius_Rec,&agv_drv[1]);
		MotorNode_Update_Spd((float)input->speed[2]*19.0f/2.0f/pi*Drv_Wheel_Radius_Rec,&agv_drv[2]);
		MotorNode_Update_Spd((float)input->speed[3]*19.0f/2.0f/pi*Drv_Wheel_Radius_Rec*(-1.0f),&agv_drv[3]);
	
	
	MotorNode_Update_Angle(input->angle[0]*Srv_Slowdown_Rate*36.0f,&agv_srv[0]);
	MotorNode_Update_Angle(input->angle[1]*Srv_Slowdown_Rate*36.0f,&agv_srv[1]);
	MotorNode_Update_Angle(input->angle[2]*Srv_Slowdown_Rate*36.0f,&agv_srv[2]);
	MotorNode_Update_Angle(input->angle[3]*Srv_Slowdown_Rate*36.0f,&agv_srv[3]);
}

#define VESC_EXCUTE_COUNT 2
#define VESC_EXCUTE_TIMER 19
/**
 @name:AGV_Execute
 @brief:将解算结果下发给舵轮底盘（让电调来控制轮子速度的版本）
@note: 3.183来自于20/2/pi，再与Drv_Wheel_Radius_Rec相乘即可得到需要给电调发送的RPM，因为在vesc tool发送1500时u8的转速约为75转/min，相除得到20
**/
static __INLINE void AGV_Execute_VESC(AGV_Speed* input)
{
	static AGV_Speed last_spd;
	
	if (Count_Delay(VESC_EXCUTE_COUNT,VESC_EXCUTE_TIMER))
	{
		MotorNode_Update_Spd(input->speed[0]*3.183f*Drv_Wheel_Radius_Rec,&agv_drv[0]);
		MotorNode_Update_Spd(input->speed[1]*3.183f*Drv_Wheel_Radius_Rec,&agv_drv[1]);
		MotorNode_Update_Spd(input->speed[2]*3.183f*Drv_Wheel_Radius_Rec,&agv_drv[2]);
		MotorNode_Update_Spd(input->speed[3]*3.183f*Drv_Wheel_Radius_Rec,&agv_drv[3]);
		last_spd.speed[0]=input->speed[0];
		last_spd.speed[1]=input->speed[1];
		last_spd.speed[2]=input->speed[2];
		last_spd.speed[3]=input->speed[3];
	}
	else
	{
		MotorNode_Update_Spd(last_spd.speed[0]*3.183f*Drv_Wheel_Radius_Rec,&agv_drv[0]);
		MotorNode_Update_Spd(last_spd.speed[1]*3.183f*Drv_Wheel_Radius_Rec,&agv_drv[1]);
		MotorNode_Update_Spd(last_spd.speed[2]*3.183f*Drv_Wheel_Radius_Rec,&agv_drv[2]);
		MotorNode_Update_Spd(last_spd.speed[3]*3.183f*Drv_Wheel_Radius_Rec,&agv_drv[3]);
	}
	
	MotorNode_Update_Angle(input->angle[0]*Srv_Slowdown_Rate*36,&agv_srv[0]);
	MotorNode_Update_Angle(input->angle[1]*Srv_Slowdown_Rate*36,&agv_srv[1]);
	MotorNode_Update_Angle(input->angle[2]*Srv_Slowdown_Rate*36,&agv_srv[2]);
	MotorNode_Update_Angle(input->angle[3]*Srv_Slowdown_Rate*36,&agv_srv[3]);
}

void AGV_Test_VESC(void)
{
	MotorNode_Update_Spd(1500,&agv_drv[0]);
	MotorNode_Update_Spd(1500,&agv_drv[1]);
	MotorNode_Update_Spd(1500,&agv_drv[2]);
	MotorNode_Update_Spd(1500,&agv_drv[3]);
}


float AGV_Pid_Pos[4];
float AGV_current_pos[4];
void AGV_Test_ANGLE(void)
{
	MotorNode_Update_Angle(AGV_Pid_Pos[0],&agv_srv[0]);
	MotorNode_Update_Angle(AGV_Pid_Pos[1],&agv_srv[1]);
	MotorNode_Update_Angle(AGV_Pid_Pos[2],&agv_srv[2]);
	MotorNode_Update_Angle(AGV_Pid_Pos[3],&agv_srv[3]);
//	MotorNode_Update_Spd(AGV_Pid_Pos[0],&agv_srv[0]);
//	MotorNode_Update_Spd(AGV_Pid_Pos[1],&agv_srv[1]);
//	MotorNode_Update_Spd(AGV_Pid_Pos[2],&agv_srv[2]);
//	MotorNode_Update_Spd(AGV_Pid_Pos[3],&agv_srv[3]);
	
	AGV_current_pos[0]=MotorNode_Get_Angle(&agv_srv[0]);
	AGV_current_pos[1]=MotorNode_Get_Angle(&agv_srv[1]);
	AGV_current_pos[2]=MotorNode_Get_Angle(&agv_srv[2]);
	AGV_current_pos[3]=MotorNode_Get_Angle(&agv_srv[3]);
}
uint8_t Inited[4];
uint8_t srv_state[5];
float srv_zero[5];
float Get_IrZero_Pos(uint8_t id)
{	
    if (RCS_GPIO_Read(GPIOC,GPIO_Pin_0)==0) 
    {
        srv_zero[1]=Get_Motor_Float_Angle(1)+30.0f*36.0f*Srv_Slowdown_Rate;
        srv_state[1]=1;
            Inited[0] =1;
    }
    else
    {
        srv_state[1]=0;
    }
    
    if (RCS_GPIO_Read(GPIOC,GPIO_Pin_5)==0) 
    {
        srv_zero[2]=Get_Motor_Float_Angle(2)-30.0f*36.0f*Srv_Slowdown_Rate;
        srv_state[2]=1;
            Inited[1] =1;
    }
    else
    {
        srv_state[2]=0;
    }
    
    if (RCS_GPIO_Read(GPIOB,GPIO_Pin_1)==0) 
    {
        srv_zero[3]=Get_Motor_Float_Angle(3)+30.0f*36.0f*Srv_Slowdown_Rate;
        srv_state[3]=1;
            Inited[2] =1;
    }
    else
    {
        srv_state[3]=0;
    }
    
    if (RCS_GPIO_Read(GPIOC,GPIO_Pin_3)==0) 
    {
        srv_zero[4]=Get_Motor_Float_Angle(4)-30.0f*36.0f*Srv_Slowdown_Rate;
        srv_state[4]=1;
            Inited[3] =1;
    }
    else
    {
        srv_state[4]=0;
    }
    
    return (Get_Motor_Float_Angle(id)-srv_zero[id]);
}

uint8_t AGV_Init_Angle(void)
{
	Get_IrZero_Pos(1);
	
	if(Inited[0] == 1 && Inited[1] == 1 && Inited[2] == 1 && Inited[3] == 1)
	{
		MotorNode_Update_Spd(0,&agv_srv[0]);
		MotorNode_Update_Spd(0,&agv_srv[1]);
		MotorNode_Update_Spd(0,&agv_srv[2]);
		MotorNode_Update_Spd(0,&agv_srv[3]);
		return 1;
	}
	else
	{
		MotorNode_Update_Spd(3000,&agv_srv[0]);
		MotorNode_Update_Spd(3000,&agv_srv[1]);
		MotorNode_Update_Spd(3000,&agv_srv[2]);
		MotorNode_Update_Spd(3000,&agv_srv[3]);
		return 0;
	}
}

void Zero_CANID1_EXTI_Isr(void)
{
	srv_zero[1]=Get_Motor_Float_Angle(1);
}
void Zero_CANID2_EXTI_Isr(void)
{
	srv_zero[2]=Get_Motor_Float_Angle(2);
}
void Zero_CANID3_EXTI_Isr(void)
{
	srv_zero[3]=Get_Motor_Float_Angle(3);
}
void Zero_CANID4_EXTI_Isr(void)
{
	srv_zero[4]=Get_Motor_Float_Angle(4);
}

static void IrZero_Init(void)
{
	RCS_GPIO_Input_Init(GPIOC,GPIO_Pin_0);
	RCS_GPIO_Input_Init(GPIOC,GPIO_Pin_3);
	RCS_GPIO_Input_Init(GPIOC,GPIO_Pin_4);
	RCS_GPIO_Input_Init(GPIOC,GPIO_Pin_5);
	RCS_GPIO_Input_Init(GPIOB,GPIO_Pin_1);
//	RCS_InitEXTI(GPIOC,GPIO_Pin_0,EXTI_Trigger_Rising_Falling,Zero_CANID3_EXTI_Isr,0x01);//PC0 -- ID3
//	RCS_InitEXTI(GPIOC,GPIO_Pin_3,EXTI_Trigger_Rising_Falling,Zero_CANID2_EXTI_Isr,0x11);//PC3 -- ID2
//	RCS_InitEXTI(GPIOC,GPIO_Pin_4,EXTI_Trigger_Rising_Falling,Zero_CANID4_EXTI_Isr,0x21);//PC4 -- ID4  
//	RCS_InitEXTI(GPIOB,GPIO_Pin_1,EXTI_Trigger_Rising_Falling,Zero_CANID3_EXTI_Isr,0x31);//PB1 -- ID1  
}

float simulate_x, simulate_y, simulate_z;
void AGV_Silumate_Move(float speed_x, float speed_y, float speed_z)
{
	simulate_x += speed_x / 12.0f;
	simulate_y += speed_y / 12.0f;
	simulate_z += speed_z * 0.005f;
}

float Get_Simulate_GPS_X()
{
	return simulate_x;
}

float Get_Simulate_GPS_Y()
{
	return simulate_y;
}

float Get_Simulate_GPS_Z()
{
	return simulate_z;
}

/**
 @name AGV_Read_Angle
 @note 读取底盘当前面向的角度（换陀螺仪则该这里面的函数即可）
**/
static __INLINE float AGV_Read_Angle(void)
{
	return Get_GPS_Z();
}

static __INLINE float AGV_Read_X(void)
{
	return Get_GPS_X();
}
static __INLINE float AGV_Read_Y(void)
{
	return Get_GPS_Y();
}