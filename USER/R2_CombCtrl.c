#include "R2_CombCtrl.h"
#include "Ch_Ctrl.h"

/*********************************************************************
 * 
 * @addtogroup:宏定义
 * 
 * *******************************************************************/
void pick_ball_init(void);
#define SCARA_MAX_SIZE 600.0f
#define ABSOLUTE_SCARA 1
#define RELATIVE_SCARA 0
#define Cup_Center_Pixcel 317
#define Cup_Window_Len    10
#define Cup_Tor_Valid_Percent 0.5f
#define Cup_Tor_Valid_Var_X   15
#define Cup_Tor_Valid_Var_D   50

#define GANTRY_FINDBALL_POS   700.0f

#define ACTOR_UP_POS    0.0f
#define ACTOR_DOWN_POS  -5000.0f

#define SCARA_MAIN_CUPCMA_CENTER_DEG 150.0f   //奥比相机正视车体中心时的大臂角度
#define SCARA_END_FOLD_POS_DEG       -160.0f  //小臂完全折叠的角度
#define FINDBALL_SCAN_SPD            200.0f   //找球时扫描的速度
#define FINDBALL_SCAN_P_DEG                
extern Motor_Ctrl_Node R2_Scara_Main;
/*********************************************************************
 * 
 * @addtogroup:全局变量
 * 
 * *******************************************************************/

//桶框的位置
volatile float Pos_Silo[5][2][3];//[silo][zone][x/y/z]
//大臂找球PID
PID_Struct     FindBall_Main_Pid;
//底盘找球PID
DacePID_Struct FindBall_Chassis_Pid;
//底盘跑点PID
DacePID_Struct     agv_z_pid;
DacePID_Struct     agv_y_pid;
DacePID_Struct     agv_x_pid;

//传入底盘进行速度控制的值
float agv_target_x;
float agv_target_y;
float agv_target_z;

float agv_start_x;
float agv_start_y;

AGV_Speed HAL_Last_Output;

//复合动作日志
LogServer_t Combo_Log;

//上一次的输出值
float target_scara_x=450.0f;
float target_scara_y=0.0f;
float target_gantry;
float target_actor;
uint8_t punk_status;
uint8_t swiv_status;
float safe_x;
float safe_y;

//日志
int8_t scara_err;
int8_t comm_err;

//lasered xy
volatile float test_X;
volatile float test_Y1;
volatile float test_Y2;

//奥比中光返回的球位置的滑动窗口
volatile Comm_WindowData_t Vision_Cup_Window_X;
volatile Comm_WindowData_t Vision_Cup_Window_Y;
volatile Comm_WindowData_t Vision_Cup_Window_D;

volatile Comm_WindowData_t Vision_Pick_Window_X;
volatile Comm_WindowData_t Vision_Pick_Window_Y;

//奥比中光相机返回数据，经处理过后得到有效参数
float valid_cup_var_x;
float valid_cup_var_y;
float valid_cup_mean_x;
float valid_cup_mean_y;
float valid_cup_mean_d;
float valid_cup_percent;
float valid_vision_cup_xyd[3][60];

//面前8个球的相对坐标
float ball_id_pos[8][2];
//窗口内允许的最小有效数据比
float ball_min_valid_percent=VISION_MIN_VALID_PERCENT;
//为了看到球，允许的最高龙门架高度
float ball_gantry_max_pos=17000.0f;
//为了看到球，允许的最小龙门架高度
float ball_gantry_min_pos=13000.0f;
//捡球到球的龙门架高度
float ball_pick_gantry=600.0f;
//捡球scara的加减速pid
DacePID_Struct pick_x_pid;
DacePID_Struct pick_y_pid;
//捡球的目标中心点
uint16_t pick_center_pixcel_x;
uint16_t pick_center_pixcel_y;
//捡球窗口的大小
uint8_t pick_window_len=5;

//捡整球状态机
int8_t Pick_BlockBall_State;
//捡球摄像头窗口内的有效占比
volatile float  pick_valid_percent;
//捡球摄像头窗口内的有效方差
volatile float  pick_valid_var;
//捡球摄像头窗口内的均值
volatile float  pick_mean_x;
volatile float  pick_mean_y;
//scara闭环捡球输出值
float  pick_output_x;
float  pick_output_y;
//当前龙门架的高度
float  current_gantry;
//相机坐标系下球的位置
volatile DESCARTES_AXIS pick_camera_mean_pos;
volatile float pic_camera_pos_x;
volatile float pic_camera_pos_y;
//车体坐标系下球的位置
volatile DESCARTES_AXIS pick_chassis_mean_pos;
volatile float pic_chassis_pos_x;
volatile float pic_chassis_pos_y;

//isolate ball performance
volatile float isolate_ball_valid_percent;
volatile float isolate_ball_var_x;
volatile float isolate_ball_var_y;
volatile float isolate_ball_mean_x;
volatile float isolate_ball_mean_y;
volatile float isolate_ball_mean_d;
volatile float isolate_ball_scara_out_x;
volatile float isolate_ball_scara_out_y;

volatile float isolate_ball_chassis_output_r;
volatile float isolate_ball_chassis_output_x;
volatile float isolate_ball_chassis_output_y;
volatile float isolate_ball_scara_output_main;
volatile float isolate_ball_chassis_output_follow_main_x;
volatile float isolate_ball_chassis_output_follow_main_y;

int8_t Pick_IsolateBall_State;

/*********************************************************************
 * 
 * @addtogroup:初始化
 * 
 * *******************************************************************/

void CombCtrl_Param_Init(RCS_PIN_USART USARTx_MAP)
{
	//距离环PID参数
	FindBall_Main_Pid.P=-3.0f;
	FindBall_Main_Pid.Limit_Output=3200.0f;
	
	FindBall_Chassis_Pid.pid.P=10.0f;
	FindBall_Chassis_Pid.pid.Limit_Output=10000.0f;
	FindBall_Chassis_Pid.dead_zone=15.0f;
	FindBall_Chassis_Pid.max_acce=100.0f;
	FindBall_Chassis_Pid.max_dcce=200.0f;

	agv_x_pid.pid.P=0.2f;//0.1f;//
	agv_x_pid.pid.Limit_Output=600.0f;//330.0f;//
	agv_x_pid.max_acce=2.0f;//1.0f;//
	agv_x_pid.max_dcce=agv_x_pid.pid.Limit_Output*0.8;
	agv_x_pid.dead_zone=50.0f;
	
	agv_y_pid.pid.P=0.2f;//0.1f;//
	agv_y_pid.pid.Limit_Output=600.0f;//330.0f;//
	agv_y_pid.max_acce=2.0f;//1.0f;//
	agv_y_pid.max_dcce=agv_y_pid.pid.Limit_Output*0.8;
	agv_y_pid.dead_zone=50.0f;
	
	agv_z_pid.pid.P=6.0f;
	agv_z_pid.pid.Limit_Output=300.0f;//200.0f;
	agv_z_pid.max_acce=13.0f;
	agv_z_pid.max_dcce=50.0f;
	agv_z_pid.dead_zone=0.5f;

	
	//速度环PID参数

	//滑动窗口
	WindowFloat_Init_Slide(&Vision_Cup_Window_X,10);
	WindowFloat_Init_Slide(&Vision_Cup_Window_Y,10);
	WindowFloat_Init_Slide(&Vision_Cup_Window_D,10);
	WindowFloat_Init_Slide(&Vision_Pick_Window_X,15);
	WindowFloat_Init_Slide(&Vision_Pick_Window_Y,15);

	//日志系统
	LogServer_Init(&Combo_Log,USARTx_MAP.USARTx,"Combo\0");
	
	pick_ball_init();
}


/*********************************************************************
 * 
 * @addtogroup:HAL层-底盘
 * 
 * *******************************************************************/
//底盘控制函数根据此函数获取X坐标
float HAL_Chassis_Get_X(void)
{
	//return ChCtrl_DT35_Get_X();
	return Get_GPS_X();
}
//底盘控制函数根据此函数获取Y坐标
float HAL_Chassis_Get_Y(void)
{
	return Get_GPS_Y();
}
//底盘控制函数根据此函数获取Z坐标
float HAL_Chassis_Get_Z(void)
{
	return Get_GPS_Z();
}
//设置底盘上次的速度输出，一般用于速度控制转距离控制，避免控制不连续
void HAL_Chassis_Reset(float x_last_output,float y_last_output,float z_last_output)
{
	agv_x_pid.last_output=x_last_output;
	agv_y_pid.last_output=y_last_output;
	agv_z_pid.last_output=z_last_output;
}
//
uint8_t HAL_Chassis_Ctrl(float param_x,float param_y,float param_z)
{
		agv_target_x=DacePID_Normal_Ctrl(param_x,Get_GPS_X(),&agv_x_pid);
		agv_target_y=DacePID_Normal_Ctrl(param_y,Get_GPS_Y(),&agv_y_pid);
		agv_target_z=DacePID_Normal_Ctrl(param_z,Get_GPS_Z(),&agv_z_pid);
		BaseMove_P2P(agv_target_x,agv_target_y,agv_target_z);
		if (agv_target_x==0 && agv_target_y==0 && agv_target_z==0)
				return 2;
		else if ((fabsf(param_x-Get_GPS_X())<=50.0f)&&(fabsf(param_y-Get_GPS_Y())<=50.0f))   //sz--pilo
				return 6;
		else                                                       
				return 0;
}
void Laser_Reset_GPS_III(uint8_t zone)
{
	float zone_compensate;
	if (zone==ZONE_BLUE) zone_compensate=-1.0f;
	else                 zone_compensate=1.0f;
	
	test_X=zone_compensate*(5275.0f-ChCtrl_Laser_Get_cY());
	test_Y1=11000.0f-ChCtrl_Laser_Get_lX();
	test_Y2=11000.0f-ChCtrl_Laser_Get_rX();
	
	if (zone==ZONE_BLUE)
	{
		if ((fabsf(HAL_Chassis_Get_Z()-90.0f*zone_compensate)<0.5f)&&(HAL_Chassis_Get_X()<=5000.0f*zone_compensate))
		{
			Set_GPS_X(zone_compensate*(5475.0f-ChCtrl_Laser_Get_cY()));
			Set_GPS_Y(11000.0f-ChCtrl_Laser_Get_lX());
		}
	}
	else
	{
		 if ((fabsf(HAL_Chassis_Get_Z()-90.0f*zone_compensate)<0.5f)&&(HAL_Chassis_Get_X()>=5000.0f*zone_compensate))
		{
			Set_GPS_X(zone_compensate*(5475.0f-ChCtrl_Laser_Get_cY()));
			Set_GPS_Y(11000.0f-ChCtrl_Laser_Get_rX());
		}
	}
}

/*********************************************************************
 * 
 * @addtogroup:HAL层-上层
 * 
 * *******************************************************************/
//大臂角度重映射函数
float Gyro_Main_Scara_Get_Angle(uint8_t id)
{
	return Get_Motor_Rad_Angle2_M3508(id)+HAL_Chassis_Get_Z()/R2_SCARA_MAIN_SLOWDOWN_RATE*DEG2RAD;
}
//判断尺寸是否即将超过
uint8_t HAL_Scara_Limit_Judge(void)
{
	DESCARTES_AXIS current_scara_pos=R2_Scara_Get_Pos();
	float size=sqrtf(current_scara_pos.x*current_scara_pos.x+current_scara_pos.y*current_scara_pos.y);
	if (size >=SCARA_MAX_SIZE)    return 1;
	else                          return 0;
}
//修正尺寸输出
uint8_t HAL_Scara_Limit_Param(float* x_or_main,float* y_or_end,uint8_t* ctrl_type)
{
	float direction;
	DESCARTES_AXIS current_scara_pos;
	volatile uint8_t ctrl_typer=*ctrl_type;
	if (HAL_Scara_Limit_Judge()==1)
	{
		switch(ctrl_typer)
		{	
			//笛卡尔坐标控制：同方向，收缩到限位
			case 0:
				if (sqrtf((*x_or_main)*(*x_or_main)+(*y_or_end)*(*y_or_end))>=SCARA_MAX_SIZE)
				{
					direction=atan2f(*y_or_end,*x_or_main);
					*x_or_main=cosf(direction)*SCARA_MAX_SIZE;
					*y_or_end=sinf(direction)*SCARA_MAX_SIZE;
				}
			break;

			//笛卡尔速度控制：同方向，不允许向半径外有速度
			case 1:
				current_scara_pos=R2_Scara_Get_Pos();
				float out_direction=atan2f(*y_or_end,*x_or_main);//output spd vector
				float cur_direction=atan2f(current_scara_pos.y,current_scara_pos.x);//current pos vector
				float oct_cur_dir_spd=sqrtf((*y_or_end)*(*y_or_end)+(*x_or_main)*(*x_or_main))*sinf(out_direction-cur_direction);
				float pct_cur_dir_spd=sqrtf((*y_or_end)*(*y_or_end)+(*x_or_main)*(*x_or_main))*cosf(out_direction-cur_direction);
				
				if (pct_cur_dir_spd>=0)
				{
					*y_or_end=cosf(cur_direction)*oct_cur_dir_spd;
					*x_or_main=-sinf(cur_direction)*oct_cur_dir_spd;
				}
			break;

			//Scara电机角度控制：不允许小臂大于-60度或小于-140度
			case 2:
				if (*y_or_end>=-60.0f) *y_or_end=-60.0f;
				if (*y_or_end<=-140.0f) *y_or_end=-140.0f;
			break;

			//Scara电机速度控制：超过限制，直接不允许小臂转动
			case 3:
				*y_or_end=0;
			break;
		}
		return 1;
	}
	else
	{
		return 0;
	}
}
//安全限制完善的Scara机械臂控制
int8_t HAL_Scara_Action(float x_or_main,float y_or_end,uint8_t ctrl_type,uint8_t is_absolute)
{
	
	volatile uint8_t safe_ct=ctrl_type;
	volatile int8_t  reval;
	safe_x=x_or_main;
	safe_y=y_or_end;
	//相对于底盘坐标系的scara控制
	if (is_absolute==RELATIVE_SCARA)
	{
		R2_Scara_Main.Get_Node_Angle=&Get_Motor_Rad_Angle2_M3508;
		scara_err=HAL_Scara_Limit_Param(&safe_x,&safe_y,&safe_ct);
		reval=R2_Scara_Action(safe_x,safe_y,safe_ct);
	}
	//相对于场地坐标系的scara控制
	else if (is_absolute==ABSOLUTE_SCARA)//bu yong
	{
		R2_Scara_Main.Get_Node_Angle=&Gyro_Main_Scara_Get_Angle;
		scara_err=HAL_Scara_Limit_Param(&safe_x,&safe_y,&safe_ct);
		reval=R2_Scara_Action(safe_x,safe_y,safe_ct);
	}
	
	return reval;
}
//安全限制完善的上层控制
int8_t HAL_Comm_Action(float gantry_pos,float actor_pos,uint8_t punk_status,uint8_t swiv_status,uint8_t gantry_ctrl_type)
{
	static float last_gantry_pos=0;
	float safe_gantry_pos;
	float safe_actor_pos;
	volatile int8_t reval;
	
	switch(gantry_ctrl_type)
	{
		case GANTRY_CTRL_TYPE_POS:
			if (gantry_pos<=100.0f) safe_gantry_pos=100.0f;
			else                    safe_gantry_pos=gantry_pos;
			if (safe_gantry_pos>=20000.0f) safe_gantry_pos=20000.0f;
//			if (actor_pos<=-2400.0f)  safe_actor_pos=-2400.0f;
//			else                     safe_actor_pos=actor_pos;
		safe_actor_pos=actor_pos;//替换上文
			reval=(int8_t)R2_Comm_Action(safe_gantry_pos,safe_actor_pos,punk_status,swiv_status);
			last_gantry_pos=safe_gantry_pos;
			return reval;
		break;
		
		case GANTRY_CTRL_TYPE_DIFF_POS:
			safe_gantry_pos=last_gantry_pos+gantry_pos;
			if (safe_gantry_pos<=100.0f)   safe_gantry_pos=100.0f;
			if (safe_gantry_pos>=20000.0f) safe_gantry_pos=20000.0f;
//			if (actor_pos<=-2400.0f)  safe_actor_pos=-2400.0f;
//			else                     safe_actor_pos=actor_pos;
		safe_actor_pos=actor_pos;//替换上文
			reval=(int8_t)R2_Comm_Action(safe_gantry_pos,safe_actor_pos,punk_status,swiv_status);
			last_gantry_pos=safe_gantry_pos;
			return reval;
		break;
	}
}


/*********************************************************************
 * 
 * @addtogroup:视觉数据处理
 * 
 * *******************************************************************/
//对奥比中光相机的数据做滑动窗口处理
void Vision_Cup_ValidCheck_Main(uint8_t window_size)
{
	//更新窗口参数
	WindowFloat_Update_Size(&Vision_Cup_Window_X,window_size);
	WindowFloat_Update_Size(&Vision_Cup_Window_Y,window_size);
	WindowFloat_Update_Size(&Vision_Cup_Window_D,window_size);

	//获取当前视觉数据
	volatile float Cup_X=Vision_Cup_Get_Ball_Pos_X();
	volatile float Cup_Y=Vision_Cup_Get_Ball_Pos_Y();
	volatile float Cup_D=Vision_Cup_Get_Ball_Dist();

	//将视觉数据送入窗口
	WindowFloat_Update_Member(&Vision_Cup_Window_X,Cup_X);
	WindowFloat_Update_Member(&Vision_Cup_Window_Y,Cup_Y);
	WindowFloat_Update_Member(&Vision_Cup_Window_D,Cup_D);

	//计算滑动窗口的性能参数
	volatile float temp_float_x;
	volatile float temp_float_y;
	volatile float temp_float_d;
	uint8_t valid_vision_count; 

	if (WindowFloat_Get_Redy_Flag(&Vision_Cup_Window_X))
	{
		valid_vision_count=0;
		//遍历整个窗口，收集有效数据
		for(int i=0;i<Vision_Cup_Window_X.window_size;i++)
		{
			//拷贝浮点数
			memcpy(&temp_float_x,WindowFloat_Get_Member_Ptr(&Vision_Cup_Window_X,i),sizeof(float));
			memcpy(&temp_float_y,WindowFloat_Get_Member_Ptr(&Vision_Cup_Window_Y,i),sizeof(float));
			memcpy(&temp_float_d,WindowFloat_Get_Member_Ptr(&Vision_Cup_Window_D,i),sizeof(float));

			//判断是否有效	
			if ((temp_float_x==0.0f)&&(temp_float_y==0.0f)&&(temp_float_d==0.0f))
			{
				//无效数据不做处理
				__NOP();
			}
			else
			{
				//有效数据加入数组中
				valid_vision_cup_xyd[0][valid_vision_count]=temp_float_x;
				valid_vision_cup_xyd[1][valid_vision_count]=temp_float_y;
				valid_vision_cup_xyd[2][valid_vision_count]=temp_float_d;
				valid_vision_count++;
			}		
		}

		//求有效数据的性能参数
		valid_cup_var_x =ArrayFloat_Get_Var(valid_vision_cup_xyd[0],valid_vision_count);
		valid_cup_var_y =ArrayFloat_Get_Var(valid_vision_cup_xyd[1],valid_vision_count);
		valid_cup_mean_x=ArrayFloat_Get_Mean(valid_vision_cup_xyd[0],valid_vision_count);
		valid_cup_mean_y=ArrayFloat_Get_Mean(valid_vision_cup_xyd[1],valid_vision_count);
		valid_cup_mean_d=ArrayFloat_Get_Mean(valid_vision_cup_xyd[2],valid_vision_count);
		
		if (window_size!=0) 
			valid_cup_percent=(1.0f*valid_vision_count)/(1.0f*window_size);
		else 
			valid_cup_percent=1.0f;
	}
}

//获取滑动窗口内的性能参数
void Vision_Cup_Valid_Get_Performance(float* valid_percent,float* var_x,float* var_y)
{
	*valid_percent=valid_cup_percent;
	*var_x=valid_cup_var_x;
	*var_y=valid_cup_var_y;
}

//获取滑动窗口处理过后的平滑数据
void Vision_Cup_Valid_Get_Data(float* mean_x,float* mean_y,float* mean_d)
{
	*mean_x=valid_cup_mean_x;
	*mean_y=valid_cup_mean_y;
	*mean_d=valid_cup_mean_d;
}

/*********************************************************************
 * 
 * @addtogroup:复合动作-捡球

//捡整球的前置条件：定位要准，不能把球撞散，也不能离太远。可以稍微撞一下，但是不能撞散
//如下捡整球策略中，完全没有考虑出乎该前置条件的情况，也不应该考虑这些情况

//龙门架高低问题：如果现在就能看到球，那么把龙门架升高是不必要的操作。
//但如果视野边缘的数据容易跳动，看到球，进入case1只是偶然，龙门架就需要升高。
//因为球不可能被撞散，所以龙门架升高后，一定能看到球，但又会出现看到取不到的球的情况
//因此不如就按变量最少的最低龙门高度作为跟球高度，出了问题，再根据结果另谈
//最低龙门高度的要求是，在定位精准的前提下，该龙门高度下，其他球不能取代目标球成为返回球

//底盘移动问题：机械臂抵达限位，此时就需要进行底盘移动
//抵达限位又分为两种情况：目标球被更远的球替代，追更远的球去了；球散了，追不到
//在理想情况下，这两种情况都是不会发生的，因此仍然选择最为简单的实现方式：球乱就乱，底盘跟上，出了其他问题，再根据结果另谈

//数据有效问题：视野边缘的球往往不具备高的识别率，但是强行忽视的话会导致视野缩小。
//跟随低识别率球，有可能会导致被误差吸引。
//解决方式是通过方差，区分乱跳的数据和只能偶尔看到的数据，因为随机误差不可能具有很小的方差。
//不同于上述两个问题，本答案不需要根据结果补全后续的解决方案，能想到的可能情况，本方案都完美解决了，但要注意，大部分时候都不是这样，在编写代码时，都需要强行假定某些不那么重要的情况作为前提，然后通过实验来验证

 * *******************************************************************/
/**
 * @name: pick_ball_init
 * @brief:捡球参数初始化
 **/
void pick_ball_init(void)
{
	pick_center_pixcel_x=CAMERA_AXIS_CENTER_X;
	pick_center_pixcel_y=CAMERA_AXIS_CENTER_Y;
	
	pick_x_pid.pid.P=-6.0f;
	pick_x_pid.pid.Limit_Output=3000.0f;
	pick_x_pid.max_acce=10.0f;
	pick_x_pid.max_dcce=90.0f;
	pick_x_pid.dead_zone=10.0f;
	
	pick_y_pid.pid.P=-6.0f;
	pick_y_pid.pid.Limit_Output=3000.0f;
	pick_y_pid.max_acce=10.0f;
	pick_y_pid.max_dcce=90.0f;
	pick_y_pid.dead_zone=10.0f;
	
	WindowFloat_Init_Slide(&Vision_Pick_Window_X,pick_window_len);
	WindowFloat_Init_Slide(&Vision_Pick_Window_Y,pick_window_len);
}

uint8_t all_route_test_state;
uint8_t arriver_flag;
int8_t gongxun_state;
int8_t gongxun()
{
	float zone_compensate;
	
	switch(gongxun_state)
	{
		//to II
		case 0:
			arriver_flag=HAL_Chassis_Ctrl(0.0f,5000.0f,0.0f);
			//=HAL_Chassis_Ctrl(0.0f*zone_compensate,5400.0f+150.0f,0.0f,AGV_CTRL_TYPE_POS);
			if (arriver_flag==1) gongxun_state=1;//1
		break;
		//II at 1
		case 1:
			HAL_Scara_Action(0.0f,-200.0f,SCARA_CTRL_TYPE_DPOS,RELATIVE_SCARA);
			HAL_Comm_Action(0.0f,ACTOR_UP_POS,1,1,GANTRY_CTRL_TYPE_POS);
			arriver_flag=HAL_Chassis_Ctrl(0.0f*zone_compensate,5400.0f+150.0f,0.0f);
			if (arriver_flag==1) gongxun_state++;//1
		break;
//		//II at 2
//		case 2:
//			HAL_Scara_Action(0.0f,-200.0f,SCARA_CTRL_TYPE_DPOS,RELATIVE_SCARA);
//			HAL_Comm_Action(0.0f,ACTOR_UP_POS,1,1,GANTRY_CTRL_TYPE_POS);
//			if (arriver_flag==3) gongxun_State++;//3
//		break;
//		//go up
//		case 3:
//			HAL_Scara_Action(0.0f,-200.0f,SCARA_CTRL_TYPE_DPOS,RELATIVE_SCARA);
//			HAL_Comm_Action(0.0f,ACTOR_UP_POS,1,1,GANTRY_CTRL_TYPE_POS);
//			if (arriver_flag==4) gongxun_State++;//4
//		break;
//		
//		case 5:
//			arriver_flag=HAL_Chassis_Ctrl(1000.0f*zone_compensate,8750.0f+150.0f,45.0f*zone_compensate,AGV_CTRL_TYPE_POS);
//			if (arriver_flag==2) return 1;
//		break;
	}
	return 0;
}

void pickball_Ctrl_test(uint8_t gantary,uint8_t claw)
{
	int catch_fsm = 0;
	switch (catch_fsm)
	{
		
	}	
}

/*********************************************************************
 * 
 * @addtogroup:调试与性能测试
 * 
 * *******************************************************************/
void CombCtrl_JoyStick(void)
{
	
	static float agv_spd[3];
	
	if(abs(rocker_lx-125)<10) rocker_lx=125;
	if(abs(rocker_ly-128)<10) rocker_ly=128;
	if(abs(rocker_rx-133)<10) rocker_rx=133;
		

	agv_spd[0]=((float)(rocker_lx-125))*1;
	agv_spd[1]=((float)(rocker_ly-128))*1;
	agv_spd[2]=((float)(rocker_rx-133))*-1;
	
	if (key_flag[0]) target_gantry-=20;
	if (key_flag[1]) target_gantry+=20;
	if (key_flag[2]) target_actor-=4;
	if (key_flag[3]) target_actor+=4;
	punk_status = switch_key;
	swiv_status = key_flag[4];
		
	if (direction_key[0]) target_scara_y+=1;
	if (direction_key[1]) target_scara_y-=1;
	if (direction_key[2]) target_scara_x-=1;
	if (direction_key[3]) target_scara_x+=1;
	
	if (stop_key)
	{
		HAL_Chassis_Ctrl(agv_spd[0],agv_spd[1],agv_spd[2]);
	}
	else
	{
		
	}
	
}
