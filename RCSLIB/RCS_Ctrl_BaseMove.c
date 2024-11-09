/**
 * @filename:RCS_Ctrl_BaseMove
 * @brief:�����ۺϿ���
 * @changlog: 2023-7-28 ���˹� �޸�����
 * @changlog: 2024-2-20 ���Ͽ� ������ֵ���
*/

/* =================ͷ�ļ�=================================*/
#include "RCS_Ctrl_BaseMove.h"

/* =================˽��ȫ�ֱ���============================*/
static int angle_flag = 0;
static int distance_flag = 0;
static float dece_distance = 0;
static int count[100] = {0};
static int i = 0;
static int acce_time;
static float acce_speed,dece_speed;

/* ==================�ⲿ����================================*/
extern __inline void RCS_USART_Send_Char( USART_TypeDef *_USARTx, uint8_t _character);
extern int rocker_lx,rocker_ly;  //��ҡ�˿����˶�
extern int rocker_rx,rocker_ry;  //��ң�п�����ת
extern int direction_key[4];

/* ==================��̬����================================*/
static double Calculate_Direction(Route_Point *rpoint,int index,int lead_point_num);
static float Speed_Limit(float speed,float max_speed,float min_speed);
static int Point_Limit(int num);
static float EndSpeedCalculate(int index,float max_speed);

/* ==================����ʵ��================================*/

/**
 * @name: Ctrl_BaseMove_Init
 * @brief: ���̿��Ƴ�ʼ��
**/
void Ctrl_BaseMove_Init(void)
{
	#ifdef USE_OMNI_CHASSIS
		M_BaseMove_Init();		//���ֵ��̳�ʼ��
	#endif 
	
	#ifdef USE_AGV_CHASSIS 
		AGV_Init();           //���ֵ��̳�ʼ��
	#endif
}

/**
 * @name:Chassis_Move
 * @brief:ֱ�ӿ��Ƶ��̵��ٶ�
*/
void Chassis_Move(float spd_x,float spd_y,float spd_z)
{
	#ifdef USE_OMNI_CHASSIS
		BaseMove_P2P(spd_x,spd_y,spd_z);
	#endif 

	#ifdef USE_AGV_CHASSIS 
		AGV_Re_Center_Move_v3_3(spd_x,spd_y,spd_z);
	#endif
}
/**
 * @name:BaseMove_Joystick
 * @brief:ң�ؿ��Ƶ��̵��ٶ�
*/
void BaseMove_Joystick(void)
{
	float thetas;
	int16_t speeds[3];
	float x_speed,y_speed;
	//chassis���̿�������
	int16_t output[2],speed[2];
	thetas=Get_Gyro_Z();
		
	  if(abs(rocker_lx-125)<10) rocker_lx=125;//��ֵ
	  if(abs(rocker_ly-128)<10) rocker_ly=128;
	  if(abs(rocker_rx-133)<10) rocker_rx=133;
		
		if (switch_key)
	  {
			speeds[0]=((float)(rocker_lx-125))*3;
			speeds[1]=((float)(rocker_ly-128))*3;
			speeds[2]=((float)(rocker_rx-133))*-3;
		}
		else
		{
			speeds[0]=((float)(rocker_lx-125))*1.5;
			speeds[1]=((float)(rocker_ly-128))*1.5;
			speeds[2]=((float)(rocker_rx-133))*-1.5;
		}
//		if (direction_key[0])
//		{
//			y_speed=1500;
//			x_speed=0;
//		}
//		else if (direction_key[1])
//		{
//			y_speed=-1500;
//			x_speed=0;
//		}
//		else if (direction_key[2])
//		{
//			y_speed=0;
//			x_speed=-1500;
//		}
//		else if (direction_key[3])
//		{
//			y_speed=0;
//			x_speed=1500;
//		}
		
		
		x_speed=speeds[0]*cos(thetas*DEG2RAD)+speeds[1]*sin(thetas*DEG2RAD);//x=Xc+Ys,X��ʱ��תz��x
		y_speed=speeds[1]*cos(thetas*DEG2RAD)-speeds[0]*sin(thetas*DEG2RAD);//y=Yc-Xs	

		Chassis_Move(x_speed,y_speed,0);
}

/**
	@name: Point2Point_ER
	@brief: ���˹����ֱ���ܵ��㷨
	@param: Coord_Point* cp  Ŀ���,����xyz,���RCS_Ctrl_BaseMove.h
	@param: Trace_Param* tp  �ܵ����,���RCS_Ctrl_BaseMove.h
	@param: int *ER_flag     ����ܵ�ı�־λ
**/
void Point2Point_ER(Coord_Point* cp,Trace_Param* tp,int *ER_flag)
{
	Coord_Point input_cp=*cp;//���ڱ������Ż�
	Trace_Param input_tp=*tp;

	float x,y,z;
	float direction,speed,rotate_speed;
	float distance;
	
	//��ȡ��ǰ����
	x = Get_GPS_Action_X();
	y = Get_GPS_Action_Y();
	z = Get_GPS_Z();
	
	direction = -z -(atan2((input_cp.x - x),(input_cp.y - y))) * RAD2DEG;      //������ǰ�˶�����
	distance = sqrt((x - input_cp.x)*(x - input_cp.x) + (y - input_cp.y)*(y - input_cp.y));   //�ж����յ����
	dece_distance = (input_tp.run_speed * input_tp.run_speed)/(2 * input_tp.DECE);    //���㿪ʼ����ʱ�ľ���
	acce_time = (int)(input_tp.run_speed/input_tp.ACCE);    //���ٴ���/ʱ��
	
	//���ٹ���
	if(count[i] < acce_time)
	{
		acce_speed = count[i] * input_tp.run_speed / acce_time;
		count[i] ++;
	}
	
	//�ж��Ƿ񵽴��յ�
	if(distance <5)
	{
		distance_flag = 1;
		dece_speed = 0;
	}
	else
	{
		distance_flag = 0;
		
		//�����˶���Χ
		if(distance > dece_distance)
		{
			dece_speed = input_tp.run_speed;
		}
		//����
		else
		{
			dece_speed = sqrt(2 * input_tp.DECE * distance);
//			dece_speed = (2 * DECE * distance)*(2 * DECE * distance);
//			dece_speed = run_speed * distance / dece_distance;;
		}
	}
	
	//�ж��Ǽ��ٻ��Ǽ���
	if(acce_speed >= dece_speed)
	{
		speed = dece_speed;
	}
	else
	{
		speed = acce_speed;
	}
	
	//��ת
	rotate_speed = PID_Angle_Ctrl(input_cp.z,Get_GPS_Z(),&tp->rotate_angle_pid);
	if(((input_cp.z-Get_GPS_Z())>=-(input_tp.R_ANGLE))&&((input_cp.z-Get_GPS_Z())<=input_tp.L_ANGLE))
	{
		angle_flag = 1;
	}
	else
	{
		angle_flag = 0;
	}	
	
	BaseMove_Polar(direction,speed,rotate_speed);
  
	//�ж��Ƿ񵽴�
	if((distance_flag == 1)&&(angle_flag == 1))
	{
		distance_flag = 0;
		angle_flag = 0;
		i++;
		*ER_flag = 1;
//		upspeed_flag = 0;
	}
	else
	{
		*ER_flag =  0;
	}
	return;
}


/**
	@name: Point2Point
	@brief: ��Ե��ƶ��Զ�����
	@param: float target_x		Ŀ��X����
	@param: float target_y		Ŀ��Y����
	@param: float target_z		Ŀ��ƫ����
**/
void Point2Point(float target_x,float target_y,float target_z,PID_Struct distance_x_pid,PID_Struct distance_y_pid,PID_Struct rotate_angle_pid)
{
	float speed[3] = {0,0,0};
	float direction,run_speed;
	
	speed[0] = PID_Normal_Ctrl(target_x,Get_GPS_X(),&distance_x_pid);
	speed[1] = PID_Normal_Ctrl(target_y,Get_GPS_Y(),&distance_y_pid);
	speed[2] = PID_Normal_Ctrl(target_z,Get_GPS_Z(),&rotate_angle_pid);
	
	direction = -Get_GPS_Z() -(atan2(speed[0],speed[1])) * RAD2DEG;
	run_speed = sqrt(speed[0] * speed[0] + speed[1] * speed[1]);
	if(run_speed>500)
	{
		run_speed = 500;
	}
	
	BaseMove_Polar(direction,run_speed,speed[2]);
}

/**
	@name: Bessel_P2P
	@brief: ���������ߵ�Ե��ƶ��Զ�����
	@param: Route_Point rpoint		���������ߵ�λ��Ϣ
	@return: ��ǰ���ߵĵ�λ��
	@explain:·���滮һ��Ҫ�����ǶȾ���ֵ��45��~135��֮�����'x'��������'y',��������һ��Ҫ�����ܵ���
**/
void Bessel_P2P(Route_Point *rpoint,int *index,int AUTO_SPEED,float target_z,PID_Struct distance_x_pid,PID_Struct distance_y_pid,PID_Struct rotate_angle_pid,PID_Struct angle_pid)
{
	float x,y,z;
	float direction,run_speed,rotate_speed;
	float temp[2],nearest_dis;
	float speed[2];
	float nearest_point[2]; 
	int int_lead[2];
	
  //��ȡ��ǰ����
	x = Get_GPS_X();
	y = Get_GPS_Y();
	z = Get_GPS_Z();
	
	temp[0] = rpoint->point[POINT_NUM-1].x - rpoint->point[0].x;
	temp[1] = rpoint->point[POINT_NUM-1].y - rpoint->point[0].y;
	
	if((temp[0] == 0) && (temp[1]== 0))								//ԭ����ת
	{
		//���㵱ǰ��λ
		nearest_point[0] = POINT_NUM * ((z - rpoint->point[0].z)/(rpoint->point[POINT_NUM-1].z - rpoint->point[0].z));
		//ǣ�����޷�
		int_lead[0] = Point_Limit(nearest_point[0])+10;
		if(Judge_Pos_B(int_lead[0]))
		{
			*index =  int_lead[0];
			BaseMove_P2P(0,0,0);
			return;
		}
		if(((rpoint->point[POINT_NUM-1].z) - (rpoint->point[0].z))>0)
		{
			rotate_speed = 100.0f;
		}
		else
		{
			rotate_speed = -100.0f;
		}
		BaseMove_P2P(0,0,rotate_speed);
		*index =  int_lead[0];					//���������
		return;
	}
	
	else if(temp[0] == 0)					//��Y�᷽���ƶ�
	{
		nearest_point[1] = POINT_NUM * ((y - rpoint->point[0].y)/temp[1]);
		int_lead[0] = Point_Limit(nearest_point[1]);
	}
	else if(temp[1] == 0)				//��X�᷽���ƶ�
	{
		nearest_point[0] = POINT_NUM * ((x - rpoint->point[0].x)/temp[0]);
		int_lead[0] = Point_Limit(nearest_point[0]);
	}
	else										//�������������ж���һ��Ҫ�ǵ������ߣ�
	{
		nearest_point[0] = POINT_NUM * ((x - rpoint->point[0].x)/temp[0]);		//X�̶���Y����������
		if(nearest_point[0] > POINT_NUM-1)
			nearest_point[0] = POINT_NUM-1;
		if(nearest_point[0] < 0)
			nearest_point[0] = 0;		
		int_lead[0] = (int)(nearest_point[0]);
		
		nearest_point[1] = POINT_NUM * ((y - rpoint->point[0].y)/temp[1]);	  //Y�̶���X����������
		if(nearest_point[1] > POINT_NUM-1)
			nearest_point[1] = POINT_NUM-1;
		if(nearest_point[1] < 0)
			nearest_point[1] = 0;		
		int_lead[1] = (int)(nearest_point[1]);
		
		temp[0] = nearest_point[0] - nearest_point[1];												//���������ȡ��
		if(temp[0] != 0)													//������������������ͬ������㲻�䣬��֮��ȡ���ߵ�������ϣ�(cos2��+1)/2 = ��n = nx-nearest
		{
			temp[1] = atan2(rpoint->point[int_lead[0]].x - rpoint->point[int_lead[1]].x,rpoint->point[int_lead[1]].y - rpoint->point[int_lead[0]].y);
			int_lead[0] =(int)(nearest_point[0] - (cos(2*temp[1])+1.0f) * temp[0] /2.0f);
			int_lead[0] = Point_Limit(int_lead[0]);
		}
	}
	
	if(Judge_Pos_B(int_lead[0]))
	{
		*index =  int_lead[0];
		BaseMove_Polar(0,0,0);
		return;
	}
	temp[0] = Calculate_Direction(rpoint,int_lead[0],1);  //������
	
	//�ж���x������y�����˶�
	if(fabsf(rpoint->speed[POINT_NUM-1].direction * DEG2RAD) < pi/4.0f*3.0f && fabsf(rpoint->speed[POINT_NUM-1].direction * DEG2RAD) > pi/4.0f)
		rpoint->direction = 'x';
	else
		rpoint->direction = 'y';
	
	//����У׼�� 
	if(rpoint->direction == 'x')
	{
		nearest_dis = (y - rpoint->point[int_lead[0]].y) * sin(temp[0]);
		run_speed = PID_Normal_Ctrl(0,nearest_dis,&distance_y_pid);
	}
	else
	{
		nearest_dis = (x - rpoint->point[int_lead[0]].x) * cos(temp[0]);
		run_speed = PID_Normal_Ctrl(0,nearest_dis,&distance_x_pid);
	}
	
	//·�߷�������
	direction = temp[0] - pi/2.0f;
	speed[0] = -run_speed * sin(direction);
	speed[1] =  run_speed * cos(direction);
	
	if(rpoint->end_speed_ctrl == 0)
	{
		//������
		speed[0] = speed[0] - sin(temp[0]) * AUTO_SPEED;
		speed[1] = speed[1] + cos(temp[0]) * AUTO_SPEED;
	}
	else              
	{
		//�����յ�ʱ����
		speed[0] = speed[0] - sin(temp[0]) * EndSpeedCalculate(int_lead[0],AUTO_SPEED);
		speed[1] = speed[1] + cos(temp[0]) * EndSpeedCalculate(int_lead[0],AUTO_SPEED);
	}
	
	//����ɼ�����
	direction = atan2(-speed[0],speed[1]) * RAD2DEG - z;
	run_speed = sqrt(speed[0] * speed[0] + speed[1] * speed[1]);
	
	//ƫ���Ƕ�����
	rotate_speed = PID_Normal_Ctrl(rpoint->point[int_lead[0]].z,z,&angle_pid);
	
	BaseMove_Polar(direction,run_speed,rotate_speed);
	*index =  int_lead[0];					//���������
	
	return;
}

/**
	@name: EndSpeedCalculate
	@brief: �ٶȼ���
	@param: float max_speed ����ٶ�
	@param:	int index ·�ߵ�����
	@return: �ٶ�ֵ
**/
static float EndSpeedCalculate(int index,float max_speed)
{
	if(index <= SLOW_POINT)
		return max_speed;
	else
		return ((max_speed-400.0f) * (1.0f - (index-SLOW_POINT)/(POINT_NUM-SLOW_POINT)))+400.0f;			//kx+b
}

/**
	@name: Calculate_Direction
	@brief: ����Ǽ���
	@param: Route_Point  *rpoint	����
	@param: int index							����
	@param:	int lead_point_num		��ǰ��
	@return: ������ķ���Ƕȣ��Ƕ��ƣ�
**/
static double Calculate_Direction(Route_Point *rpoint,int index,int lead_point_num)
{
	float temp[2];
	if(index == POINT_NUM)
	{
		temp[0] = rpoint->ctrl_point[1].x - rpoint->point[POINT_NUM-1].x;
		temp[1] = rpoint->point[POINT_NUM-1].y - rpoint->ctrl_point[1].y;
	}
	else if(index == 0)
	{
		temp[0] = rpoint->point[0].x - rpoint->ctrl_point[0].x;
		temp[1] = rpoint->ctrl_point[0].y - rpoint->point[0].y;
	}
	else
	{
		temp[0] = rpoint->point[index-1].x - rpoint->point[Point_Limit(index-1+lead_point_num)].x;
		temp[1] = rpoint->point[Point_Limit(index-1+lead_point_num)].y - rpoint->point[index-1].y;
	}
	return (atan2(temp[0],temp[1]));
}


/**
	@name: Judge_Pos
	@brief: λ�õ����ж�
	@param: float target_x					Ŀ��X
	@param: float target_y					Ŀ��Y
	@param: float target_z					Ŀ��Z
	@return: 1���0δ����
**/
int Judge_Pos(float target_x,float target_y,float target_z)
{
		float err_xy,err_z,err_speed;
		
		err_xy = fabsf(Get_KalmanFilter_X()-target_x)+fabsf(Get_KalmanFilter_Y()-target_y);
		err_z = fabsf(Get_Ave_Z()-target_z);
		for(int i =0;i<4;i++)
			err_speed = abs(Get_Motor_Speed(i+1));
	
		if((err_xy < 20) && (err_speed < 400) && (err_z < 1 ))
			return 1;
		else
			return 0;
}

/**
	@name: Judge_Pos_B
	@brief: λ�õ����ж�(������)
	@param: int index					��ǰ���������ߵ�λ
	@return: 1���0δ����
**/
int Judge_Pos_B(int index)
{
	if(index >= (POINT_NUM - 1))
		return 1;
	else
		return 0;
}

/**
	@name: Point_Limit
	@brief: �����޷�
	@param: int num				����
	@return: int 					�޷������
**/
static int Point_Limit(int num)
{
	if(num > POINT_NUM-1)
		return POINT_NUM-1;
	else if(num <0)
		return 0;
	else
		return num;
}

/**
	@name: Speed_Limit
	@brief: �ٶ��޷�
	@param: float speed				�ٶ�
	@param: float max_speed		����ٶ�
	@param: float min_speed		��С�ٶ�
	@return: float 						�޷����ٶ�
**/
static float Speed_Limit(float speed,float max_speed,float min_speed)
{
	if(speed > max_speed)
		speed = max_speed;
	if(speed < min_speed)
		speed = min_speed;
	return speed;
}
