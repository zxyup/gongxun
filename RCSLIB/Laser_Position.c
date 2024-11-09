/*@filename: Laser_Positioning.c
 *@author     ���˹�       
 *@brief:     ���ⶨλ
 *@brief:     DT35��֡�ʴ���1000Hz
 *@date: 2023-7-27
*/

#include "Laser_Position.h" 


/*
��������ϵ�������£�
       ��Y
       |
       |
       |
     Z +---------�� X  
		 
�����ڳ����ϵİ�װλ������ͼ��ʾ�����ĸ����⣬���Ҹ�һ������������)��
							-----------
	1						|					|								2
--------------+--**	 *--+----------------
							|		|	 *	|
							----+--+---
									|	 |
								 3|  | 4
									
*/

/*****************ȫ�ֱ���*****************/
static float init_distance[4];
static int init_yaw;
static int i=0,j=0,k=0;
static float x[100] = {0};
static float y[100] = {0};
//static float z[20] = {0};
static float Laser_x,Laser_y,Laser_z;
/*****************��̬����*****************/

static void Get_R_Pos(float *now_pos_x,float *now_pos_y,float now_pos_z);
static void Get_L_Pos(float *now_pos_x,float *now_pos_y,float now_pos_z);
/*****************************************/


/**
	@name: Laser_Init
	@brief: ����ͨ����ʼ��
**/
void Laser_Init(void)
{
    RCS_ADC_Init(LASER_ADC_ONE, LASER_GPIO_ONE, LASER_PIN_ONE);
    RCS_ADC_Init(LASER_ADC_TWO, LASER_GPIO_TWO, LASER_PIN_TWO);
    RCS_ADC_Init(LASER_ADC_THREE, LASER_GPIO_THREE, LASER_PIN_THREE);
    RCS_ADC_Init(LASER_ADC_FOUR, LASER_GPIO_FOUR, LASER_PIN_FOUR);
}

/**
	@name: Laser_Task
	@brief: ���ⶨλ����ʹ��ʱ��Ҫ�������񣬷���ִ��
**/
void Laser_Task(void)
{
	x[i] = RCS_Get_ADC(LASER_ADC_ONE,LASER_CHANNEL_ONE);
	y[i] = RCS_Get_ADC(LASER_ADC_TWO, LASER_CHANNEL_TWO);
//	z[i] = Get_GPS_Z();
	i++;
	if(i==5)
	{
		//��λ���˲�
		Laser_x = Get_Median(x,5);
		Laser_y = Get_Median(y,5);
//		Laser_z = Get_Median(z,5);
		
		//�������˲�
		Laser_x = KalmanFilter_x(Laser_x);
		Laser_y = KalmanFilter_y(Laser_y);
		
		//ת����ʵ�����꣬�����뼤��У׼�й�
		Laser_x = (Laser_x * 1.3792f + 4172.0f+388.0f) * cos(Laser_z * DEG2RAD);
		Laser_y = (Laser_y * 0.3313f + 315.4f+388.0f) * cos(Laser_z * DEG2RAD);
		
		i = 0;
	}
}

/**
	@name: Get_Laser_GPS_X
	@brief: ��ȡX���򼤹�����
	@brief: ����2023���֮�����������
	@return: X����
**/
float Get_Laser_GPS_X(void)
{
	x[i] = RCS_Get_ADC(LASER_ADC_ONE,LASER_CHANNEL_ONE);
	i++;
	if(i==TIMES)
	{
		Laser_x = Get_Median(x,TIMES);
//		Laser_x = KalmanFilter_x(Laser_x);  //�������˲��������죬��δʹ��
		Laser_x = (5950.0f - (0.4371f * Laser_x + 4970.2f + 385.0f)) * cos(Get_GPS_Z() * DEG2RAD);  //ģ����ת��������ֵ
		i = 0;
	}
	return Laser_x;
}

/**
	@name: Get_Laser_GPS_X
	@brief: ��ȡY���򼤹�����
	@brief: ����2023���֮�����������
	@return: X����
**/
float Get_Laser_GPS_Y(void)
{
	y[j] = RCS_Get_ADC(LASER_ADC_TWO, LASER_CHANNEL_TWO);
	j++;
	if(j==TIMES)
	{
		Laser_y = Get_Median(y,TIMES);
//		Laser_y = KalmanFilter_y(Laser_y);    //�������˲��������죬��δʹ��
		Laser_y = (Laser_y * 0.3214f + 718.45f + 90.0f - 420.0f * tan(Get_GPS_Z() * DEG2RAD)) * cos(Get_GPS_Z() * DEG2RAD);  //ģ����ת��������ֵ
		j = 0;
	}	
	return Laser_y;
}

/**
	@name: Get_Laser_GPS_X
	@brief: ��ȡZ���򼤹�����
	@brief: ����2023���֮�����������
	@return: Z����
**/
float Get_Laser_GPS_Z(void)
{
	return Get_GPS_Z();
}

/**
	@name: Get_Laser_Distance_X
	@brief: ��ȡX���򼤹�����
	@brief: ����2021���ⶨλby��־ΰ
	@return: X����
**/
float Get_Laser_Distance_X(void)
{
	static float databuffx[BUFFERLEN] = {0};
	static int index = 0;
	float distance_x;
	
	databuffx[index] = (LASER_EQUIVALENT*(RCS_Get_ADC(LASER_ADC_TWO,LASER_CHANNEL_TWO) - init_distance[1]))*cos(Get_GPS_Z()*DEG2RAD);
	index++;
		
	if(index >= BUFFERLEN)
		index = 0;
		
	distance_x = -Get_Average(databuffx,BUFFERLEN);
	
	return distance_x;
	
}

/**
	@name: Get_Laser_Distance_Y
	@brief: ��ȡY���򼤹�����
	@brief: ����2021���ⶨλby��־ΰ
	@return: Y����
**/
float Get_Laser_Distance_Y(void)
{
	static float databuffy[BUFFERLEN] = {0};
	static int index = 0;
	float distance_y;
	databuffy[index] = (LASER_EQUIVALENT*(RCS_Get_ADC(LASER_ADC_THREE,LASER_CHANNEL_THREE) - init_distance[2]))*cos(Get_GPS_Z()*DEG2RAD);
	index ++;
	if(index >= BUFFERLEN)
		index = 0;
	distance_y = Get_Average(databuffy,BUFFERLEN);
	return distance_y;
	
}

/**
	@name: Get_Laser_Z
	@brief: �����ȡƫ���ǣ���Ҫע�⣬����3��4������ں󵲰����Ч������������ƺ�ƫ������ʹ�øú�����
	@brief: ����2021���ⶨλby��־ΰ
	@return: ƫ����
**/
float Get_Laser_Z(void)
{
		float temp,laser_z;
		float distance[2];
		static float databuffz[2][BUFFERLEN] = {0};
		static int index = 0;
		databuffz[0][index] = RCS_Get_ADC(LASER_ADC_THREE, LASER_CHANNEL_THREE);
    databuffz[1][index] = RCS_Get_ADC(LASER_ADC_FOUR, LASER_CHANNEL_FOUR);
		distance[0] = Recursive_Filter(databuffz[0],BUFFERLEN,index);
		distance[1] = Recursive_Filter(databuffz[1],BUFFERLEN,index);
		temp = LASER_EQUIVALENT * (distance[0] - distance[1])/LASER3X4;
		laser_z = (RAD2DEG * atan(temp) - init_yaw);
			index ++;
		if(index >= BUFFERLEN)
			index = 0;		
		return laser_z;
}


/**
	@name: Laser_Get_Pos
	@brief: ���ⶨλ
	@param:float *now_pos_x           ��������x
	@param:float *now_pos_y           ��������y
	@param:float now_pos_z            ����ƫ����
**/
void Laser_Get_Pos(float *now_pos_x,float *now_pos_y,float now_pos_z)
{
		static float databuffx[BUFFERLEN] = {0};
		static float databuffy[BUFFERLEN] = {0};
		static int index = 0;
		static int flag = 0;
		float tempx,tempy;
		
		if(*now_pos_x >=-MIDPOSX)
			flag = 0;
		if(*now_pos_x <=-MIDPOSX)
			flag = 1;
		
		if(flag == 0)
			Get_R_Pos(&tempx,&tempy,now_pos_z);
		else
			Get_L_Pos(&tempx,&tempy,now_pos_z);
		
		databuffx[index] = tempx;
		databuffy[index] = tempy;
		
		index++;
		
		if(index >= BUFFERLEN)
			index = 0;
		
		*now_pos_x = Get_Average(databuffx,BUFFERLEN);
		*now_pos_y = Get_Average(databuffy,BUFFERLEN);
}

/**
	@name: Get_R_Pos
	@brief: �Ұ볡���ⶨλ
	@param:float *now_pos_x           ��������x
	@param:float *now_pos_y           ��������y
	@param:float now_pos_z            ����ƫ����
**/
static void Get_R_Pos(float *now_pos_x,float *now_pos_y,float now_pos_z)
{
		float distance[3];
	
    distance[0] = RCS_Get_ADC(LASER_ADC_TWO, LASER_CHANNEL_TWO);
    distance[1] = RCS_Get_ADC(LASER_ADC_THREE, LASER_CHANNEL_THREE);
    distance[2] = RCS_Get_ADC(LASER_ADC_FOUR, LASER_CHANNEL_FOUR);
		
		for(int i=0;i<3;i++)
			distance[i] = distance[i]  * cos(now_pos_z * DEG2RAD) - init_distance[i+1];
		
		*now_pos_x =-LASER_EQUIVALENT * distance[0];
		*now_pos_y = LASER_EQUIVALENT * ((distance[1] + distance[2])/2.0f);
}


/**
	@name: Get_L_Pos
	@brief: ��볡���ⶨλ
	@param:float *now_pos_x           ��������x
	@param:float *now_pos_y           ��������y
	@param:float now_pos_z            ����ƫ����
**/
static void Get_L_Pos(float *now_pos_x,float *now_pos_y,float now_pos_z)
{
		float distance[3];
	
    distance[0] = RCS_Get_ADC(LASER_ADC_ONE, LASER_CHANNEL_ONE);
    distance[1] = RCS_Get_ADC(LASER_ADC_THREE, LASER_CHANNEL_THREE);
    distance[2] = RCS_Get_ADC(LASER_ADC_FOUR, LASER_CHANNEL_FOUR);
		
		for(int i=0;i<3;i++)
			distance[i] *=cos(now_pos_z * DEG2RAD);
		
		*now_pos_x = LASER_EQUIVALENT * distance[0] - 11200.0f;			//���ؿ�ȼ�ȥ����ֵ
		*now_pos_y = LASER_EQUIVALENT * (distance[1] + distance[2] - init_distance[2] - init_distance[3])/2.0f;
}


