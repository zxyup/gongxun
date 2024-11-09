/*@filename: GPS.c
 *@author     ���˹�      
 *@brief:     ȫ����λ���������ư�
 *@brief:     ���̵�֡�ʴ���1000Hz    
 *@brief:     ��λ֡��С��20Hzʱ���̻��ܲ���
 *@date: 2023-8-31
*/

/* =========ͷ�ļ�=================================*/
#include "GpsTask.h"
#include "G431_GPS.h"
/* =========��̬��������============================*/
static OS_STK my_gps_task_stk[NORMAL_TASK_STK_SIZE];
void my_gps_task(void *p_arg);

volatile float GPS_Start_X;
volatile float GPS_Start_Y;

#ifdef RCS16_GPS
/* =========˽��ȫ�ֱ���=============================*/
RCS_Spd_T Now_Encoder_Spd,Now_Base_Spd;
RCS_Pos_T Now_Pos,Start_Pos;

/* =========���������ӿ�=============================*/
/**
	@name: GPS_Init
	@brief:����ȫ����λ��ʼ��
**/
void GPS_Init(void)
{	
	GPS_Encoder_Init();							//��������ʼ��
	OSTaskCreate(my_gps_task,					//������Ϣ��������
	           (void *)0,
	           &my_gps_task_stk[NORMAL_TASK_STK_SIZE - 1],
	           my_gps_task_PRIO);
}

/**
	@name: my_gps_task
	@brief:GPS��Ϣ���������ٶȻ��ְ�
**/
void my_gps_task(void *p_arg)
{
	p_arg = p_arg;
	while(1)   
	{
		//���ݱ�������̵õ��������ٶ�
		gpos_diff_gspd(&Now_Encoder_Spd);
		//���ݱ������ٶȵõ��������ٶ�
		gspd_Kin_spd(&Now_Encoder_Spd,Get_Gyro_Z(),&Now_Base_Spd);
		//�������ٶȻ���
		bspd_cumtrupz_bpos(&Now_Base_Spd,Start_Pos,&Now_Pos);
		//200Hz����֡��
		delay_ms(5);
	}
}

#endif


#ifdef RCS13_GPS
static int32_t gps_cycle_x = 0;			//���������Ȧ��
static int32_t gps_cycle_y = 0;			
static float gps_origin_x = 0;				//ԭʼ����������
static float gps_origin_y = 0;
static volatile float gps_x = 0;			//���XYֵ
static volatile float gps_y = 0;
static volatile float raw_gps_x = 0;			//���XYֵ
static volatile float raw_gps_y = 0;
static float raw_z = 0;				
static float k1=0;
static float k2 = 0;
static float delta_zangle;						//�ָ���ƫ��������
static float rad_zangle;							//ƫ���ǣ������ƣ�
static float gps_sum_zangle = 0;			//��չ���ƫ����
static float gps_last_sum_zangle=0;
static float last_zangle = 0;				//��һʱ�̵�ƫ����
static float angle_error=0,angle_move=0; 	
static int cycle_z = 0;				//z��Ȧ��	
static float last_org_x=0,last_org_y =0;		//��һʱ��X��Y�����������Ϣ
static double delta_org[3]={0,0,0};		//�������

/**
	@name: GPS_Init
	@brief:����ȫ����λ��ʼ��
**/
void GPS_Init(void)
{	
	GPS_Encoder_Init();							//��������ʼ��
	OSTaskCreate(my_gps_task,					//������Ϣ��������
	           (void *)0,
	           &my_gps_task_stk[NORMAL_TASK_STK_SIZE - 1],
	           my_gps_task_PRIO);

}


///**
//	@name: my_gps_task
//	@brief:GPS��Ϣ�����������̰�
//**/
//void my_gps_task(void *p_arg)
//{
//	p_arg = p_arg;
//	while(1)   
//	{
//		raw_z = Get_Gyro_Z();  //��ȡƫ����
//		rad_zangle = raw_z * DEG2RAD;   //ƫ���ǻ�����
//		if(last_zangle >= 160.0f && raw_z <= -160.0f)
//			cycle_z ++;
//		if(last_zangle <= -160.0f && raw_z >= 160.0f)
//			cycle_z --;
//		
//		gps_sum_zangle = cycle_z * 360.0f + raw_z;	//��չ�Ƕ�����������		

//    //���̶�λ
//		gps_origin_x = G431GPS_Get_X()* TRANSFER_CONST;//(gps_cycle_x * 65536.0 + GPS_X_TIM->CNT) * TRANSFER_CONST * ENCODER_X_DIR;	//��ȡ����������ֵ��ת��Ϊ���ֵ
//		gps_origin_y = G431GPS_Get_Y()* TRANSFER_CONST;//(gps_cycle_y * 65536.0 + GPS_Y_TIM->CNT) * TRANSFER_CONST * ENCODER_Y_DIR;
//		
//		delta_org[0] = gps_origin_x - last_org_x;				//�Ƕ�ֵת�������ֵ���복�ְ뾶�й�
//		delta_org[1] = gps_origin_y - last_org_y;

//		raw_gps_x += delta_org[0] * cos(rad_zangle) - delta_org[1] * sin(rad_zangle);  //����任֮ǰ����ֵ
//		raw_gps_y += delta_org[0] * sin(rad_zangle) + delta_org[1] * cos(rad_zangle);
//		

// 	//������һʱ�����ֵ
//		last_org_x = gps_origin_x;								//������һʱ�̵����
//		last_org_y = gps_origin_y;
//		last_zangle = raw_z;											//������һʱ��ƫ����
//		
//    //���̶�λ
//		gps_x = raw_gps_x * cos(TRANSFORM_DZ) + raw_gps_y * sin(TRANSFORM_DZ);// + TRANSFORM_DX * (1 - cos(rad_zangle));
//		gps_y =-raw_gps_x * sin(TRANSFORM_DZ) + raw_gps_y * cos(TRANSFORM_DZ);// - sin(rad_zangle) * TRANSFORM_DX;
//		
//		delay_ms(10);
//	}
//}


/**
	@name: my_gps_task
	@brief:GPS��Ϣ�������񣬵����
**/
void my_gps_task(void *p_arg)
{
	p_arg = p_arg;
	while(1)   
	{
		
		raw_z = Get_Gyro_Z();  //��ȡƫ����
		rad_zangle = raw_z * DEG2RAD;   //ƫ���ǻ�����
		if(last_zangle >= 160.0f && raw_z <= -160.0f)
			cycle_z ++;
		if(last_zangle <= -160.0f && raw_z >= 160.0f)
			cycle_z --;
		
		
		gps_sum_zangle = cycle_z * 360.0f + raw_z;	//��չ�Ƕ�����������		
			
		gps_origin_x = Get_Motor_Float_Angle(1);	//��ȡ����������ֵ��ת��Ϊ���ֵ
		gps_origin_y = -Get_Motor_Float_Angle(2);
		
		
		delta_org[0] = (gps_origin_x - last_org_x)*TRANSFORM_X;				//�Ƕ�ֵת�������ֵ���복�ְ뾶�й�
		delta_org[1] = (gps_origin_y - last_org_y)*TRANSFORM_Y;
		
		raw_gps_x += delta_org[0] * cos(rad_zangle) - delta_org[1] * sin(rad_zangle);  //����任֮ǰ����ֵ
		raw_gps_y += delta_org[0] * sin(rad_zangle) + delta_org[1] * cos(rad_zangle);
		

 	//������һʱ�����ֵ
		last_org_x = gps_origin_x;								//������һʱ�̵����
		last_org_y = gps_origin_y;
		last_zangle = raw_z;											//������һʱ��ƫ����
		
	//����ռ�任����λ��-��������
		gps_x =-raw_gps_x * cos(ANGLE_DX) - raw_gps_y * sin(ANGLE_DY) + sin(rad_zangle) * TRANSFORM_AX;  //TRANSFORM_AX/AY�ǳ������붨λ���ĵľ���
		gps_y = raw_gps_x * sin(ANGLE_DX) - raw_gps_y * cos(ANGLE_DY) + TRANSFORM_AY * (1 - cos(rad_zangle));
		
		gps_x = gps_x/10.0f;
		gps_y = gps_y/10.0f;
		
		
		delay_ms(2);
	}
}

volatile float refer_gps_x;
volatile float refer_gps_y;
volatile float true_laser_x;
volatile float true_laser_y;
void Set_GPS_X(float laser_x)
{
	refer_gps_x=Get_GPS_X();
	true_laser_x=laser_x;
}
void Set_GPS_Y(float laser_y)
{
	refer_gps_y=Get_GPS_Y();
	true_laser_y=laser_y;
}


/**
	@name: Get_GPS_X
	@brief:��ȡX����
**/
float Get_GPS_X(void)
{
	//return Get_GPS_Action_X();
	return gps_x;
}

/**
	@name: Get_GPS_Y
	@brief:��ȡY����
**/
float Get_GPS_Y(void)
{
	//return Get_GPS_Action_Y();
	return gps_y;
}

/**
	@name: Get_GPS_Z
	@brief:��ȡ��չƫ����
**/
float Get_GPS_Z(void)
{
//	return Get_GPS_Action_Z();
	return gps_sum_zangle;
}

/**
	@name: Get_GPS_Ori_X
	@brief:���شӶ���ת��·��
**/
float Get_GPS_Ori_X(void)
{
	return gps_origin_x;
}

/**
	@name: Get_GPS_Ori_Y
	@brief:���شӶ���ת��·��
**/
float Get_GPS_Ori_Y(void)
{
	return gps_origin_y;
}

/**
	@name: Get_GPS_Ori_X
	@brief:���شӶ���ת��·��
**/
float Get_GPS_Raw_X(void)
{
	return raw_gps_x;
}

/**
	@name: Get_GPS_Ori_Y
	@brief:���شӶ���ת��·��
**/
float Get_GPS_Raw_Y(void)
{
	return raw_gps_y;
}

int32_t gps_cycle_test_x(void)
{
	return gps_cycle_x;
}

int32_t gps_cycle_test_y(void)
{
	return gps_cycle_y;
}

/**
	@name: X_Cycle
	@brief: X������Ȧ���Լ�
	@param:uint8_t flag  �Լӻ����Լ��ź�
**/
void X_Cycle(uint8_t flag)
{
	if(flag)
		gps_cycle_x++;
	else 
		gps_cycle_x--;
}
/**
	@name: Y_Cycle
	@brief: Y������Ȧ���Լ�
	@param:uint8_t flag  �Լӻ����Լ��ź�
**/
void Y_Cycle(uint8_t flag)
{
	if(flag)
		gps_cycle_y++;
	else 
		gps_cycle_y--;
}
#endif