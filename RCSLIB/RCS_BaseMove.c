/*@filename: RCS_BaseMove.c
 *@author     ���˹�       
 *@brief:     �����ٶȽ���
 *@date: 2023-7-28
*/
#include "RCS_BaseMove.h"

/*******************
X�ͳ��������ֵ��̣�Ĭ��CAN1ͨѶ
�����š��������ϵ�������תʱ���ӵ���Ч��������
        ��Y
        |
 1�J----+---�K2
   |    |   |
   |    +---+----��X
   |        |��
 4�I--------�L3
       ��

��������ϵ��������
       ��Y
       |
       |
       |
     Z +---------�� X      
********************/

/********ȫ�ֱ���********/
static float wheel_dir[4];		   //�����תʱ�����˶���������
static float wheel_distance;		 //���Ӻ͵������ĵþ���
static float wheel_w_dir[4];		 //������תʱ��ת���ٶ��ڸ������ϵķ���

Motor_Ctrl_Node base_motor[4];
PID_Struct base_speed_pid[4];

/********��̬����********/
static void Wheel_Init(void);

/************************/


/**
	@name: M_BaseMove_Init
	@brief: ���ֵ��̳�ʼ��
**/
void M_BaseMove_Init(void)
{
	for(int i=0;i<4;i++)
	{
		base_speed_pid[i]=PID_Get_RM3508_Speed_Pid();
		MotorNode_Init_C620(CAN_GROUP_1,i,&base_motor[i]);
		MotorNode_Add_SpeedPid(&base_motor[i],&base_speed_pid[i]);
		MotorNode_Add(CAN_GROUP_1,&(base_motor[i]));
		
	}
		Motor_Init();		//�����ʼ��
		Wheel_Init();		//���Ӳ�����ʼ��
}

/**
	@name: BaseMove_P2P
	@brief: ���ֵ��̺����㷨(ֱ������ʽ)
	@param:float move_speed_x             ����ƽ���ٶ�(r/min)
	@param:float move_speed_y							����ƽ���ٶ�(r/min)
	@param:float rotate_speed             ��תģʽ(rad/s��ʱ��Ϊ��,�ֶ�����ʱ����)
**/
void BaseMove_P2P(float move_speed_x,float move_speed_y,float rotate_speed)
{
    float target_speed[4] = {0,0,0,0};			//Ŀ��ת��
		float max_speed= 0;
		int32_t out[4]={0,0,0,0};

    for(int i=0;i<4;i++)
		{
				target_speed[i] += move_speed_x * cos(-pi/2.0f - wheel_dir[i]) + move_speed_y * cos( 0 - wheel_dir[i]); //ƽ���ٶ������������ͶӰ
				target_speed[i] += rotate_speed * cos(wheel_w_dir[i] - wheel_dir[i]);									//ת�����ٶ������������ͶӰ
				target_speed[i] *= MM_S2R_MIN;				//��������������ٶ�ת��Ϊ����ת��
				target_speed[i] *= SLOWDOWN_RATE;		  //������ת��ת��Ϊ������ת���Ա���PID����
				if(fabsf(target_speed[i]) > max_speed)
					max_speed = fabsf(target_speed[i]);
		}
		
		if(max_speed > MAX_MOTOR_SPEED)
		{
			for(int i=0;i<4;i++)
				target_speed[i] = target_speed[i] * MAX_MOTOR_SPEED / max_speed;
		}
		
		for(int i= 0;i<4;i++)
			out[i] = PID_Motor_Ctrl(target_speed[i],Get_Motor_Speed(i+1),&base_speed_pid[i]); //�ٶȻ�

		Motor_Send(out[0], out[1], out[2], out[3]);
}

/**
	@name: BaseMove_Polar
	@brief:���ֵ��̺����㷨(������ʽ)
	@param:float direction_angle	�г�����
	@param:float run_speed				�г��ٶ�
	@param:float rotate_speed		��ת�ٶ�
**/
void BaseMove_Polar(float direction_angle,float run_speed,float rotate_speed)
{
    float target_speed[4] = {0};			//Ŀ��ת��
		float max_speed = 0;							//����ٶ�����
		int32_t out[4];

    for(int i=0;i<4;i++)
		{
				target_speed[i] += run_speed * cos(direction_angle * DEG2RAD - wheel_dir[i]); 			//ƽ���ٶ������������ͶӰ
				target_speed[i] += rotate_speed * cos(wheel_w_dir[i] - wheel_dir[i]);								//ת�����ٶ������������ͶӰ
				if(fabsf(target_speed[i]) > max_speed)
					max_speed = fabsf(target_speed[i]);
		}
		if(max_speed > MAX_MOTOR_SPEED)
		{
			for(int i=0;i<4;i++)
				target_speed[i] = target_speed[i] * MAX_MOTOR_SPEED / max_speed;
		}
		for(int i= 0;i<4;i++)
			out[i] = PID_Motor_Ctrl(target_speed[i],Get_Motor_Speed(i+1),&base_speed_pid[i]); //�ٶȻ�

		Motor_Send(out[0], out[1], out[2], out[3]);
}


/**
	@name: BaseMove_Vision
	@brief:���ֵ��̺����㷨(�Ӿ�ʽ)
	@param:float move_speed     	       	�Ӿ�����ƽ���ٶ�
	@param:float rotate_speed             ��ת�ٶ�
**/
void BaseMove_Vision(float move_speed,float rotate_speed)
{
    float target_speed[4] = {0,0,0,0};			//Ŀ��ת��
		float max_speed= 0;	
		int32_t out[4]={0,0,0,0};

    for(int i=0;i<4;i++)
		{
				target_speed[i] += move_speed * cos(-wheel_dir[i]); //ƽ���ٶ������������ͶӰ
				target_speed[i] += rotate_speed * cos(wheel_w_dir[i] - wheel_dir[i]);									//ת�����ٶ������������ͶӰ
				target_speed[i] *= MM_S2R_MIN;				//��������������ٶ�ת��Ϊ����ת��
				target_speed[i] *= SLOWDOWN_RATE;		  //������ת��ת��Ϊ������ת���Ա���PID����
				if(fabsf(target_speed[i]) > max_speed)
					max_speed = fabsf(target_speed[i]);
		}
		if(max_speed > MAX_MOTOR_SPEED)
		{
			for(int i=0;i<4;i++)
				target_speed[i] = target_speed[i] * MAX_MOTOR_SPEED / max_speed;
		}
		for(int i= 0;i<4;i++)
			out[i] = PID_Motor_Ctrl(target_speed[i],Get_Motor_Speed(i+1),&base_speed_pid[i]); //�ٶȻ�

		Motor_Send(out[0], out[1], out[2], out[3]);
}

/**
	@name: Wheel_Init
	@brief:���Ӳ�����ʼ��
**/
static void Wheel_Init(void)
{
	
	  //������������򣨼����Ϸ�ͼ����
		wheel_dir[0] = -pi/4.0f;			
		wheel_dir[1] = 3 * wheel_dir[0];
		wheel_dir[2] = -wheel_dir[1];
		wheel_dir[3] = -wheel_dir[0];
		//������תʱ��ת���ٶ��ڸ������ϵķ���
		wheel_w_dir[0] = atan2( BASE_WIDTH, BASE_LENGTH) + pi/2.0f;
		wheel_w_dir[1] = atan2(-BASE_WIDTH, BASE_LENGTH) + pi/2.0f;
		wheel_w_dir[2] = atan2(-BASE_WIDTH,-BASE_LENGTH) + pi/2.0f;
		wheel_w_dir[3] = atan2( BASE_WIDTH,-BASE_LENGTH) + pi/2.0f;
		//����������������ľ���
		wheel_distance = sqrt(BASE_LENGTH * BASE_LENGTH + BASE_WIDTH * BASE_WIDTH) / 2.0f;
}
