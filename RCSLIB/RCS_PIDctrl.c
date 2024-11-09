/*@filename: RCS_PIDctrl.c
 *@author     ���˹�       
 *@brief:     PID�㷨
 *@date: 2023-7-27
*/

#include "RCS_PIDctrl.h"

/*-------------�������PID------------*/

/**
	@name: PID_Init
	@brief: PID��ʼ��
**/
void PID_Init(void)
{
		
}

PID_Struct PID_Get_RM3508_Speed_Pid(void)
{
	PID_Struct speed_pid;
	//3508����ٶȻ�PID
	speed_pid.P = 8.3;
	speed_pid.I = 0.25;
	speed_pid.D = 0.38;
	speed_pid.Last_Error = 0;			
	speed_pid.Limit_Output = 16000;  	
	speed_pid.Limit_Integral = LIMIT_ERR; 
	speed_pid.Integral = 0; 			
	return speed_pid;
}

PID_Struct PID_Get_RM3508_Angle_Pid(void)
{
	PID_Struct angle_pid;
	//3508����ٶȻ�PID
	angle_pid.P = 20;
	angle_pid.I = 0;
	angle_pid.D = 0;
	angle_pid.Last_Error = 0;			
	angle_pid.Limit_Output = 1000;  	
	angle_pid.Limit_Integral = LIMIT_ERR; 
	angle_pid.Integral = 0; 			
	return angle_pid;
}

PID_Struct PID_Get_RM2006_Speed_Pid(void)
{
	PID_Struct speed_pid;
	//2006����ٶȻ�PID
	speed_pid.P = 8.3;
	speed_pid.I = 0.25;
	speed_pid.D = 0.38;
	speed_pid.Last_Error = 0;			
	speed_pid.Limit_Output = 10000;  	
	speed_pid.Limit_Integral = LIMIT_ERR; 
	speed_pid.Integral = 0; 			
	return speed_pid;
}

PID_Struct Get_Valve_Speed_Pid(void)
{
	PID_Struct valve_speed_pid;			//�����ٶȻ�

    valve_speed_pid.P = 5;
	valve_speed_pid.I = 0;
	valve_speed_pid.D = 0;
	valve_speed_pid.Last_Error = 0;			
	valve_speed_pid.Limit_Output = 5000;//LIMIT_SPEED;  	
	valve_speed_pid.Limit_Integral = LIMIT_ERR; 
	valve_speed_pid.Integral = 0; 
	return valve_speed_pid;
}

PID_Struct Get_Valve_Angle_Pid(void)
{
	PID_Struct valve_angle_pid;			//���ŽǶȻ�
	
	valve_angle_pid.P = 500;
	valve_angle_pid.I = 0;
	valve_angle_pid.D = 0;
	valve_angle_pid.Last_Error = 0;			
	valve_angle_pid.Limit_Output = 1000;//LIMIT_SPEED;  	
	valve_angle_pid.Limit_Integral = LIMIT_ERR; 
	valve_angle_pid.Integral = 0; 
	return valve_angle_pid;
}


/**
	@name: PID_Normal_Ctrl
	@brief: ͨ��PID����
	@param:float target 				Ŀ��ֵ
	@param:float current 				��ǰֵ
	@param:PID_Struct *_pid   	PID
	@return:float								����ֵ
**/
float PID_Normal_Ctrl(float target,float current,PID_Struct *_pid)
{
		float err;
		float	different_err;
    float output;         
    
    err =target - current;														//���
    _pid->Integral += err;														//�������
    different_err = err - _pid->Last_Error;						//΢�����
    
    if(_pid->Integral >= _pid->Limit_Integral)        //�����޷�
        _pid->Integral = _pid->Limit_Integral;
    if(_pid->Integral <= -_pid->Limit_Integral)
        _pid->Integral =-_pid->Limit_Integral;
    
    output = _pid->P * err + _pid->I * _pid->Integral + _pid->D * different_err;			//PID����
    
    if(output >= _pid->Limit_Output)          			//����޷�
        output = _pid->Limit_Output;
    if(output <= - _pid->Limit_Output)
        output = - _pid->Limit_Output;
    
		_pid->Last_Error = err;													//��¼�ϴ����
		return output;
}

/**
	@name: PID_ANGLE1_Ctrl
	@brief: �Ƕȿ���PID����
	@param:float target 				Ŀ��ֵ
	@param:float current 				��ǰֵ
	@param:PID_Struct *_pid   	PID
	@return:float								����ֵ
**/
float PID_Angle_Ctrl(float target,float current,PID_Struct *_pid)
{
		float err;
		float	different_err;
    float output;         
    
    err =target - current;														//���
    _pid->Integral += err;														//�������
    different_err = err - _pid->Last_Error;						//΢�����
    
    if(_pid->Integral >= _pid->Limit_Integral)        //�����޷�
        _pid->Integral = _pid->Limit_Integral;
    if(_pid->Integral <= -_pid->Limit_Integral)
        _pid->Integral =-_pid->Limit_Integral;
    
    output = _pid->P * err + _pid->I * _pid->Integral + _pid->D * different_err;			//PID����
    
    if(output >= _pid->Limit_Output)          			//����޷�
        output = _pid->Limit_Output;
    if(output <= - _pid->Limit_Output)
        output = - _pid->Limit_Output;
    
		_pid->Last_Error = err;													//��¼�ϴ����
		return output;
}

/**
	@name: PID_Motor_Ctrl
	@brief: ���PID���ƺ���
	@param:float target 				Ŀ��ֵ
	@param:float current 				��ǰֵ
	@param:PID_Struct *_pid   	PID
	@return:int32_t							�������ֵ
**/
int32_t PID_Motor_Ctrl(float target,float current,PID_Struct *_pid)
{
		int32_t err;						
		int32_t	different_err;	
    int32_t output;         
    
    err =target - current;														//���
    _pid->Integral += err;														//�������
    different_err = err - _pid->Last_Error;						//΢�����
    
    if(_pid->Integral >= _pid->Limit_Integral)        //�����޷�
        _pid->Integral = _pid->Limit_Integral;
    if(_pid->Integral <= -_pid->Limit_Integral)
        _pid->Integral =-_pid->Limit_Integral;
    
    output = _pid->P * err + _pid->I * _pid->Integral + _pid->D * different_err;			//PID����
    
    if(output >= _pid->Limit_Output)          			//����޷�
        output = _pid->Limit_Output;
    if(output <= - _pid->Limit_Output)
        output = - _pid->Limit_Output;
    
		_pid->Last_Error = err;													//��¼�ϴ����
		return output;
}

/**
	@name:  DoubleRing_Float_Ctrl
	@brief: ����˫��pid����
**/
float DoubleRing_Float_Ctrl(float target_angle,float now_angle,float now_speed,PID_Struct* DR_Angle_pid,PID_Struct* DR_Speed_pid)
{
	return PID_Normal_Ctrl(PID_Normal_Ctrl(target_angle,now_angle,DR_Angle_pid),now_speed,DR_Speed_pid);
}

