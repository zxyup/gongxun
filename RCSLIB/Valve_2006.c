#include "Valve_2006.h"
void Servo_position( PWM_Device_t *devicename,float degree)
{
	float percent = 2.5f + 0.1f*(degree/270.f);
	Servo_Output(devicename,percent);
}
void Tray_Setup(Motor_Ctrl_Node* node,uint8_t CAN_GROUP_x,uint8_t id,PID_Struct* spd_pid,DacePID_Struct* pos_pid)
{
	*spd_pid=PID_Get_RM3508_Speed_Pid();
	MotorNode_Init_C620(CAN_GROUP_x,id,node);
	MotorNode_Add_SpeedPid(node,spd_pid);
	MotorNode_Add_DaceAnglePid(node,pos_pid);
	
	
	node->Max_Current=5000;
	MotorNode_Add(CAN_GROUP_x,node);
}

uint8_t Tray_Set0(Motor_Ctrl_Node* node)
{
	return MotorNode_Update_AngleFull(0.0f,node);
}


uint8_t Tray_Set1(Motor_Ctrl_Node* node)
{
	return MotorNode_Update_AngleFull(60.0f,node);
}


uint8_t Tray_Set2(Motor_Ctrl_Node* node)
{
	return MotorNode_Update_AngleFull(-60.0f,node);
}



void Arm_Setup(Motor_Ctrl_Node* node,uint8_t CAN_GROUP_x,uint8_t id,PID_Struct* spd_pid,DacePID_Struct* pos_pid)
{
	*spd_pid=PID_Get_RM2006_Speed_Pid();
	MotorNode_Init_C610(CAN_GROUP_x,id,node);
	MotorNode_Add_SpeedPid(node,spd_pid);
	MotorNode_Add_DaceAnglePid(node,pos_pid);
	
	node->Max_Current=5000;
	MotorNode_Add(CAN_GROUP_x,node);
}

uint8_t Arm_down(Motor_Ctrl_Node* node)
{
	return MotorNode_Update_AngleFull(0.0f,node);
}

uint8_t Arm_up(Motor_Ctrl_Node* node)
{
	return MotorNode_Update_AngleFull(-29167.0f,node);
}

int32_t Get_UpCtrl_angle(uint8_t Motor_ID)
{
	return Get_Motor_Integer_Angle2(Motor_ID);
}
void Claw_Setup(Motor_Ctrl_Node* node,uint8_t CAN_GROUP_x,uint8_t id,PID_Struct* spd_pid,DacePID_Struct* pos_pid)
{
	*spd_pid=PID_Get_RM2006_Speed_Pid();
	MotorNode_Init_C610(CAN_GROUP_x,id,node);
	MotorNode_Add_SpeedPid(node,spd_pid);
	MotorNode_Add_DaceAnglePid(node,pos_pid);
	
	node->Max_Current=5000;
	MotorNode_Add(CAN_GROUP_x,node);
}

uint8_t Claw_open(Motor_Ctrl_Node* node)
{
	return MotorNode_Update_AngleFull(1280.0f,node);
}

uint8_t Claw_close(Motor_Ctrl_Node* node)
{
	return MotorNode_Update_AngleFull(0.0f,node);
}
void Yuntai_Setup(Motor_Ctrl_Node* node,uint8_t CAN_GROUP_x,uint8_t id,PID_Struct* spd_pid,DacePID_Struct* pos_pid)
{
	*spd_pid=PID_Get_RM2006_Speed_Pid();
	MotorNode_Init_C610(CAN_GROUP_x,id,node);
	MotorNode_Add_SpeedPid(node,spd_pid);
	MotorNode_Add_DaceAnglePid(node,pos_pid);
	
	node->Max_Current=5000;
	MotorNode_Add(CAN_GROUP_x,node);
}

uint8_t Yuntai_forword(Motor_Ctrl_Node* node)
{
	return MotorNode_Update_AngleFull(0.0f,node);
}

uint8_t Yuntai_back(Motor_Ctrl_Node* node)
{
	return MotorNode_Update_AngleFull(6650.0f,node);
}

void Stop(Motor_Ctrl_Node* node)
{
	return MotorNode_Update_Spd(0,node);
}