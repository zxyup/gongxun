#ifndef _VALVE_2006_H_
#define _VALVE_2006_H_

#include "RCS_Motor_Upctrl.h"
void Servo_position( PWM_Device_t *devicename,float degree);
void PWM_deviceInit(RCS_PIN_TIM TIMERx_MAP,uint16_t Gh_channel,uint32_t _CLKHZ,uint32_t _PWMHZ,PWM_Device_t* device_name);
void Tray_Setup(Motor_Ctrl_Node* node,uint8_t CAN_GROUP_x,uint8_t id,PID_Struct* spd_pid,DacePID_Struct* pos_pid);
uint8_t Tray_Set0(Motor_Ctrl_Node* node);
uint8_t Tray_Set1(Motor_Ctrl_Node* node);
uint8_t Tray_Set2(Motor_Ctrl_Node* node);
void Arm_Setup(Motor_Ctrl_Node* node,uint8_t CAN_GROUP_x,uint8_t id,PID_Struct* spd_pid,DacePID_Struct* pos_pid);
uint8_t Arm_down(Motor_Ctrl_Node* node);
uint8_t Arm_up(Motor_Ctrl_Node* node);
void Claw_Setup(Motor_Ctrl_Node* node,uint8_t CAN_GROUP_x,uint8_t id,PID_Struct* spd_pid,DacePID_Struct* pos_pid);
uint8_t Claw_open(Motor_Ctrl_Node* node);
uint8_t Claw_close(Motor_Ctrl_Node* node);
void Yuntai_Setup(Motor_Ctrl_Node* node,uint8_t CAN_GROUP_x,uint8_t id,PID_Struct* spd_pid,DacePID_Struct* pos_pid);
uint8_t Yuntai_forword(Motor_Ctrl_Node* node);
uint8_t Yuntai_back(Motor_Ctrl_Node* node);
int32_t Get_UpCtrl_angle(uint8_t Motor_ID);
void Stop(Motor_Ctrl_Node* node);
#endif