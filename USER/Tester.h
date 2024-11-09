#ifndef TESTERRR_H_
#define TESTERRR_H_

#include "rcs.h"




void Alien_Chassis_Init(void);
void Alien_Move(float spd_x,float spd_y,float spd_z);
void Alien_Joystick(void);

void Tester_Add_RM3508(uint8_t CAN_Group,uint8_t id);
void Tester_Test_RM3508(uint8_t CAN_Group,uint8_t id,uint8_t type,int32_t output);

void Tester_MBVESC_Init(void);
void Tester_MBVESC_Run(float* current_ptr,float target);
#endif