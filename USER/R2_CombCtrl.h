#ifndef R2_COMBCTRL_H_
#define R2_COMBCTRL_H_

#include "stdint.h"
#include "RCS_PIDctrl.h"
#include "Up_Ctrl.h"
#include "RCS_AGV_BaseMove.h"
#include "RCS_AcceCtrl.h"
#include "RCS_Stastic.h"
#include "RCS_LogServer.h"
#include "RCS_Scara.h"

#define AGV_CTRL_TYPE_SPD 0
#define AGV_CTRL_TYPE_POS 1
#define AGV_CTRL_TYPE_SRV_SPD 2
#define AGV_CTRL_TYPE_LKZ_SPD 3
#define AGV_CTRL_TYPE_SFT_SPD 4

#define GANTRY_CTRL_TYPE_POS 0
#define GANTRY_CTRL_TYPE_DIFF_POS 1

typedef struct descartes DESCARTES_AXIS;
typedef struct agv_status_vector AGV_Speed;

void HAL_Chassis_Reset(float x_last_output,float y_last_output,float z_last_output);
void CombCtrl_Param_Init(RCS_PIN_USART USARTx_MAP);
int8_t HAL_Scara_Action(float x_or_main,float y_or_end,uint8_t ctrl_type,uint8_t is_absolute);
int8_t HAL_Comm_Action(float gantry_pos,float actor_pos,uint8_t punk_status,uint8_t swiv_status,uint8_t gantry_ctrl_type);
void Limit_Param_Test(void);
void CombCtrl_JoyStick(void);
uint8_t HAL_Chassis_Ctrl(float param_x,float param_y,float param_z);

int8_t gongxun();

extern int8_t gongxun_state;
void All_Route_Test(uint8_t zone);
void Laser_Reset_GPS_III(uint8_t zone);
void Block_Pick_Test(uint8_t zone);
#endif