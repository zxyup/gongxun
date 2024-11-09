/*@filename:   Up_Ctrl.h
 *@author:     RCS16-RC
 *@brief:      �ϲ����
 *@date:       2024
*/

/*---------.h head-----------------------------------------*/
#ifndef _UP_CTRL_H_
#define _UP_CTRL_H_

/*---------�����ļ�-----------------------------------------*/

//tips:ֱ�Ӱ�����ͷ�ļ�rcs.h�ᵼ��upctrl.c�ļ��޷�ʶ������.h�е�typedef�ṹ��
#include "RCS_LogServer.h"
#include "RCS_Motor_Upctrl.h"
#include "RCS_Scara.h"
#include "R2_Upctrl.h"
#include "RCS_AcceCtrl.h"

/*---------��������------------------------------------------*/
#define R2_SCARA_MAIN_SLOWDOWN_RATE -0.2365591397849      //��ۼ��ٱ�
#define R2_SCARA_END_SLOWDOWN_RATE  0.3650793650793       //С�ۼ��ٱ�
#define R2_SCARA_MAIN_LEN           300.0                 //��۳���
#define R2_SCARA_END_LEN            350.0                 //С�۳���
#define R2_SCARA_P_INV_EPSILON      0.001                 //α��������򻯲���

#define R2_SCARA_END_MAX_UPOS_RAD   0.5*pi                //С�����Ƕ�
#define R2_SCARA_END_MAX_LPOS_RAD   -0.5*pi               //С����С�Ƕ�
#define R2_SCARA_ACTOR_OFFSET_RAD   -90.0f*DEG2RAD         
#define R2_SCARA_ACTOR_OFFSET_X     0.0f*DEG2RAD
#define R2_SCARA_ACTOR_OFFSET_Y     0.0f*DEG2RAD

#define R2_COMM_ACTOR_START_POS     2000.0f

#define VISION_WINDOW_SIZE          6                    //�ƶ����ڳ���(15*5=75ms)
#define VISION_MIN_VALID_PERCENT    0.43f                 //��������Ч����ռ�ȸ���VISION_MIN_VALID_PERCENT����Ϊ����������Ч
#define VISION_MAX_VALID_VAR        5.0f                  //��������Ч���ݵķ���С��VISION_MAX_VALID_VAR����Ϊ����������Ч


#define CAMERA_AXIS_CENTER_X        320                   //���������������ͷX����
#define CAMERA_AXIS_CENTER_Y        210//230                    //���������������ͷY����
#define R2_SCARA_CTRL_TOR_PIXEL     2//5                  //������̵�����ͷ�������


#define R2_UPCTRL_CAN_GROUP         CAN_GROUP_2
#define Tray_CAN_GROUP				CAN_GROUP_1
#define Arm_CAN_GROUP`				CAN_GROUP_2
#define Claw_CAN_GROUP				CAN_GROUP_2
#define Yuntai_CAN_GROUP			CAN_GROUP_1
#define R2_SCARA_MAIN_CAN_ID        4
#define R2_SCARA_END_CAN_ID         8
#define R2_GANTRY_P_CAN_ID          3
#define R2_GANTRY_N_CAN_ID          7
#define R2_SCARA_ACTOR_CAN_ID       5
#define TRAY_CAN_ID             	6
#define ARM_CAN_ID             		3
#define YUNTAI_CAN_ID             	5
#define CLAW_CAN_ID             	1



#define R2_SCARA_ACTOR_PUNK_GPIO    GPIOB
#define R2_SCARA_ACTOR_PUNK_PIN     GPIO_Pin_3

#define R2_SCARA_ACTOR_PUNK_GPIO_1    GPIOD
#define R2_SCARA_ACTOR_PUNK_PIN_1     GPIO_Pin_12

#define R2_SCARA_ACTOR_PUNK_GPIO_2    GPIOD
#define R2_SCARA_ACTOR_PUNK_PIN_2     GPIO_Pin_15

#define R2_SCARA_ACTOR_PUNK_GPIO_3    GPIOD
#define R2_SCARA_ACTOR_PUNK_PIN_3     GPIO_Pin_14

#define R2_SCARA_ACTOR_SWIV_GPIO    GPIOE
#define R2_SCARA_ACTOR_SWIV_PIN     GPIO_Pin_2

#define R2_SCARA_ACTOR_SWIV_GPIO_1    GPIOE
#define R2_SCARA_ACTOR_SWIV_PIN_1     GPIO_Pin_3

#define R2_SCARA_ACTOR_SWIV_GPIO_2    GPIOE
#define R2_SCARA_ACTOR_SWIV_PIN_2     GPIO_Pin_4

#define R2_SCARA_ACTOR_SWIV_GPIO_3    GPIOE
#define R2_SCARA_ACTOR_SWIV_PIN_3     GPIO_Pin_5

/*---------�����ṹ��----------------------------------------*/

typedef enum{
	//�������򣺱����ö���_����_�Ӷ���_��������
	R2_ERR_NONE            =0,
	SCARA_ERR_MAIN_LIMIT   =1, //���ײ�������λ
	SCARA_ERR_END_LIMIT    =2, //С��ײ�������λ
}R2_ERROR_ENUM;

typedef enum{
    //�������򣺶���_����_����_Ŀ��

    R2_EVENT_NONE        =0, //��������ƨ��û��
	R2_EVENT_CPLT        =1,
    SCARA_EVENT_LOST_RADM=2, //������»�е�۶�ʧĿ��
    SCARA_EVENT_LOST_COMP=3, //�������Ұ���»�е�۶�ʧĿ��
    SCARA_EVENT_LOST_TOTL=4, //��ȫ��֪��������
    SCARA_EVENT_CPLT     =5, //�ɹ�����

    CHASSIS_EVENT_ARIV =6, //���̵���

}R2_EVENT_ENUM;

typedef struct{
	uint8_t side_color;
	float   field_target_x;
	float   field_target_y;
	float   ball_disappared_pos_x;
	float   ball_disappared_pos_y;
}R2_FSM_Var_t;

#define SCARA_CTRL_TYPE_DPOS 0
#define SCARA_CTRL_TYPE_DSPD 1
#define SCARA_CTRL_TYPE_SPOS 2
#define SCARA_CTRL_TYPE_SSPD 3

/*---------��������----------------------------------------*/
void R2_Upctrl_Init(RCS_PIN_USART USARTx_MAP);
void R2_Log_Outout(void);

void myServo_Init();
void test2();
uint8_t R2_Scara_Action(double x_or_main,double y_or_end,uint8_t ctrl_type);
SCARA_AXIS    R2_Scara_Get_SPos(void);
uint8_t R2_Comm_Action(float gantry_pos,float actor_pos,uint8_t punk_status,uint8_t swiv_status);
float R2_Comm_Get_Gantry_Pos(void);
DESCARTES_AXIS R2_Actor_Axis_Convert(DESCARTES_AXIS Actor_Camera_Axis);
DESCARTES_AXIS R2_Scara_Get_Pos(void);
void R2_Scara_Debug_Joystick(void);
void R2_Scara_Debug_Forward_Back(void);
void R2_Scara_Debug_Easy_Trace(void);

uint32_t R2_Actor_Vision_ValidCheck(uint16_t jump_limit,uint16_t count_limit,uint8_t window_count);
uint32_t R2_Actor_Vision_ValidCheck_Filter(uint16_t window_size,float* valid_percent,float* valid_var,float* mean_x,float* mean_y);

int16_t R2_Upctrl_RstArena1_State(void* global_var);
int16_t R2_Upctrl_RstArena1_Quit(void* global_var);
int16_t R2_Upctrl_RstArena2_State(void* global_var);
int16_t R2_Upctrl_RstArena2_Quit(void* global_var);

int16_t R2_Aim(void);
int16_t R2_Look_For_Scan(void);

int16_t R2_Upctrl_PickBall_State(void* global_var);
int16_t R2_Upctrl_PickBall_Quit(void* global_var);

int16_t R2_Chctrl_PickBall_State(void* global_var);
int16_t R2_Chctrl_PickBall_Quit(void* global_var);

int16_t R2_Upctrl_PickBall_State_Scan(void* global_var);
int16_t R2_Chctrl_PickBall_Quit_Scan(void* global_var);

int Bottom_Num(int side_color);
int Basket_State_Judge(int side_color);
int Basket_Decide(int side_color);

int16_t R2_Upctrl_DropBall_State(int decision);

uint8_t camera_debug(void);
void punk_pick_test(int high_or_low);

void count_test(void);
void actor_test(int method);
void scan_test(void);
void Get_Scara_Angle(void);
void main_test(void);

uint16_t Judge_Rival(int color);
uint16_t Store(void);
uint16_t Pick_Store(void* global_var);
uint16_t Strategy(void* global_var,int color);
/*---------.h head-----------------------------------------*/

#endif
