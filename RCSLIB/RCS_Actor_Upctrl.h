/* ---------.h head----------------------------*/
#ifndef RCS_ACTOR_UPCTRL_H_
#define RCS_ACTOR_UPCTRL_H_

/* ---------ͷ�ļ�-----------------------------*/
#include "rcs.h"
/* ---------�����ṹ��-------------------------*/
typedef enum{
    Device_Type_Servo = 0,
    Device_Type_Bdc   = 1,
}PWM_Device_Type;

struct pwm_device
{
    //�豸Ӳ����Ϣ(TIM)
    PWM_Device_Type Device_Type;
	RCS_PIN_TIM Drv_TIMERx_MAP;      //���ذ�Ķ�ʱ��gh�ӿ���
    uint16_t    Drv_TIMERx_MAP_Chl;  //gh1.25�ڵ�channel
    //�豸Ӳ����Ϣ(GPIO)
    RCS_PIN_IO  IN1_IOx_MAP;         //���ذ��IO��gh�ӿ���
    uint16_t    IN1_IOx_MAP_Chl;     //gh1.25�ڵ�channel
    RCS_PIN_IO  IN2_IOx_MAP;         //���ذ��IO��gh�ӿ���
    uint16_t    IN2_IOx_MAP_Chl;     //gh1.25�ڵ�channel
    //�豸������Ϣ
    float       PWM_Percent;
    uint8_t     direction;
};
typedef struct pwm_device PWM_Device_t;

/* ---------��������---------------------------*/
void Cylinder_Init(RCS_PIN_CYLINDER CYx_MAP);
void Cylinder_SetBits(RCS_PIN_CYLINDER CYx_MAP,uint8_t channel);
void Cylinder_ResetBits(RCS_PIN_CYLINDER CYx_MAP,uint8_t channel);

void IO_Init(RCS_PIN_IO IOx_MAP);
uint8_t IO_Read(RCS_PIN_IO IOx_MAP,uint8_t channel);
void IO_SetBits(RCS_PIN_IO IOx_MAP,uint8_t channel);
void IO_ResetBits(RCS_PIN_IO IOx_MAP,uint8_t channel);

void Servo_Init(RCS_PIN_TIM TIMERx_MAP,uint16_t Gh_channel,uint32_t _CLKHZ,uint32_t _PWMHZ,PWM_Device_t* device_name);
void Servo_Output(PWM_Device_t* Servo_Device_Name,float _percent);
void Bdc_Init(RCS_PIN_TIM TIMERx_MAP,uint8_t Gh_channel,uint32_t _CLKHZ,uint32_t _PWMHZ,
              RCS_PIN_IO IN1_IOx_MAP,uint8_t IN1_Gh_channel,
              RCS_PIN_IO IN2_IOx_MAP,uint8_t IN2_Gh_channel,
              PWM_Device_t* device_name);
void Bdc_Ctrl(PWM_Device_t* Bdc_Device_Name,float real_percent);

/*
����/�г̿���
*/
#define SWITCH_INFRARED_GPIO   	 GPIOB
#define SWITCH_INFRARED_PIN		   GPIO_Pin_10
#define INIT_INFRARED_GPIO			 GPIOB
#define INIT_INFRARED_PIN				 GPIO_Pin_11
#define TURN_INFRARED_GPIO			 GPIOB
#define TURN_INFRARED_PIN				 GPIO_Pin_15

/*
���
*/
#define ACT1_TIM                     TIM4
#define ACT1_TIM_CH                    3
#define ACT1_GPIO                    GPIOD
#define ACT1_PIN                GPIO_Pin_14

#define ACT2_TIM                     TIM4
#define ACT2_TIM_CH                    1
#define ACT2_GPIO                    GPIOD
#define ACT2_PIN                GPIO_Pin_12

#define ACT3_TIM                     TIM4
#define ACT3_TIM_CH                    2
#define ACT3_GPIO                    GPIOD
#define ACT3_PIN                GPIO_Pin_13

#define ACT4_TIM                     TIM1
#define ACT4_TIM_CH                    4
#define ACT4_GPIO                    GPIOE
#define ACT4_PIN                GPIO_Pin_14

#define ACT5_TIM                     TIM1
#define ACT5_TIM_CH                    2
#define ACT5_GPIO                    GPIOE
#define ACT5_PIN                GPIO_Pin_11

#define ACT_CLKHZ               84000
#define ACT_PWMHZ               333


/*
�����
*/
#define ACTUATOR_TIM                     TIM4
#define ACTUATOR_TIM_CH                    1
#define ACTUATOR_GPIO                    GPIOD
#define ACTUATOR_GPIO_Pin                GPIO_Pin_12
#define ACTUATOR_COUNT_FREQUENCY         84000
#define ACTUATOR_OUTPUT_FREQUENCY        333


#endif