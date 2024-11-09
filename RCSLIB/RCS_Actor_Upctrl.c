#include "RCS_Actor_Upctrl.h"


/*************************************************************************
 *                      @addtogroup:���׿���                             *
*************************************************************************/
/**
 * @name:Cylinder_Init
 * @brief:�������ذ��ϵ�ĳ��6pin���׽ӿ�
 * @param:RCS_PIN_CYLINDER CYx_MAP ��RCS_Pin_Mapping.c�в鿴��������ĸ���
*/
void Cylinder_Init(RCS_PIN_CYLINDER CYx_MAP)
{
    for(int i=0;i<4;i++)
        RCS_GPIO_Output_Init(CYx_MAP.IO[i].GPIOx,CYx_MAP.IO[i].GPIO_Pin_x);
}

/**
 * @name:Cylinder_SetBits
 * @brief:�����׽ӿ��е�ĳһ·��Ϊ�ߵ�ƽ
 * @param:RCS_PIN_CYLINDER CYx_MAP �ĸ����׿�
 * @param:uint8_t channel          ��һ·,����������Ϊ0~3
*/
void Cylinder_SetBits(RCS_PIN_CYLINDER CYx_MAP,uint8_t channel)
{
    RCS_GPIO_Set(CYx_MAP.IO[channel].GPIOx,CYx_MAP.IO[channel].GPIO_Pin_x);
}

/**
 * @name:Cylinder_ResetBits
 * @brief:�����׽ӿ��е�ĳһ·��Ϊ�͵�ƽ
 * @param:RCS_PIN_CYLINDER CYx_MAP �ĸ����׿�
 * @param:uint8_t channel          ��һ·,����������Ϊ0~3
*/
void Cylinder_ResetBits(RCS_PIN_CYLINDER CYx_MAP,uint8_t channel)
{
    RCS_GPIO_Reset(CYx_MAP.IO[channel].GPIOx,CYx_MAP.IO[channel].GPIO_Pin_x);
}



/*************************************************************************
 *                  @addtogroup:���⿪��/��ͨIO����                       *
*************************************************************************/
void IO_Init(RCS_PIN_IO IOx_MAP)
{
    for(int i=0;i<4;i++)
        RCS_GPIO_Output_Init(IOx_MAP.IO[i].GPIOx,IOx_MAP.IO[i].GPIO_Pin_x);
}
uint8_t IO_Read(RCS_PIN_IO IOx_MAP,uint8_t channel)
{
    return RCS_GPIO_Read(IOx_MAP.IO[channel].GPIOx,IOx_MAP.IO[channel].GPIO_Pin_x);
}
void IO_SetBits(RCS_PIN_IO IOx_MAP,uint8_t channel)
{
    RCS_GPIO_Set(IOx_MAP.IO[channel].GPIOx,IOx_MAP.IO[channel].GPIO_Pin_x);
}
void IO_ResetBits(RCS_PIN_IO IOx_MAP,uint8_t channel)
{
    RCS_GPIO_Reset(IOx_MAP.IO[channel].GPIOx,IOx_MAP.IO[channel].GPIO_Pin_x);
}


/*************************************************************************
 *                  @addtogroup:�������/ֱ���������                      *
*************************************************************************/

/**
 * @name:Servo_Init
 * @brief:���ö�ʱ����ĳ���˿�ΪPWM�����
 * @param:RCS_PIN_TIM ���ذ��ϵ�6pin��ʱ���ڣ�����TIMER1_MAP~TIMER4_MAP
 * @param:channel     6pin��ʱ���ڹ���4·��ʱ������������ҷֱ���0123��ֱ������0/1/2/3����
 * @param:_PWMHZ      ��Ҫ�����PWMƵ�ʡ�ע�⣬�޷���֤���Ƶ�ʾ�ȷ��ֻ�ܽ���
*/
void Servo_Init(RCS_PIN_TIM TIMERx_MAP,uint16_t Gh_channel,uint32_t _CLKHZ,uint32_t _PWMHZ,PWM_Device_t* device_name)
{
    //PWM��ʼ��
    PWMInit(TIMERx_MAP.TIMx[Gh_channel],TIMERx_MAP.Channel[Gh_channel],
            TIMERx_MAP.IO[Gh_channel].GPIOx,TIMERx_MAP.IO[Gh_channel].GPIO_Pin_x,
            _CLKHZ,_PWMHZ);
    
    //�豸��ʼ��
    device_name->Device_Type         =Device_Type_Servo;
    device_name->Drv_TIMERx_MAP      =TIMERx_MAP;
    device_name->Drv_TIMERx_MAP_Chl  =Gh_channel;
    device_name->PWM_Percent         =0;
}

/**
 * @name:Servo_Output
 * @brief:�������ϵĶ�ʱ�������PWM��
 * @param:PWM_Device_t* ����ʼ��Ϊ�����PWM�豸��ָ��
 * @param:_percent      ������ռ�ձ�,��֧�ָ���ռ�ձ�
 * @todo:��Ϊֱ�ӿ��ƽǶ�
*/
void Servo_Output(PWM_Device_t* Servo_Device_Name,float _percent)
{
    uint16_t gh_channel=Servo_Device_Name->Drv_TIMERx_MAP_Chl;

    PWMOutput(Servo_Device_Name->Drv_TIMERx_MAP.TIMx[gh_channel],
              Servo_Device_Name->Drv_TIMERx_MAP.Channel[gh_channel],
             _percent);

    Servo_Device_Name->PWM_Percent=_percent;
}

/**
 * @name:Bdc_Init
 * @brief:��һ·PWM����·IO�󶨵�TB6612�߼���ֱ��PWM�����豸��
 * @param:RCS_PIN_TIM TIMERx_MAP,uint8_t Gh_channel ָ�����ذ��ϵ�һ����ʱ��gh1.25�ӿں�ͨ����Ϊ�������
 * @param:RCS_PIN_IO IN1_IOx_MAP,uint8_t IN1_Gh_channel ָ�����ذ��ϵ�һ��IOgh1.25�ӿں�ͨ����ΪIN1
 * @param:RCS_PIN_IO IN2_IOx_MAP,uint8_t IN2_Gh_channel ָ�����ذ��ϵ�һ��IOgh1.25�ӿں�ͨ����ΪIN2
 * @param:PWM_Device_t* device_name Ҫ����ʼ��Ϊֱ�������PWM�豸��ָ��
*/
void Bdc_Init(RCS_PIN_TIM TIMERx_MAP,uint8_t Gh_channel,uint32_t _CLKHZ,uint32_t _PWMHZ,
              RCS_PIN_IO IN1_IOx_MAP,uint8_t IN1_Gh_channel,
              RCS_PIN_IO IN2_IOx_MAP,uint8_t IN2_Gh_channel,
              PWM_Device_t* device_name)
{
    //PWM��ʼ��
    PWMInit(TIMERx_MAP.TIMx[Gh_channel],TIMERx_MAP.Channel[Gh_channel],
            TIMERx_MAP.IO[Gh_channel].GPIOx,TIMERx_MAP.IO[Gh_channel].GPIO_Pin_x,
            _CLKHZ,_PWMHZ);
    //IO��ʼ��
    RCS_GPIO_Output_Init(IN1_IOx_MAP.IO[IN1_Gh_channel].GPIOx,IN1_IOx_MAP.IO[IN1_Gh_channel].GPIO_Pin_x);
    RCS_GPIO_Output_Init(IN2_IOx_MAP.IO[IN2_Gh_channel].GPIOx,IN2_IOx_MAP.IO[IN2_Gh_channel].GPIO_Pin_x);

    //�豸��ʼ��
    device_name->Device_Type         =Device_Type_Bdc;
    device_name->Drv_TIMERx_MAP      =TIMERx_MAP;
    device_name->Drv_TIMERx_MAP_Chl  =Gh_channel;
    device_name->IN1_IOx_MAP         =IN1_IOx_MAP;
    device_name->IN1_IOx_MAP_Chl     =IN1_Gh_channel;
    device_name->IN2_IOx_MAP         =IN2_IOx_MAP;
    device_name->IN2_IOx_MAP_Chl     =IN2_Gh_channel;
    device_name->PWM_Percent         =0;
}

/**
 * @name:Bdc_Ctrl
 * @brief:����ֱ������Ĺ���
 * @param:PWM_Device_t* Bdc_Device_Name ����ʼ��Ϊֱ�������PWM�豸��ָ��
 * @param:float real_percent -1~+1��ռ�ձȣ�������Ϊ��ת
*/
void Bdc_Ctrl(PWM_Device_t* Bdc_Device_Name,float real_percent)
{
    //�ܽŻ�ȡ
    RCS_MAP_IO   IN1_IO=Bdc_Device_Name->IN1_IOx_MAP.IO[Bdc_Device_Name->IN1_IOx_MAP_Chl]; 
    RCS_MAP_IO   IN2_IO=Bdc_Device_Name->IN2_IOx_MAP.IO[Bdc_Device_Name->IN2_IOx_MAP_Chl]; 
    TIM_TypeDef* Drv_TIMx=Bdc_Device_Name->Drv_TIMERx_MAP.TIMx[Bdc_Device_Name->Drv_TIMERx_MAP_Chl];
    uint8_t      Drv_TIMx_Chl=Bdc_Device_Name->Drv_TIMERx_MAP.Channel[Bdc_Device_Name->Drv_TIMERx_MAP_Chl];
    //���
    if (real_percent <0)
    {
				if(real_percent>=-1)
					Bdc_Device_Name->PWM_Percent=-real_percent;
				else
					Bdc_Device_Name->PWM_Percent=-0.96;
				
        RCS_GPIO_Reset(IN1_IO.GPIOx,IN1_IO.GPIO_Pin_x);
        RCS_GPIO_Set(IN2_IO.GPIOx,IN2_IO.GPIO_Pin_x);
        PWMOutput(Drv_TIMx,Drv_TIMx_Chl,Bdc_Device_Name->PWM_Percent);
    }
    else if (real_percent >0)
    {
				if(real_percent<=1)
					Bdc_Device_Name->PWM_Percent=-real_percent;
				else
					Bdc_Device_Name->PWM_Percent=0.96;
       
        RCS_GPIO_Set(IN1_IO.GPIOx,IN1_IO.GPIO_Pin_x);
        RCS_GPIO_Reset(IN2_IO.GPIOx,IN2_IO.GPIO_Pin_x);
        PWMOutput(Drv_TIMx,Drv_TIMx_Chl,Bdc_Device_Name->PWM_Percent);
    }
    else
    {
        Bdc_Device_Name->PWM_Percent=0;
        RCS_GPIO_Reset(IN1_IO.GPIOx,IN1_IO.GPIO_Pin_x);
        RCS_GPIO_Reset(IN2_IO.GPIOx,IN2_IO.GPIO_Pin_x);
        PWMOutput(Drv_TIMx,Drv_TIMx_Chl,0);
    }
}

/* =============��̬����====================================================*/
