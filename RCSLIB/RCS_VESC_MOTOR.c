/**
 * @name:RCS_VESC_MOTOR
 * @brief:ΪCAN�����ṩ�������������
 * @tips:���ڱ�����Դ��comm_can.c�е�
 * @changelog:2024-3-5 CYK-Dot ���δ���
*/

/* ==============ͷ�ļ�========================================*/
#include "RCS_VESC_MOTOR.h"

/* ==============˽��ȫ�ֱ���===================================*/
volatile float   vesc_rpm[CAN_PERI_COUNT][MAX_VESC_ON_CAN];     //��ǰת��
volatile float   vesc_cur[CAN_PERI_COUNT][MAX_VESC_ON_CAN];     //��ǰ����
volatile float   vesc_pos[CAN_PERI_COUNT][MAX_VESC_ON_CAN];     //��ǰ�Ƕ�(360)
volatile int16_t vesc_power[CAN_PERI_COUNT][MAX_VESC_ON_CAN];   //��ǰ����

volatile float vesc_init_pos[CAN_PERI_COUNT][MAX_VESC_ON_CAN];  //��ʼ����ʵ�Ƕ�
volatile float vesc_last_pos[CAN_PERI_COUNT][MAX_VESC_ON_CAN];  //�ϴνǶ�(360)
volatile float vesc_loop_pos[CAN_PERI_COUNT][MAX_VESC_ON_CAN];  //�ػ��Ƕ�(��������)


/* ==============��̬��������===================================*/
static void Bldc_Variable_Memset(CAN_TypeDef* CANx);
static void Bldc_Angle_Init(CAN_TypeDef* CANx);
static inline void Bldc_AngleLoop_Update(void);
static inline void Bldc_AngleLoop_Update2(void);

/* ==============�ӿں�������===================================*/

/**
 * @name:VESC_CAN_Init
 * @brief:��ʼ��CAN����+����CAN�ܽ�+ΪCAN���ñ������������
*/
void VESC_CAN_Init(CAN_TypeDef* CANx)
{
	//ȫ�ֱ�������
	Bldc_Variable_Memset(CANx);

	//����CAN����
	if (CANx==CAN1)
	{
		if (RCS_Judge_CAN_Service(CAN1,RM_Motor_Data_Filter1)==0)
			RCS_CANx_Add_Handler(CAN1,RM_Motor_Data_Filter1);
	}
	else if (CANx==CAN2)
	{
		if (RCS_Judge_CAN_Service(CAN2,RM_Motor_Data_Filter2)==0)
			RCS_CANx_Add_Handler(CAN2,RM_Motor_Data_Filter2);
	}

	//����CAN����
	RCS_CANx_Config_With_Buffer(CAN1_MAP,CAN1_BAUD);//ʹ�����buffer����rm���

	//��ֵ��ʼ�Ƕ�
	delay_ms(DLY_AFTER_CAN_RDY);
	Bldc_Angle_Init(CANx);
}

/**
 @name:VESC_Excute_Current
 @brief:ͨ��CAN1����VESC������������
 @param:Id,VESC�����ID
 @param:Current,mA����
 @tips:�ɺ�VESC_MAX_CURRENT���𱣻����
**/
void VESC_Excute_Current(CAN_TypeDef* CANx,uint16_t ext_id,int16_t current)
{
	volatile uint8_t vesc_cmd_data[4];
	volatile uint32_t vesc_cmd_id;
	
	vesc_cmd_id=(VESC_PACKET_SET_CURRENT<<8) | ext_id;
	Int32_to_Int8(current,vesc_cmd_data);

	RCS_CANx_Send_EXTID(CANx,vesc_cmd_id,4,vesc_cmd_data);
}

/**
 @name:VESC_Excute_Speed
 @brief:ͨ��CAN1����VESC��������ת��(��λERPM)
**/
void VESC_Excute_Speed(CAN_TypeDef* CANx,uint16_t ext_id,int32_t speed)
{
	volatile uint8_t vesc_cmd_data[4];
	volatile uint32_t vesc_cmd_id;
	
	vesc_cmd_id=(VESC_PACKET_SET_RPM<<8) | ext_id;
	Int32_to_Int8(speed,vesc_cmd_data);
	RCS_CANx_Send_EXTID(CANx,vesc_cmd_id,4,vesc_cmd_data);
}
/**
 @name:VESC_Excute_Angle
 @brief:ͨ��CAN1����VESC���������Ƕ�(��λDeg)
**/
void VESC_Excute_Angle(CAN_TypeDef* CANx,uint16_t ext_id,float angle)
{
	//todo
}


//��ȡVESCת��,��λERPM
int16_t VESC_Get_Speed_Erpm(CAN_TypeDef* CANx,uint16_t ext_id)
{
	if (CANx==CAN1)
		return (int16_t)vesc_rpm[0][ext_id];
	else if (CANx==CAN2)
		return (int16_t)vesc_rpm[1][ext_id];
}

//��ȡVESC�����Ƕ�
float Get_VESC_Pos(uint8_t Motor_ID)
{
	return vesc_pos[0][Motor_ID];
}
//��ȡVESCת��,��λRPM
int16_t Get_VESC_Speed2(uint8_t Motor_ID)
{
	return (int16_t)vesc_rpm[1][Motor_ID];
}
//��ȡVESC�����Ƕ�
float Get_VESC_Pos2(uint8_t Motor_ID)
{
	return vesc_pos[1][Motor_ID];
}
//��ȡVESC����
//todo:�ڱ����л�ȡ���ʶ����������õ�������
int16_t Get_VESC_Power(uint8_t Motor_ID)
{
	return vesc_power[0][Motor_ID];
}
//��ȡVESC����
//todo:�ڱ����л�ȡ���ʶ����������õ�������
int16_t Get_VESC_Power2(uint8_t Motor_ID)
{
	return vesc_power[1][Motor_ID];
}


/**
 * @name:Bldc_Motor_Data_Filter1
 * @brief:��CAN1�Ϲ����ı��������շ���
*/
void Bldc_Motor_Data_Filter1(RCS_CAN2B_DATA_FM_RX rx_message)
{
	if ((rx_message.IDE == CAN_Id_Extended) && (rx_message.RTR == CAN_RTR_Data) && (rx_message.DLC == 0x08)) //��չ֡������֡����Ϊ8
	{
		/*--------VESC����ID����----------------*/
		volatile uint8_t motor_number = (rx_message.ExtId & 0x00ff);
		volatile uint8_t motor_cmd = rx_message.ExtId >> 8;
		
		/*--------VESC�������ݽ���--------------*/
		switch(motor_cmd)
		{
			//1��״̬�ش����İ���ת�ٺ͵�ǰ����
			case VESC_PACKET_STATUS:
				vesc_rpm[0][motor_number]=(rx_message.Data[0]<<24)|(rx_message.Data[1]<<16)|(rx_message.Data[2]<<8)|(rx_message.Data[3]);
				vesc_cur[0][motor_number]=((rx_message.Data[4]<<8)|(rx_message.Data[5]))*10;
				vesc_power[0][motor_number]=(int16_t)(BATTERY_VOLTAGE*vesc_cur[0][motor_number]);
			break;
			//4��״̬�ش����İ�����ǰ�Ƕ�
			case VESC_PACKET_STATUS_4:
				vesc_pos[0][motor_number]=((rx_message.Data[6]<<8)|(rx_message.Data[7]))*50;
			break;
		}

		/*--------VESC���Ĵ���--------------*/
		Bldc_AngleLoop_Update();
	}	
}
/**
 * @name:Bldc_Motor_Data_Filter2
 * @brief:��CAN2�Ϲ����ı��������շ���
*/
void Bldc_Motor_Data_Filter2(RCS_CAN2B_DATA_FM_RX rx_message)
{
	if ((rx_message.IDE == CAN_Id_Extended) && (rx_message.RTR == CAN_RTR_Data) && (rx_message.DLC == 0x08)) //��չ֡������֡����Ϊ8
	{
		/*--------VESC����ID����----------------*/
		volatile uint8_t motor_number = (rx_message.ExtId & 0x00ff);
		volatile uint8_t motor_cmd = rx_message.ExtId >> 8;
		
		/*--------VESC�������ݽ���--------------*/
		switch(motor_cmd)
		{
			case VESC_PACKET_STATUS:
				vesc_rpm[1][motor_number]=(rx_message.Data[0]<<24)|(rx_message.Data[1]<<16)|(rx_message.Data[2]<<8)|(rx_message.Data[3]);
				vesc_cur[1][motor_number]=((rx_message.Data[4]<<8)|(rx_message.Data[5]))*10;
				vesc_power[1][motor_number]=(int16_t)(BATTERY_VOLTAGE*vesc_cur[1][motor_number]);
			break;
			
			case VESC_PACKET_STATUS_4:
				vesc_pos[1][motor_number]=((rx_message.Data[6]<<8)|(rx_message.Data[7]))*50;
			
			break;
		}

		/*--------VESC���Ĵ���--------------*/
		Bldc_AngleLoop_Update2();
	}
}

/* ==============��̬��������===================================*/
static void Bldc_Variable_Memset(CAN_TypeDef* CANx)
{
	if (CANx == CAN1)
	{
		memset(vesc_rpm[0],0,sizeof(vesc_rpm[0]));
		memset(vesc_pos[0],0,sizeof(vesc_pos[0]));
		memset(vesc_cur[0],0,sizeof(vesc_cur[0]));
		memset(vesc_power[0],0,sizeof(vesc_power[0]));
		memset(vesc_init_pos[0],0,sizeof(vesc_init_pos[0]));
		memset(vesc_last_pos[0],0,sizeof(vesc_last_pos[0]));
		memset(vesc_loop_pos[0],0,sizeof(vesc_loop_pos[0]));
	}
	else if (CANx == CAN2)
	{
		memset(vesc_rpm[1],0,sizeof(vesc_rpm[1]));
		memset(vesc_pos[1],0,sizeof(vesc_pos[1]));
		memset(vesc_cur[1],0,sizeof(vesc_cur[1]));
		memset(vesc_power[1],0,sizeof(vesc_power[1]));
		memset(vesc_init_pos[1],0,sizeof(vesc_init_pos[1]));
		memset(vesc_last_pos[1],0,sizeof(vesc_last_pos[1]));
		memset(vesc_loop_pos[1],0,sizeof(vesc_loop_pos[1]));
	}
}
static void Bldc_Angle_Init(CAN_TypeDef* CANx)
{
	if (CANx == CAN1)
	{
		for (int i=0;i<MAX_VESC_ON_CAN;i++)
		{
			vesc_init_pos[0][i]=Get_VESC_Pos(i+1);
		}
	}
}

static inline void Bldc_AngleLoop_Update(void)
{
	for(int i=0;i<MAX_VESC_ON_CAN;i++)
	{
		vesc_loop_pos[0][i]=vesc_loop_pos[0][i] + (vesc_pos[0][i]-vesc_last_pos[0][i]);
		vesc_last_pos[0][i]=vesc_pos[0][i];
	}
}

static inline void Bldc_AngleLoop_Update2(void)
{
	for(int i=0;i<MAX_VESC_ON_CAN;i++)
	{
		vesc_loop_pos[1][i]=vesc_loop_pos[1][i] + (vesc_pos[1][i]-vesc_last_pos[1][i]);
		vesc_last_pos[1][i]=vesc_pos[1][i];
	}
}