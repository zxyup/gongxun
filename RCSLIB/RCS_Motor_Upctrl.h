#ifndef _RCS_MTR_UPCTRL_H_
#define _RCS_MTR_UPCTRL_H_

/* ------------ͷ�ļ�--------------------------------------*/
#include "RCS_MOTOR.h"
#include "RCS_VESC_MOTOR.h"
#include "RCS_PIDctrl.h"
#include "RCS_AcceCtrl.h"


/* ------------���ú�---------------------------------------*/
#define CTRL_LIST_MAX_LEN     30    //�����������󳤶�
#define CTRL_LIST_GROUP_COUNT 2     //��·CAN
//#define RCS_MOTOR_UPCTRL_DEBUG

/* ------------������--------------------------------------*/
typedef enum{
	TELEPORT_RMESC =0, //�������ͣ����״�ʦ���  (CAN��׼֡)
	TELEPORT_VESC = 1, //�������ͣ�VESC���     (CAN��չ֡)
}MOTOR_TELEPORT_TYPE;       

typedef enum{
	ESC_C610 =0,     //RM���C610
	ESC_C620 =1,     //RM���C620
	ESC_GM6020=2,    //6020������һ��
	ESC_MBVESC =3,   //������������ͻ���Mini V6.7
}ESC_TYPE;

typedef enum motor_type{
	RMESC_M3508=0,         //M3508���,������20A
	RMESC_M2006=1,         //M2006���,������10A
	RMESC_GM602=2,         //GM6020���,���
	VESC_U8_LiteL_KV110=3, //U8���
	VESC_N5065_KV140=4,    //N5065���
}MOTOR_TYPE;

typedef enum{
	CTRL_TYPE_CUR =0, //�������ͣ���������,RMESC/VESC���ɽ��е�������
	CTRL_TYPE_RPM =1, //�������ͣ��ٶȿ���,����VESC֧���ٶȿ���
	CTRL_TYPE_TOR =2, //�������ͣ����ؿ���
	CTRL_TYPE_POS =3, //�������ͣ��Ƕȿ���
}CTRL_TYPE;

typedef enum{
	CAN_GROUP_1 =0,  //CAN1�µĵ����
	CAN_GROUP_2 =1,  //CAN2�µĵ����
}MOTOR_CAN_GROUP;

typedef enum{
	MOTOR_ERR = 0,
	MOTOR_FINE= 1,
}MOTOR_SELFCHECK_REVAL;

typedef float    (*FNCT_FLOAT_CAN_U8) (CAN_TypeDef*,uint8_t);
typedef int16_t  (*FNCT_S16_CAN_U8) (CAN_TypeDef*,uint8_t);
typedef void     (*FNCT_VOID_CAN_S16) (CAN_TypeDef*,int16_t);
typedef void     (*FNCT_VOID_CAN_F32) (CAN_TypeDef*,float);
typedef void     (*FNCT_S16_F32_F32) (float,float);

/* ------------������--------------------------------------*/
//���õ���ļ��޲���
#define MBVESC_MAX_CURRENT 50000       //���ͻ���VESC���50A
#define MBVESC_MAX_POWER   1200000     //50*24W
#define MBVESC_MAX_ERPM    150000      //RPM*������=ERPM

#define C610_MAX_CURRENT 10000         //C610���10A
#define C620_MAX_CURRENT 20000         //C620���20A
#define GM6020_MAX_CURRENT 16384       //6020���3A
#define GM6020_MAX_VOLTAGE 25000       //6020���25V 

//���õ���ļ��޲���
#define U8LiteL_KV110_MAX_CURRENT      45800    //U8��180s�ĳ����������ô���45.8A
#define U8LiteL_KV110_MAX_POWER        1790000  //U8�����1790W
#define U8LiteL_KV110_POLEPAIRS        21       //U8��36��42��,��21��

#define N5065_KV140_MAX_CURRENT        80000    //N5065����80A
#define N5065_KV140_MAX_POWER          1820000  //1820W   
#define N5065_KV140_POLEPAIRS          7        //5065��14����,��7��


/* ------------�����ṹ�� -----------------------------*/

////���״̬
//typedef struct motor_status{
//	int16_t     Current;       //��ǰ����
//	int16_t     Speed;         //��ǰ�ٶ�
//	float       Angle;         //��ǰ�Ƕ�
//}M_Status_t;

//typedef union ctrl_32bit{
//	int32_t Current;
//	int32_t Speed;
//	float_t Angle;    
//}Msg_Ctrl;

//typedef struct limit_32bit{
//	int32_t Current;
//	int32_t Speed;
//	float_t Angle;    
//}ESC_Limit_t;

//typedef enum type_bus{
//	BUS_CAN_1 = 0,
//	BUS_CAN_2 = 1,
//	BUS_PWM_1 = 2,
//};

////������Ϣ
//typedef struct esc_status{
//	uint8_t     Bus_Type;          //��ʲô���͵������Ϸ��Ϳ�����Ϣ  @Arg:  ���� Type_Bus
//	uint8_t     Ctrl_Type;         //���Ϳ���ʲô�����ı���  @Arg:  ���� CTRL_TYPE
//	Msg_Ctrl    Ctrl_Msg;          //������Ϣ����  @Arg:  ���� Msg_Ctrl
//}ESC_Status_t;

////������ָ��FNCT_CONTROLLER������1-Ŀ��ֵָ�� ����2-��ǰֵָ�� ����3-���Ʋ���ָ�� ���1-������Ϣ
//typedef ESC_Status_t (*FNCT_CONTROLLER) (M_Status_t*,M_Status_t*,void*);


////�۲����ӿ�
//typedef struct motor_interface{
//	FNCT_FLOAT_CAN_U8 Angle_Observer;   //�Ƕȹ۲���
//	FNCT_S16_CAN_U8   Speed_Observer;   //�ٶȹ۲���
//	FNCT_VOID_CAN_S16 Current_Observer; //�����۲���
//}M_Interface_t;

////�������ӿ�
//typedef struct controller{
//	FNCT_CONTROLLER  Ang_Ctrl_Func;   //�Ƕȿ�����
//	FNCT_CONTROLLER  Spd_Ctrl_Func;   //�ٶȿ�����
//	FNCT_CONTROLLER  Cur_Ctrl_Func;   //����������
//	void*            Ang_Ctrl_Param;  //�Ƕȿ��Ʋ���
//	void*            Spd_Ctrl_Param;  //�ٶȿ��Ʋ���
//	void*            Cur_Ctrl_Param;  //�������Ʋ���
//}Controller_t;

//typedef enum can_msg_pro{
//	
//}

////ִ�����ӿ�
//typedef struct esc_interface{
//	FNCT_VOID_CAN_S16 Speed_Updater;    //�ٶ�ִ��������������ٶ����ֵ�ĺ���
//	FNCT_VOID_CAN_F32 Angle_Updater;    //�Ƕ�ִ���������𽫽Ƕȿ����ź��·�������ĺ���,������ر�����Ӧ����ȱʡ
//	FNCT_VOID_CAN_S16 Current_Updater;  //����ִ���������𽫵��������ź��·�������ĺ���,������ر�����Ӧ����ȱʡ
//}ESC_Interface_t;

////����ڵ�
//typedef struct motor_node{
//	uint8_t         ESC_ID;        //����������ϵ�ID    @Arg:  �������ּ��� 
//	ESC_Status_t    ESC_Output;    //׼������Ŀ�����Ϣ  @Arg:����ESC_Status_t
//	ESC_Limit_t     ESC_Limit;     //����޷�           @Arg:����ESC_Limit_t
//	
//	
//	M_Interface_t   ESC_Observer;  //�۲����ӿ�         @Arg:����M_Interface_t
//	Controller_t    ESC_Controller;//�������ӿ�         @Arg:����Controller_t
//	ESC_Interface_t ESC_Updater;   //ִ�����ӿ�         @Arg:����ESC_Interface_t
//}Motor_Node_T;
//typedef uint8_t      (*FNCT_EXCUTER) (uint8_t,Motor_Node_T*);

////�����������
//typedef struct mlist{
//	uint8_t       List_Len;                        //��ǰ���������
//	Motor_Node_T* List_Node[CTRL_LIST_MAX_LEN];    //�����ڵĵ���ڵ�
//}Motor_List_T;

typedef struct Node_Stat{
	int8_t is_PosProtect;
	int8_t is_SpdProtect;
	
	int8_t IsrBit_PosProtect;
	int8_t IsrBit_SpdProtect;
}MotorCtrl_Status;


//����ڵ�ṹ��
struct Node{
	//�ڵ����
	uint8_t     CAN_GROUP;
	uint8_t     CAN_ID;            //����ID
	uint8_t     Teleport_Type;     //�����ͺ�
	uint8_t     ESC_Type;          //����ͺ�
	uint8_t     Motor_Type;        //����ͺ�
	uint8_t     Ctrl_Type;         //���Ʒ�ʽ
	PID_Struct* Motor_Speed_PID;   //�ٶȻ�PID
	PID_Struct* Motor_Angle_PID;   //�ǶȻ�PID
	DacePID_Struct* Motor_DaceAngle_PID;
	int16_t     Max_Current;       //������(ֱ������)
	int16_t     Max_Power;         //�����(��Ҫ��ҵ���߼���Ӧ��)
	int16_t     Max_Speed;         //����ٶ�(ֱ������)
	float       Max_UPos;
	float       Max_LPos;
	int16_t     Max_USpd;
	int16_t     Max_LSpd;
	
	//�ڵ�״̬
	MotorCtrl_Status Now_CtrlStatus;
	int16_t     Now_Speed;         //��ǰ�ٶ�
	float       Now_Angle;         //��ǰ�Ƕ�
	int16_t     Now_Power;         //��ǰ����
	int16_t     Out_Current;       //Ŀ�����(������ǵ������������������)
	int16_t     Out_Spd;           //Ŀ���ٶ�
	float       Out_Angle;         //Ŀ��Ƕ�
	
	//�ڵ�Ӳ���ӿ�
	FNCT_FLOAT_U8 Get_Node_Angle;  //��ȡ����Ƕȵĺ���
	FNCT_S16_U8   Get_Node_Speed;  //��ȡ���ת�ٵĺ���
	FNCT_S16_U8   Get_Node_Power;  //��ȡ������ʵĺ���

	FNCT_VOID_S16 Excute_Node_Current;//CAN���ߵ�������ָ��
	FNCT_VOID_S16 Excute_Node_Speed;  //CAN�����ٶȿ���ָ��
	FNCT_VOID_F32 Excute_Node_Angle;  //CAN���߽Ƕȿ���ָ��

	//�ڵ�˽�в���(���߱������������)
	int16_t     Current;           //�����µ��������
	int16_t     Speed;             //�����µ�Ŀ���ٶ�
	float       Angle;             //�����µ�Ŀ��Ƕ�
	float       Torque;            //�����µ�Ŀ��Ť��
};
typedef struct Node Motor_Ctrl_Node;//��typedef��struct�ֿ�д,���ܿ��ļ�������

//�����������ṹ��
struct List{
	uint8_t          List_Len;
	Motor_Ctrl_Node* Node_Ctrl[CTRL_LIST_MAX_LEN];//��RTOS��ʹ��malloc���������ǲ���ȫ��
};
typedef struct List Motor_Ctrl_List;

/* ------------��������-----------------------------------*/

//������1��Ϊ����ڵ����õ������
void MotorNode_Init_C620(uint8_t CAN_Group,uint8_t ID,Motor_Ctrl_Node* Node);
void MotorNode_Init_C610(uint8_t CAN_Group,uint8_t ID,Motor_Ctrl_Node* Node);
void MotorNode_Init_GM6020(uint8_t CAN_Group,uint8_t ID,Motor_Ctrl_Node* Node);
void MotorNode_Init_MBVESC(uint8_t CAN_Group,uint8_t ID,CTRL_TYPE CTRL_TYPE_xxx,Motor_Ctrl_Node* Node);

//������2��Ϊ����ڵ����õ������
void MotorNode_Add_BldcProtect(Motor_Ctrl_Node* Node,MOTOR_TYPE Motor);
void MotorNode_Add_PosProtect(Motor_Ctrl_Node* Node,float min_pos,float max_pos);
void MotorNode_Add_SpdProtect(Motor_Ctrl_Node* Node,float n_max_spd,float p_max_spd);
void MotorNode_Add_SpeedPid(Motor_Ctrl_Node* Node,PID_Struct* spd_pid);
void MotorNode_Add_AnglePid(Motor_Ctrl_Node* Node,PID_Struct* angle_pid);
void MotorNode_Add_DaceAnglePid(Motor_Ctrl_Node* Node,DacePID_Struct* dace_pid);

//������3��������ڵ������������
void MotorNode_Add(uint8_t CAN_Group,Motor_Ctrl_Node* Node);

//������4����ȡ����ڵ��״̬
int16_t MotorNode_Get_Speed(Motor_Ctrl_Node* Node);        //��ǰ���ٶ�
float MotorNode_Get_Angle(Motor_Ctrl_Node* Node);          //��ǰ���Ƕ�
int16_t MotorNode_Get_TargetCurrent(Motor_Ctrl_Node* Node);//���������
int16_t MotorNode_Get_TargetSpeed(Motor_Ctrl_Node* Node);  //������ٶ�
float MotorNode_Get_TargetAngle(Motor_Ctrl_Node* Node);    //������Ƕ�
uint8_t MotorNode_Judge_PowerOverload(Motor_Ctrl_Node* Node);
uint8_t MotorNode_Judge_Selfcheck(Motor_Ctrl_Node* Node,uint8_t state);
//������5�����µ���ڵ�����ֵ
void MotorNode_Update_Current(int16_t Cur,Motor_Ctrl_Node* Node);//ֱ�Ӹ��µ���������
void MotorNode_Update_Spd(int16_t Spd,Motor_Ctrl_Node* Node);//ָ���ٶȲ����µ���
void MotorNode_Update_Angle(float angle,Motor_Ctrl_Node* Node);//ָ���Ƕȸ��µ���
uint8_t MotorNode_Update_AngleEasy(float angle,float speed,float tor_angle,Motor_Ctrl_Node* Node);//ָ����Χ�ļ򵥽Ƕȿ���
uint8_t MotorNode_Update_AngleFull(float angle,Motor_Ctrl_Node* Node);

void MotorNode_Config_PosProtect(Motor_Ctrl_Node* Node,uint8_t new_state);
void MotorNode_Config_SpdProtect(Motor_Ctrl_Node* Node,uint8_t new_state);

//������6����ȫ��������Ϣ�·������
void MotorList_Excute(uint8_t CAN_Group);


#endif