/**
 * @filename:RCS_Motor_Upctrl.c
 * @brief:ʹ��ͳһ�Ŀ�ܿ��Ƶ��
 * @tips:�뱣��RM�����IDλ��1~8֮��,�����������ID>8
 * @todo:6020��δ����ÿ��;CAN1�µ�ID1~4Ϊ���̵��,��Ϊ����ÿ��,��Ȼ��Motor_Send����
 * @contribute:CYK-dot @2024-2-2
 * ------------------------------------------------------------
 * @changelog: CYK-dot  @2024-3-25 �����˵��״̬��صĽӿڡ�����ע��
 * ------------------------------------------------------------
 * @changelog: CYK-dot  @2024-5-21 ������Ұָ����صı���
*/

/* ==========ͷ�ļ� ==========================================*/
#include "RCS_Motor_Upctrl.h"
#include "RCS_dsp.h"

/* ==========��̬���� ========================================*/
static inline void Reset_RMESC_OutputCurrent(uint8_t CAN_Group);
static inline void Update_RMESC_OutputCurrent(uint8_t CAN_Group,Motor_Ctrl_Node* Node);
static inline void Excute_RMESC_OutputCurrent(uint8_t CAN_Group);
static inline void Excute_VESC_Output(uint8_t CAN_Group,Motor_Ctrl_Node* Node);

static inline int8_t Judge_MotorNode_isPosProtect(Motor_Ctrl_Node* Node);
static inline int8_t Judge_MotorNode_isSpdProtect(Motor_Ctrl_Node* Node);
/* ==========ȫ�ֱ��� ========================================*/
Motor_Ctrl_List CtrlList[CTRL_LIST_GROUP_COUNT];    //����ڵ�����
int16_t RMESC_Current[CTRL_LIST_GROUP_COUNT][9];    //3508/2006����������
int16_t GM6020_Current[CTRL_LIST_GROUP_COUNT][9];   //6020�������

uint8_t Teleport_Status[CTRL_LIST_GROUP_COUNT];     //RM�������״̬
//[7~4]��ʹ�� [3]�Ƿ���ҪMotor_Send [2]�Ƿ���ҪMotor_Send_ADD [1]�Ƿ���ҪGM6020_Send [0]�Ƿ���ҪGM6020_Send_ADD 

/* ==========�������� ========================================*/

/**
 * @name:MotorNode_Add
 * @brief:������ڵ����CAN����
 * @param:CAN_Group={CAN_GROUP_x,x=1~2}
 * @param:Motor_Ctrl_Node* Node ��д��ɵ���ڵ��,�ٰ��������ڵ��ָ������˺���
*/
void MotorNode_Add(uint8_t CAN_Group,Motor_Ctrl_Node* Node)
{
	if (CtrlList[CAN_Group].List_Len ==CTRL_LIST_MAX_LEN) 
	{
		#ifdef RCS_MOTOR_UPCTRL_DEBUG
			RCS_Shell_Logs("ERROR:in RCS_Motor_Upctrl.c, add too many motors");
		#endif
		return;
	}
	CtrlList[CAN_Group].Node_Ctrl[CtrlList[CAN_Group].List_Len]=Node;
	CtrlList[CAN_Group].List_Len++;
}

/**
 * @name:Motor_Ctrl_Update_Current
 * @brief:���µ���ڵ�ĵ������ֵ
 * @param:Node ��Ҫ�ı�״̬�ĵ���ڵ�
 * @tips:�������Ƶĵ��,ʹ����������Ϊ������Լ��
*/
void MotorNode_Update_Current(int16_t Cur,Motor_Ctrl_Node* Node)
{
	//------------δ�ﵽ�޷�----------------------------
	if (abs(Cur) <= Node->Max_Current)
	{
		//��ֵ����
		Node->Current=Cur;    //�·�����������ֵ
		Node->Out_Current=Cur;//�����۲��debugֵ

		//��־��¼
		#ifdef RCS_MOTOR_UPCTRL_DEBUG
		if (Node->Ctrl_Type != CTRL_TYPE_CUR)
		RCS_Shell_Logs("Warning:in RCS_Motor_Upctrl.c,Tying to use current control mistakenly");
		#endif
	}
	//------------�ﵽ�޷�------------------------------
	else
	{
		//��ֵ����
		if (Cur>0)
		{
			Node->Current=Node->Max_Current;
			Node->Out_Current=Node->Max_Current;
		}
		else 
			Node->Current=-1*Node->Max_Current;
			Node->Out_Current=Node->Current;

		//��־��¼
		#ifdef RCS_MOTOR_UPCTRL_DEBUG
			RCS_Shell_Logs("Warning:in RCS_Motor_Upctrl.c,Current overload!");
			if (Node->Ctrl_Type != CTRL_TYPE_CUR)
			RCS_Shell_Logs("Warning:in RCS_Motor_Upctrl.c,Tying to use current control mistakenly");
		#endif
	}
}

/**
 * @name:Motor_Ctrl_Update_Spd
 * @brief:���µ���ڵ��Ŀ���ٶ�,��������õ�������,��ʹ���ٶȻ�PID����Ŀ�����
 * @param:Node ��Ҫ�ı�״̬�ĵ���ڵ�
 * @tips:����ת�ٿ��Ƶĵ��:��ҪΪ������õ������������ú�����Զ���0���Ա������
 * @tips:���ڵ������Ƶĵ��:��Ȼʹ����������Ϊ������Լ��
*/
void MotorNode_Update_Spd(int16_t Spd,Motor_Ctrl_Node* Node)
{	
		//update variables
		Node->Out_Spd=Spd;
		Node->Now_Speed=Node->Get_Node_Speed(Node->CAN_ID);
	
		//protect variables
		if (Node->Now_CtrlStatus.is_SpdProtect==1)
		if (Judge_MotorNode_isPosProtect(Node)==1)
			Node->Out_Spd=Node->Max_USpd;
		
		if (Node->Now_CtrlStatus.is_SpdProtect==1)
		if (Judge_MotorNode_isPosProtect(Node)==-1)
			Node->Out_Spd=Node->Max_LSpd;
		
		if (Node->Now_CtrlStatus.is_PosProtect==1)
		if (Judge_MotorNode_isPosProtect(Node)!=0)
			Node->Out_Spd=0;
		
		//excute for variables
		if (Node->Ctrl_Type==CTRL_TYPE_CUR)
		{
			Node->Out_Current=PID_Normal_Ctrl(Node->Out_Spd,Node->Now_Speed,Node->Motor_Speed_PID);
			MotorNode_Update_Current(Node->Out_Current,Node);
		}
		else if (Node->Ctrl_Type==CTRL_TYPE_RPM)
		{
			Node->Speed=Node->Out_Spd;
		}
}

/**
 * @name:Motor_Ctrl_Update_Angle
 * @brief:���µ���ڵ��Ŀ��Ƕ�
 * @param:Node ��Ҫ�ı�״̬�ĵ���ڵ�
 * @tips:����ת�ٿ��Ƶĵ��:��ҪΪ������õ�������������ٶ���ԶΪ0�Ա������
 * @tips:���ڵ������Ƶĵ��:��Ȼʹ����������Ϊ������Լ��
*/
void MotorNode_Update_Angle(float angle,Motor_Ctrl_Node* Node)
{
	//����Ŀ��ֵ
	Node->Out_Angle=angle;
	//���µ�ǰֵ
	Node->Now_Angle=Node->Get_Node_Angle(Node->CAN_ID);
	
	//protect
	if (Node->Now_CtrlStatus.is_PosProtect==1)
	if (Judge_MotorNode_isPosProtect(Node)==1)
		Node->Out_Angle=Node->Max_UPos;
	
	if (Node->Now_CtrlStatus.is_PosProtect==1)
	if (Judge_MotorNode_isPosProtect(Node)==-1)
		Node->Out_Angle=Node->Max_LPos;
	
	//excute
	if ((Node->Ctrl_Type==CTRL_TYPE_CUR)||(Node->Ctrl_Type==CTRL_TYPE_RPM))
	{
		//�������ֵ
		Node->Out_Spd=PID_Normal_Ctrl(Node->Out_Angle,Node->Now_Angle,Node->Motor_Angle_PID);
		//���¿���ֵ
		MotorNode_Update_Spd(Node->Out_Spd,Node);
	}
	else if (Node->Ctrl_Type==CTRL_TYPE_POS)
	{
		Node->Angle=Node->Out_Angle;
	}
}

/**
 * @name:MotorNode_Update_AngleEasy
 * @brief:���µ���ڵ��Ŀ��Ƕ�,��ָ��������ʽ����Ŀ�����
 * @param:angle Ŀ��Ƕ�
 * @param:speed �ƶ���Ŀ��Ƕȵ��ٶ�
 * @param:tor_angle �ݲ�,���ݲ����ڵĻ��õ���ٶ�Ϊ0
 * @param:Node ��Ҫ�ı�״̬�ĵ���ڵ�
 * @reval:�Ƿ�ִ�λ��
 * @tips:�������ٶȿ���/����������ĵ��
*/
uint8_t MotorNode_Update_AngleEasy(float angle,float speed,float tor_angle,Motor_Ctrl_Node* Node)
{
	//update
	Node->Out_Angle=angle;
	Node->Now_Angle=Node->Get_Node_Angle(Node->CAN_ID);
	
	//protect
	if (Node->Now_CtrlStatus.is_PosProtect==1)
	if (Judge_MotorNode_isPosProtect(Node)==1)
		Node->Out_Angle=Node->Max_UPos;
	
	if (Node->Now_CtrlStatus.is_PosProtect==1)
	if (Judge_MotorNode_isPosProtect(Node)==-1)
		Node->Out_Angle=Node->Max_LPos;
	
	//caculate & excute
	if (Node->Ctrl_Type==CTRL_TYPE_CUR)
	{
		//smaller
		if (Node->Now_Angle <= (Node->Out_Angle-tor_angle))
		{
			MotorNode_Update_Spd(speed,Node);
		}
		//bigger
		else if (Node->Now_Angle > (Node->Out_Angle-tor_angle))
		{
			MotorNode_Update_Spd(-speed,Node);
		}
		//suitable
		else
		{
			MotorNode_Update_Spd(0,Node);
		}
	}
}



uint8_t MotorNode_Update_AngleFull(float angle,Motor_Ctrl_Node* Node)
{
	//����Ŀ��ֵ
	Node->Out_Angle=angle;
	//���µ�ǰֵ
	Node->Now_Angle=Node->Get_Node_Angle(Node->CAN_ID);
	
	//protect
	if (Node->Now_CtrlStatus.is_PosProtect==1)
	if (Judge_MotorNode_isPosProtect(Node)==1)
		Node->Out_Angle=Node->Max_UPos;
	
	if (Node->Now_CtrlStatus.is_PosProtect==1)
	if (Judge_MotorNode_isPosProtect(Node)==-1)
		Node->Out_Angle=Node->Max_LPos;
	
	//excute
	if ((Node->Ctrl_Type==CTRL_TYPE_CUR)||(Node->Ctrl_Type==CTRL_TYPE_RPM))
	{
		//�������ֵ
		Node->Out_Spd=DacePID_Normal_Ctrl(Node->Out_Angle,Node->Now_Angle,Node->Motor_DaceAngle_PID);
		//���¿���ֵ
		MotorNode_Update_Spd(Node->Out_Spd,Node);
		
		if (Node->Out_Spd==0) return 1;
	}
	else if (Node->Ctrl_Type==CTRL_TYPE_POS)
	{
		Node->Angle=Node->Out_Angle;
	}
	
	return 0;
}
/**
 * @name:MotorList_Excute
 * @brief:�����е���ڵ�ĵ���ֵ�·������
 * @param:CAN_Group={CAN_GROUP_x,x=1~2}
 * @tips:�ú�����Ŀ���Ǳ���ʹ�õ����ĵ�����ƺ�����������������ҵ���߼��������delay_msǰ����Ӧ����ҵ���߼���
*/
void MotorList_Excute(uint8_t CAN_Group)
{
	//-----��λ--------------------------------------------

	//��λ����ͬ���ĵ����ֵ
	Reset_RMESC_OutputCurrent(CAN_Group);
	//��λ����������ĵ����ֵ
	__NOP();//�������Լ��ᳬʱͣ��,����Ҫ������ϱ�����Ʋ�����

	//-----����--------------------------------------------

	for(int i=0;i<CtrlList[CAN_Group].List_Len;i++)
	{
		//���¶���ͬ���ĵ����ֵ
		Update_RMESC_OutputCurrent(CAN_Group,(CtrlList[CAN_Group].Node_Ctrl[i]));
		//���µ���������ĵ����ֵ
		__NOP();
	}

	//------ִ��-------------------------------------------

	//ִ�ж��������ĵ����ֵ
	Excute_RMESC_OutputCurrent(CAN_Group);
	//ִ�е���������ĵ����ֵ
	for(int i=0;i<CtrlList[CAN_Group].List_Len;i++)
	{
		Excute_VESC_Output(CAN_Group,(CtrlList[CAN_Group].Node_Ctrl[i]));
	}
}

/**
 * @name:MotorNode_Init_C620
 * @brief:����RM����Ĳ���
 * @param:uint8_t CAN_Group      ����CAN_GROUP_x
 * @param:uint8_t ID             ���ID,�����ó�1~8
 * @param:Motor_Ctrl_Node* Node  ��Ҫ�����ó�RM����ĵ���ڵ�
*/
void MotorNode_Init_C620(uint8_t CAN_Group,uint8_t ID,Motor_Ctrl_Node* Node)
{
	Node->CAN_ID=ID;
	Node->Teleport_Type=TELEPORT_RMESC;    //RM����
	Node->Ctrl_Type=CTRL_TYPE_CUR;         //��������
	Node->ESC_Type=ESC_C620;               //C620���
	Node->Max_Current=C620_MAX_CURRENT;    //��������

	if (CAN_Group==CAN_GROUP_1)
	{
		Node->Get_Node_Speed=&Get_Motor_Speed;
		Node->Get_Node_Angle=&Get_Motor_Float_Angle;
	}
	else
	{
		Node->Get_Node_Speed=&Get_Motor_Speed2;
		Node->Get_Node_Angle=&Get_Motor_Float_Angle2;
	}
}

/**
 * @name:Motor_Ctrl_Init_C610_Motor
 * @brief:����RM����Ĳ���
 * @param:uint8_t CAN_Group      ����CAN_GROUP_x
 * @param:uint8_t ID             ���ID,�����ó�1~8
 * @param:Motor_Ctrl_Node* Node  ��Ҫ�����ó�RM����ĵ���ڵ�
*/
void MotorNode_Init_C610(uint8_t CAN_Group,uint8_t ID,Motor_Ctrl_Node* Node)
{
	Node->CAN_ID=ID;
	Node->Teleport_Type=TELEPORT_RMESC;    //RM����
	Node->Ctrl_Type=CTRL_TYPE_CUR;         //��������
	Node->ESC_Type=ESC_C610;               //C610���
	Node->Max_Current=C610_MAX_CURRENT;    //��������

	if (CAN_Group==CAN_GROUP_1)
	{
		Node->Get_Node_Speed=&Get_Motor_Speed;
		Node->Get_Node_Angle=&Get_Motor_Float_Angle;
	}
	else
	{
		Node->Get_Node_Speed=&Get_Motor_Speed2;
		Node->Get_Node_Angle=&Get_Motor_Float_Angle2;
	}
}

/**
 * @name:MotorNode_Init_GM6020_Cur
 * @brief:����GM6020���,��ʹ�õ���������
*/
void MotorNode_Init_GM6020(uint8_t CAN_Group,uint8_t ID,Motor_Ctrl_Node* Node)
{
	#ifdef RCS_MOTOR_UPCTRL_DEBUG
		if (ID <= 4) RCS_Shell_Logs("Warning:in RCS_Motor_Upctrl.c,attempt to set GM6020 ID in range of 1~4");
	#endif
	Node->CAN_ID=ID;
	Node->Teleport_Type=TELEPORT_RMESC;    //RM����
	Node->Ctrl_Type=CTRL_TYPE_CUR;         //��������
	Node->ESC_Type=ESC_GM6020;             //6020���
	Node->Max_Current=GM6020_MAX_CURRENT;  //��������
	
	if (CAN_Group==CAN_GROUP_1)
	{
		Node->Get_Node_Speed=&Get_GM6020_Speed;
		Node->Get_Node_Angle=&Get_GM6020_Float_Angle;
	}
	else
	{
		Node->Get_Node_Speed=&Get_GM6020_Speed2;
		Node->Get_Node_Angle=&Get_GM6020_Float_Angle2;
	}
}

/**
 * @name:Motor_Ctrl_Init_MBVESC_Motor_Cur
 * @brief:���ô��ͻ��ر���������Ĳ���,��ʹ�ñ���������ĵ������Ʒ�ʽ
 * @param:uint8_t CAN_Group       ����CAN_GROUP_x
 * @param:uint8_t ID              ���ID
 * @param:CTRL_TYPE CTRL_TYPE_xxx ��������,ʹ�õ�������ת�ٿ���,CTRL_TYPE_CUR/CTRL_TYPE_RPM
 * @param:Motor_Ctrl_Node* Node   ��Ҫ�����óɱ���������ĵ���ڵ�
*/
void MotorNode_Init_MBVESC(uint8_t CAN_Group,uint8_t ID,CTRL_TYPE CTRL_TYPE_xxx,Motor_Ctrl_Node* Node)
{
	if (Node->Ctrl_Type == CTRL_TYPE_TOR)//@todo:��ʱ������ô�ñ����������������
	{
		#ifdef RCS_MOTOR_UPCTRL_DEBUG
			RCS_Shell_Logs("ERROR: in RCS_Motor_Upctrl.c,Could not init VESC with torque control type!");
			return;
		#endif
	}

	Node->CAN_ID=ID;
	Node->Teleport_Type=TELEPORT_VESC;   //����������
	Node->ESC_Type=ESC_MBVESC;           //���ͻ��ص��
	Node->Ctrl_Type=CTRL_TYPE_xxx;       //��������/ת�ٿ���
	Node->Max_Current=MBVESC_MAX_CURRENT;//��������
	Node->Max_Power=MBVESC_MAX_POWER;    //ת�ٵĹ��ʱ���(û�б���)
	Node->Max_Speed=MBVESC_MAX_ERPM;     //��ת�ٱ���(û�б���)

	if (CAN_Group==CAN_GROUP_1)
	{
		Node->Get_Node_Speed=&Get_VESC_Speed2;
		Node->Get_Node_Angle=&Get_VESC_Pos;
	}
	else
	{
		Node->Get_Node_Speed=&Get_VESC_Speed2;
		Node->Get_Node_Angle=&Get_VESC_Pos2;
	}
}



/**
 * @name:MotorNode_Add_SpeedPid
 * @brief:Ϊ�ڵ�����ٶȻ�pid
*/
void MotorNode_Add_SpeedPid(Motor_Ctrl_Node* Node,PID_Struct* spd_pid)
{
	Node->Motor_Speed_PID=spd_pid;
}

/**
 * @name:MotorNode_Add_AnglePid
 * @brief:Ϊ�ڵ���ӽǶȻ�pid
*/
void MotorNode_Add_AnglePid(Motor_Ctrl_Node* Node,PID_Struct* angle_pid)
{
	Node->Motor_Angle_PID=angle_pid;
}

void MotorNode_Add_DaceAnglePid(Motor_Ctrl_Node* Node,DacePID_Struct* dace_pid)
{
	Node->Motor_DaceAngle_PID=dace_pid;
}

/**
 * @name:MotorNode_Add_BldcProtect
 * @brief:��������֮��,Ϊ����ڵ���ӵ������
 * @param:Motor_Ctrl_Node* Node ��Ҫ��ӵ�������Ľڵ��ָ��
 * @param:MOTOR_TYPE Motor      �������,Ŀǰ��֧��VESC_U8_LiteL_KV110��VESC_N5065_KV140
*/
void MotorNode_Add_BldcProtect(Motor_Ctrl_Node* Node,MOTOR_TYPE Motor)
{
	//�������С�ڵ����������ѡ����������Ϊ���޲���
	switch (Motor)
	{
		case VESC_U8_LiteL_KV110:
			if (Node->Max_Current > U8LiteL_KV110_MAX_CURRENT)
			{
				Node->Max_Current=U8LiteL_KV110_MAX_CURRENT;
			}
			if (Node->Max_Power > U8LiteL_KV110_MAX_POWER)
			{
				Node->Max_Power=U8LiteL_KV110_MAX_POWER;
			}
			Node->Max_Speed=(Node->Max_Speed)/U8LiteL_KV110_POLEPAIRS;//������ת����Ҫ����
		break;

		case VESC_N5065_KV140:
			if (Node->Max_Current > N5065_KV140_MAX_CURRENT)
			{
				Node->Max_Current=N5065_KV140_MAX_CURRENT;
			}
			if (Node->Max_Power > N5065_KV140_MAX_POWER)
			{
				Node->Max_Power=N5065_KV140_MAX_POWER;
			}
			Node->Max_Speed=(Node->Max_Speed)/N5065_KV140_POLEPAIRS;//������ת����Ҫ����
		break;
			
		case RMESC_M3508:
			Node->Max_Current=20000;
		break;
		
		case RMESC_M2006:
			Node->Max_Current=10000;
		break;
	}
}


void MotorNode_Add_PosProtect(Motor_Ctrl_Node* Node,float min_pos,float max_pos)
{
	(Node->Now_CtrlStatus).is_PosProtect=1;
	Node->Max_LPos=min_pos;
	Node->Max_UPos=max_pos;
}

void MotorNode_Add_SpdProtect(Motor_Ctrl_Node* Node,float n_max_spd,float p_max_spd)
{
	(Node->Now_CtrlStatus).is_SpdProtect=1;
	Node->Max_LSpd=n_max_spd;
	Node->Max_USpd=p_max_spd;
}

void MotorNode_Config_PosProtect(Motor_Ctrl_Node* Node,uint8_t new_state)
{
	(Node->Now_CtrlStatus).is_PosProtect=new_state;
}

void MotorNode_Config_SpdProtect(Motor_Ctrl_Node* Node,uint8_t new_state)
{
	(Node->Now_CtrlStatus).is_SpdProtect=new_state;
}

/**
 *@name:Motor_Ctrl_Get_Motor_Speed
 *@brief:��ȡ�����ǰ���ٶ�
**/
int16_t MotorNode_Get_Speed(Motor_Ctrl_Node* Node)
{
	if (Node->Get_Node_Speed != NULL)
		return Node->Get_Node_Speed(Node->CAN_ID);
	else
	{
		return 0;
		#ifdef RCS_MOTOR_UPCTRL_DEBUG
			RCS_Shell_Logs("Err:Get Spd From a Motor which not initialized before!");
		#endif
	}
	
}
/**
 *@name:Motor_Ctrl_Get_Motor_Angle
 *@brief:��ȡ�����ǰ�ĽǶ�
**/
float MotorNode_Get_Angle(Motor_Ctrl_Node* Node)
{
	if (Node->Get_Node_Angle != NULL)
		return Node->Get_Node_Angle(Node->CAN_ID);
	else
	{
		return 0;
		#ifdef RCS_MOTOR_UPCTRL_DEBUG
			RCS_Shell_Logs("Err:Get Pos From a Motor which not initialized before!");
		#endif
	}
}
/**
 *@name:Motor_Ctrl_Get_Motor_Target_Speed
 *@brief:��ȡ�����ǰ��Ŀ���ٶ�
 *@tips:����ֱ��ͨ��Node->Spd����ȡĿ��Ƕȣ������ȡ���Ĳ�����ȷ��Ŀ���ٶȣ�������Node->Out_Spd
**/
int16_t MotorNode_Get_TargetSpeed(Motor_Ctrl_Node* Node)
{
	return Node->Out_Spd;
}
/**
 *@name:Motor_Ctrl_Get_Motor_Target_Angle
 *@brief:��ȡ�����ǰ��Ŀ��Ƕ�
 *@tips:����ֱ��ͨ��Node->Angle����ȡĿ��Ƕȣ������ȡ���Ĳ�����ȷ��Ŀ��Ƕȣ�������Node->Out_Angle
**/
float MotorNode_Get_TargetAngle(Motor_Ctrl_Node* Node)
{
	return Node->Out_Angle;
}

/**
 * @name:MotorNode_Get_TargetCurrent
 * @brief:��ȡ�����Ŀ�����
 * @tips:����ֱ��ͨ��Node->Current����ȡĿ��Ƕȣ������ȡ���Ĳ�����ȷ��Ŀ�������������Node->Out_Current
*/
int16_t MotorNode_Get_TargetCurrent(Motor_Ctrl_Node* Node)
{
	return Node->Out_Current;
}

/**
 * @name:Motor_Node_Get_Power
 * @brief:�жϵ�ǰ����Ĺ���
*/
int16_t Motor_Node_Get_Power(Motor_Ctrl_Node* Node)
{
	if (Node->Get_Node_Power !=NULL)
		return Node->Now_Power=Node->Get_Node_Power(Node->CAN_ID);
	else
	{
		return 0;
		#ifdef RCS_MOTOR_UPCTRL_DEBUG
			RCS_Shell_Logs("Err:Get Power From a Motor which not initialized before!");
		#endif
	}
}
/**
 * @name:MotorNode_Judge_PowerOverload
 * @brief:�жϵ�ǰ����Ƿ񳬹������
*/
uint8_t MotorNode_Judge_PowerOverload(Motor_Ctrl_Node* Node)
{
	if (Node->Get_Node_Power !=NULL)
	{
		Node->Now_Power=Node->Get_Node_Power(Node->CAN_ID);
		if (Node->Now_Power > Node->Max_Power)
			return 1;
		else
			return 0;
	}
	else
	{
		return 1;
		#ifdef RCS_MOTOR_UPCTRL_DEBUG
		RCS_Shell_Logs("Err:Get Power From a Motor which not initialized before!");
		#endif
	}
}

uint8_t MotorNode_Judge_Selfcheck(Motor_Ctrl_Node* Node,uint8_t state)
{
	
}



/* ============��̬����=============================================*/
/**
 * @name:Reset_RMESC_OutputCurrent
 * @brief:��λRM����Ļ������ֵ
*/
static inline void Reset_RMESC_OutputCurrent(uint8_t CAN_Group)
{
	memset(RMESC_Current[CAN_Group],0,sizeof(RMESC_Current[CAN_Group]));
	memset(GM6020_Current[CAN_Group],0,sizeof(GM6020_Current[CAN_Group]));
}
/**
 * @name:Update_RMESC_OutputCurrent
 * @brief:����RM����Ļ������ֵ
*/
static inline void Update_RMESC_OutputCurrent(uint8_t CAN_Group,Motor_Ctrl_Node* Node)
{
	//GM6020
	if (Node->ESC_Type ==ESC_GM6020)
	{
		//���µ��������ֵ
		GM6020_Current[CAN_Group][Node->CAN_ID]=Node->Current;
		//�������״̬��
		if (Node->CAN_ID <=4) Teleport_Status[CAN_Group] |= 0B0010;
		else                  Teleport_Status[CAN_Group] |= 0B0001;
		//������㣬������Ʋ�����
		Node->Current=0;
	}
	//RM3508 & RM2006
	else if (Node->ESC_Type ==ESC_C610 ||Node->ESC_Type ==ESC_C620)
	{
		//���µ��������ֵ
		RMESC_Current[CAN_Group][Node->CAN_ID]=Node->Current;
		//�������״̬��
		if (Node->CAN_ID <=4) Teleport_Status[CAN_Group] |= 0B1000;
		else                  Teleport_Status[CAN_Group] |= 0B0100;
		//������㣬������Ʋ�����
		Node->Current=0;
	}
}

/**
 * @name:Excute_RMESC_OutputCurrent
 * @brief:ִ��RM����Ļ������ֵ
*/
static inline void Excute_RMESC_OutputCurrent(uint8_t CAN_Group)
{
	if (CAN_Group == CAN_GROUP_1)
	{
		if ((Teleport_Status[CAN_Group] & 0b0100)>>2)
		Motor_Send_ADD(RMESC_Current[CAN_Group][5],RMESC_Current[CAN_Group][6],RMESC_Current[CAN_Group][7],RMESC_Current[CAN_Group][8]);
		if ((Teleport_Status[CAN_Group] & 0b1000)>>3)
		Motor_Send    (RMESC_Current[CAN_Group][1],RMESC_Current[CAN_Group][2],RMESC_Current[CAN_Group][3],RMESC_Current[CAN_Group][4]);
		if ((Teleport_Status[CAN_Group] & 0b0010)>>1)
		Motor_Send_6020(GM6020_Current[CAN_Group][1],GM6020_Current[CAN_Group][2],GM6020_Current[CAN_Group][3],GM6020_Current[CAN_Group][4]);
		if ((Teleport_Status[CAN_Group] & 0b0001)>>0)
		Motor_Send_6020_ADD(GM6020_Current[CAN_Group][5],GM6020_Current[CAN_Group][6],GM6020_Current[CAN_Group][7]);
	}
	else if (CAN_Group == CAN_GROUP_2)
	{
		if ((Teleport_Status[CAN_Group] & 0b1000)>>3)
		Motor_Send2    (RMESC_Current[CAN_Group][1],RMESC_Current[CAN_Group][2],RMESC_Current[CAN_Group][3],RMESC_Current[CAN_Group][4]);
		if ((Teleport_Status[CAN_Group] & 0b0100)>>2)
		Motor_Send2_ADD(RMESC_Current[CAN_Group][5],RMESC_Current[CAN_Group][6],RMESC_Current[CAN_Group][7],RMESC_Current[CAN_Group][8]);
		if ((Teleport_Status[CAN_Group] & 0b0010)>>1)
		Motor_Send2_6020   (GM6020_Current[CAN_Group][1],GM6020_Current[CAN_Group][2],GM6020_Current[CAN_Group][3],GM6020_Current[CAN_Group][4]);
		if ((Teleport_Status[CAN_Group] & 0b0001)>>0)
		Motor_Send2_6020_ADD(GM6020_Current[CAN_Group][5],GM6020_Current[CAN_Group][6],GM6020_Current[CAN_Group][7]);
	}
}
/**
 * @name:Excute_RMESC_OutputCurrent
 * @brief:���²�ִ��VESC��������ֵ
*/
static inline void Excute_VESC_Output(uint8_t CAN_Group,Motor_Ctrl_Node* Node)
{
	if (Node->Teleport_Type == TELEPORT_VESC)
	{
	 	//--------------�������Ƶı�����-------------------------
	 	if (Node->Ctrl_Type == CTRL_TYPE_CUR)
	 	{
			if (CAN_Group == CAN_GROUP_1)
	 			VESC_Excute_Current(CAN1,Node->CAN_ID,Node->Current);
	 		else if (CAN_Group == CAN_GROUP_2)
				VESC_Excute_Current(CAN2,Node->CAN_ID,Node->Current);

			Node->Current=0;//������㣬������Ʋ�����
	 	}
	 	//--------------ת�ٿ��Ƶı�����------------------------
	 	else if (Node->Ctrl_Type == CTRL_TYPE_RPM)
	 	{
			if (CAN_Group == CAN_GROUP_1)
	 			VESC_Excute_Speed(CAN1,Node->CAN_ID,Node->Speed);
			else if (CAN_Group == CAN_GROUP_2)
				VESC_Excute_Speed(CAN2,Node->CAN_ID,Node->Speed);
	 		Node->Speed=0;
	 	}
	}
}





static inline int8_t Judge_MotorNode_isPosProtect(Motor_Ctrl_Node* Node)
{
	Node->Now_Angle=Node->Get_Node_Angle(Node->CAN_ID);
	
	if ((Node->Now_Angle >= Node->Max_LPos) && (Node->Now_Angle <= Node->Max_UPos ))
	{
		Node->Now_CtrlStatus.IsrBit_PosProtect=0;
		return 0;
	}
	else if (Node->Now_Angle <= Node->Max_LPos)
	{
		Node->Now_CtrlStatus.IsrBit_PosProtect=-1;
		return -1;
	}
	else if (Node->Now_Angle >= Node->Max_UPos)
	{
		Node->Now_CtrlStatus.IsrBit_PosProtect=1;
		return 1;
	}
}

static inline int8_t Judge_MotorNode_isSpdProtect(Motor_Ctrl_Node* Node)
{
	Node->Now_Speed=Node->Get_Node_Speed(Node->CAN_ID);
	
	if ((Node->Now_Speed >= Node->Max_LSpd) && (Node->Now_Speed <= Node->Max_USpd ))
	{
		Node->Now_CtrlStatus.IsrBit_SpdProtect=0;
		return 0;
	}
	else if (Node->Now_Speed <= Node->Max_LSpd)
	{
		Node->Now_CtrlStatus.IsrBit_SpdProtect=-1;
		return -1;
	}
	else if (Node->Now_Speed >= Node->Max_USpd)
	{
		Node->Now_CtrlStatus.IsrBit_SpdProtect=1;
		return 1;
	}
}

//Motor_List_T CtrlList_CAN_1,CtrlList_CAN_2;
//Motor_List_T CtrlList_PWM_1;
//ESC_Status_t Err_Controller(M_Status_t* target,M_Status_t* now,void* param);
//ESC_Status_t None_Spd_Through_Controller(M_Status_t* target,M_Status_t* now,void* param);
//ESC_Status_t None_Cur_Through_Controller(M_Status_t* target,M_Status_t* now,void* param);
//ESC_Status_t PID_Spd_Cur_Comm_Controller(M_Status_t* target,M_Status_t* now,void* param);
//ESC_Status_t PID_Ang_Cur_Comm_Controller(M_Status_t* target,M_Status_t* now,void* param);
//static uint8_t Invalid_CtrlList_Check(Motor_List_T* CtrlList_XXX_n,Motor_Node_T* Node_tobe_Checked);
//static void Excute_Singl_Ctrl_Motor(Motor_Node_T* Node);
//static uint8_t Update_Multi_Ctrl_Motor(Motor_Node_T* Node);
//static void Excute_Multi_Ctrl_Motor(uint8_t ctrl_flag);

//void MotorNode_Init_C610_New(Motor_Node_T* Node,uint8_t id)
//{
//	//����ID
//	Node->ESC_ID=id;

//	//�����޷�
//	Node->ESC_Limit.Current=10000;

//	//���ù۲���
//	Node->ESC_Observer.Speed_Observer=RM3508_Get_Speed_Rpm;
//	
//	//���ÿ�����
//	Node->ESC_Controller.Cur_Ctrl_Func=None_Cur_Through_Controller;//2006��������C610�������ջ�����
//	Node->ESC_Controller.Spd_Ctrl_Func=PID_Spd_Cur_Comm_Controller;//2006�ٶȻ�����pid������
//	Node->ESC_Controller.Ang_Ctrl_Func=Err_Controller;//2006�ǶȻ�ȱʡ,��Ҫ����ʵ������������

//	
//}

//void MotorNode_Add_New(Motor_List_T* CtrlList_XXX_x,Motor_Node_T* Node)
//{
//	//����Ƿ񳬳����ɿ��Ƶ������
//	if (CtrlList_XXX_x->List_Len ==CTRL_LIST_MAX_LEN) 
//	{
//		#ifdef RCS_MOTOR_UPCTRL_DEBUG
//			RCS_Shell_Logs("ERROR:in RCS_Motor_Upctrl.c, add too many motors");
//		#endif
//		return;
//	}

//	//����Ƿ�����õ������ÿ�����
//	#ifdef RCS_MOTOR_UPCTRL_DEBUG
//		uint8_t err=Invalid_CtrlList_Check(CtrlList_XXX_x,Node);
//		if (err!=0) 
//		{
//			if (err==1) RCS_Shell_Logs("ERR:same teleport in different motor ctrl list");
//			if (err==2) RCS_Shell_Logs("ERR:add same id to list for twice");
//			return;
//		}
//	#endif

//	//������ڵ���������
//	CtrlList_XXX_x->List_Node[CtrlList_XXX_x->List_Len]=Node;
//	CtrlList_XXX_x->List_Len++;
//}

//void MotorList_Excute_New(Motor_List_T* CtrlList_XXX_x)
//{
//	//step1: Ҫ���͵Ķ������ı�־λ
//	uint16_t multi_mesg_flag=0;

//	//step2: �������нڵ�
//	for(int i=0;i<CtrlList_XXX_x->List_Len;i++)
//	{
//		//��������ģ�ֱ��ִ��,���ִ�к�λ���
//		Excute_Singl_Ctrl_Motor(CtrlList_XXX_x->List_Node[i]);
//		//�������ģ��������ֵ+���±��ı�־λ,��ɸ��º�λ���
//		multi_mesg_flag|=Update_Multi_Ctrl_Motor(CtrlList_XXX_x->List_Node[i]);
//	}

//	//step3: ���ݱ��ı�־λ�����Ͷ�������
//	Excute_Multi_Ctrl_Motor(multi_mesg_flag);
//}

//static void Excute_Singl_Ctrl_Motor(Motor_Node_T* Node)
//{
//	static uint8_t bus_type=Node->ESC_Output.Bus_Type;
//	static int32_t output;

//	switch(Node->ESC_Output.Ctrl_Type)
//	{
//		case CTRL_TYPE_CUR:
//			output = Node->ESC_Output.Ctrl_Msg.Current;
//			if      (bus_type==BUS_CAN_1) Node->ESC_Updater.Current_Updater(CAN1,output);
//			else if (bus_type==BUS_CAN_2) Node->ESC_Updater.Current_Updater(CAN2,output);
//		break;

//		case CTRL_TYPE_RPM:
//			output = Node->ESC_Output.Ctrl_Msg.Speed;
//			if      (bus_type==BUS_CAN_1) Node->ESC_Updater.Speed_Updater(CAN1,output);
//			else if (bus_type==BUS_CAN_2) Node->ESC_Updater.Speed_Updater(CAN2,output);
//		break;
//	}
//}
//static uint8_t Update_Multi_Ctrl_Motor(Motor_Node_T* Node)
//{
//	uint8_t reval;
//	Excute_Singl_Ctrl_Motor(Node);
//	switch(Node->ESC_Controller.Cur_Ctrl_Func)
//	{
//		case RM3508_Update_Current
//	}
//}

//static void Excute_Multi_Ctrl_Motor(uint8_t ctrl_flag)
//{

//}
////���
//static uint8_t Invalid_CtrlList_Check(Motor_List_T* CtrlList_XXX_n,Motor_Node_T* Node_tobe_Checked)
//{
//	uint8_t bus_types=(Node_tobe_Checked->ESC_Output).Bus_Type;
//	uint8_t bus_Id=Node_tobe_Checked->ESC_ID;

//	for(int i=0;i<CtrlList_XXX_n->List_Len;i++)
//	{
//		//ԭ��1:���Ʊ��Ĳ���ͬһ�������ϵĵ�����������ͬһ����������
//		if (CtrlList_XXX_n->List_Node[i]->ESC_Output.Bus_Type != bus_types) return 1;
//		//ԭ��2:ͬһ������ID�ĵ��������ڶ��μ�������
//		if (CtrlList_XXX_n->List_Node[i]->ESC_ID == bus_Id) return 2;
//	}

//	return 0;	
//}
////ȱʡ������
//inline ESC_Status_t Err_Controller(M_Status_t* target,M_Status_t* now,void* param)
//{
//	ESC_Status_t reval;
//	reval.Ctrl_Type=CTRL_TYPE_CUR;
//	reval.Ctrl_Msg.Current=0;
//	return reval;
//}
////�ɵ����������ٶ�,��Ƭ������ӿ�����
//inline ESC_Status_t None_Spd_Through_Controller(M_Status_t* target,M_Status_t* now,void* param)
//{
//	ESC_Status_t reval;
//	reval.Ctrl_Type=CTRL_TYPE_RPM;
//	reval.Ctrl_Msg.Speed=target->Speed;
//	return reval;
//}
////�ɵ��������������,��Ƭ������ӿ�����
//inline ESC_Status_t None_Cur_Through_Controller(M_Status_t* target,M_Status_t* now,void* param)
//{
//	ESC_Status_t reval;
//	reval.Ctrl_Type=CTRL_TYPE_CUR;
//	reval.Ctrl_Msg.Current=target->Current;
//	return reval;
//}
////�ٶ�-������PID������
//inline ESC_Status_t PID_Spd_Cur_Comm_Controller(M_Status_t* target,M_Status_t* now,void* param)
//{
//	ESC_Status_t reval;
//	//Ӳ��֧�ֵ�������
//	reval.Ctrl_Type=CTRL_TYPE_CUR;
//	//PID���ص���ֵ
//	reval.Ctrl_Msg.Current=PID_Normal_Ctrl(target->Speed,now->Speed,(PID_Struct*)param);
//}
////�Ƕ�-������PID������
//inline ESC_Status_t PID_Ang_Cur_Comm_Controller(M_Status_t* target,M_Status_t* now,void* param)
//{
//	ESC_Status_t reval;
//	//Ӳ��֧�ֵ�������
//	reval.Ctrl_Type=CTRL_TYPE_CUR;
//	//PID���ص���ֵ
//	reval.Ctrl_Msg.Current=PID_Normal_Ctrl(target->Speed,now->Speed,(PID_Struct*)param);
//}
