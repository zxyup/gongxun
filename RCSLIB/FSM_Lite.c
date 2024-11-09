/**
 @name:FSM_Lite.h
 @brief:�˸������״̬��
 @readme:״̬ת�ƹ��ܱ�������֧����Ϊ��������֧��״̬-�¼�ת�Ʊ�
**/
#include "FSM_Lite.h"

/**
 * @name:FSM_Server
 * @brief:����״̬���Ĺ���ʵ��
 * @param:fsm    ��Ҫ���е�״̬��
 * @reval:EVENT  ״̬��������¼�
 * @readme:״̬���ڴ����¼�֮ǰ�����ǻ��Ȱ��¼��Ӻ����з��أ���ʱ�Ϳ����þ��߻��޸�״̬ת�Ʊ�ʵ��ʵʱ����
**/
EVENT FSM_Server(FSM_Struct* fsm)
{
    FSM_Struct input_fsm=(*fsm); //���������Ż��ı���
    //----------�����ں��¼�(״ִ̬�����)-------------------
    if (input_fsm.Event==EVENT_FINISH)                     //����EVENT_FINISH�¼�����ʱ�Ų���״̬ת��
	  {
        fsm->State=input_fsm.nState_Table[fsm->State];     //״̬ת��
        fsm->Event=EVENT_NONE;                             //�¼���λ
    }
	//-----------ִ��״̬����--------------------------------	
	if (input_fsm.State_Func[fsm->State]!=NULL)
	{
		fsm->Event=input_fsm.State_Func[fsm->State]();    //״ִ̬��+�¼����
	}
	else
	{
		fsm->Error_Func();       //����
	}
	
	return fsm->Event; //�����Ȱ��¼����ͳ�ȥ��������ִ��
}

/**
 * @name:FSM_Jump
 * @brief:�ֶ�����״̬����״̬
 * @param:fsm   ��Ҫ��ת״̬��״̬��
 * @param:state ��Ҫת������״̬
*/
void FSM_Jump(FSM_Struct* fsm,int32_t new_state)
{
    fsm->State=new_state;
}

/**
 @name: FSM_DeInit
 @brief:�������״̬��������,Ӧ��״̬����ʼ��֮��ִ��
**/
void FSM_DeInit(FSM_Struct* fsm)
{
	for(FSM_TYPE i=0;i<=FSM_SIZE;i++)
	{
		fsm->State_Func[i]=NULL;
		fsm->nState_Table[i]=FSM_SIZE;
	}
	fsm->Event=EVENT_NONE;
	fsm->Error_Func=NOP_VOID;
}


void FSM_ErrState_Init(FSM_Struct* fsm,FSM_VOID_Func func)
{
    fsm->Error_Func=func;
}

FSM_TYPE FSM_Get_State(FSM_Struct* fsm)
{
    return fsm->State;
}

EVENT NOP_VOID(void)
{
	return EVENT_NONE;
}



