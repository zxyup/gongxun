#ifndef _FSM_H_
#define _FSM_H_

#include <stdint.h>
#include <string.h>
//#include "rcs.h"  �궨����Ҫ��ͷ�ļ����õĻ�������include��ͷ�ļ�

/*-״̬���������Ͷ���-*/
#define FSM_SIZE      25             //״̬������
#define FSM_TYPE      int32_t        //32λ��Ƭ������32λ����������
#define EVENT FSM_TYPE               //��������Ϊ�¼��ĺ���
typedef EVENT(*FSM_VOID_Func)(void); //�����룬�����¼��ĺ���ָ��
typedef EVENT(*FSM_INPT_Func)(void*);//�����룬�����¼��ĺ���ָ��

/*-״̬���ں��¼�-*/
#define EVENT_NONE    0     //�¼������¼�
#define EVENT_FINISH  -1    //�¼���״̬����ִ�����
#define EVENT_PAUSE   -2    //�¼���״̬��������

/*-״̬���ں�״̬����-*/
EVENT NOP_VOID(void);        //û�����ĺ���

/*-״̬������-*/
typedef struct 
{
    FSM_TYPE Event;                    //��ǰ�¼�ID 
    FSM_TYPE State;                    //��ǰ״̬ID
    FSM_TYPE nState_Table[FSM_SIZE+1];   //״̬ת�Ʊ�,������EVENT_FINISH�¼���,ת�Ƶ���һ��״̬
    FSM_VOID_Func State_Func[FSM_SIZE+1];//״̬����ָ������
    FSM_VOID_Func Error_Func;
}FSM_Struct;

/*-״̬������-*/
EVENT FSM_Server(FSM_Struct* fsm);
void FSM_Jump(FSM_Struct* fsm,int32_t new_state);
void FSM_ErrState_Init(FSM_Struct* fsm,FSM_VOID_Func func);

/*-״̬��������*/
FSM_TYPE FSM_Get_State(FSM_Struct* fsm);



#endif