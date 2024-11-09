#ifndef _RCS_FSM_H__
#define _RCS_FSM_H__

/* -----------------ͷ�ļ�------------------------------------*/
//C���Թ�����
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

//RCSLIBģ��������(�㷨��)
#include "RCS_DataStructure.h" //�������ݽṹ
#include "RCS_Types.h"

//RCSLIB�����˹��ܲ�
#include "RCS_Debug.h"
#include "Core407_RCS12_Debug.h"


/* ----------------���ú�-------------------------------------*/
#define FSM_MAX_STATE_COUNT     50 //״̬������״̬����
#define FSM_MAX_ACTION_COUNT    10 //״̬�����Ķ�������
#define FSM_MAX_EVENT_COUNT     50 //״̬�������¼�����
#define FSM_MAX_LOG_LEN         70 //�����־��¼����

#define FSM_DEBUG                  //�������ڵ������
#define FSM_HOOK_ENABLE            //�������Ӻ���֧��
#define FSM_ENABLE_RECORDER        //������־�ռ�����

/* ----------------˽�к�-------------------------------------*/
#define FSM_NONE_EVENT_ID       0 //���·������¼�ID


/* ----------------�������ݽṹ--------------------------------*/
typedef uint8_t FSM_Num_t;      //״̬����״̬/����������<255����ʹ��uint8_t
typedef uint16_t FSM_EVENT_t;   //״̬�����¼������п���>255����ʹ��uint16_t

//����״̬��
typedef struct{
    //Public����
    FSM_EVENT_t   Current_Event;      //��ǰ�¼���ID
    FSM_Num_t     Current_State_ID;   //��ǰ״̬������ID
    void*         FSM_Shared_Var;     //״̬�������ı���ָ��

    //Private����
    FNCT_VOID_PVD States_Func[FSM_MAX_STATE_COUNT];
    FNCT_VOID_PVD Action_Func[FSM_MAX_ACTION_COUNT];
    FSM_Num_t     SE2S_Tbl   [FSM_MAX_STATE_COUNT][FSM_MAX_EVENT_COUNT];//����State��Event��ID,����ת�ƺ�State��ID
    FSM_Num_t     SE2A_Tbl   [FSM_MAX_STATE_COUNT][FSM_MAX_EVENT_COUNT];//����State��Event��ID,����Ҫִ�е�Action��ID
    FSM_Num_t     State_Count;                       //״̬����
    FSM_Num_t     Action_Count;                      //��Ϊ����
}RCS_FSM_T;

//��չ״̬��
typedef struct{
    //Public����
    FSM_EVENT_t   Current_Event;
    FSM_Num_t     Current_State_ID;
    void*         FSM_Shared_Var;

    //Private����
    FNCT_VOID_PVD States_Func[FSM_MAX_STATE_COUNT];
    FNCT_VOID_PVD ExiAct_Func[FSM_MAX_ACTION_COUNT];
    FNCT_VOID_PVD EntAct_Func[FSM_MAX_ACTION_COUNT];
    FSM_Num_t     SE2S_Tbl   [FSM_MAX_STATE_COUNT][FSM_MAX_EVENT_COUNT];//����State��Event��ID,����ת�ƺ�State��ID
    FSM_Num_t     SE2A_Tbl   [FSM_MAX_STATE_COUNT][FSM_MAX_EVENT_COUNT];
    FSM_Num_t     State_Count;                       //״̬����
    FSM_Num_t     ExtAct_Count;                      //�˳���������
    FSM_Num_t     EntAct_Count;                      //���붯������
}RCS_FSM_EXT_T;

//״̬����־
typedef struct 
{
    FSM_EVENT_t   Event_ID_Log[FSM_MAX_LOG_LEN];
    FSM_Num_t     State_ID_Log[FSM_MAX_LOG_LEN];
    FSM_Num_t     Action_ID_Log[FSM_MAX_LOG_LEN];
    uint16_t      log_len;
}RCS_FSM_DEBUG_T;


/* -----------------��������------------------------------------*/

//����״̬��
void FSM_Init(RCS_FSM_T* fsm,void* Global_Var);
void FSM_Add_State(RCS_FSM_T* fsm,FNCT_VOID_PVD state);
void FSM_Add_Action(RCS_FSM_T* fsm,FNCT_VOID_PVD action);
void FSM_Set_Startup_Status(RCS_FSM_T* fsm,FNCT_VOID_PVD startup_state,FSM_EVENT_t startup_event);

//����״̬ת�Ʊ�
void FSM_Add_Event_State(RCS_FSM_T* fsm,FNCT_VOID_PVD From_State,FNCT_VOID_PVD Target_State,FSM_EVENT_t Event);
void FSM_Add_Event_Action(RCS_FSM_T* fsm,FNCT_VOID_PVD From_State,FNCT_VOID_PVD Trigger_Action,FSM_EVENT_t Event);

//״̬������
void FSM_Main(RCS_FSM_T* fsm);
void FSM_Set_Event(RCS_FSM_T* fsm,FSM_EVENT_t startup_event);

//���Ը�������
FSM_Num_t FSM_Find_State_ID(RCS_FSM_T* fsm,FNCT_VOID_PVD State);
FSM_Num_t FSM_Find_Action_ID(RCS_FSM_T* fsm,FNCT_VOID_PVD Action);
void*     FSM_Get_GlobalVar(RCS_FSM_T* fsm);
FSM_Num_t FSM_Get_Event(RCS_FSM_T* fsm);
int       FSM_Set_State(RCS_FSM_T* fsm,FNCT_VOID_PVD State);//����ʹ�øú�������ת״̬
void      FSM_Hook_On_Action_Skip(FNCT_VOID_PVD hook_func);
void      FSM_Hook_On_State_Err(FNCT_VOID_PVD hook_func);
void      FSM_Hook_On_FSM_Main(FNCT_VOID_PVD hook_func);
RCS_FSM_DEBUG_T FSM_Get_Recorder(void);

//״̬���ײ�ĳ�����
void FSM_Func_None(void* ipt);
void FSM_Func_Error(void* ipt);
void FSM_Action_Skip(void* ipt);
void FSM_Hook_None(void* ipt);

/* -----------------������--------------------------------------*/
#define FSM_EVENT_NONE       0
#define FSM_ERR_STATE_FUNC   FSM_Func_Error


#endif