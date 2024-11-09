/**
 * @name:FSM.c
 * @brief:�ṩ����״̬��֧��
 * @changelog:2024-3-10 CYK-Dot ���δ���
*/

/* ========ͷ�ļ�=============================================*/
#include "FSM.h"

/* ========ȫ�ֱ���===========================================*/
static volatile FNCT_VOID_PVD action_skip_hook=FSM_Hook_None;
static volatile FNCT_VOID_PVD state_err_hook =FSM_Hook_None;
static volatile FNCT_VOID_PVD state_none_hook=FSM_Hook_None;
static volatile FNCT_VOID_PVD fsm_main_hook  =FSM_Hook_None;
static volatile RCS_FSM_DEBUG_T fsm_log;

/* ========��̬����===========================================*/
static inline void FSM_Log(RCS_FSM_T* fsm);
static void FSM_Assert(RCS_FSM_T* fsm);
static void FNCT_Memset(FNCT_VOID_PVD* arr,FNCT_VOID_PVD target,FSM_Num_t arr_size);
FSM_Num_t FSM_Find_State_ID(RCS_FSM_T* fsm,FNCT_VOID_PVD State);
FSM_Num_t FSM_Find_Action_ID(RCS_FSM_T* fsm,FNCT_VOID_PVD Action);

/* ========��������===========================================*/

/**
 * @name:FSM_Init
 * @brief:��ղ���ʼ��һ��һ��״̬��
 * @param:RCS_FSM_T* fsm    ״̬���ĵ�ַ
 * @param:void* Global_Var  ��״̬�乲���αȫ�ֱ�����ָ��
 * @tips:0����Ϊ�ǲ�����Ϊ,0��״̬��ʲôҲ����,0���¼������·���
 * @tips:������Ӧ����Ϊ��ʼ���ĵ�һ�����������һ�����������һ�������״̬����ԭ�ȵ�����
*/
void FSM_Init(RCS_FSM_T* fsm,void* Global_Var)
{
	//״̬�����
	FNCT_Memset(fsm->Action_Func,FSM_Action_Skip,FSM_MAX_ACTION_COUNT);
	FNCT_Memset(fsm->States_Func,FSM_Func_Error ,FSM_MAX_STATE_COUNT);
	memset(fsm->SE2A_Tbl,0,sizeof(fsm->SE2A_Tbl));
	memset(fsm->SE2S_Tbl,0,sizeof(fsm->SE2S_Tbl));

	//ע�������Ϣ
	FSM_Add_Action(fsm,FSM_Action_Skip);
	FSM_Add_State(fsm,FSM_Func_None);
	FSM_Set_Event(fsm,FSM_NONE_EVENT_ID);
	fsm->FSM_Shared_Var=Global_Var;
}

/**
 * @name:FSM_Add_State
 * @brief:Ϊ״̬��ע��һ��״̬
 * @param:RCS_FSM_T* fsm ״̬���ĵ�ַ
 * @param:FNCT_VOID_PVD state ״̬����
 * @tips:���������Ϊx��Ӧ�ñ������void x(void* ipt) 
*/
void FSM_Add_State(RCS_FSM_T* fsm,FNCT_VOID_PVD state)
{
    fsm->States_Func[fsm->State_Count]=state;
    fsm->State_Count++;//����+1
    FSM_Assert(fsm);//�Լ�
}

/**
 * @name:FSM_Add_Action
 * @brief:Ϊ״̬��ע��һ����Ϊ
 * @param:RCS_FSM_T* fsm ״̬���ĵ�ַ
 * @param:FNCT_VOID_PVD action ��Ϊ����
*/
void FSM_Add_Action(RCS_FSM_T* fsm,FNCT_VOID_PVD action)
{
    fsm->Action_Func[fsm->Action_Count]=action;
    fsm->Action_Count++;//����+1
    FSM_Assert(fsm);//�Լ�
}

/**
 * @name:FSM_Add_Event_State
 * @param:RCS_FSM_T* fsm ״̬��ָ��
 * @param:FNCT_VOID_PVD From_State ��ʼ״̬������ֱ�ӰѺ��������ȥ����
 * @param:FNCT_VOID_PVD Target_State ת�ƺ�Ҫ����ĺ�����ֱ�ӰѺ��������ȥ����
 * @param:FSM_EVENT_t Event ������Event�¼���״̬��From_Stateת�Ƶ�Target_State��
 * @brief:Ϊ�¼�ָ��״̬ת��
*/
void FSM_Add_Event_State(RCS_FSM_T* fsm,FNCT_VOID_PVD From_State,FNCT_VOID_PVD Target_State,FSM_EVENT_t Event)
{
    fsm->SE2S_Tbl[FSM_Find_State_ID(fsm,From_State)][Event]=FSM_Find_State_ID(fsm,Target_State);
    fsm->SE2A_Tbl[FSM_Find_State_ID(fsm,From_State)][Event]=FSM_Find_Action_ID(fsm,FSM_Action_Skip);
}

/**
 * @name:FSM_Add_Event_Action
 * @param:RCS_FSM_T* fsm ״̬��ָ��
 * @param:FNCT_VOID_PVD From_State ��ʼ״̬������ֱ�ӰѺ��������ȥ����
 * @param:FNCT_VOID_PVD Trigger_Action �¼�������Ҫ�������˳�����
 * @param:FSM_EVENT_t Event ��From_State״̬�·�����Event�¼���ִ��Trigger_Action����һ�Ρ�
 * @brief:Ϊ�¼�ָ��״̬ת��ʱ�����Ķ���
*/
void FSM_Add_Event_Action(RCS_FSM_T* fsm,FNCT_VOID_PVD From_State,FNCT_VOID_PVD Trigger_Action,FSM_EVENT_t Event)
{
    fsm->SE2A_Tbl[FSM_Find_State_ID(fsm,From_State)][Event]=FSM_Find_Action_ID(fsm,Trigger_Action);
}

/**
 * @name:FSM_Set_Startup_Status
 * @brief:ָ��״̬���ĳ�ʼ״̬
 * @tips:���仰Ӧ����״̬ע�ᣬ�����޷���λ״̬������ID
*/
void FSM_Set_Startup_Status(RCS_FSM_T* fsm,FNCT_VOID_PVD startup_state,FSM_EVENT_t startup_event)
{
    if (FSM_Set_State(fsm,startup_state)==-1)
    {
        #ifdef FSM_DEBUG
            RCS_Shell_Logs(&Core407_RCSLIB_Debug,"ERROR:FSM.c,setup startup state earlier than state-function was registered!");
        #endif
    }
    FSM_Set_Event(fsm,startup_event);
}

/**
 * @name:FSM_Main
 * @param:RCS_FSM_T* fsm
 * @brief:״̬������
 * @tips:��ֱ�ӽ��ú�������������upctrl_task�У������ɸú������𴥷�״̬������ִ��
 * @tips:
*/
void FSM_Main(RCS_FSM_T* fsm)
{
    //���·���
    if (fsm->Current_Event == FSM_NONE_EVENT_ID)
    {
        //����״̬������ִ��
        fsm->States_Func[fsm->Current_State_ID](fsm->FSM_Shared_Var);
    }
    //���鷢��
    else
    {
        //���Ӻ���
        #ifdef FSM_HOOK_ENABLE
            fsm_main_hook(fsm->FSM_Shared_Var);
        #endif
        //��־����
        #ifdef FSM_ENABLE_RECORDER
            FSM_Log(fsm);
        #endif

        //������Ϊ������ִ��
        fsm->Action_Func[fsm->SE2A_Tbl[fsm->Current_State_ID][fsm->Current_Event]](fsm->FSM_Shared_Var);
        //״̬ת��
        fsm->Current_State_ID=fsm->SE2S_Tbl[fsm->Current_State_ID][fsm->Current_Event];
        //�¼�����
        fsm->Current_Event = FSM_NONE_EVENT_ID;
				//����״̬������ִ��
        fsm->States_Func[fsm->Current_State_ID](fsm->FSM_Shared_Var);
    }
}

/**
 * @name:FSM_Trigger_Event
 * @brief:����һ���¼�
*/
void FSM_Set_Event(RCS_FSM_T* fsm,FSM_EVENT_t Event)
{
    fsm->Current_Event=Event;
}

/**
 * @name:FSM_Set_State
 * @brief:ǿ��ָ��״̬����ǰ������״̬
 * @tips:�ú��������ڵ��ԣ���Ҫ������������״̬��ת��
 * @reval:�����Ƿ�ɹ�,�ɹ�����1�����򷵻�-1
*/
int FSM_Set_State(RCS_FSM_T* fsm,FNCT_VOID_PVD State)
{
    FSM_Num_t state_id=FSM_Find_State_ID(fsm,State);
    if (state_id!=-1)
    {
        fsm->Current_State_ID=FSM_Find_State_ID(fsm,State);
        return 1;
    }
    else
        return -1;
}

/**
 * @name:FSM_Find_State_ID
 * @brief:����ĳ��״̬������ע��ɵ�ID
*/
FSM_Num_t FSM_Find_State_ID(RCS_FSM_T* fsm,FNCT_VOID_PVD State)
{
    for(int i=0;i<fsm->State_Count;i++)
    {
        if (fsm->States_Func[i]==State) return i;
    }

    return -1;
    #ifdef FSM_DEBUG
        RCS_Shell_Logs(&Core407_RCSLIB_Debug,"Error:Can`t Find Target State");
    #endif
}

/**
 * @name:FSM_Find_Action_ID
 * @brief:����ĳ����Ϊ������ע���ID
*/
FSM_Num_t FSM_Find_Action_ID(RCS_FSM_T* fsm,FNCT_VOID_PVD Action)
{
    for(int i=0;i<fsm->Action_Count;i++)
    {
        if (fsm->Action_Func[i]==Action) return i;
    }

    #ifdef FSM_DEBUG
        RCS_Shell_Logs(&Core407_RCSLIB_Debug,"Error:Can`t Find Target Action");
    #endif
}

/**
 * @name:FSM_Get_GlobalVar
 * @brief:����״̬���ڲ�ȫ�ֱ�����ָ��
*/
void*  FSM_Get_GlobalVar(RCS_FSM_T* fsm)
{
    return fsm->FSM_Shared_Var;
}

/**
 * @name:FSM_Get_Event
 * @brief:��ȡ״̬����ǰ���¼�
*/
FSM_Num_t FSM_Get_Event(RCS_FSM_T* fsm)
{
    return fsm->Current_Event;
}

/**
 * @name:FSM_Hook_On_Action_Skip
 * @brief:ΪĬ����Ϊ����ע�ṳ�Ӻ���
 * @tips:���û��Ϊ״̬-�¼�ת��ע��Action,��Ĭ�ϻ�ִ��Action_Skip
*/
void FSM_Hook_On_Action_Skip(FNCT_VOID_PVD hook_func)
{
    #ifdef FSM_HOOK_ENABLE
        action_skip_hook=hook_func;
    #else
        #error FSM.c:��ͼ��δ���ù��ӵ�ʱ��ѹ��Ӻ���ע���Action Skip
    #endif
}
/**
 * @name:FSM_Hook_On_Action_Skip
 * @brief:Ϊ�����״̬ע�ṳ�Ӻ��������⳵���ܷ�
 * @tips:���԰���ع��ܹ������״̬�������Ų�����
*/
void FSM_Hook_On_State_Err(FNCT_VOID_PVD hook_func)
{
    #ifdef FSM_HOOK_ENABLE
        state_err_hook=hook_func;
    #else
        #error FSM.c:��ͼ��δ���ù��ӵ�ʱ��ѹ��Ӻ���ע���State Err
    #endif
}
/**
 * @name:FSM_Hook_On_FSM_Main
 * @brief:���û��������״̬���ں���
 * @tips:�ù��ӽ�����״̬ת��ʱ����������
*/
void FSM_Hook_On_FSM_Main(FNCT_VOID_PVD hook_func)
{
    #ifdef FSM_HOOK_ENABLE
        fsm_main_hook=hook_func;
    #else
        #error FSM.c:��ͼ��δ���ù��ӵ�ʱ��ѹ��Ӻ���ע���FSM_Main
    #endif
}
/**
 * @name:FSM_Get_Recorder
 * @brief:����״̬����־���
*/
RCS_FSM_DEBUG_T FSM_Get_Recorder(void)
{
    return fsm_log;
}
/* =================��̬��������====================================*/
/**
 * @name:FSM_Assert
 * @brief:������״̬��ʱ����Ƿ�Ϸ�
*/
static void FSM_Assert(RCS_FSM_T* fsm)
{
    if ((fsm->Action_Count >= FSM_MAX_ACTION_COUNT)||(fsm->State_Count  >= FSM_MAX_EVENT_COUNT))
    {
        #ifdef FSM_DEBUG
        RCS_Shell_Logs(&Core407_RCSLIB_Debug,"Error:FSM Action or State function overflow!");
        uint8_t err=10/0.0f;
        #endif
    }
    return;
}
/**
 * @name:
*/
static FSM_Num_t FSM_State_Trans_Assert(RCS_FSM_T* fsm)
{
    if (fsm->States_Func[fsm->SE2S_Tbl[fsm->Current_State_ID][fsm->Current_Event]]==FSM_Func_Error)
    {
        return 0;
        RCS_Shell_Logs(&Core407_RCSLIB_Debug,"Warning:FSM.c,jump to an undefined state!");
    }
    else 
        return 1;
}
/**
 * @name:FNCT_Memset
 * @brief:ר�ű�д��memset����
*/
static void FNCT_Memset(FNCT_VOID_PVD* arr,FNCT_VOID_PVD target,FSM_Num_t arr_size)
{
	for(int i=0;i<arr_size;i++)
	{
		arr[i]=target;
	}
}

/**
 * @name:FSM_Log
 * @brief:���ζ��У���¼״̬����״̬ת��ʱ�������Ϣ�������ռ�����
*/
static inline void FSM_Log(RCS_FSM_T* fsm)
{
    fsm_log.Event_ID_Log[fsm_log.log_len]=fsm->Current_Event;
    fsm_log.State_ID_Log[fsm_log.log_len]=fsm->Current_State_ID;
    fsm_log.Action_ID_Log[fsm_log.log_len]=fsm->SE2A_Tbl[fsm->Current_State_ID][fsm->Current_Event];
    fsm_log.log_len++;
    if (fsm_log.log_len==FSM_MAX_LOG_LEN) fsm_log.log_len=0;
}
/**
 * @name:FSM_Func_Error
 * @brief:״̬���ں�˽�еĺ���������ͼת��һ��δ�����״̬ʱ���ͻ����ú���
*/
void FSM_Func_Error(void* ipt)
{
    #ifdef FSM_DEBUG
    RCS_Shell_Logs(&Core407_RCSLIB_Debug,"Error:FSM Entered error state or action");
    #endif

    #ifndef FSM_HOOK_ENABLE
        return;
    #else
        state_err_hook(ipt);
        return;
    #endif
}
/**
 * @name:FSM_Action_Skip
 * @brief:״̬���ں�˽�еĺ�������δָ����ת����ʱ��Ĭ��ִ�иú�����֧�ֹ���
*/
void FSM_Action_Skip(void* ipt)
{
    #ifndef FSM_HOOK_ENABLE
    return;
    #else
    action_skip_hook(ipt);
    #endif
}
/**
 * @name:FSM_Func_None
 * @brief:״̬���ں�˽�еĺ�������ʾʲô��������״̬������֧�ֹ���
*/
void FSM_Func_None(void* ipt)
{
    #ifndef FSM_HOOK_ENABLE
    return;
    #else
    state_none_hook(ipt);
    #endif
}

void FSM_Hook_None(void* ipt)
{
    return;
}
