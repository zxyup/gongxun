/* =============ͷ�ļ�=================================*/
#include "Cup23_FSM_Round1.h"

#define EVENT_CHANGE_LED 1
#define EVENT_CHANGE_DIRECTION 2

/* =============ȫ�ֱ���===============================*/
context_var_t var;
RCS_FSM_T fsm;

void FSM_Blink_Main(void)
{
    FSM_Main(&fsm);
}

void FSM_Blink_Init(void)
{
    //�����������ʼ��
    IO_Init(IO_DEBUG_MAP);
    FSM_Init(&fsm,&var);

    //ע��״̬�����붯������
    FSM_Add_State(&fsm,LED1_Blink);
    FSM_Add_State(&fsm,LED2_Blink);
    FSM_Add_State(&fsm,LED3_Blink);
    FSM_Add_State(&fsm,LED4_Blink);
    FSM_Add_Action(&fsm,LED_Change_Freq);

    //��д״̬ת�Ʊ��״̬�¼���
    FSM_Add_Event_State(&fsm,LED1_Blink,LED2_Blink,EVENT_CHANGE_LED);//led1����led2��
    FSM_Add_Event_State(&fsm,LED2_Blink,LED3_Blink,EVENT_CHANGE_LED);//led2����led3��
    FSM_Add_Event_State(&fsm,LED3_Blink,LED4_Blink,EVENT_CHANGE_LED);
    FSM_Add_Event_State(&fsm,LED4_Blink,LED1_Blink,EVENT_CHANGE_LED);
    FSM_Add_Event_Action(&fsm,LED4_Blink,LED_Change_Freq,EVENT_CHANGE_LED);//led4���괥���¼����ı���˸Ƶ��

    //����״̬����ʼ״̬
    FSM_Set_Startup_Status(&fsm,LED1_Blink,FSM_NONE_EVENT_ID);
    
    //���ù���ʵ��״̬��λ
    FSM_Hook_On_Action_Skip(Reset_Context_StatusFlag);
}

void LED1_Blink(void* ipt)
{
    /*---�����ı�����ȡ-------------------------------------*/
    context_var_t* ipts=ipt;
    /*---ִ��״̬����---------------------------------------*/
    switch(ipts->status)
    {
        case 0:
            IO_SetBits(IO_DEBUG_MAP,0);
            if (Count_Delay(ipts->blink_freq,0)) ipts->status++;
        break;
        case 1:
            IO_ResetBits(IO_DEBUG_MAP,0);
            if (Count_Delay(ipts->blink_freq,0)) ipts->status++;
        break;
    }
    /*---�����¼�-----------------------------------------*/
    if (ipts->status==2) FSM_Set_Event(&fsm,EVENT_CHANGE_LED);
}
void LED2_Blink(void* ipt)
{
    /*---�����ı�����ȡ-------------------------------------*/
    context_var_t* ipts=ipt;
    /*---ִ��״̬����---------------------------------------*/
    switch(ipts->status)
    {
        case 0:
            IO_SetBits(IO_DEBUG_MAP,1);
            if (Count_Delay(ipts->blink_freq,0)) ipts->status++;
        break;
        case 1:
            IO_ResetBits(IO_DEBUG_MAP,1);
            if (Count_Delay(ipts->blink_freq,0)) ipts->status++;
        break;
    }
    /*---�����¼�-----------------------------------------*/
    if (ipts->status==2) FSM_Set_Event(&fsm,EVENT_CHANGE_LED);
}
void LED3_Blink(void* ipt)
{
    /*---�����ı�����ȡ-------------------------------------*/
    context_var_t* ipts=ipt;
    /*---ִ��״̬����---------------------------------------*/
    switch(ipts->status)
    {
        case 0:
            IO_SetBits(IO_DEBUG_MAP,2);
            if (Count_Delay(ipts->blink_freq,0)) ipts->status++;
        break;
        case 1:
            IO_ResetBits(IO_DEBUG_MAP,2);
            if (Count_Delay(ipts->blink_freq,0)) ipts->status++;
        break;
    }
    /*---�����¼�-----------------------------------------*/
    if (ipts->status==2) FSM_Set_Event(&fsm,EVENT_CHANGE_LED);
}
void LED4_Blink(void* ipt)
{
    /*---�����ı�����ȡ-------------------------------------*/
    context_var_t* ipts=ipt;
    /*---ִ��״̬����---------------------------------------*/
    switch(ipts->status)
    {
        case 0:
            IO_SetBits(IO_DEBUG_MAP,3);
            if (Count_Delay(ipts->blink_freq,0)) ipts->status++;
        break;
        case 1:
            IO_ResetBits(IO_DEBUG_MAP,3);
            if (Count_Delay(ipts->blink_freq,0)) ipts->status++;
        break;
    }
    /*---�����¼�-----------------------------------------*/
    if (ipts->status==2) FSM_Set_Event(&fsm,EVENT_CHANGE_LED);
}

void LED_Change_Freq(void* ipt)
{
    context_var_t* ipts=ipt;
    //��ʱ����+1
    ipts->blink_freq=ipts->blink_freq+2;
		if (ipts->blink_freq==60) ipts->blink_freq=0;
    //״̬��λ
    ipts->status=0;
}

void Reset_Context_StatusFlag(void* ipt)
{
    context_var_t* ipts=ipt;
    ipts->status=0;
}

