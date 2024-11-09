#ifndef RCS_RTOS_H_
#define RCS_RTOS_H_
/*============������=====================================================*/
//----------C���Թ�����----------------------
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

//----------BSP���м��------------------------
#include "stm32f4xx.h"        //STM32�Ĵ�������
#include "stm32f4xx_conf.h"   //STM32��׼��ͷ�ļ�
#include "bsp.h"              //uCOS�ײ����
#include "includes.h"         //uCOS�ر����
#include "delay.h"            //��ʱ����
//----------Ӳ�������------------------------
#include "RCS_Timer.h"
//----------ģ��������-------------------------
#include "RCS_filter.h"

/*============ʱ�������=====================================================*/
//-------------���ú�---------------------
#define RTOS_TIMETICK_US  1000000.0f/OS_TICKS_PER_SEC//һ��RTOS��tick�Ƕ���΢��
#define RTOS_S_TIMETICK   OS_TICKS_PER_SEC           //һ���м���RTOS��tick 
#define HARD_MS_TIMETICK  168000                     //һ�������м���Ӳ��ʱ��tick
#define HARD_TICKS_PER_US 0.0005952380f              //һ��Ӳ��ʱtick��ʱ����us
#define SOFT_TICKER_LEN   200

//------------�������ݽṹ----------------
struct rtos_hard_timer
{
    uint8_t  enable;        //�Ƿ���Ӳʱ���
    uint64_t total_tick;    //Ӳʱ�������
    TIM_TypeDef* TIMx;      //����Ӳʱ����Ķ�ʱ��
    uint8_t overflow_count; //��ʱ���������
    uint16_t cnt;           //��ʱ����ǰ����
};
typedef struct rtos_hard_timer RCS_Hard_Stamper_T;

//------------���������ӿ�----------------
void RCS_Hard_Stamper_Init(TIM_TypeDef* TIMx);
void RCS_Hard_Stamper_Disable(void);
void RCS_Hard_Stamper_enable(void);
uint64_t Hard_Get_RealTime_Stamp(void);
float Hard_Get_RealTime_Us(void);
void Hard_Delay_Us(uint32_t nus);
void RCS_Hard_Stamper_Reset(void);

uint32_t RTOS_Get_RealTime_Stamp(void);
uint32_t RTOS_Get_RealTime_Us(void);
void RTOS_Delay_Ms(uint16_t nms);
uint8_t RTOS_Count_Delay_Ms(uint8_t nms,uint8_t cycle_ms,uint8_t timer_id);
int Count_Delay(int count_num,int num);
/*============����ͬ������=====================================================*/
//-------------���ú�---------------------
#define RCS_EVENT_GROUP_COUNT 3

//------------�������ݽṹ----------------

typedef uint8_t RCS_MUTEX_T;
typedef uint32_t RCS_EVENT_T;
//------------���������ӿ�----------------
uint8_t RCS_Pend_Mutex(RCS_MUTEX_T* mutex);
void RCS_Rel_Mutex(RCS_MUTEX_T* mutex);
void RCS_Event_Set(uint8_t event_group,uint8_t event_id);
void RCS_Event_Reset(uint8_t event_group,uint8_t event_id);
uint8_t RCS_Event_Read(uint8_t event_group,uint8_t event_id);

/*============�ڴ������=====================================================*/
//-------------���ú�---------------------
#define BLOCK_LEN    35    //������      
#define BLOCK_SIZE   40    //ÿ����Ĵ�С(byte) 
#define STK_SIZE     25     //ջ�ռ��С
//STM32F407VG��128k RAM,ƽ�������ռȥ20k���ң���˲�����BLOCK_LEN* BLOCK_SIZE>1650
//�������Ŀռ�ᵼ��L6406E����
typedef uint8_t BLOCK_NUM_T;    
//------------�������ݽṹ----------------
 
struct block_type
{
    uint8_t data[BLOCK_SIZE];
};
struct stk_type
{
    uint8_t data[STK_SIZE];
};
typedef struct block_type Block_t;      
typedef struct stk_type   Stack_t;

struct soft_mmu
{
    //��ռ����
    Block_t     block_partion[BLOCK_LEN][BLOCK_SIZE]; //��ռ�
    uint8_t     block_ability[BLOCK_LEN]; //�ռ��Ƿ����

    //ջ�ռ����
    Stack_t     stk_partion;//ջ�ռ�
    void*       stk_top;    //ջ��
};
typedef struct soft_mmu RCS_MMU_T;

//------------���������ӿ�----------------
void* RCS_MMU_Malloc(RCS_MMU_T* mmu,uint32_t size);
void RCS_MMU_Free(RCS_MMU_T* mmu,void* addr);
uint8_t RCS_MMU_Push(RCS_MMU_T* mmu,void* data,uint32_t size);
void* RCS_MMU_Pop(RCS_MMU_T* mmu);

#endif