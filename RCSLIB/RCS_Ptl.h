/* .h head --------------------------------------------------------------*/
#ifndef RCS_PROTOCOL_H_
#define RCS_PROTOCOL_H_

/* ͷ�ļ� ---------------------------------------------------------------*/
#include "stdint.h"
#include "string.h"
#include "RCS_Types.h"

/* ˽������ --------------------------------------------------------------*/

//��ͷβЭ�����
typedef struct protocol{
    uint8_t Start_Byte;
    uint8_t Len;
    uint8_t End_Byte;
		FNCT_VOID_PU8 Finish_Callback;
}RCS_Easy_Ptl;


//�򵥳���У��Э��
typedef struct{

    uint8_t Start_Byte;  //ͷ�ֽ�
    uint8_t Len;         //������Ч����

    uint8_t  RcvStat_Fsm; //����״̬����ǰ��״̬
    uint8_t  RcvBufr_Pnt; //���������е�ǰ�±�
    uint8_t* RcvBufr_Arr; //����������

}RCS_LenVerify_Ptl;

//�䳤ͷβЭ����־����
typedef enum comm_ptl_log{
    RCV_PROC =0,          //֡��δ��ɽ���
    RCV_CPLT =1,          //֡��ɽ��ն���У��ɹ�
    RCV_FAIL =2,          //֡��ɽ��յ���У��ʧ��
    RCV_ERR  =3,          //����״̬����״̬������ı�,�����˲�Ӧ�ý����case
    RCV_IDLE =4,          //����,��δ���ڹ���״̬
}RCS_Common_Ptl_Log;

//�䳤ͷβЭ��֡����
typedef enum comm_ptl_cfg{
    Pkg_Type_Verify     =0,   //�����У������ݰ�
    Pkg_Type_Direct     =1,   //����βУ������ݰ�
    Pkg_Type_FullVerify =2,   //�����+��βУ������ݰ�(todo
		Pkg_Type_Fix        =3,
}RCS_Common_Ptl_PkgType;

//�䳤ͷβЭ�����
typedef struct comm_protocol{
    volatile uint8_t                Start_Byte;    //ʹ�õİ�ͷ�ֽ�
    volatile RCS_Common_Ptl_PkgType Pkg_Type;      //ʹ�õ�֡����
    volatile uint8_t                End_Byte;      //ʹ�õİ�β�ֽ�
		volatile uint8_t                Max_Len;
}RCS_Common_Ptl;

//�䳤ͷβЭ�黺����
typedef struct comm_ptl_buffer{
    volatile int8_t   status_flag;  //����״̬����״̬
    volatile uint8_t* rcv_buffer;   //���ջ�����,�ڳ�ʼ����ʱ��ָ��,��С��ӦС�����ݳ���+2
    volatile uint8_t  rcv_pointer;  //����д����ջ��������±�

    volatile uint8_t  rcv_pkg_len;  //���յ��ı��ĳ���
    volatile uint8_t  rcv_pkg_type; //���յ���֡������
    volatile uint8_t  crc_h;        //ʵ�ʼ�������ܺ͵ĵ�16~8λ
    volatile uint8_t  crc_l;        //ʵ�ʼ�������ܺ͵ĵ�7~0λ
}RCS_Common_Ptl_Buffer;

/* �������� --------------------------------------------------------------*/
//��·��Э�麯��ָ��
typedef uint8_t (*FNCT_PTL_PACK)      (void*,uint8_t*,uint8_t*,uint8_t);//�����ݽ��д���ĺ���ָ��
typedef uint8_t (*FNCT_PTL_UNPACK_ARR)(void*,uint8_t*,uint8_t*,uint8_t);//����һ�����ݵĺ���ָ��
typedef uint8_t (*FNCT_PTL_UNPACK_IT) (void*,uint8_t ,uint8_t*);        //���������ֽڵĺ���ָ��


/* �������� --------------------------------------------------------------*/
typedef struct comm_ptl_handler{
    //֡��ʽ
    RCS_Common_Ptl        PTL_Param;
    //֡����״̬
    RCS_Common_Ptl_Buffer PTL_Rcv_Buffer;
    RCS_Common_Ptl_Log    PTL_Log;
    //��·��ӿ�
    FNCT_PTL_PACK         PTL_Get;
    FNCT_PTL_UNPACK_ARR   PTL_RcvArr;
    FNCT_PTL_UNPACK_IT    PTL_RcvChr;
}RCS_Comm_Ptl_Handler;



/* �������� ---------------------------------------------------------------*/

//Easy Protocol
void Protocol_Rcv_Easy(uint8_t input_byte,  uint8_t* temp_arr,
                       uint8_t* output_flag,uint8_t* output_arr,
                       RCS_Easy_Ptl* protocol);

void Protocol_Get_Easy(uint8_t* input_arr,
                       uint8_t* output_arr,
                       RCS_Easy_Ptl* protocol);

//LenVerify Protocol
void Protocol_Init_LenVerify(RCS_LenVerify_Ptl* ptl,uint8_t start_byte,uint8_t len,uint8_t* buffer_arr);
uint8_t Protocol_Rcv_LenVerify_IT(void* ptl_handler_void,uint8_t input_byte,uint8_t* output_arr);
uint8_t Protocol_Get_LenVerify(void* ptl_handler_void,uint8_t* output_arr,uint8_t* input_arr,uint8_t len);

//BleNet Protocol
uint8_t Protocol_Rcv_RCSBleNet_IT(void* ptl_handler_void,uint8_t input_byte,uint8_t* output_arr);
uint8_t Protocol_Rcv_RCSBleNet_ARR(void* ptl_handler_void,uint8_t* input_byte,uint8_t* output_arr,uint8_t input_len);//todo
uint8_t Protocol_Get_RCSBleNet(void* ptl_handler_void,uint8_t* output_arr,uint8_t* input_arr,uint8_t len);
uint8_t Protocol_Init_RCSBleNet(RCS_Comm_Ptl_Handler* ptl_Handler_p,uint8_t start_byte,uint8_t end_byte,RCS_Common_Ptl_PkgType pkg_type,uint8_t* buffer_arr);

/* ������������ ----------------------------------------------------------*/
inline void PTL_CHANGE_BIT(uint32_t* target_byte,uint8_t position,uint8_t new_state)
{
    if (new_state==1)
    {
        *target_byte |= 1 << position;
    }
    else
    {
        *target_byte &= ~(1 << position);
    }
}
inline void PTL_SET_BIT(uint32_t* target_byte,uint8_t position)
{
    *target_byte |= 1 << position;
}
inline void PTL_RESET_BIT(uint32_t* target_byte,uint8_t position)
{
    *target_byte &= ~(1 << position);
}
/* .h head --------------------------------------------------------------*/
#endif
