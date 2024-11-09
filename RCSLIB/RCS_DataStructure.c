/**
 * @filename:RCS_DataStructure.c
 * @brief:ʵ�ֻ��������ݽṹ
 * @changlog: 2024-2-8 CYK-Dot  ���δ���
*/

/* =========ͷ�ļ�====================================*/
#include "RCS_DataStructure.h"

/* =========˽�к�====================================*/
#define QUENE_EMPTY_SIGN 0
#define QUENE_FILLED_SIGN 1

/* =======����ʵ��====================================*/

/**
 * @name:Quene_Init
 * @brief:��ʼ�����ζ���
 * @param:RCS_QUENE* quene  ָ����е�ָ��
 * @param:void* Quene_Data_Arr ��Ŷ������ݵ���������ڴ�ռ�
 * @param:uint16_t Quene_Len ���г���
 * @param:uint16_t Member_Size ��Ա����
 * @tips:��������һ�����float�Ķ���,��ôMember_Size������Sizeof(float)
*/
void Quene_Init(RCS_QUENE* quene,void* Quene_Data_Arr,uint16_t Quene_Len,uint16_t Member_Size)
{
	quene->bottom=0;
	quene->top=0;
	quene->is_empty=QUENE_EMPTY_SIGN;
	quene->len=Quene_Len;
	quene->data=Quene_Data_Arr;
	quene->data_size=Member_Size;
}

/**
 * @name:Quene_Add_Member
 * @brief:Ϊ���ζ�����ӳ�Ա
 * @param:RCS_QUENE* quene  ָ����е�ָ��
 * @param:void* member      ָ���Ա��ָ��
 * @tips:�ǵ��Ȱ�ָ������ǿ��ת��Ϊvoid*�������������
*/
void Quene_Add_Member(RCS_QUENE* quene,void* member)
{
	//�������
	memcpy(quene->data+(quene->top)*(quene->data_size),member,quene->data_size);
	//*(quene->data+(quene->top)*(quene->data_size))=*(member); ������
	
	//���±�־λ
	quene->top++;
	quene->is_empty=QUENE_FILLED_SIGN;
	if (quene->top >= quene->len) quene->top=0;	
}

/**
 * @name:Quene_Quit_Member
 * @brief:�ӻ��ζ�����ȡ��һ����Ա
 * @param:RCS_QUENE* quene  ָ����е�ָ��
 * @reval:void*             ָ����ӳ�Ա��ָ��
 * @tips:�ѷ��ص�void*ָ��ǿ��ת��Ϊ��Ҫ�����ͼ��ɻ�ȡ����
*/
void* Quene_Quit_Member(RCS_QUENE* quene)
{
	static void* reval;

	if (quene->is_empty==QUENE_EMPTY_SIGN)//��ͷ=��β����ζ�Ŷӿ�
	{
		reval=QUENE_MEMBER_EMPTY;//��ָ��
	}
	else
	{
		//����
		reval=quene->data+(quene->bottom)*(quene->data_size);
		quene->bottom++;
		
		//���±�־λ
		if (quene->bottom >= quene->len) quene->bottom=0;	
		if (quene->bottom == quene->top) quene->is_empty=QUENE_EMPTY_SIGN;//����֮���ͷ=��β���Ƕӿ�
	}
	
	return reval;
}

/**
 * @name:Quene_Clear
 * @brief:��ջ��ζ���
 * @param:RCS_QUENE* quene  ָ����е�ָ��
*/
void Quene_Clear(RCS_QUENE* quene)
{
	quene->bottom=0;
	quene->top=0;
	//quene->is_empty=QUENE_EMPTY_SIGN;
}



