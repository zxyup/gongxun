/* ---------.h head-------------------------------------*/
#ifndef RCS_DATA_STRUCTURE_H_
#define RCS_DATA_STRUCTURE_H_

/* ---------������--------------------------------------*/
//C���Թ�����
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stddef.h>

/* ---------�����ṹ��------------------------------------*/
typedef struct arr_quene{
	void* data;
	uint16_t data_size;
	uint16_t len;
	uint16_t top;
	uint16_t bottom;
	uint8_t  mutex;
	uint8_t  is_empty;
}RCS_QUENE;

/* ---------������---------------------------------------*/
#define QUENE_MEMBER_EMPTY NULL

/* ---------��������-------------------------------------*/
void Quene_Init(RCS_QUENE* quene,void* Quene_Data_Arr,uint16_t Quene_Len,uint16_t Member_Size);
void Quene_Add_Member(RCS_QUENE* quene,void* member);
void* Quene_Quit_Member(RCS_QUENE* quene);
void Quene_Clear(RCS_QUENE* quene);

/* ---------.h head-------------------------------------*/
#endif