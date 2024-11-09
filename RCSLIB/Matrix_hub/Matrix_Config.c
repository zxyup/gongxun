/**
 * @name:Matrix_Config.c
 * @brief:ΪMatrix_Hub�ṩӲ���������������
 * @changlog:2024-3-18 CYK-Dot ������ֲ
*/

/* =============ͷ�ļ�========================================*/
#include "matrix_os_config.h"

/* ==============ȫ�ֱ���=====================================*/
RCS_MMU_T matrix_mmu;

/* ==============������������=================================*/

/**
 * @name:OS_Malloc
 * @brief:�ڴ˴�����ʵ���ڴ����ĺ������Թ�Matrix_Hubʹ��
*/
void* OS_Malloc(uint16_t size_byte)
{
    #ifdef MALLOC_DISABLE
        return RCS_MMU_Malloc(&matrix_mmu,size_byte);
    #else
        return malloc(size_byte);
    #endif
}

/**
 * @name:OS_Free
 * @brief:�ڴ˴�����ʵ���ڴ��ͷŵĺ������Թ�Matrix_Hubʹ��
*/
void OS_Free(void* mem_p)
{
    #ifdef MALLOC_DISABLE
        RCS_MMU_Free(&matrix_mmu,mem_p);
    #else
        free(mem_p);
    #endif
}

/**
 * @name:OS_Err_Collector
 * @brief:�ڴ˴�ʵ���ռ���־�ĺ������Թ�Matrix_Hubչʾ�����Ϣ
*/
void OS_Err_Collector(const char* format, ...)
{
    RCS_Shell_Logs(format);
}

/**
 * @name:OS_Commander
 * @brief:�ڴ˴�ʵ���ն˿��Ƶĺ������Թ�Matrix_Hub�����ն���ʾ
*/
void OS_Commander(const char* format, ...)
{
	
}

/**
 * @name:OS_Matrix_Init
 * @brief:�����������������Ҫ��ʼ������ʹ�ã����ڴ˴�ʵ�ֳ�ʼ��
*/
void OS_MatrixHub_Init(void)
{
    
}