#include "sys.h"  
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//系统时钟初始化	
//包括时钟设置/中断管理/GPIO设置等
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/2
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved
//********************************************************************************
//修改说明
//无
//////////////////////////////////////////////////////////////////////////////////  

//THUMB???????§????±à????
//????????·?·¨??????????±à????WFI  
void WFI_SET(void)
{
	__asm volatile("WFI");	  
}
//??±??ù??????(??????°ü?¨fault??NMI????)
void INTX_DISABLE(void)
{
	__asm volatile("CPSID I");
	__asm volatile("BX LR");	  
}
//?????ù??????
void INTX_ENABLE(void)
{
	__asm volatile("CPSIE I");
	__asm volatile("BX LR");  
}
//?è?????????·
//addr:???????·
void MSR_MSP(u32 addr) 
{
	__asm volatile("MSR MSP, R0"); 			//set Main Stack value
	__asm volatile("BX R14");
}
















