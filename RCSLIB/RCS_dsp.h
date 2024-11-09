/**
 @filename:RCS_dsp.h
 @brief:�����׼�ؽ�����ֵ����
 @tips:���������ARM_MATH_CM4�����Զ�ѡ��DSP��ʵ�ָ������㣬��֮��ֱ��ʹ��FPU���и�������
 @tips:��.h��ͨ��inline���庯������ͷ�ļ��ظ����������⣬������𽫸�ͷ�ļ�����rcs.h�У���������Ҫ�÷�����
 @date:2023/10/30
 @author:���Ͽ�
**/
#ifndef RCS_DSP_H_
#define RCS_DSP_H_
#include <math.h>
#include <stdint.h>
#include "cmsis_armclang.h"


/*****************************************************
        @addtogroup:���ó�������
*****************************************************/
#define F32_PI          3.1415926535f 
#define F32_PI_TWICE    6.2831853072f 
#define F32_PI_HALF     1.5707963268f
#define F32_COS_45      0.707106781187f
#define F32_COS_30      0.866025403784f
#define F32_COS_60      0.5f

#define pi 				3.14159265357f        //��
#define DEG2RAD         1.74532925199433e-2f  //�Ƕȱ任�ɻ���     
#define RAD2DEG         5.72957795130823e1f		//���ȱ任�ɽǶ�
#define FLOAT_ZERO      1e-10f                //��������ȵ��ݲ�
#define G               9.8f                  //�������ٶ�
/*****************************************************
        @addtogroup:ʹ��FPU���м򵥼���
*****************************************************/
__INLINE float RCS_F32_Pow(float input)
{
	return (input*input);
}

__INLINE float RCS_F32_Sqrt(float input)
{
	float temp_output; 

	#ifdef ARM_MATH_CM4
	    arm_sqrt_f32(input,&temp_output);
	#else
	    temp_output=sqrtf(input);
	#endif
	
	return temp_output;
}

__INLINE float RCS_F32_Atan2(float y,float x)
{
    if ((y==0)&&(x==0))//�������hard_fault
    {
        return 0;
    }
    else
    {
	    float temp_output;
	    #ifdef ARM_MATH_CM4 
	        arm_atan2_f32(y,x,&temp_output);
	    #else 
	        temp_output=atan2f(y,x);
	    #endif
        return temp_output;
    }
}

__INLINE float RCS_F32_Sin(float input)
{
    #ifdef ARM_MATH_CM4 
	    return arm_sin_f32(input);
	#else 
	    return sinf(input);
	#endif
}

__INLINE float RCS_F32_Cos(float input)
{
    #ifdef ARM_MATH_CM4 
	    return arm_cos_f32(input);
	#else 
	    return cosf(input);
	#endif
}

__INLINE float RCS_F32_Modf_Fpart(float input)
{
    float temp;
    return modff(input,&temp);
}

__INLINE float RCS_F32_Trunc(float input)
{
    return truncf(input);
}

/*****************************************************
        @addtogroup:ʹ����ֵ�ȶ����㷨������ֵ����
*****************************************************/
/**
 @brief:sqrt(a*a+b*b)�����ǲ�����Ϊa��b���������ʧ����
 @todo:��ʱû����cmsis_dsp����дhypot�Ĺ��ܣ���Ȼ�õ�c��׼��hypot
**/
__INLINE float RCS_F32_Hypot(float a,float b)
{
    #ifdef ARM_MATH_CM4
        return hypotf(a,b);
    #else
        return hypotf(a,b);
    #endif
}

/**
 @brief:x*y+z������ֻ����һ������������������
 @todo:CM4ò�Ʋ�֧����Ԫ����ָ��������������Ƿ������Ч
**/
__INLINE float RCS_F32_Multi_Add(float x,float y,float z)
{
    return fmaf(x,y,z);
}

/**
 @brief: ����num/den��������
**/
__INLINE int32_t RCS_F32_Rem_Quot(float num,float den)
{
    int32_t quot_output;
    remquof(num,den,&quot_output);
    return quot_output;
}

/**
 @brief: ����num/den�ĸ�������
**/
__INLINE float RCS_F32_Rem_Remainder(float num,float den)
{
    return remainderf(num,den);
}

/**
 @brief:��-��~+�޵ĽǶȻػ���[0,2*pi),����֤��ֵ�ȶ�
 @param:target_angle,�����ƽǶ�
**/
__INLINE float Get_Loop_Angle_R(float target_angle)
{
	float temp_output=RCS_F32_Rem_Remainder(target_angle,F32_PI_TWICE);
	if (temp_output>=0) return temp_output;
	else                return temp_output+F32_PI_TWICE;
}
/*****************************************************
        @addtogroup:����DSP����
*****************************************************/

/**
 * @name:cumtrapz
 * @brief:���ι�ʽ�����ֵ����
*/
__INLINE void cumtrapz(double* input,double* output,uint16_t len)
{
    double sum=input[0];
    output[0]=0;

    for (int i=1;i<len;i++)
    {
        sum+=input[i];
        output[i]=sum-0.5*(input[0]+input[i]);
    }
}



#endif