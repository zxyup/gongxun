/* =======ͷ�ļ�================================*/
#include "GPS_DSP.h"
#include "RCS_dsp.h"

/* =======˽��ȫ�ֱ���===========================*/
RCS_MUTEX_T Encoder_Mutex_X,Encoder_Mutex_Y;//��ȡ�����������Ļ�����
RCS_Oth_Encoder_T Encoder_Value;
RCS_Pos_T Last_Base_Pos;
RCS_Spd_T Last_Base_Spd;

/* =======�����ӿ�===============================*/
/**
 * @name:gspd_Kin_spd
 * @brief:�����ٶ�->�����ٶ�
*/
void gspd_Kin_spd(RCS_Spd_T* Gyro_Spd,double Chassis_Angle,RCS_Spd_T* reval) 
{
    double Sin_Angle=sin(Chassis_Angle);
    double Cos_Angle=cos(Chassis_Angle);

    double temp_vcpx = (B1_MULT_C0*Gyro_Spd->spdw
                       -B0_MULT_C1*Gyro_Spd->spdw
                       +B0*Gyro_Spd->spdy
                       -B1*Gyro_Spd->spdx) 
                       / (A1_MULT_B0 - A0_MULT_B1);

    double temp_vcpy = -(A1_MULT_C0*Gyro_Spd->spdw 
                       - A0_MULT_C1*Gyro_Spd->spdw 
                       + A0*Gyro_Spd->spdy 
                       - A1*Gyro_Spd->spdx) 
                       / (A1_MULT_B0 - A0_MULT_B1);

    reval->spdx = (temp_vcpx * Cos_Angle + temp_vcpy * Sin_Angle);
    reval->spdy = (temp_vcpy * Cos_Angle - temp_vcpx * Sin_Angle);
    reval->spdw = Gyro_Spd->spdw;

}

/**
 * @name:spd_inv_gspd
 * @brief:�����ٶ�->�����ٶ�
*/
void spd_inv_gspd(RCS_Spd_T* Base_Spd,double Chassis_Angle,RCS_Spd_T* reval) 
{
    RCS_Spd_T tmp_spd;
		int i;
    double Cos_Angle=cos(Chassis_Angle);
    double Sin_Angle=sin(Chassis_Angle);

    // ���ص�����
    tmp_spd.spdx = Base_Spd->spdx * Cos_Angle - Base_Spd->spdy * Sin_Angle;
    tmp_spd.spdy = Base_Spd->spdx * Sin_Angle + Base_Spd->spdy * Cos_Angle;
        
    // ���嵽����
    reval->spdx = COS_CENTER_Z * tmp_spd.spdx + SIN_CENTER_Z * tmp_spd.spdy - sin(Wheel_Ang_X - Center_Offset_Z) * Base_Spd->spdw;
    reval->spdy = COS_CENTER_Z * tmp_spd.spdy - SIN_CENTER_Z * tmp_spd.spdx + cos(Center_Offset_Z - Wheel_Ang_Y) * Base_Spd->spdw;
    reval->spdw = Base_Spd->spdw;
}

/**
 * @name:gpos_diff_gspd
 * @brief:��ֻ�ȡ�����ٶ�
 * @tips:��Ҫ�ϸ�֤����ʱ�䣬����ɸú����������뻥����
*/
void gpos_diff_gspd(RCS_Spd_T* reval)
 {
	RCS_Pend_Mutex(&Encoder_Mutex_X);
	RCS_Pend_Mutex(&Encoder_Mutex_Y);
	reval->spdx                =Encoder_Value.X_Count - Encoder_Value.Last_X_Count;
	Encoder_Value.Last_X_Count =Encoder_Value.X_Count;

	reval->spdy                 =Encoder_Value.Y_Count - Encoder_Value.Last_Y_Count;
	Encoder_Value.Last_Y_Count  =Encoder_Value.Y_Count;
	RCS_Rel_Mutex(&Encoder_Mutex_Y);
	RCS_Rel_Mutex(&Encoder_Mutex_X);
 }

/**
 * @name:Encoder_Update_X
 * @brief:���ж��и���X����������ֵ
 * @tips:�ж�����������һ����룬��˽����жϻ������Ƿ�ռ��
*/
void Encoder_Update_X(void)
 {
    if (RCS_Pend_Mutex(&Encoder_Mutex_X))
    {
        Encoder_Value.X_Count=Encoder_Value.X_Count+Encoder_Value.Buffered_X_Count+1;
        Encoder_Value.Buffered_X_Count=0;
        RCS_Rel_Mutex(&Encoder_Mutex_X);
    }
    else
    {
        Encoder_Value.Buffered_X_Count++;
    }
 }

 /**
 * @name:Encoder_Update_Y
 * @brief:���ж��и���Y����������ֵ
 * @tips:�ж�����������һ����룬��˽����жϻ������Ƿ�ռ��
*/
void Encoder_Update_Y(void)
 {
    if (RCS_Pend_Mutex(&Encoder_Mutex_Y))
    {
        Encoder_Value.Y_Count=Encoder_Value.Y_Count+Encoder_Value.Buffered_Y_Count+1;
        Encoder_Value.Buffered_Y_Count=0;
        RCS_Rel_Mutex(&Encoder_Mutex_Y);
    }
    else
    {
        Encoder_Value.Buffered_Y_Count++;
    }
 }

/**
 * @name:bspd_cumtrupz_bpos
 * @brief:ͨ�����λ��֣��ó������ٶ�->��������
*/
void bspd_cumtrupz_bpos(RCS_Spd_T* spd,RCS_Pos_T* start_pos,RCS_Pos_T* output)
{
    output->posx=Last_Base_Pos.posx+0.5*(spd->spdx+Last_Base_Spd.spdx);
    output->posy=Last_Base_Pos.posy+0.5*(spd->spdy+Last_Base_Spd.spdy);
    output->posw=Last_Base_Pos.posw+0.5*(spd->spdw+Last_Base_Spd.spdw);

    Last_Base_Spd.spdx=spd->spdx;
    Last_Base_Spd.spdy=spd->spdy;
    Last_Base_Spd.spdw=spd->spdw;
}

