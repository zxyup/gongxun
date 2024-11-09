/**
 * @name:RCS_Scara
 * @brief:��Ŷ����ɶȻ�е�۽����㷨
 * @usage:��ѭ�ȳ�ʼ����ʹ�õ��߼���ʹ��SCARA_PARAM���������е��ʵ�������ɻ��ڶ��е�۵Ŀ���

 * @contribute:2024-1-13 CYK-dot ���δ���
 * -----------------------------------------------------
 * @changelog:2024-4-14 CYK-dot  ���ƽӿ�
**/

/* ͷ�ļ� ======================================================*/
#include "RCS_Scara.h"
#include "RCS_dsp.h"

/* ˽��ȫ�ֱ��� ================================================*/
volatile SCARA_AXIS output_scara;

/* ��̬�������� ================================================*/


/* Scara�ӿں������� ===========================================*/

/**
 *@name: Scara_Param_Init
 *@brief:��ʼ��һ��SCARA��е��ʵ��
 *@param:Main_Len/End_Len            ��С�۳���
 *@param:Main_Slowdown/End_Slowdown  ��С�ۼ��ٱ�,��������ϵΪ�����ٱ�,��֮Ϊ��
 *@param:Main_StartPos_Deg           ��С�۵ĳ�ʼ�Ƕ�,����������ϵ����
 *@tips:����Ƕ�Ӧ���ڱ�����֮ǰ�ʹ����,�˺���ֻ��ע��е�۶���չ�ֳ��ĽǶ�
**/
SCARA_PARAM Scara_Param_Init(SCARA_FLOAT_T Main_Len,         SCARA_FLOAT_T End_Len,
                             SCARA_FLOAT_T Main_Slowdown,    SCARA_FLOAT_T End_Slowdown,
                             SCARA_FLOAT_T Main_StartPos_Deg,SCARA_FLOAT_T End_StartPos_Deg)
{
	SCARA_PARAM reval;
	reval.Main_Len=Main_Len;
	reval.End_Len=End_Len;
	reval.Pinv_Epsilon=SCARA_PINV_COMMON;

	reval.Main_SlDn=Main_Slowdown;
	reval.End_SlDn=End_Slowdown;
	reval.Start_Pos.joint_angle_main=Main_StartPos_Deg*DEG2RAD;
	reval.Start_Pos.joint_angle_end=End_StartPos_Deg*DEG2RAD;
	return reval;
}


/**
 *@name: Scara_Spd_Ctrl
 *@brief:��SCARA��е�۽����ٶȿ���
 *@param:spd_x/spd_y                         Ŀ���ٶ�
 *@param:Main_MotorPos_Rad/End_MotorPos_Rad  ��С�۵�������ĽǶ�(������)
 *@param:scara                               ��Ҫ�����ƵĻ�е��ʵ��
 *@reval:scara���������Ŀ���ٶ�
**/
SCARA_AXIS Scara_Spd_Ctrl(SCARA_FLOAT_T spd_x,            SCARA_FLOAT_T spd_y,
                          SCARA_FLOAT_T Main_MotorPos_Rad,SCARA_FLOAT_T End_MotorPos_Rad,
                          SCARA_PARAM* scara)
{
		SCARA_AXIS Now_Pos_Scara;
		SCARA_AXIS Now_Pos_Motor;
		DESCARTES_AXIS Input_Spd;
		SCARA_AXIS     Output_Spd_s,Output_Spd_m;

		Now_Pos_Motor.joint_angle_main=Main_MotorPos_Rad;
		Now_Pos_Motor.joint_angle_end=End_MotorPos_Rad;
		Input_Spd.x=spd_x;
		Input_Spd.y=spd_y;

		//������������٣���ȡ��е�۽Ƕ�
    Now_Pos_Scara=Motor_Trans_Scara_Pos(&Now_Pos_Motor,scara);

    //΢�����˶�ѧ����ȡ��е���ٶ�
    Input_Spd.x=spd_x*100.0;
    Input_Spd.y=spd_y*100.0;
    Output_Spd_s=Scara_Jcb_PInvKin(&Input_Spd,&Now_Pos_Scara,scara);

    //��������ٷ���������ȡ����ٶ�
    Output_Spd_m=Scara_Trans_Motor_Spd(&Output_Spd_s,scara);

    return Output_Spd_m;
}


/**
 *@name: Scara_Pos_Ctrl
 *@brief:��SCARA��е�۽���λ�ÿ���
 *@param:desc_pos_x/desc_pos_y               Ŀ������
 *@param:Main_MotorPos_Rad/End_MotorPos_Rad  ��С�۵�������ĽǶ�(������)
 *@param:scara                               ��Ҫ�����ƵĻ�е��ʵ��
 *@reval:scara���������λ��
**/
SCARA_AXIS Scara_Pos_Ctrl(SCARA_FLOAT_T desc_pos_x,SCARA_FLOAT_T desc_pos_y,
                          SCARA_FLOAT_T Main_MotorPos_Rad,SCARA_FLOAT_T End_MotorPos_Rad,
                          SCARA_PARAM* scara)
{
		volatile SCARA_AXIS Now_Pos_Motor,Now_Pos_Scara;
		volatile SCARA_AXIS Output_Pos_Scara,Output_Pos_Motor;
		volatile DESCARTES_AXIS Input_Pos_Scara;
		//��ȡ�����ǰ�Ƕ�
		Now_Pos_Motor.joint_angle_main=Main_MotorPos_Rad;
		Now_Pos_Motor.joint_angle_end=End_MotorPos_Rad;

		//������������٣���ȡ��е�۵�ǰ�Ƕ�
		Now_Pos_Scara=Motor_Trans_Scara_Pos(&Now_Pos_Motor,scara);

		//���˶�ѧ��ȡ��е��Ŀ��Ƕ�
		Input_Pos_Scara.x=desc_pos_x;
		Input_Pos_Scara.y=desc_pos_y;
		Output_Pos_Scara=Scara_InvKin(&Input_Pos_Scara,&Now_Pos_Scara,scara);

		 //��������ٷ���������ȡ���Ŀ��Ƕ�(rad)
		Output_Pos_Motor=Scara_Trans_Motor_Pos(&Output_Pos_Scara,scara);

		//debug test
		output_scara=Output_Pos_Motor;

		return Output_Pos_Motor;
}

/**
 *@name: Scara_Pos_Ctrl_Direct
 *@brief:ֱ�ӿ���scara����ĽǶ�
 *@param:Main_Pos/End_Pos       Ŀ��Ƕ�(������)
 *@param:scara                  ��Ҫ�����ƵĻ�е��ʵ��
 *@reval:scara���������λ��
**/
SCARA_AXIS Scara_Pos_Ctrl_Direct(SCARA_FLOAT_T Main_Pos,SCARA_FLOAT_T End_Pos,SCARA_PARAM* scara)
{
	volatile SCARA_AXIS Target_Pos_Scara;
	volatile SCARA_AXIS Target_Pos_Motor;

	Target_Pos_Scara.joint_angle_main=Main_Pos;
	Target_Pos_Scara.joint_angle_end=End_Pos;
	Target_Pos_Motor=Scara_Trans_Motor_Pos(&Target_Pos_Scara,scara);
	output_scara=Target_Pos_Motor;
	return Target_Pos_Motor;
}

/**
 *@name: Scara_Get_Scara_Pos
 *@brief:ֱ�ӻ�ȡscara��е�۵ĽǶ�
 *@param:Main_MotorPos_Rad      ����Ƕ�(������)
 *@param:scara                  ��е��ʵ��
**/
SCARA_AXIS Scara_Get_Scara_Pos(SCARA_FLOAT_T Main_MotorPos_Rad,SCARA_FLOAT_T End_MotorPos_Rad,SCARA_PARAM* scara)
{
	SCARA_AXIS Now_Pos_Motor,Now_Pos_Scara;
	//��ȡ�����ǰ�Ƕ�
	Now_Pos_Motor.joint_angle_main=Main_MotorPos_Rad;
	Now_Pos_Motor.joint_angle_end=End_MotorPos_Rad;

	//������������٣���ȡ��е�۵�ǰ�Ƕ�
	Now_Pos_Scara=Motor_Trans_Scara_Pos(&Now_Pos_Motor,scara);

	return Now_Pos_Scara;
}

/**
 *@name: Scara_Get_Desc_Pos
 *@brief:��ȡ��е��ĩ�˵ĵѿ�������ϵλ��
 *@param:Main_MotorPos_Rad      ����Ƕ�(������)
 *@param:scara                  ��е��ʵ��
**/
DESCARTES_AXIS Scara_Get_Desc_Pos(SCARA_FLOAT_T Main_MotorPos_Rad,SCARA_FLOAT_T End_MotorPos_Rad,SCARA_PARAM* scara)
{
	SCARA_AXIS Now_Pos_Motor,Now_Pos_Scara;
	DESCARTES_AXIS Now_Pos_Scara_Desc;
	//��ȡ�����ǰ�Ƕ�
	Now_Pos_Motor.joint_angle_main=Main_MotorPos_Rad;
	Now_Pos_Motor.joint_angle_end=End_MotorPos_Rad;

	//������������٣���ȡ��е�۵�ǰ�Ƕ�
	Now_Pos_Scara=Motor_Trans_Scara_Pos(&Now_Pos_Motor,scara);

	Now_Pos_Scara_Desc=Scara_Kin(&Now_Pos_Scara,scara);
	return Now_Pos_Scara_Desc;
}




/* ��̬�������� ======================================*/

/**
 * @name:Scara_Kin
 * @brief:���˶�ѧ�����ȡ��ǰ��ĩ��λ��
 * @param:SCARA_AXIS* jonit_vector �ؽڵ������Ƕ�
 * @param:SCARA_PARAM* scara       ��е�۵Ļ�������
 * @reval:ĩ����Ի��������ڵĵѿ�������ϵ
*/
DESCARTES_AXIS Scara_Kin(SCARA_AXIS* jonit_vector,SCARA_PARAM* scara)
{
	//�������Ż�����
	DESCARTES_AXIS reval;
	SCARA_AXIS jonit_angle=*jonit_vector;
	//��x=l2*cos(a1+a2)+l1*cos(a1)
	reval.x= scara->End_Len   *SCARA_COS(jonit_angle.joint_angle_end+jonit_angle.joint_angle_main)
	        +scara->Main_Len  *SCARA_COS(jonit_angle.joint_angle_main);
	//��y=l2*sin(a1+a2)+l1*sin(a1)
	reval.y= scara->End_Len   *SCARA_SIN(jonit_angle.joint_angle_end+jonit_angle.joint_angle_main)
	        +scara->Main_Len  *SCARA_SIN(jonit_angle.joint_angle_main);

	return reval;
}
/**
 * @name:Scara_InvKin
 * @brief:���˶�ѧ����õ��ؽڽǶ�
 * @param:target_vector            Ŀ��λ������ڳ���ѿ�������ϵ������
 * @param:SCARA_AXIS* now_axis     ��ǰ�Ļ�е�۽Ƕ�(����)
 * @param:SCARA_PARAM* scara       ��е�۵Ļ�������
 * @reval:���뵱ǰλ�������һ�����˶�ѧ��
*/
SCARA_AXIS Scara_InvKin(DESCARTES_AXIS* target_vector,SCARA_AXIS* now_axis,SCARA_PARAM* scara)
{
	SCARA_AXIS reval,reval_1,reval_2;
	DESCARTES_AXIS target_desc=*(target_vector);

	SCARA_FLOAT_T e_end_1,e_main_1,e_end_2,e_main_2;
	SCARA_FLOAT_T tar1_end,tar1_main,tar2_end,tar2_main;

	//��е��Ŀ��Ƕ�
	reval_1.joint_angle_end=acos(((target_desc.x)*(target_desc.x)+(target_desc.y)*(target_desc.y)-(scara->Main_Len)*(scara->Main_Len)-(scara->End_Len)*(scara->End_Len))/(2*scara->Main_Len*scara->End_Len));
	reval_1.joint_angle_main=atan2(target_desc.y,target_desc.x)-atan2(scara->End_Len*sin(reval_1.joint_angle_end),scara->Main_Len+scara->End_Len*cos(reval_1.joint_angle_end));

	reval_2.joint_angle_end=-reval_1.joint_angle_end;
	reval_2.joint_angle_main=atan2(target_desc.y,target_desc.x)-atan2(scara->End_Len*sin(reval_2.joint_angle_end),scara->Main_Len+scara->End_Len*cos(reval_2.joint_angle_end));

	//���Ŀ��Ƕ�
	tar1_end=-(reval_1.joint_angle_end)*RAD2DEG *36.0;
	tar1_main=-(reval_1.joint_angle_main)*RAD2DEG *36.0/20.0*33.0;

	tar2_end=-(reval_2.joint_angle_end)*RAD2DEG *36.0;
	tar2_main=-(reval_2.joint_angle_main)*RAD2DEG *36.0/20.0*33.0;


	if((tar1_end-(now_axis->joint_angle_end))<=0)
	{
		e_end_1=now_axis->joint_angle_end-tar1_end;
	}
	else
	{
		e_end_1=tar1_end-now_axis->joint_angle_end;
	}

	if((tar1_main-(now_axis->joint_angle_main))<=0)
	{
		e_main_1=now_axis->joint_angle_main-tar1_main;
	}
	else
	{
		e_main_1=tar1_main-now_axis->joint_angle_main;
	}

	if((tar2_end-(now_axis->joint_angle_end))<=0)
	{
		e_end_2=now_axis->joint_angle_end-tar2_end;
	}
	else
	{
		e_end_2=tar2_end-now_axis->joint_angle_end;
	}

	if((tar2_main-(now_axis->joint_angle_main))<=0)
	{
		e_main_2=now_axis->joint_angle_main-tar2_main;
	}
	else
	{
		e_main_2=tar2_main-now_axis->joint_angle_main;
	}

	//��1
	if((e_end_1+e_main_1)<=(e_end_2+e_main_2))
	{
		reval=reval_1;
	}
	if((e_end_1+e_main_1)>(e_end_2+e_main_2))
	{
		reval=reval_2;
	}

	return reval_2;
}
/**
 * @name:Scara_Jcb_Kin
 * @brief:���ݹؽ��ٶȻ�ȡĩ���ٶ�
 * @param:SCARA_AXIS* now_axis     ��ǰ�Ļ�е�۽Ƕ�(����)
 * @param:SCARA_PARAM* scara       ��е�۵Ļ�������
 * @todo
*/
DESCARTES_AXIS Scara_Jcb_Kin(SCARA_AXIS* jonit_vector,SCARA_AXIS* now_axis,SCARA_PARAM* scara)
{

}
/**
 * @name:Scara_Jcb_PInvKin
 * @brief:����Ŀ���ٶȻ�ȡ�ؽ��ٶ�(�Ÿ���α�����+���򻯾���)
 * @param:DESCARTES_AXIS* target_speed Ŀ��ѿ����ٶ�
 * @param:SCARA_AXIS* now_axis         ��ǰ�Ļ�е�۽Ƕ�(����)
 * @param:SCARA_PARAM* scara       ��е�۵Ļ�������
 * @attention:ƽ����ʱ110us,����2.2%�Ŀ���֡��
*/
SCARA_AXIS Scara_Jcb_PInvKin(DESCARTES_AXIS* target_speed,SCARA_AXIS* now_axis,SCARA_PARAM* scara)
{
	//�ֲ�����
	SCARA_AXIS reval_spd;
	SCARA_FLOAT_T vt1_num;
	SCARA_FLOAT_T vt2_num;
	SCARA_FLOAT_T vtx_den;

	//Ԥ�ȼ���
	SCARA_FLOAT_T lj         = scara->Pinv_Epsilon;
	SCARA_FLOAT_T L1_Sint1   = scara->Main_Len *SCARA_SIN(now_axis->joint_angle_main);
	SCARA_FLOAT_T L1_Cost1   = scara->Main_Len *SCARA_COS(now_axis->joint_angle_main);
	SCARA_FLOAT_T L2_Sin_t1t2= scara->End_Len  *SCARA_SIN(now_axis->joint_angle_end+now_axis->joint_angle_main);
	SCARA_FLOAT_T L2_Cos_t1t2= scara->End_Len  *SCARA_COS(now_axis->joint_angle_end+now_axis->joint_angle_main);
	SCARA_FLOAT_T L1L2_Sin_t12t2=scara->Main_Len*scara->End_Len*SCARA_SIN(now_axis->joint_angle_main+2*now_axis->joint_angle_end);
	SCARA_FLOAT_T L1L2_Cos_t12t2=scara->Main_Len*scara->End_Len*SCARA_COS(now_axis->joint_angle_main+2*now_axis->joint_angle_end);
	SCARA_FLOAT_T L2L2      = scara->End_Len*scara->End_Len;
	SCARA_FLOAT_T L1L1      = scara->Main_Len*scara->Main_Len;

	//vt1 num
	vt1_num= ( scara->End_Len*L1L2_Sin_t12t2 - L2L2*L1_Sint1 - 2*lj*L2_Sin_t1t2 - 2*lj*L1_Sint1)*target_speed->x
            +(-scara->End_Len*L1L2_Cos_t12t2 + L2L2*L1_Cost1 + 2*lj*L2_Cos_t1t2 + 2*lj*L1_Cost1)*target_speed->y;

	//vt2 num
	vt2_num=-target_speed->x*(2*lj*L2_Sin_t1t2 + L1L1*L2_Sin_t1t2 - scara->End_Len*L1L1*SCARA_SIN(now_axis->joint_angle_main - now_axis->joint_angle_end) - L2L2*L1_Sint1 +  scara->End_Len*L1L2_Sin_t12t2)
            +target_speed->y*(2*lj*L2_Cos_t1t2 + L1L1*L2_Cos_t1t2 - scara->End_Len*L1L1*SCARA_COS(now_axis->joint_angle_main - now_axis->joint_angle_end) - L2L2*L1_Cost1 +  scara->End_Len*L1L2_Cos_t12t2);

	//vt1&vt2 den
	vtx_den= (2*(L1L1*lj + 2*L2L2*lj + L1L1*L2L2 + lj*lj - L1L1*L2L2*SCARA_COS(now_axis->joint_angle_end)*SCARA_COS(now_axis->joint_angle_end) + 2*scara->End_Len*scara->Main_Len*lj*SCARA_COS(now_axis->joint_angle_end)));

	reval_spd.joint_angle_main=(vt1_num/vtx_den);
	reval_spd.joint_angle_end =(vt2_num/vtx_den);

	return reval_spd;
}
/**
 * @name:Scara_Jcb_InvKin
 * @brief:����Ŀ���ٶȻ�ȡ�ؽ��ٶ�(�Ÿ��������)
 * @tips:������ʹ�ã���������������޷����������
*/
SCARA_AXIS Scara_Jcb_InvKin(DESCARTES_AXIS* target_speed,SCARA_AXIS* now_axis,SCARA_PARAM* scara)
{
	SCARA_AXIS reval_spd;

	 if (fabs(now_axis->joint_angle_end) < FLOAT_ZERO)
	 {
	 	if (now_axis->joint_angle_end<0) now_axis->joint_angle_end-=FLOAT_ZERO;
	 	else                             now_axis->joint_angle_end+=FLOAT_ZERO;
	 }

	 reval_spd.joint_angle_main=  (target_speed->x*  (SCARA_COS(now_axis->joint_angle_main + now_axis->joint_angle_end)                  /SCARA_SIN(now_axis->joint_angle_end))
	                              +target_speed->y*  (SCARA_SIN(now_axis->joint_angle_main + now_axis->joint_angle_end)                  /SCARA_SIN(now_axis->joint_angle_end))
	 														 )
	                             /(scara->Main_Len);

	 reval_spd.joint_angle_end = -(scara->End_Len *target_speed->x* (SCARA_COS(now_axis->joint_angle_main + now_axis->joint_angle_end)/SCARA_SIN(now_axis->joint_angle_end))
	                              +scara->End_Len *target_speed->y* (SCARA_SIN(now_axis->joint_angle_main + now_axis->joint_angle_end)/SCARA_SIN(now_axis->joint_angle_end))
	                              +scara->Main_Len*target_speed->x* (SCARA_COS(now_axis->joint_angle_main)                            /SCARA_SIN(now_axis->joint_angle_end))
	                              +scara->Main_Len*target_speed->y* (SCARA_SIN(now_axis->joint_angle_main)                            /SCARA_SIN(now_axis->joint_angle_end))
	 															)
	                             /(scara->Main_Len*scara->End_Len);
}
/**
 * @name:Scara_Trans_Motor_Pos
 * @brief:���ݹؽڵ�ǰ�ĽǶȣ���ȡ��ǰ�ĵ���Ƕ�
 * @param:SCARA_AXIS* jonit_vector    �ؽڵľ��ԽǶ�(��x���᷽��Ϊ���0�Ƚ�,�Դ�����ⷽ��ΪС��0�Ƚ�)
 * @param:SCARA_AXIS* start_jonit_pos �ؽڵľ�����ʼ�Ƕ�(��x���᷽��Ϊ���0�Ƚ�,�Դ�����ⷽ��ΪС��0�Ƚ�)
 * @param:SCARA_PARAM* scara          ��е�۵Ļ������ٲ���
 * @reval:SCARA_AXIS                  �������ʵ�Ƕ�
*/
SCARA_AXIS Scara_Trans_Motor_Pos(SCARA_AXIS* jonit_vector,SCARA_PARAM* scara)
{
	SCARA_AXIS reval;
	reval.joint_angle_main=(jonit_vector->joint_angle_main - (scara->Start_Pos).joint_angle_main) / scara->Main_SlDn;
	reval.joint_angle_end =(jonit_vector->joint_angle_end  - (scara->Start_Pos).joint_angle_end)  / scara->End_SlDn;
	return reval;
}

/**
 * @name:Motor_Trans_Scara_Pos
 * @brief:���ݵ����ǰ�ĽǶȣ���ȡ��ǰ�ĹؽڽǶ�
 * @param:SCARA_AXIS* motor_vector    �������ʵ�Ƕ�
 * @param:SCARA_AXIS* start_jonit_pos �ؽڵľ�����ʼ�Ƕ�(��x���᷽��Ϊ���0�Ƚ�,�Դ�����ⷽ��ΪС��0�Ƚ�)
 * @param:SCARA_PARAM* scara          ��е�۵Ļ������ٲ���
 * @reval:SCARA_AXIS                  �ؽڵľ��ԽǶ�(��x���᷽��Ϊ���0�Ƚ�,�Դ�����ⷽ��ΪС��0�Ƚ�)
*/
SCARA_AXIS Motor_Trans_Scara_Pos(SCARA_AXIS* motor_vector,SCARA_PARAM* scara)
{
	SCARA_AXIS reval;
	reval.joint_angle_main=motor_vector->joint_angle_main * scara->Main_SlDn;
	reval.joint_angle_end=motor_vector->joint_angle_end * scara->End_SlDn;

	reval.joint_angle_main+=(scara->Start_Pos).joint_angle_main;
	reval.joint_angle_end+=(scara->Start_Pos).joint_angle_end;
	return reval;
}

/**
 * @name:Scara_Trans_Motor_Spd
 * @brief:���ݹؽڵ�ǰ���ٶȣ���ȡ��ǰ�ĵ���ٶ�
*/
SCARA_AXIS Scara_Trans_Motor_Spd(SCARA_AXIS* jonit_vector,SCARA_PARAM* scara)
{
	SCARA_AXIS reval;
	reval.joint_angle_main=jonit_vector->joint_angle_main / scara->Main_SlDn;
	reval.joint_angle_end =jonit_vector->joint_angle_end  / scara->End_SlDn;
	return reval;
}

/**
 * @name:Motor_Trans_Scara_Spd
 * @brief:���ݵ����ǰ���ٶȣ���ȡ��ǰ�Ĺؽ��ٶ�
 * @param:SCARA_AXIS* motor_vector �����ǰ���ٶ�
 * @param:SCARA_PARAM* scara       ��е�۵Ļ������ٲ���
 * @reval:SCARA_AXIS               �ؽڵ�ǰ���ٶ�
*/
SCARA_AXIS Motor_Trans_Scara_Spd(SCARA_AXIS* motor_vector,SCARA_PARAM* scara)
{
	SCARA_AXIS reval;
	reval.joint_angle_main=motor_vector->joint_angle_main * scara->Main_SlDn;
	reval.joint_angle_end=motor_vector->joint_angle_end * scara->End_SlDn;

	return reval;
}

/**
 * @name:Judge_Scara_Jcb_InvKin
 * @brief:�жϻ�е���ڵ�ǰλ�˵������
 * @tips:���ڶ����ɶȻ�е��,С����ȫ��չ(�Ƕ�=0)ʱ�޷��ṩx�ٶȣ���ʱʧȥһ�����ɶ�
 * @todo:ʹ�ö����ͺ����˶�����
*/
SCARA_FLOAT_T Judge_Scara_Jcb_InvKin(DESCARTES_AXIS* target_speed,SCARA_AXIS* now_axis,SCARA_PARAM* scara)
{

}

/**
 * @name:Judge_Scara_Jcb_InvKin
 * @brief:�ж����˶�ѧλ�ý��Ƿ����
 * @tips:���ڶ����ɶȻ�е��,�޷��ƶ���Բ�������
*/
uint8_t Judge_Scara_InvKin(DESCARTES_AXIS* target_pos,SCARA_AXIS* now_axis,SCARA_PARAM* scara)
{
	if (((target_pos->x*target_pos->x+target_pos->y*target_pos->y)
	    -(scara->Main_Len+scara->End_Len)*(scara->Main_Len+scara->End_Len)
	    )
		>= -FLOAT_ZERO
	   )
		return 0;
	else
		return 1;

}



