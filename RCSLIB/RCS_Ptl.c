/**
 * @filename:RCS_Ptl.c
 * @brief:ʵ�ֶ��ֳ������ͨ��Э��
 * @layers:ISO������·��
 * @contribute:CYK-Dot @2024-4-6 ʵ�ּ򵥶���ͷβЭ��
 * ----------------------------------------------
 * @changelog:CYK-Dot @2024-5-14 ʵ�����ֱ䳤ͷβЭ��(��ȷ��/����ȷ��),��ͳһ��·��ĺ�����ʽ
*/
#include "RCS_Ptl.h"

/**
 * @name:RCS_Rcv_EasyProtocol
 * @brief:�򵥶���ͷβЭ�����
 * @param:input_byte  һ������һ���ֽ�
 * @param:temp_arr    ����м���������
 * @param:output_flag ���Э��״̬��״̬�ı���ָ��
 * @param:output_arr  ������ս��������
 * @param:protocol    ʹ�õ�Э��ָ��
*/
void Protocol_Rcv_Easy(uint8_t input_byte,  uint8_t* temp_arr,
                        uint8_t* output_flag,uint8_t* output_arr,
                        RCS_Easy_Ptl* protocol)
{
    //--------�ֲ���������------------------------------
    static volatile int rcv_number=0;
    volatile int i;
    //--------Э��״̬��--------------------------------
	switch(*output_flag)
	{
		//________�ȴ���ͷ_______________________________
		case 0:
			if (input_byte==protocol->Start_Byte)
			{
				memset(temp_arr,0,sizeof(temp_arr));
				rcv_number=0;
				*output_flag=1;
			}
		break;
		//________��ȡ����_______________________________
		case 1:
			temp_arr[rcv_number]=input_byte;
			rcv_number=rcv_number+1;

			if ((rcv_number==(protocol->Len+1))&&(input_byte==protocol->End_Byte))
			{
				//���
				for(i=0;i<protocol->Len;i++)
				{
					output_arr[i]=temp_arr[i];
				}
				//callback
				protocol->Finish_Callback(output_arr);
				
				//��λ״̬��
				*output_flag=0;
				rcv_number=0;
			}

			if (rcv_number>(protocol->Len+1))
			{
				//��λ״̬��
				*output_flag=0;
				rcv_number=0;
			}
		break;
	}
}
/**
 * @name:RCS_Get_EasyProtocol
 * @brief:�����ݴ���ɼ򵥶���ͷβЭ��
*/
void Protocol_Get_Easy(uint8_t* input_arr,uint8_t* output_arr,RCS_Easy_Ptl* protocol)
{
    int i;
    output_arr[0]=protocol->Start_Byte;
    output_arr[protocol->Len+1]=protocol->End_Byte;
    for(i=0;i<protocol->Len;i++) output_arr[i+1]=input_arr[i];
}

/**
 * @name:Protocol_Init_LenVerify
 * @brief:��ʼ��һ������У��Э��
**/
void Protocol_Init_LenVerify(RCS_LenVerify_Ptl* ptl,uint8_t start_byte,uint8_t len,uint8_t* buffer_arr)
{
    ptl->Start_Byte=start_byte;
    ptl->Len=len;
    ptl->RcvBufr_Arr=buffer_arr;
    ptl->RcvBufr_Pnt=0;
    ptl->RcvStat_Fsm=0;
}

/**
 * @name:Protocol_Rcv_LenVerify_IT
 * @brief:���ֽ����������ú���,ʵ�ֳ���У��Э��Ľ���
 * @param:ptl_handler_void  Э������ָ��,���ڳ���У��Э��,��������ΪRCS_LenVerify_Ptl,����ָ��ת��Ϊvoid*����ú�������
 * @param:input_byte        ���յ����ֽ�
 * @param:output_arr        ������ı��Ľ�����µ��������С�����ʧ�ܲ��Ὣ�������е�������0,���Ǳ��ֲ���
 * @reval:uint8_t           ���ص�ǰ�Ľ������,��������Ƿ���ɡ����RCS_Ptl.h��RCS_Common_Ptl_Log�Ķ���
 * @tips:�ڴ���ptl_handler_void֮ǰ,��Ҫ�ȶ�������ʼ��
**/
uint8_t Protocol_Rcv_LenVerify_IT(void* ptl_handler_void,uint8_t input_byte,uint8_t* output_arr)
{
    RCS_LenVerify_Ptl* ptl=(RCS_LenVerify_Ptl*)ptl_handler_void;

    switch(ptl->RcvStat_Fsm)
    {
        //---------------------�ȴ���ͷ-------------------------------------
        //�ٰ�ͷ�ֽڳ��֣���ʼ��������
        //�ڰ�ͷ�ֽ�δ���֣������κ����
        case 0:
            if (input_byte==ptl->Start_Byte)
            {
                ptl->RcvStat_Fsm=1;            //ת����һ��״̬
                ptl->RcvBufr_Pnt=0;            //��0��ʼ�������
                ptl->RcvBufr_Arr[1+ptl->Len]=ptl->Start_Byte;//��Len+1���������ڴ��У���
                return RCV_IDLE;
            }
            break;

        //----------------��������+ʵʱ�ۼ�У��-----------------------------
        //�ٵ����յ��ֽ����ﵽԤ�ڣ�����У����ȷ���򿽱����ݣ������¿�ʼ��
        //�ڵ����յ��ֽ����ﵽԤ�ڣ�����У�鲻��ȷ�������¿�ʼ��
        //�۽��յ��ֽ���δ��Ԥ�ڣ���������ա�
        case 1:
            ptl->RcvBufr_Arr[ptl->RcvBufr_Pnt]=input_byte;
            ptl->RcvBufr_Arr[1+ptl->Len]+=input_byte;
            ptl->RcvBufr_Pnt++;

            if (ptl->RcvBufr_Pnt == 1+ptl->Len)
            {
                //�����
                if (ptl->RcvBufr_Arr[1+ptl->Len] == (uint8_t)(ptl->RcvBufr_Arr[ptl->Len]+ptl->RcvBufr_Arr[ptl->Len]))
                {
                    memcpy(output_arr,ptl->RcvBufr_Arr,ptl->Len);
                    ptl->RcvStat_Fsm=0;
                    return RCV_CPLT;

                }
                //�����
                else
                {
                    ptl->RcvStat_Fsm=0;
                    return RCV_FAIL;
                }
            }
            else
            {
                //�����
                return RCV_PROC;
            }
            break;
    }
}

/**
 * @name:Protocol_Get_LenVerify
 * @brief:�������͵����鰴�ճ���У��Э����
 * @param:ptl_handler_void  Э������ָ��,���ڳ���У��Э��,��������ΪRCS_LenVerify_Ptl,����ָ��ת��Ϊvoid*����ú�������
 * @param:output_arr        ����õ����ݽ�����µ���������
 * @param:input_arr         ��Ҫ���������
 * @param:len               ��Ҫ��������ݵĳ��ȣ����ڳ���У��Э�飬�����������Ч�ģ����Բ���
 * @tips:�ڴ���ptl_handler_void֮ǰ,��Ҫ�ȶ�������ʼ��
 * @reval:��װ�õ��ֽ�����
*/
uint8_t Protocol_Get_LenVerify(void* ptl_handler_void,uint8_t* output_arr,uint8_t* input_arr,uint8_t len)
{
    RCS_LenVerify_Ptl* ptl=(RCS_LenVerify_Ptl*)ptl_handler_void;
    uint16_t i;

    output_arr[0]=ptl->Start_Byte;
    output_arr[ptl->Len+1]=ptl->Start_Byte;
    for(i=0;i<ptl->Len;i++)
    {
        output_arr[1+i]=input_arr[i];
        output_arr[ptl->Len+1]+=input_arr[i];
    }
    return ptl->Len+2;
}



/**
 * @name:Protocol_Rcv_RCSBleNet_IT
 * @brief:���ֽ����������ú���,ʵ�ֱ䳤ͷβЭ��Ľ���
 * @param:ptl_handler_void  Э������ָ��,���ڱ䳤ͷβЭ��,��������ΪRCS_Comm_Ptl_Handler,����ָ��ת��Ϊvoid*����ú�������
 * @param:input_byte        ���յ����ֽ�
 * @param:output_arr        ������ı��Ľ�����µ��������С�����ʧ�ܲ��Ὣ�������е�������0,���Ǳ��ֲ���
 * @reval:uint8_t           ���ص�ǰ�Ľ������,��������Ƿ���ɡ����RCS_Ptl.h��RCS_Common_Ptl_Log�Ķ���
 * @tips:�ڴ���ptl_handler_void֮ǰ,��Ҫ�ȶ�������ʼ��
*/
uint8_t Protocol_Rcv_RCSBleNet_IT(void* ptl_handler_void,uint8_t input_byte,uint8_t* output_arr)
{
	//�ֲ�����
	uint32_t total_crc;
	uint8_t i;
	//��������
	RCS_Comm_Ptl_Handler*  ptl_handler_p=(RCS_Comm_Ptl_Handler*)ptl_handler_void;
	RCS_Common_Ptl*        ptl_param=&(ptl_handler_p->PTL_Param);
	RCS_Common_Ptl_Buffer* ptl_buf=&(ptl_handler_p->PTL_Rcv_Buffer);
	RCS_Common_Ptl_Log*    ptl_log=&(ptl_handler_p->PTL_Log);

	//״̬��
	switch(ptl_buf->status_flag)
	{
		//-------------------�ȴ���ͷ--------------------------------
		case 0:
			if (input_byte==ptl_param->Start_Byte)
			{
				//���ݸ�λ
				ptl_buf->rcv_pointer=0;
				ptl_buf->crc_h=0;
				ptl_buf->crc_l=0;
				memset(ptl_buf->rcv_buffer,0,sizeof(ptl_buf->rcv_buffer));
				//���յ����ֽںͰ�ͷһ�£�����ת
				ptl_buf->status_flag=2;
			}
			//δ��ɽ���
			*ptl_log=RCV_PROC;
			return RCV_PROC;
            break;

		//-----------------����֡����---------------------------------
		case 2:
			//���ݽ���(buffer[0]=֡����)
			ptl_buf->rcv_buffer[ptl_buf->rcv_pointer]=input_byte;
			ptl_buf->rcv_pkg_type=input_byte;
			ptl_buf->rcv_pointer++;
			//ֱ����ת
			ptl_buf->status_flag=3;
			//δ��ɽ���
			*ptl_log=RCV_PROC;
			return RCV_PROC;
            break;

		//------------------���ձ��ĳ���--------------------------------
		case 3:
			//���ݽ���(buffer[1]=���ĳ���)
			ptl_buf->rcv_buffer[ptl_buf->rcv_pointer]=input_byte;
			ptl_buf->rcv_pkg_len=input_byte;
			ptl_buf->rcv_pointer++;
			//ֱ�ӽ�����һ��case
			ptl_buf->status_flag=4;
			//δ��ɽ���
			*ptl_log=RCV_PROC;
			return RCV_PROC;
		break;

		//-------------------��������-----------------------------------
		case 4:
			//���ݽ���(buffer[2~2+N]=����)
			ptl_buf->rcv_buffer[ptl_buf->rcv_pointer]=input_byte;
			ptl_buf->rcv_pointer++;

			//���յ����ֽ�������ָ������������У�鷽ʽ�Ĳ�ͬ������ת��case��׼������У������
			if (ptl_buf->rcv_pointer == (2 + ptl_buf->rcv_pkg_len) )
			{
				if (ptl_buf->rcv_pkg_type==Pkg_Type_Verify)
					ptl_buf->status_flag=10;
				else if (ptl_buf->rcv_pkg_type==Pkg_Type_Direct)
					ptl_buf->status_flag=20;
				else if (ptl_buf->rcv_pkg_type==Pkg_Type_FullVerify)
					ptl_buf->status_flag=30;//todo
			}

			//���յ�һ���ֽ�,���ν���ʧ��,ǿ�����¿�ʼ����
			if ((ptl_buf->rcv_pkg_type==Pkg_Type_Direct)||(ptl_buf->rcv_pkg_type==Pkg_Type_FullVerify))
			{
				//�����յ������ֽ�+��ʼ�ֽ�,���������ų�������ʶ��Ϊ��β�����
				if ((ptl_buf->rcv_buffer[ptl_buf->rcv_pointer-1]==ptl_param->Start_Byte)&&(ptl_buf->rcv_buffer[ptl_buf->rcv_pointer-2]==ptl_param->End_Byte))
				{
					//���ݸ�λ
					ptl_buf->rcv_pointer=0;
					ptl_buf->crc_h=0;
					ptl_buf->crc_l=0;
					memset(ptl_buf->rcv_buffer,0,sizeof(ptl_buf->rcv_buffer));
					//���¿�ʼ����
					ptl_buf->status_flag=2;
					//����ʧ��
					*ptl_log=RCV_FAIL;
					return RCV_FAIL;
				}
			}

			//δ��ɽ���
			*ptl_log=RCV_PROC;
			return RCV_PROC;
		break;

		//-------------------����У��-----------------------------------------------
		case 10:
			//���ݽ���(buffer[3+N]=�߰�λУ��ֵ)
			ptl_buf->rcv_buffer[ptl_buf->rcv_pointer]=input_byte;
			ptl_buf->rcv_pointer++;
			//������һ��case
			ptl_buf->status_flag=11;
			//δ��ɽ���
			*ptl_log=RCV_PROC;
			return RCV_PROC;
		break;
		case 11:
			//���ݽ���(buffer[4+N]=�Ͱ�λУ��ֵ)
			ptl_buf->rcv_buffer[ptl_buf->rcv_pointer]=input_byte;
			ptl_buf->rcv_pointer++;
			//����У���
			total_crc=ptl_param->Start_Byte;
			for(i=1;i<=1+ptl_buf->rcv_pkg_len;i++) total_crc+=ptl_buf->rcv_buffer[i];
			ptl_buf->crc_h=(total_crc&0xff00)>>8;
			ptl_buf->crc_l= total_crc&0xff;
			//������һ��case
			if ((ptl_buf->crc_h ==ptl_buf->rcv_buffer[2+ptl_buf->rcv_pkg_len])
			  &&(ptl_buf->crc_l ==ptl_buf->rcv_buffer[3+ptl_buf->rcv_pkg_len]))
			{
				for(i=0;i<ptl_buf->rcv_pkg_len;i++) output_arr[i]=ptl_buf->rcv_buffer[i+2];
				ptl_buf->status_flag=0;
				*ptl_log=RCV_CPLT;
				return RCV_CPLT;
			}
			else
			{
				ptl_buf->status_flag=0;
				*ptl_log=RCV_FAIL;
				return RCV_FAIL;
			}
		break;

		//------------------ֱͨУ��--------------------------------------
		case 20:
			//���ݽ���(buffer[3+N]=��β)
			ptl_buf->rcv_buffer[ptl_buf->rcv_pointer]=input_byte;
			ptl_buf->rcv_pointer++;
			//У��
			if (input_byte==ptl_param->End_Byte)
			{
				for(i=0;i<ptl_buf->rcv_pkg_len;i++) output_arr[i]=ptl_buf->rcv_buffer[i+2];
				ptl_buf->status_flag=0;
				*ptl_log=RCV_CPLT;
				return RCV_CPLT;
			}
			else
			{
				ptl_buf->status_flag=0;
				*ptl_log=RCV_FAIL;
				return RCV_FAIL;
			}
		break;
		//------------------����У��--------------------------------------
		case 30:
			//���ݽ���(buffer[3+N]=�߰�λУ��ֵ)
			ptl_buf->rcv_buffer[ptl_buf->rcv_pointer]=input_byte;
			ptl_buf->rcv_pointer++;
			//������һ��case
			ptl_buf->status_flag=31;
			//δ��ɽ���
			*ptl_log=RCV_PROC;
			return RCV_PROC;
		break;
		case 31:
			//���ݽ���(buffer[4+N]=�Ͱ�λУ��ֵ)
			ptl_buf->rcv_buffer[ptl_buf->rcv_pointer]=input_byte;
			ptl_buf->rcv_pointer++;
			//����У���
			total_crc=ptl_param->Start_Byte;
			for(i=0;i<=1+ptl_buf->rcv_pkg_len;i++) total_crc+=ptl_buf->rcv_buffer[i];
			ptl_buf->crc_h=(total_crc&0xff00)>>8;
			ptl_buf->crc_l= total_crc&0xff;
			//������һ��case
			if ((ptl_buf->crc_h ==ptl_buf->rcv_buffer[2+ptl_buf->rcv_pkg_len])
			  &&(ptl_buf->crc_l ==ptl_buf->rcv_buffer[3+ptl_buf->rcv_pkg_len]))
			{
				for(i=0;i<ptl_buf->rcv_pkg_len;i++) output_arr[i]=ptl_buf->rcv_buffer[i+2];
				ptl_buf->status_flag=32;
				*ptl_log=RCV_PROC;
				return RCV_PROC;
			}
			else
			{
				ptl_buf->status_flag=0;
				*ptl_log=RCV_FAIL;
				return RCV_FAIL;
			}
		break;
		case 32:
			//���ݽ���(buffer[3+N]=��β)
			ptl_buf->rcv_buffer[ptl_buf->rcv_pointer]=input_byte;
			ptl_buf->rcv_pointer++;
			//У��
			if (input_byte==ptl_param->End_Byte)
			{
				for(i=0;i<ptl_buf->rcv_pkg_len;i++) output_arr[i]=ptl_buf->rcv_buffer[i+2];
				ptl_buf->status_flag=0;
				*ptl_log=RCV_CPLT;
				return RCV_CPLT;
			}
			else
			{
				ptl_buf->status_flag=0;
				*ptl_log=RCV_FAIL;
				return RCV_FAIL;
			}
		break;

		//------------------����״̬--------------------------------------
		default:
			*ptl_log=RCV_ERR;
			return RCV_ERR;
		break;
	}
}
/**
 * @name:Protocol_Get_RCSBleNet
 * @brief:�������͵����鰴�ձ䳤ͷβЭ����
 * @param:ptl_handler_void  Э������ָ��,���ڱ䳤ͷβЭ��,��������ΪRCS_Comm_Ptl_Handler,����ָ��ת��Ϊvoid*����ú�������
 * @param:output_arr        ����õ����ݽ�����µ���������
 * @param:input_arr         ��Ҫ���������
 * @param:len               ��Ҫ��������ݵĳ���
 * @usage:�䳤ͷβЭ��������֡��ʽ,�������ָ�ʽ�����õ�������˵��͸����,ѡ����ȡ����ʵ������,������Ҫ׼ȷ������Ҫ����
 * @tips:�ڴ���ptl_handler_void֮ǰ,��Ҫ�ȶ�������ʼ��
 * @reval:��װ�õ��ֽ�����
*/
uint8_t Protocol_Get_RCSBleNet(void* ptl_handler_void,uint8_t* output_arr,uint8_t* input_arr,uint8_t len)
{
	//�ֲ�����
	int i;
	uint32_t crc_total=0;
	//һ��ָ�����
	RCS_Common_Ptl* ptl_param=(RCS_Common_Ptl*)ptl_handler_void;
	//��������
	switch(ptl_param->Pkg_Type)
	{
		case Pkg_Type_Direct:
			output_arr[0]=ptl_param->Start_Byte;
			output_arr[1]=Pkg_Type_Direct;
			output_arr[2]=len;
			for(i=0;i<len;i++) output_arr[3+i]=input_arr[i];
			output_arr[3+len]=ptl_param->End_Byte;
			return 4+len;
		break;

		case Pkg_Type_Verify:
			output_arr[0]=ptl_param->Start_Byte; crc_total+=output_arr[0];
			output_arr[1]=Pkg_Type_Verify; crc_total+=output_arr[1];
			output_arr[2]=len;             crc_total+=output_arr[2];
			for(i=0;i<len;i++)
			{
				output_arr[3+i]=input_arr[i];
				crc_total+=input_arr[i];
			}
			output_arr[3+len]=(crc_total & 0xff00) >>8;
			output_arr[4+len]= crc_total & 0xff;
			return 5+len;
		break;

		case Pkg_Type_FullVerify:
			output_arr[0]=ptl_param->Start_Byte;crc_total+=output_arr[0];
			output_arr[1]=Pkg_Type_FullVerify;  crc_total+=output_arr[1];
			output_arr[2]=len;                  crc_total+=output_arr[2];
			for(i=0;i<len;i++)
			{
				output_arr[3+i]=input_arr[i];
				crc_total+=input_arr[i];
			}
			output_arr[3+len]=(crc_total & 0xff00) >>8;
			output_arr[4+len]= crc_total & 0xff;
			output_arr[5+len]= ptl_param->End_Byte;
			return 6+len;
		break;
	}
}

/**
 * @name:Protocol_Init_RCSBleNet
 * @brief:��ʼ��һ���䳤ͷβЭ��(�������ڻ���Э��)
 * @param:ptl         Э�����ĵ�ַ
 * @param:start_byte  ֡����ʼ�ֽ�,��Ҫ�������ݵķֲ����,ѡ��һ������ܸ������ݵ�ֵ
 * @param:end_byte    ֡�Ľ����ֽ�,��Ҫ�������ݵķֲ����,ѡ��һ������ܸ������ݵ�ֵ
 * @param:pkg_type    ֡�İ�װ��ʽ,���@ref:RCS_Common_Ptl_PkgType
 * @param:buffer_arr  Ϊ��Э��Ľ���,��Ҫռ��һ�����ڴ�ռ�,�ⲿ�ֿռ���Ҫ�ֶ�ָ��,������Ĵ�С��ӦС����Ч���ݵ���С����+6
 * @reval:�Ƿ�ɹ���ɳ�ʼ��
*/
uint8_t Protocol_Init_RCSBleNet(RCS_Comm_Ptl_Handler* ptl_Handler_p,
                                uint8_t start_byte,uint8_t end_byte,RCS_Common_Ptl_PkgType pkg_type,
								                uint8_t* buffer_arr)
{
	ptl_Handler_p->PTL_Param.Start_Byte=start_byte;
	ptl_Handler_p->PTL_Param.End_Byte=end_byte;
	ptl_Handler_p->PTL_Param.Pkg_Type=pkg_type;

	ptl_Handler_p->PTL_Rcv_Buffer.rcv_buffer=buffer_arr;
	ptl_Handler_p->PTL_Rcv_Buffer.crc_h=0;
	ptl_Handler_p->PTL_Rcv_Buffer.crc_l=0;
	ptl_Handler_p->PTL_Rcv_Buffer.rcv_pkg_len=0;
	ptl_Handler_p->PTL_Rcv_Buffer.rcv_pkg_type=0;
	ptl_Handler_p->PTL_Rcv_Buffer.rcv_pointer=0;
	ptl_Handler_p->PTL_Rcv_Buffer.status_flag=0;

	ptl_Handler_p->PTL_Log=RCV_IDLE;

	return 1;
}