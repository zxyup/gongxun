/*@filename: RCS_Filter.c
 *@author     ��־ΰ       
 *@brief:    �˲�����ѧ���㺯��
 *@date: 2020-11-20
 *@author     ���˹�     
 *@brief:    �������˲�
 *@date: 2023-5-5
*/


#include "RCS_Filter.h"

/************��̬ȫ�ֱ���*************/
static float filter_x = 0;	//Kalman�˲��������			
static float filter_y = 0;
static float xy_cov = 0;		//����Ԥ��ֵЭ����
static float filter_z = 0;	//��ֵ�˲����ƫ����

/*************��̬����****************/
static float Average_Filter_Z(float gps_z,float laser_z,float gps_weight);

/**************************************/


/**
	@name: Kalman_Filter
	@brief: kalman�˲��㷨���˲�Ŀ��X��Y���꣩
	@explain:�ں���������ͼ������꣬��δʹ��
**/
void Kalman_Filter(void)
{
	
	float rcs_pos_x = 0;			//ȫ����λ����
	float rcs_pos_y = 0;	
	
	float rcs_delta_x;				//��������
	float rcs_delta_y;
	
	float temp_filter_x = 0;	//Ԥ��ֵ���꣨Ԥ��ֵ��
	float temp_filter_y = 0;
	
	float kalman_gen = 0;			//����������
	float temp_cov = 0;				//��ʱԤ��ֵЭ����
	
	static float rcs_last_x;	//��һʱ��ȫ����λ��������
	static float rcs_last_y;
	
	static float laser_x = 0;	//�������꣨����ֵ��
	static float laser_y = 0;
	
	rcs_pos_x = Get_GPS_X();	//��ȡȫ����λ����
	rcs_pos_y = Get_GPS_Y();

	filter_z = Get_GPS_Z();//0.9f * filter_z + 0.1f * Average_Filter_Z(RCS_Get_Z(),Get_Laser_Z(),0.7);		//�ͺ��ֵ�˲�ƫ����
	Laser_Get_Pos(&laser_x,&laser_y,filter_z);				//��ȡ���ⶨλ����	

	rcs_delta_x = rcs_pos_x - rcs_last_x;	//��ȡ��������
	rcs_delta_y = rcs_pos_y - rcs_last_y;	
	
	temp_filter_x = filter_x + rcs_delta_x;	//Ԥ��ֵ
	temp_filter_y = filter_y + rcs_delta_y;
	
	temp_cov = xy_cov + GPS_COV;						//Ԥ��ֵЭ����
	kalman_gen = temp_cov / (temp_cov + LASER_COV);	//���¿���������
	
	
	filter_x = temp_filter_x + kalman_gen * (laser_x - temp_filter_x);	//�������Ż�Ԥ��ֵ
	filter_y = temp_filter_y + kalman_gen * (laser_y - temp_filter_y);

	rcs_last_x = rcs_pos_x;			//��¼��һʱ��ȫ����λ����
	rcs_last_y = rcs_pos_y;
	
	xy_cov = (1.0f - kalman_gen) * temp_cov;	//�������Ż�Ԥ��Э����
}

/**
	@name: Get_KalmanFilter_X
	@brief: ��ȡ�������˲����x����
	@return: �˲����X����ֵ
**/
float Get_KalmanFilter_X(void)
{
	return Get_GPS_X();//filter_x;
}

/**
	@name: Get_KalmanFilter_Y
	@brief: ��ȡ�������˲����y����
	@return: �˲����Y����ֵ
**/
float Get_KalmanFilter_Y(void)
{
	return Get_GPS_Y();;//filter_y;
}

/**
	@name: Get_Ave_Z
	@brief: ��ȡ��ֵ�˲����ƫ����
	@return: �˲����Z����ֵ
**/
float Get_Ave_Z(void)
{
	return Get_GPS_Z();
}

/**
	@name: KalmanFilter_x
	@brief:�������˲��㷨������Ӧ�ó�������p q r����
	@param:p���Э�����ʼֵ
	@param:q����������Ӱ������ƽ���̶ȣ�qԽСԽƽ��������Ӱ��ԽС��qԽ��̬��ӦԽ�á�QֵԽ��
         ����Խ���β���ֵ��Qֵ����󣬴���ֻ�ò���ֵ��QֵԽС������Խ����ģ��Ԥ��ֵ��QֵΪ0������ֻ��ģ��Ԥ��ֵ
  @param:r�۲�����������������Խ��rԽС�������˲����������ʵ�����ߵ�����̶ȣ�rԽСԽ����ʵ��ֵ��rԽ��Խ����Ԥ��ֵ��r���޴�ֻ����Ԥ��ֵ��rԽ��̬��ӦԽ��
**/
float KalmanFilter_x(float inData)
{
  static float prevData = 0;                                 //��һ������
//  static float p = 1.0f, q = 0.001f, r = 0.05f, kGain = 0;      //���Ʋ���
//	static float p = 1.0f, q = 0.5f, r = 0.01f, kGain = 0; 
	static float p = 1.0f, q = 0.001f, r = 0.05f, kGain = 0;
  p = p + q;
  kGain = p / (p + r);          //���㿨��������
  inData = prevData + (kGain * (inData - prevData));    //׼ȷλ��=��һ��Ԥ��λ��+K*��ʵ������-��һ��Ԥ��λ�ã�
  p = (1 - kGain) * p;           //���²�������
  prevData = inData;
  return inData;                 //���ع���ֵ
}

/**
	@name: KalmanFilter_y
	@brief:�������˲��㷨������Ӧ�ó�������p q r����
	@param:���Э�����ʼֵ
	@param:q����������Ӱ������ƽ���̶ȣ�qԽСԽƽ����qԽ��̬��ӦԽ�á�QֵԽ��
         ����Խ���β���ֵ��Qֵ����󣬴���ֻ�ò���ֵ��QֵԽС������Խ����ģ��Ԥ��ֵ��QֵΪ0������ֻ��ģ��Ԥ��ֵ
  @param:r�۲������������˲����������ʵ�����ߵ�����̶ȣ�rԽСԽ����ʵ��ֵ��rԽ��Խ����Ԥ��ֵ��r���޴�ֻ����Ԥ��ֵ��rԽ��̬��ӦԽ��
**/
float KalmanFilter_y(float inData)
{
  static float prevData = 0;                                 //��һ������
	static float p = 1.0f, q = 0.001f, r = 0.05f, kGain = 0;
  p = p + q;
  kGain = p / (p + r);                                      //���㿨��������
  inData = prevData + (kGain * (inData - prevData));      //���㱾���˲�����ֵ
  p = (1 - kGain) * p;                                      //���²�������
  prevData = inData;
  return inData;                                             //���ع���ֵ
}


/**
	@name: Get_Median
	@brief:������λ���˲�
	@param:float *data          		  ����
	@param:int num            			�����С
	@return	��λ��
**/
float Get_Median(float data[],int num)
{
	float buf[num];
	float temp;
	
	for(int i =0;i<num;i++)
		buf[i] = data[i];
	
	for(int i =0;i<=num-2;i++)
	{
		for(int j = 0;j<=num-2;j++)
		{
			if(buf[j] > buf[j+1])
			{
				temp = buf[j];
				buf[j] = buf[j+1];
				buf[j+1] = temp;
			}
		}
	}
	
	if(num%2 == 1)
		return buf[(num-1)/2];
	else
		return ((buf[num/2-1] + buf[num/2])/2.0f);
}


/**
	@name: Get_Average
	@brief:����ƽ��ֵ�˲�
	@param:float *data          		  ����
	@param:int num            			�����С
	@return	ƽ��ֵ
**/
float Get_Average(float data[],int num)
{
	float sum = 0;
	for(int i=0;i<num;i++)
		sum += data[i];
	return sum/((float)num);
}


/**
	@name: Average_Filter_Z
	@brief:ƫ���Ǽ�Ȩƽ���˲�
	@param:float gps_z          		  ȫ����λƫ����
	@param:float laser_z            	���ⶨλƫ����
	@param:float gps_weight           ȫ����λ�ı��أ�0~1��
	@return	ƽ��ֵ
**/
static float Average_Filter_Z(float gps_z,float laser_z,float gps_weight)
{
	return (gps_weight * gps_z + (1.0f - gps_weight) * laser_z);
}

/**
	@name: Recursive_Filter
	@brief:��Ȩ���ƾ�ֵ�˲�
	@param:float *data          		  ����
	@param:int num            			�����С
	@param:int begin_index          ����ֵ��������λ��
	@return	�˲�ֵ
**/
float Recursive_Filter(float data[],int num,int begin_index)
{
	int index;
	float sum = 0;
	float sum_i = 0;
	index = begin_index;

	for(int i=num; i>0 ; i--)
	{
		sum += data[index] * i;
		sum_i += i;
		index --;
		if(index <0)
			index = num-1;
	}
	
	sum /= sum_i;
	return sum;
}


/**
	@name: Recursive_Filter
	@brief:��ֵ�˲�
	@param:float *data          		  ����
	@param:int num            			�����С
	@param:int begin_index          ����ֵ��������λ��
	@return	�˲�ֵ
**/
float Median_Ave_Filter(float data[],int num)
{
	int max_num,min_num;
	float sum;
	max_num = data[0];
	min_num = data[0];
	sum = data[0];
	for(int i =1;i<num;i++)
	{
		sum += data[i];
		if(data[i] > max_num)
			max_num = data[i];
		if(data[i] < min_num)
			min_num = data[i];
	}
	sum = (sum - max_num - min_num)/(num-2);
	return sum;
}


/**
	@name: Count_Delay
	@brief:����ת�ַ���
	@param:int num           			����
	@return	char str[].�ַ���ָ��
**/
void Int2Str(int num,char str[])
{
	char temp[16];
	int index,len;
	int temp_num = num;
	int dir_flag = 0;
	
	len = sizeof(str);
	if(temp_num == 0)
	{
		str[0] = '0';
		for(index = 1;index<len-1;index++)
			str[index] = ' ';
		str[len-1] = '\0';
	}
	else
	{
		if(temp_num <0)
		{
			str[0] = '-';
			dir_flag = 1;
			temp_num = -temp_num;
		}
		for(index = 0;temp_num != 0;index++)
		{
			temp[index] = '0'+ temp_num % 10;
			temp_num = temp_num/10;
		}
		temp_num = index-1;
		for(index = 0;index<=temp_num;index++)
			str[index+dir_flag] = temp[temp_num - index];
		for(index = temp_num+1;index<len-1;index++)
			str[index] = ' ';
		str[len-1] = '\0';
	}	
}

/**
	@name: Float2Str
	@brief:������ת�ַ���
	@param:float 		num           		������
	@param:uint8_t  length          	�ַ�������
	@param:uint8_t  decimal_length    ������С��λ��
	@return	char str[].�ַ���ָ��
**/
void Float2Str(float num, char str[], uint8_t length, uint8_t decimal_length)
{
	int temp_num;
	uint8_t tmp[10];
	int i = 0, j = 0;

	temp_num = (int)(num * pow(10, decimal_length));
	if (temp_num < 0)
	{
		str[j++] = '-';
		temp_num = -temp_num;
	}
	do
	{
		tmp[i++] = temp_num % 10 + '0';
		temp_num /= 10;
	}
	while (i < decimal_length);
	tmp[i++] = '.';
	do
	{
		tmp[i++] = temp_num % 10 + '0';
		temp_num /= 10;
	}
	while (temp_num);
	while (i)
		str[j++] = tmp[--i];
	while (j < length)
		str[j++] = ' ';
	str[j] = '\0';
}
