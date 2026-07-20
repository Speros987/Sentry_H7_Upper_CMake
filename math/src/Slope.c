#include "Slope.h"

#include <math.h>

//��ʼ��б�²���
void Slope_Init(Slope *slope,float step,float deadzone)
{
	slope->target=0;
	slope->step=step;
	slope->value=0;
	slope->deadzone=deadzone;
}

//�趨б��Ŀ��
void Slope_SetTarget(Slope *slope,float target)
{
	slope->target=target;
}

//�趨б�²���
void Slope_SetStep(Slope *slope,float step)
{
	slope->step=step;
}

//������һ��б��ֵ������slope->value�����ظ�ֵ
float Slope_NextVal(Slope *slope)
{
	float error=slope->value-slope->target;//��ǰֵ��Ŀ��ֵ�Ĳ�ֵ
	
	if(ABS(error)<slope->deadzone)//���������������ǰֵ�������仯
		return slope->value;
	
	if(ABS(error)<slope->step)//�����㲽����ǰֱֵ����ΪĿ��ֵ
		slope->value=slope->target;
	else if(error<0)
		slope->value+=slope->step;
	else if(error>0)
		slope->value-=slope->step;
	return slope->value;
}

//��ȡб�µ�ǰֵ
float Slope_GetVal(Slope *slope)
{
	return slope->value;
}
