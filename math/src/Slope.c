#include "Slope.h"

#include <math.h>

//��ʼ��б�²���
//初始化斜坡参数
void Slope_Init(Slope *slope,float step,float deadzone)
{
    slope->target=0;
    slope->step=step;
    slope->value=0;
    slope->deadzone=deadzone;
}

//�趨б��Ŀ��
//设定斜坡目标
void Slope_SetTarget(Slope *slope,float target)
{
    slope->target=target;
}

//�趨б�²���
//设定斜坡步长
void Slope_SetStep(Slope *slope,float step)
{
    slope->step=step;
}

//������һ��б��ֵ������slope->value�����ظ�ֵ
//计算下一个斜坡值，更新slope->value并返回该值
float Slope_NextVal(Slope *slope)
{
    float error=slope->value-slope->target; //当前值与目标值的差值 //��ǰֵ��Ŀ��ֵ�Ĳ�ֵ

    if(fabsf(error)<slope->deadzone)//���������������ǰֵ�������仯
	    return slope->value;

    if(fabsf(error)<slope->step)//�����㲽����ǰֱֵ����ΪĿ��ֵ
	    slope->value=slope->target;
    else if(error<0)
	    slope->value+=slope->step;
    else if(error>0)
	    slope->value-=slope->step;
    return slope->value;
}

//��ȡб�µ�ǰֵ
//获取斜坡当前值
float Slope_GetVal(Slope *slope)
{
    return slope->value;
}
