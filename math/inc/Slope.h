#ifndef _SLOPE_H_
#define _SLOPE_H_

#include "main.h"

//б�½ṹ��
//斜坡结构体
typedef struct{
    float target; //目标值 //Ŀ��ֵ
    float step; //步进值 //����ֵ
    float value; //当前值 //��ǰֵ
    float deadzone; //死区，若差值小于该值则不进行增减 //����������ֵС�ڸ�ֵ�򲻽�������
}Slope;

void Slope_Init(Slope *slope,float step,float deadzone);
void Slope_SetTarget(Slope *slope,float target);
void Slope_SetStep(Slope *slope,float step);
float Slope_NextVal(Slope *slope);
float Slope_GetVal(Slope *slope);

#endif
