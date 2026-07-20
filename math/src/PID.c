/****************PID����****************/

#include "PID.h"

#include <math.h>

//��ʼ��pid����
void PID_Init(PID *pid,float p,float i,float d,float maxI,float maxOut)
{
	pid->kp=p;
	pid->ki=i;
	pid->kd=d;
	pid->maxIntegral=maxI;
	pid->maxOutput=maxOut;
	pid->deadzone=0;
}

//��ʼ��΢������pid����
void DEPID_Init(DEPID *pid,float p,float i,float d,float maxI,float maxOut,float gama)
{
	pid->kp=p;
	pid->ki=i;
	pid->kd=d;
	pid->maxIntegral=maxI;
	pid->maxOutput=maxOut;
	pid->gama=gama;
}

void PD_Init(PD_Controller *pd,float kp,float kd,float maxTorque)
{
    pd->kp = kp;
    pd->kd = kd;

    pd->deadzone = 0.0f;
    pd->torque_ff = 0.0f;

    pd->maxTorque = maxTorque;

    pd->outputTorque = 0.0f;
}


//����΢������pid����
void PIDRegulation(DEPID *vPID,float reference, float feedback, float differentiation)
{
	//��������
	vPID->lasterror=vPID->error;
	vPID->error=reference-feedback;
	//΢���˲�
	differentiation = vPID->gama * differentiation + (1-vPID->gama) * vPID-> lastPv;
	//����΢��
	vPID->output = differentiation * vPID->kd;
	//�������
	vPID->output+=vPID->error*vPID->kp;
	//�������
	vPID->integral+=vPID->error*vPID->ki;
	LIMIT(vPID->integral,-vPID->maxIntegral,vPID->maxIntegral);//�����޷�
	vPID->output+=vPID->integral;
	//����޷�
	LIMIT(vPID->output,-vPID->maxOutput,vPID->maxOutput);
	//����΢���˲�
	vPID-> lastPv = differentiation;
}

//����pid����
void PID_SingleCalc(PID *pid,float reference,float feedback)
{
	//��������
	pid->lastError=pid->error;
	if(ABS(reference-feedback) < pid->deadzone)//���������������errorֱ����0
		pid->error=0;
	else
		pid->error=reference-feedback;
	//����΢��
	pid->output=(pid->error-pid->lastError)*pid->kd;
	//�������
	pid->output+=pid->error*pid->kp;
	//�������
	pid->integral+=pid->error*pid->ki;
	LIMIT(pid->integral,-pid->maxIntegral,pid->maxIntegral);//�����޷�
	pid->output+=pid->integral;
	//����޷�
	LIMIT(pid->output,-pid->maxOutput,pid->maxOutput);
}

//����pid����
void PID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb)
{
	PID_SingleCalc(&(pid->outer),angleRef,angleFdb);//�����⻷(�ǶȻ�)
	PID_SingleCalc(&(pid->inner),pid->outer.output ,speedFdb);//�����ڻ�(�ٶȻ�)
	pid->output=pid->inner.output;
}

//����΢������pid����		��������̨ TODO�������������
void DEPID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb)
{
	PIDRegulation(&(pid->deOuter),angleRef,angleFdb,-speedFdb);//�����⻷΢������(�ǶȻ�)
	PID_SingleCalc(&(pid->inner),pid->deOuter.output ,speedFdb);//�����ڻ�(�ٶȻ�)
	pid->output=pid->inner.output;
}


void PD_ParallelCalc(PD_Controller *pd,float p_des,float v_des,float p_meas,float v_meas)
{
    float pos_err;
    float vel_err;
    /* λ����� */
    pos_err = p_des - p_meas;
    if (ABS(pos_err) < pd->deadzone)
        pos_err = 0.0f;
    /* �ٶ���� */
    vel_err = v_des - v_meas;
    if (ABS(vel_err) < pd->deadzone)
        vel_err = 0.0f;
    /* ���� PD + ǰ�� */
    pd->outputTorque = pd->kp * pos_err + pd->kd * vel_err + pd->torque_ff;
    /* �����޷� */
    LIMIT(pd->outputTorque,-pd->maxTorque,pd->maxTorque);
}

//���һ��pid����ʷ����
void PID_Clear(PID *pid)
{
	pid->error=0;
	pid->lastError=0;
	pid->integral=0;
	pid->output=0;
}

void DEPID_Clear(DEPID *pid)
{
	pid->error=0;
	pid->lasterror=0;
	pid->integral=0;
	pid->output=0;
	pid->lastPv=0;
}



//�����趨pid����޷�
void PID_SetMaxOutput(PID *pid,float maxOut)
{
	pid->maxOutput=maxOut;
}

//����PID����
void PID_SetDeadzone(PID *pid,float deadzone)
{
	pid->deadzone=deadzone;
}
