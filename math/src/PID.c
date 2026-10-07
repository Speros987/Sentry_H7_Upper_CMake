/****************PID运算****************/
/****************PID����****************/

#include "PID.h"

#include <math.h>

//��ʼ��pid����
//初始化pid参数
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
//初始化微分先行pid参数
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
//单级微分先行pid计算
void PIDRegulation(DEPID *vPID,float reference, float feedback, float differentiation)
{
	//��������
    //更新数据
    vPID->lasterror=vPID->error;
    vPID->error=reference-feedback;
	//΢���˲�
    //微分滤波
    differentiation = vPID->gama * differentiation + (1-vPID->gama) * vPID-> lastPv;
	//����΢��
    //计算微分
    vPID->output = differentiation * vPID->kd;
	//�������
    //计算比例
    vPID->output+=vPID->error*vPID->kp;
	//�������
    //计算积分
    vPID->integral+=vPID->error*vPID->ki;
    LIMIT(vPID->integral,-vPID->maxIntegral,vPID->maxIntegral); //积分限幅 //�����޷�
    vPID->output+=vPID->integral;
	//����޷�
    //输出限幅
    LIMIT(vPID->output,-vPID->maxOutput,vPID->maxOutput);
	//����΢���˲�
    //更新微分滤波
    vPID-> lastPv = differentiation;
}

//����pid����
//单级pid计算
void PID_SingleCalc(PID *pid,float reference,float feedback)
{
	//��������
    pid->lastError=pid->error;
    if(fabsf(reference-feedback) < pid->deadzone)//���������������errorֱ����0
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
//串级pid计算
void PID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb)
{
    PID_SingleCalc(&(pid->outer),angleRef,angleFdb); //计算外环(角度环) //�����⻷(�ǶȻ�)
    PID_SingleCalc(&(pid->inner),pid->outer.output ,speedFdb); //计算内环(速度环) //�����ڻ�(�ٶȻ�)
    pid->output=pid->inner.output;
}

//����΢������pid����		��������̨ TODO�������������
//串级微分先行pid计算		适用于云台 TODO拨弹等其他电机
void DEPID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb)
{
    PIDRegulation(&(pid->deOuter),angleRef,angleFdb,-speedFdb); //计算外环微分先行(角度环) //�����⻷΢������(�ǶȻ�)
    PID_SingleCalc(&(pid->inner),pid->deOuter.output ,speedFdb);//�����ڻ�(�ٶȻ�)
    pid->output=pid->inner.output;
}


void PD_ParallelCalc(PD_Controller *pd,float p_des,float v_des,float p_meas,float v_meas)
{
    float pos_err;
    float vel_err;
    /* λ����� */
    /* 位置误差 */
    pos_err = p_des - p_meas;
    if (fabsf(pos_err) < pd->deadzone)
        pos_err = 0.0f;
    /* �ٶ���� */
    /* 速度误差 */
    vel_err = v_des - v_meas;
    if (fabsf(vel_err) < pd->deadzone)
        vel_err = 0.0f;
    /* ���� PD + ǰ�� */
    /* 并级 PD + 前馈 */
    pd->outputTorque = pd->kp * pos_err + pd->kd * vel_err + pd->torque_ff;
    /* �����޷� */
    /* 力矩限幅 */
    LIMIT(pd->outputTorque,-pd->maxTorque,pd->maxTorque);
}

//���һ��pid����ʷ����
//清空一个pid的历史数据
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
//重新设定pid输出限幅
void PID_SetMaxOutput(PID *pid,float maxOut)
{
    pid->maxOutput=maxOut;
}

//����PID����
//设置PID死区
void PID_SetDeadzone(PID *pid,float deadzone)
{
    pid->deadzone=deadzone;
}
