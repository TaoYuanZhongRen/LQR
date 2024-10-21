#pragma once
#include <Eigen/Dense>

enum ModelType
{
	CONTROL_MODEL,
	TRAJECTORY_PLANNING_MODEL,
	FORMULATION_MODEL
};

struct BaseModel
{
	ModelType type;
	const char* name;
};

struct ControlModelParam
{
    // ����
    double desired[4] = {};         //����λ��
    double position[3] = {};        //��ǰλ��
    double velocity[3] = {};       //��ǰ�ٶ�
    double euler[3] = {};
    double angularSpeed[3] = {};
    // ���
    double pwm[4] = {};
    double thrust;
    double angle_cmd[3];
    // �µĸ������룬��������Ϊ�˱����ƻ��ڴ沼��
    double desired_velocity[3] = {};       //�����ٶ�
    double des_a[3] = {};                  //�������ٶ� 
};

struct ControlModel {
	BaseModel base;
	float step;
	void* (*new_instance)();
	void (*del_instance)(void*);
	void (*update)(void*, ControlModelParam*);
};

