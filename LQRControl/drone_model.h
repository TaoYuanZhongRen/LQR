#pragma once
#include <Eigen/Dense>

enum ModelType
{
	CONTROL_MODEL,
	TRAJECTORY_PLANNING_MODEL,
	FORMULATION_MODEL,
	ASTAR_CONTROL_MODEL
};

struct BaseModel
{
	ModelType type;
	const char* name;
};

struct ControlModelParam
{ 
	// 输入
	double desired[4] = {};         //期望位置
	double des_a[3] = {};
	double des_v[3] = {};           //期望速度
	double position[3] = {};        //当前位置
	double velocity[3] = {};       //当前速度
	double euler[3] = {};
	double angularSpeed[3] = {};
	// 输出
	double pwm[4] = {};
	double thrust;
	double angle_cmd[3];
};




struct LQRControlModel
{
	BaseModel base;
	float step;
	void* (*new_instance)();
	void (*del_instance)(void*);
	void (*update)(void*, ControlModelParam*);
};

