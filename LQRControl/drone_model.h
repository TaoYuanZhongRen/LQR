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
    // 输入
    double desired[4] = {};         //期望位置
    double position[3] = {};        //当前位置
    double velocity[3] = {};       //当前速度
    double euler[3] = {};
    double angularSpeed[3] = {};
    // 输出
    double pwm[4] = {};
    double thrust;
    double angle_cmd[3];
    // 新的附加输入，放在这是为了避免破坏内存布局
    double desired_velocity[3] = {};       //期望速度
    double des_a[3] = {};                  //期望加速度 
};

struct ControlModel {
	BaseModel base;
	float step;
	void* (*new_instance)();
	void (*del_instance)(void*);
	void (*update)(void*, ControlModelParam*);
};

