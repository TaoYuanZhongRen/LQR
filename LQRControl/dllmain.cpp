#include "drone_model.h"
#include "LQRControl.h"

void* new_instance()
{
	auto* obj = new LQRCalculator{};

	return obj;
}
void update(void* ptr, ControlModelParam* param)
{
	auto* obj = static_cast<LQRCalculator*>(ptr);
	obj->DLQR_Control(param);

}
LQRControlModel make_model()
{
	LQRControlModel result{};
	result.base.name = "LQR Controller";
	result.base.type = CONTROL_MODEL;

	result.new_instance = new_instance;
	result.update = update;
	//result.del_instance = del_instance;

	return result;
}
extern"C" __declspec(dllexport) BaseModel * load_model()
{
	static LQRControlModel model = make_model();

	return reinterpret_cast<BaseModel*>(&model);
}