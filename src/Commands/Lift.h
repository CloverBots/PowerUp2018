#pragma once
#include "../CommandBase.h"
#include <WPILib.h>

class Lift : public CommandBase {
public:
	Lift(double speed);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	double speed;
};
