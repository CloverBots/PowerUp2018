#pragma once
#include "../CommandBase.h"
#include <WPILib.h>

class Lift : public CommandBase {
public:
	Lift(double liftspeed, double minispeed);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	double liftspeed;
	double minispeed;
};
