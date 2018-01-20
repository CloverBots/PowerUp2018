#ifndef GrabberLift_H
#define GrabberLift_H

#include "../CommandBase.h"
#include "WPILib.h"

class GrabberLift : public CommandBase {
public:
	GrabberLift(double SetPoint);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	PIDController* pid;
	double speed;
	double SetPoint;
};

#endif  // GrabberLift_H
