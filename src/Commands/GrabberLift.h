#ifndef GrabberLift_H
#define GrabberLift_H

#include "../CommandBase.h"
#include "WPILib.h"
class GrabberLift : public CommandBase {
public:
	double Speed = 0;
	double SetPoint = 0;
	const double MAX_SPEED = .5;
	GrabberLift();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
};

#endif  // GrabberLift_H
