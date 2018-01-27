#ifndef GrabberLiftSpeed_H
#define GrabberLiftSpeed_H

#include "../CommandBase.h"

class GrabberLiftSpeed : public CommandBase {
public:
	GrabberLiftSpeed(double setpoint);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	double SetPoint;
};

#endif  // GrabberLiftSpeed_H
