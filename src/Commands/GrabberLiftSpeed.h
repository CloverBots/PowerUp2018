#ifndef GrabberLiftSpeed_H
#define GrabberLiftSpeed_H

#include "../CommandBase.h"

class GrabberLiftSpeed : public CommandBase {
public:
	GrabberLiftSpeed(double speed);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	double speed;
};

#endif  // GrabberLiftSpeed_H
