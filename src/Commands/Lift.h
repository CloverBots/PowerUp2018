#ifndef Lift_H
#define Lift_H

#include "../CommandBase.h"

class Lift : public CommandBase {
	double speed;
public:
	Lift(double speed);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // Lift_H
