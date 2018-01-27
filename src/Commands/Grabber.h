#ifndef Grabber_H
#define Grabber_H

#include "../CommandBase.h"

class Grabber : public CommandBase {
private:
	double speed;
public:
	Grabber(double speed);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // Grabber_H
