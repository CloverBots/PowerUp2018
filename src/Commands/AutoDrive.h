#ifndef AutoDrive_H
#define AutoDrive_H
#include <wpilib.h>

#include "../CommandBase.h"

class AutoDrive : public CommandBase {
	double speed;
	double turn;
public:
	AutoDrive(double speed, double turn);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // AutoDrive_H
