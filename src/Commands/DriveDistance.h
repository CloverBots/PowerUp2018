#ifndef DriveDistance_H
#define DriveDistance_H

#include "../CommandBase.h"
#include "WPILib.h"
class DriveDistance : public CommandBase {
	double Distance;
	bool half = false;
public:
	DriveDistance(double distance); //inches
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveDistance_H
