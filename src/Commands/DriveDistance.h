#ifndef DriveDistance_H
#define DriveDistance_H

#include "../CommandBase.h"

class DriveDistance : public CommandBase {
	double Distance;
public:
	DriveDistance(double distance); //inches
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveDistance_H
