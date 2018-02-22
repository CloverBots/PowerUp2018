#include "DriveToSwitchForwardRight.h"
#include <WPILib.h>
#include "CommandBase.h"
#include <iostream>
DriveToSwitchForwardRight::DriveToSwitchForwardRight() {
	std::cout << "RUNNING THSI ONE" << std::endl;
	AddSequential(new DriveDistance(149));
	AddSequential(new Rotate(-90));
	AddSequential(new DriveDistance(25));
}
