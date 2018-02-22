#include "DriveToSwitchForwardLeft.h"
#include "CommandBase.h"
DriveToSwitchForwardLeft::DriveToSwitchForwardLeft() {
	AddSequential(new DriveDistance(149));
	AddSequential(new Rotate(90));
	AddSequential(new DriveDistance(25));
}
