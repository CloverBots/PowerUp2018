#include "DriveToSwitchForwardLeft.h"
#include "CommandBase.h"
DriveToSwitchForwardLeft::DriveToSwitchForwardLeft() {
	AddSequential(new DriveDistance(147));
	AddSequential(new Rotate(90));
}
