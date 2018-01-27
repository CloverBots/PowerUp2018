#include "DriveToSwitchForwardRight.h"
#include <WPILib.h>
#include "CommandBase.h"
DriveToSwitchForwardRight::DriveToSwitchForwardRight() {
	AddSequential(new DriveDistance(147));
	AddSequential(new Rotate(-90));
}
