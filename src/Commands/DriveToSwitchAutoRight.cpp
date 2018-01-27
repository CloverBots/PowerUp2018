#include "DriveToSwitchAutoRight.h"
#include "CommandBase.h"
DriveToSwitchAutoRight::DriveToSwitchAutoRight() {
	AddSequential(new DriveDistance(21));
	AddSequential(new Rotate(-45));
	AddSequential(new DriveDistance(81.5));
	AddSequential(new Rotate(45));
	AddSequential(new DriveDistance(19));
}
