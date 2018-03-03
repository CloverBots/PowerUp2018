#include "DriveForwardAuto.h"
#include "CommandBase.h"
#include <iostream>
DriveForwardAuto::DriveForwardAuto() {
	AddSequential(new DriveDistance(140));
}
