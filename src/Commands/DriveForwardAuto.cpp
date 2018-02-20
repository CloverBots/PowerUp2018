#include "DriveForwardAuto.h"
#include "CommandBase.h"
DriveForwardAuto::DriveForwardAuto() {
	AddSequential(new DriveDistance(60));//120
}
