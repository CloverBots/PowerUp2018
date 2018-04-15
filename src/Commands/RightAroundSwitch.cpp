/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RightAroundSwitch.h"

RightAroundSwitch::RightAroundSwitch() {
	AddSequential(new AutoGrabber(-.2));
	AddSequential(new DriveDistance(216));
	AddSequential(new Rotate(-90));
	AddSequential(new DriveDistance(95));
	AddSequential(new AutoGrabber(0));
}
