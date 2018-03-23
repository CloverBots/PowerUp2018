/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "LeftAroundSwitch.h"

LeftAroundSwitch::LeftAroundSwitch() {
	AddSequential(new DriveDistance(195));
	AddSequential(new Rotate(90));
	AddSequential(new DriveDistance(175));
	AddSequential(new AutoGrabberLift(12700));
	AddSequential(new Rotate(90));
	AddSequential(new AutoDrive(-.5, 0));
	AddSequential(new DelayCommand(.4));
	AddSequential(new AutoDrive(0, 0));
	AddSequential(new DelayCommand(.3));
	AddSequential(new AutoGrabber(1));
	AddSequential(new DelayCommand(.5));
	AddSequential(new AutoGrabber(0));
}
