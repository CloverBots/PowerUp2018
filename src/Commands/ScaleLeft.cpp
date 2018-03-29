/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "ScaleLeft.h"

ScaleLeft::ScaleLeft() {
	AddSequential(new DriveDistance(200));
	AddSequential(new Rotate(20));
	AddSequential(new AutoGrabberLift(29500));
	AddSequential(new DriveDistance(56, false));
	AddSequential(new AutoGrabber(.3));
	AddSequential(new DelayCommand(.5));
	AddSequential(new AutoGrabber(0));
	AddSequential(new DriveDistance(-24, false));
	AddSequential(new AutoGrabberLift(0));
}
