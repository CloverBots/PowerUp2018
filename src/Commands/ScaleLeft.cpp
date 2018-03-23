/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "ScaleLeft.h"

ScaleLeft::ScaleLeft() {
	AddSequential(new DriveDistance(200));
	AddSequential(new Rotate(30));
	AddSequential(new AutoGrabberLift(28000));
	AddSequential(new DriveDistance(80, false, 0.02f, 0.0f, 0.08f));
	AddSequential(new AutoGrabber(1));
	AddSequential(new DelayCommand(.5));
	AddSequential(new AutoGrabber(0));
}
