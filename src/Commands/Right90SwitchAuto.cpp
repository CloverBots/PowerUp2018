/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Right90SwitchAuto.h"
#include <iostream>
Right90SwitchAuto::Right90SwitchAuto() {
	AddSequential(new DriveDistance(139));
	AddSequential(new Rotate(-90));
	AddSequential(new AutoGrabberLift(12700));
	AddSequential(new AutoDrive(-.5, 0));
	AddSequential(new DelayCommand(.5));
	AddSequential(new AutoDrive(0, 0));
	AddSequential(new DelayCommand(.3));
	AddSequential(new AutoGrabber(1));
	AddSequential(new DelayCommand(.5));
	AddSequential(new AutoGrabber(0));
}
