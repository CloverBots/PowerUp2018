/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "CenterRight.h"

CenterRight::CenterRight() {
	AddSequential(new AutoDrive(-.5, 0));
	AddSequential(new DelayCommand(.4));
	AddSequential(new AutoDrive(0, 0));
	AddSequential(new DelayCommand(.3));
	AddSequential(new Rotate(45));
	AddSequential(new DelayCommand(.3));
	AddSequential(new DriveDistance(65.5, 0.018));
	AddSequential(new Rotate(-45));
	AddSequential(new AutoGrabberLift(12700));
	AddSequential(new AutoDrive(-.5, 0));
	AddSequential(new DelayCommand(.6));
	AddSequential(new AutoDrive(0, 0));
	AddSequential(new DelayCommand(.3));
	AddSequential(new AutoGrabber(.35));
	AddSequential(new DelayCommand(.5));
	AddSequential(new AutoGrabber(0));
}
