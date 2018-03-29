/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "DoubleCubeAutoRight.h"

DoubleCubeAutoRight::DoubleCubeAutoRight() {
	AddSequential(new DriveDistance(149));
	AddSequential(new Rotate(-90));
	AddSequential(new AutoGrabberLift(12700));
	AddSequential(new AutoDrive(-.5, 0));
	AddSequential(new DelayCommand(.7));
	AddSequential(new AutoDrive(0, 0));
	AddSequential(new DelayCommand(.3));
	AddSequential(new AutoGrabber(1));
	AddSequential(new DelayCommand(.5));
	AddSequential(new AutoGrabber(0));
	AddSequential(new AutoDrive(.5, 0));
	AddSequential(new DelayCommand(.4));
	AddSequential(new AutoDrive(0, 0));
	AddSequential(new DelayCommand(.3));
	AddSequential(new Rotate(90));
	AddSequential(new DelayCommand(.4));
	AddSequential(new DriveDistance(80));
	AddSequential(new DelayCommand(.4));
	AddSequential(new Rotate(-135));
	AddSequential(new AutoGrabberLift(0));
	AddSequential(new AutoGrabber(-1));
	AddSequential(new DriveDistance(70));
	AddSequential(new AutoGrabber(0));
	AddSequential(new AutoGrabberLift(12700));
	AddSequential(new AutoGrabber(1));
	AddSequential(new DelayCommand(.5));
	AddSequential(new AutoGrabber(0));
}
