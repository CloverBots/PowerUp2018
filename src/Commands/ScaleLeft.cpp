/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "ScaleLeft.h"

ScaleLeft::ScaleLeft() {
	std::string gameData;
	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	if(gameData[1] == 'L')
	{
		//AddSequential(new DriveDistance(280.65, 0.009));
		//AddSequential(new Rotate(90));
		AddSequential(new AutoGrabberLift(12700));//AddSequential(new AutoGrabberLift(28000));
		AddSequential(new DriveDistance(19, 0.017));
		AddSequential(new AutoGrabber(-1));
		AddSequential(new DelayCommand(.5));
		AddSequential(new AutoGrabber(0));
	}
	else if(gameData[0] == 'L')
	{
		AddSequential(new DriveDistance(149));
		AddSequential(new Rotate(90));
		AddSequential(new AutoGrabberLift(12700));
		AddSequential(new AutoDrive(-.5, 0));
		AddSequential(new DelayCommand(.4));
		AddSequential(new AutoDrive(0, 0));
		AddSequential(new DelayCommand(.3));
		AddSequential(new AutoGrabber(-1));
		AddSequential(new DelayCommand(.5));
		AddSequential(new AutoGrabber(0));
	}
	else
	{
		AddSequential(new DriveDistance(140));
	}
}
