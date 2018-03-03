/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "CenterAuto.h"
#include <iostream>

CenterAuto::CenterAuto() {
	std::string gameData;
	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
//	std::cout << "Side: " << gameData[0] << std::endl;
//	if(gameData[0] == 'R')
//	{
//		std::cout << "Game data = R" << std::endl;
//	}
	if(gameData[0] == 'R')
	{
		std::cout << "Right" << std::endl;
		AddSequential(new AutoDrive(-.5, 0));
		AddSequential(new DelayCommand(.4));
		AddSequential(new AutoDrive(0, 0));
		AddSequential(new DelayCommand(.3));
		AddSequential(new Rotate(45));
		AddSequential(new DelayCommand(.3));
		AddSequential(new DriveDistance(125.5, 0.018));
		AddSequential(new Rotate(-45));
		AddSequential(new AutoGrabberLift(12700));
		AddSequential(new AutoDrive(-.5, 0));
		AddSequential(new DelayCommand(.5));
		AddSequential(new AutoDrive(0, 0));
		AddSequential(new DelayCommand(.3));
		AddSequential(new AutoGrabber(.5));
		AddSequential(new DelayCommand(.5));
		AddSequential(new AutoGrabber(0));
	}
	else if(gameData[0] == 'L')
	{

		std::cout << "Left" << std::endl;
		AddSequential(new AutoDrive(-.5, 0));
		AddSequential(new DelayCommand(.4));
		AddSequential(new AutoDrive(0, 0));
		AddSequential(new DelayCommand(.3));
		AddSequential(new Rotate(-45));
		AddSequential(new DelayCommand(.3));
		AddSequential(new DriveDistance(125.5, 0.018));
		AddSequential(new Rotate(45));
		AddSequential(new AutoGrabberLift(12700));
		AddSequential(new AutoDrive(-.5, 0));
		AddSequential(new DelayCommand(.5));
		AddSequential(new AutoDrive(0, 0));
		AddSequential(new DelayCommand(.3));
		AddSequential(new AutoGrabber(.5));
		AddSequential(new DelayCommand(.5));
		AddSequential(new AutoGrabber(0));
	}
//	std::cout << "Side: " << gameData[0] << std::endl;
}
