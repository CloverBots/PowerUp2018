/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RunCenter.h"
#include "CenterAuto.h"
#include <iostream>

RunCenter::RunCenter() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void RunCenter::Initialize() {
	std::cout << "CENTER" << std::endl;
	std::string gameData;
	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	std::unique_ptr<frc::Command> autonomousCommand;
	std::cout<< gameData << std::endl;
	autonomousCommand.reset(new CenterAuto(gameData[0]));
	autonomousCommand->Start();
}

// Called repeatedly when this Command is scheduled to run
void RunCenter::Execute() {

}

// Make this return true when this Command no longer needs to run execute()
bool RunCenter::IsFinished() {
	return true;
}

// Called once after isFinished returns true
void RunCenter::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void RunCenter::Interrupted() {

}
