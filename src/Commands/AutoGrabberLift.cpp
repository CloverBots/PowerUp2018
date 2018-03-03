/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "Subsystems/GrabberLiftSubsystem.h"
#include "AutoGrabberLift.h"
#include <iostream>

AutoGrabberLift::AutoGrabberLift(double SetPoint) : SetPoint(SetPoint) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::grabberLiftSubsystem.get());
}

// Called just before this Command runs the first time
void AutoGrabberLift::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void AutoGrabberLift::Execute() {
	if(SetPoint > CommandBase::grabberLiftSubsystem->GetDistance())
	{
		Speed = -MAX_SPEED;
	}
	else if(SetPoint < CommandBase::grabberLiftSubsystem->GetDistance())
	{
		Speed = MAX_SPEED;
	}
	else if(SetPoint == 0)
	{
		Speed = 0;
	}
	if((std::abs((SetPoint - CommandBase::grabberLiftSubsystem->GetDistance())) < 300))
	{
		std::cout << "LIFT DONE!" << std::endl;
		Done = true;
		Speed = 0;
	}
	if(!CommandBase::grabberLiftSubsystem->GetUpper())
	{
		if(Speed < 0)
		{
			Speed = 0;
		}
	}
	if(!CommandBase::grabberLiftSubsystem->GetLower())
	{
		if(Speed > 0)
		{
			Speed = 0;
		}
	}
	CommandBase::grabberLiftSubsystem->SetSpeed(Speed);
}

// Make this return true when this Command no longer needs to run execute()
bool AutoGrabberLift::IsFinished() {
	return Done;
}

// Called once after isFinished returns true
void AutoGrabberLift::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutoGrabberLift::Interrupted() {

}
