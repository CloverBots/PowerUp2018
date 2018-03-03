/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "AutoGrabber.h"
#include "Subsystems/GrabberSubsystem.h"
AutoGrabber::AutoGrabber(double speed) : speed(speed) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::grabber.get());
}

// Called just before this Command runs the first time
void AutoGrabber::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void AutoGrabber::Execute() {
	CommandBase::grabber->SetGrabberSpeed(speed);
}

// Make this return true when this Command no longer needs to run execute()
bool AutoGrabber::IsFinished() {
	return true;
}

// Called once after isFinished returns true
void AutoGrabber::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutoGrabber::Interrupted() {

}
