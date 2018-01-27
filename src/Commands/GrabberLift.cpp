#include "GrabberLift.h"
#include "Commands/GrabberLiftSpeed.h"
#include"Subsystems/GrabberLiftSubsystem.h"
#include "RobotMap.h"
#include "WPILib.h"
#include "CommandBase.h"
GrabberLift::GrabberLift(double SetPoint)
{
	this->SetPoint = SetPoint;
	Requires(CommandBase::grabberLiftSubsystem.get());
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void GrabberLift::Initialize()
{
	CommandBase::grabberLiftSubsystem->SetSpeed(0);
}

// Called repeatedly when this Command is scheduled to run
void GrabberLift::Execute()
{
	if(CommandBase::grabberLiftSubsystem->GetUpper() || CommandBase::grabberLiftSubsystem->GetLower())
	{
		CommandBase::grabberLiftSubsystem->SetPID(true, CommandBase::grabberLiftSubsystem->GetSetPoint());
	}
}

// Make this return true when this Command no longer needs to run execute()
bool GrabberLift::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void GrabberLift::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void GrabberLift::Interrupted() {

}
