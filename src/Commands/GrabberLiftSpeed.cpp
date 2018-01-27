#include "GrabberLiftSpeed.h"
#include "WPILib.h"
#include "CommandBase.h"
GrabberLiftSpeed::GrabberLiftSpeed(double setpoint) : SetPoint(setpoint)
{
	Requires(CommandBase::grabberLiftSubsystem.get());
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void GrabberLiftSpeed::Initialize()
{
	CommandBase::grabberLiftSubsystem->SetPID(true, SetPoint);
}

// Called repeatedly when this Command is scheduled to run
void GrabberLiftSpeed::Execute() {
	if(CommandBase::grabberLiftSubsystem->GetUpper() || CommandBase::grabberLiftSubsystem->GetLower())
	{
		CommandBase::grabberLiftSubsystem->SetPID(true, CommandBase::grabberLiftSubsystem->GetSetPoint());
	}
}

// Make this return true when this Command no longer needs to run execute()
bool GrabberLiftSpeed::IsFinished() {
	return true;
}

// Called once after isFinished returns true
void GrabberLiftSpeed::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void GrabberLiftSpeed::Interrupted() {

}
