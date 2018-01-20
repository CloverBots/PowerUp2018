#include "GrabberLiftSpeed.h"
#include "WPILib.h"

GrabberLiftSpeed::GrabberLiftSpeed(double speed) : speed(speed)
{
	Requires(CommandBase::grabberLiftSubsystem.get());
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void GrabberLiftSpeed::Initialize()
{
	CommandBase::grabberLiftSubsystem->SetSpeed(speed);
}

// Called repeatedly when this Command is scheduled to run
void GrabberLiftSpeed::Execute() {

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
