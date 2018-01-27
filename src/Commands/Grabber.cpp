#include "Grabber.h"
#include <WPILib.h>
#include "CommandBase.h"
Grabber::Grabber(double speed) : speed(speed)
{
	Requires (CommandBase::grabber.get());
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void Grabber::Initialize()
{
	CommandBase::grabber->SetGrabberSpeed(speed);
}

// Called repeatedly when this Command is scheduled to run
void Grabber::Execute() {

}

// Make this return true when this Command no longer needs to run execute()
bool Grabber::IsFinished() {
	return true;
}

// Called once after isFinished returns true
void Grabber::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void Grabber::Interrupted() {

}
