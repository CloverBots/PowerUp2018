#include "GrabberLift.h"
#include "Commands/GrabberLiftSpeed.h"
#include"Subsystems/GrabberLiftSubsystem.h"
#include "RobotMap.h"
#include "WPILib.h"

GrabberLift::GrabberLift(double SetPoint) : Subsystem("ExampleSubystem")
{
	this->SetPoint = SetPoint;
	Requires(CommandBase::grabberLiftSubsystem.get());
	pid = new PIDController(1,1,1, CommandBase::grabberLiftSubsystem->GetDistance(), speed);
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void GrabberLift::Initialize()
{
	pid->SetSetpoint(SetPoint);
	pid->Enable();
	CommandBase::grabberLiftSubsystem->SetSpeed(0);
}

// Called repeatedly when this Command is scheduled to run
void GrabberLift::Execute()
{
	if(CommandBase::grabberLiftSubsystem->GetUpper() || CommandBase::grabberLiftSubsystem->GetLower())
	{
		pid->Disable();
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
