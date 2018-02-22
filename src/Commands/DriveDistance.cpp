#include "DriveDistance.h"
#include "CommandBase.h"
#include <iostream>
DriveDistance::DriveDistance(double distance) : Distance(distance){
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::driveSubsystem.get());
}

// Called just before this Command runs the first time
void DriveDistance::Initialize() {
	std::cout << "DRIVE" << std::endl;
	CommandBase::driveSubsystem->Shift(DoubleSolenoid::Value::kForward);
	CommandBase::driveSubsystem->SetDrive(true, Distance);
}

// Called repeatedly when this Command is scheduled to run
void DriveDistance::Execute()
{
	//CommandBase::driveSubsystem->Drive(-.5, 0);
}
// Make this return true when this Command no longer needs to run execute()
bool DriveDistance::IsFinished()
{
	if(CommandBase::driveSubsystem->OnTarget())
	{
		CommandBase::driveSubsystem->SetDrivePIDEnabled(false);
		return true;
	}
	else
	{
		return false;
	}
//	if(std::fabs(Distance - CommandBase::driveSubsystem->GetDistance()) < 2)
//	{
//		std::cout << "DONE!" << std::endl;
//		return true;
//	}else{
//		return false;
//	}
}

// Called once after isFinished returns true
void DriveDistance::End() {
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveDistance::Interrupted() {
}
