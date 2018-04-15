#include "DriveDistance.h"
#include "CommandBase.h"
#include <iostream>
DriveDistance::DriveDistance(double distance, bool rotate,float P, float I, float D) :Distance(distance), m_P(P) ,m_I(I) ,m_D(D){
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	m_rotate = rotate;
	Requires(CommandBase::driveSubsystem.get());
}

// Called just before this Command runs the first time
void DriveDistance::Initialize() {
	CommandBase::driveSubsystem->ResetDrive();
	CommandBase::driveSubsystem->SetPID(m_P, m_I, m_D);
	CommandBase::driveSubsystem->Shift(DoubleSolenoid::Value::kForward);
	CommandBase::driveSubsystem->SetDrive(true, Distance);
	CommandBase::driveSubsystem->ResetGyro();
	CommandBase::driveSubsystem->SetDriveRotate(m_rotate, CommandBase::driveSubsystem->GetGyroAngle());
}

// Called repeatedly when this Command is scheduled to run
void DriveDistance::Execute()
{
	std::cout << CommandBase::driveSubsystem->GetDistance() << std::endl;
	CommandBase::driveSubsystem->AutoDrivePID();
}
// Make this return true when this Command no longer needs to run execute()
bool DriveDistance::IsFinished()
{
	if(CommandBase::driveSubsystem->OnTarget())
	{
		CommandBase::driveSubsystem->SetDrivePIDEnabled(false);
		CommandBase::driveSubsystem->ResetDrive();
		CommandBase::driveSubsystem->SetDriveRotatePIDEnabled(false);
		CommandBase::driveSubsystem->ResetGyro();
		std::cout << "Drive Done!" << std::endl;
		return true;
	}
	else
	{
		return false;
	}
}

// Called once after isFinished returns true
void DriveDistance::End() {
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveDistance::Interrupted() {
}
