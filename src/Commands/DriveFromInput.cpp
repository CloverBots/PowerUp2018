#include <Commands/DriveFromInput.h>
#include "../Subsystems/DriveSubsystem.h"
#include "CommandBase.h"
DriveFromInput::DriveFromInput(){
	Requires(CommandBase::driveSubsystem.get());
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
}

// Called just before thiSs Command runs the first time
void DriveFromInput::Initialize() {
	CommandBase::driveSubsystem->Drive(0,0);
	CommandBase::driveSubsystem->Shift(DoubleSolenoid::Value::kForward);
}

// Called repeatedly when this Command is scheduled to run
void DriveFromInput::Execute() {

	Joystick* pDriveStick = CommandBase::oi->GetDriveStick();
	CommandBase::driveSubsystem->Drive(
			pDriveStick->GetRawAxis(1),
			pDriveStick->GetRawAxis(4));
	//Wait(.001);
	if(pDriveStick->GetRawButton(1))
	{
		CommandBase::driveSubsystem->Shift(DoubleSolenoid::Value::kForward);
	}
	if(pDriveStick->GetRawButton(2))
	{
		CommandBase::driveSubsystem->Shift(DoubleSolenoid::Value::kReverse);
	}
}

// Make this return true when this Command no longer needs to run execute()
bool DriveFromInput::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void DriveFromInput::End() {
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveFromInput::Interrupted() {
}
