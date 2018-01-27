#include "AutoDrive.h"
#include "CommandBase.h"

AutoDrive::AutoDrive(double speed, double turn) : speed(speed),turn(turn){
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::driveSubsystem.get());
}

// Called just before this Command runs the first time
void AutoDrive::Initialize() {
	CommandBase::driveSubsystem->Drive(0,0);
	CommandBase::driveSubsystem->Shift(DoubleSolenoid::Value::kForward);
}

// Called repeatedly when this Command is scheduled to run
void AutoDrive::Execute() {
	CommandBase::driveSubsystem->Drive(speed, turn);
}

// Make this return true when this Command no longer needs to run execute()
bool AutoDrive::IsFinished() {
	return true;
}

// Called once after isFinished returns true
void AutoDrive::End() {
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutoDrive::Interrupted() {

}
