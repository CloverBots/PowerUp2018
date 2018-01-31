#include "DriveDistance.h"
#include "CommandBase.h"
DriveDistance::DriveDistance(double distance) : Distance(distance){
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::driveSubsystem.get());
}

// Called just before this Command runs the first time
void DriveDistance::Initialize() {
	CommandBase::driveSubsystem->SetDrive(true, Distance);
}

// Called repeatedly when this Command is scheduled to run
void DriveDistance::Execute() {
}

// Make this return true when this Command no longer needs to run execute()
bool DriveDistance::IsFinished() {
	if((abs(Distance - CommandBase::driveSubsystem->GetLeftDistance()) < .1) &
			(abs(Distance - CommandBase::driveSubsystem->GetRightDistance()) < .1)){
		return true;
	}else{
		return false;
	}
}

// Called once after isFinished returns true
void DriveDistance::End() {
	//CommandBase::driveSubsystem->Drive(0.0f, 0.0f);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveDistance::Interrupted() {

}
