#include "Lift.h"
#include "CommandBase.h"
Lift::Lift(double speed) : speed(speed) {
	Requires(CommandBase::lift.get());
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void Lift::Initialize() {
	CommandBase::lift->SetSpeed(speed);

}

// Called repeatedly when this Command is scheduled to run
void Lift::Execute() {

}

// Make this return true when this Command no longer needs to run execute()
bool Lift::IsFinished() {
	return true;
}

// Called once after isFinished returns true
void Lift::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void Lift::Interrupted() {

}
