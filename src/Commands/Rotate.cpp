#include "Rotate.h"
#include "CommandBase.h"
Rotate::Rotate(double angle) : m_targetAngle(angle)
{
	Requires(CommandBase::driveSubsystem.get());
}

// Called just before this Command runs the first time
void Rotate::Initialize()
{
	CommandBase::driveSubsystem->SetRotate(true, m_targetAngle);
}
// Called repeatedly when this Command is scheduled to run
void Rotate::Execute()
{
}

// Make this return true when this Command no longer needs to run execute()
bool Rotate::IsFinished()
{
	if(false)
//	if(abs(m_targetAngle - CommandBase::driveSubsystem->GetLeftDistance()) < .1)
	{
		return true;
	}else{
		return false;
	}
}

// Called once after isFinished returns true
void Rotate::End()
{
	//CommandBase::driveSubsystem->Drive(0.0f, 0.0f);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void Rotate::Interrupted()
{

}
