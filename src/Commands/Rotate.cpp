#include "Rotate.h"
#include "CommandBase.h"
#include <iostream>
Rotate::Rotate(double angle, float P, float I, float D) : m_targetAngle(angle), m_P(P) ,m_I(I) ,m_D(D)
{
	Requires(CommandBase::driveSubsystem.get());
}

// Called just before this Command runs the first time
void Rotate::Initialize()
{
	if(abs(m_targetAngle) == 90)
	{
		CommandBase::driveSubsystem->SetRotatePID(0.02f, 0.0f, 0.008f);
	}
	CommandBase::driveSubsystem->Shift(DoubleSolenoid::Value::kReverse);
	CommandBase::driveSubsystem->SetRotate(true, m_targetAngle);
}
// Called repeatedly when this Command is scheduled to run
void Rotate::Execute()
{
	//std::cout << CommandBase::driveSubsystem->GetGyroAngle() << std::endl;
}

// Make this return true when this Command no longer needs to run execute()
bool Rotate::IsFinished()
{
	if(CommandBase::driveSubsystem->RotateOnTarget())
	{
		//std::cout << "Rotate Done!" << std::endl;
		CommandBase::driveSubsystem->SetRotatePIDEnabled(false);
		CommandBase::driveSubsystem->ResetGyro();
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
