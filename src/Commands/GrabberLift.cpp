#include "GrabberLift.h"
#include"Subsystems/GrabberLiftSubsystem.h"
#include "RobotMap.h"
#include "WPILib.h"
#include "CommandBase.h"
#include <iostream>
GrabberLift::GrabberLift()
{
	Requires(CommandBase::grabberLiftSubsystem.get());
	pOperatorStick = new Joystick(1);
	AButton = new JoystickButton(pOperatorStick, 8);
	BButton = new JoystickButton(pOperatorStick, 2);
	XButton = new JoystickButton(pOperatorStick, 3);
	LButton = new JoystickButton(pOperatorStick, 9);
	RButton = new JoystickButton(pOperatorStick, 10);

	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time`
void GrabberLift::Initialize()
{

}

// Called repeatedly when this Command is scheduled to run
void GrabberLift::Execute()
{
	if(XButton->Get())
	{
		SetPoint = 0;
	}
	else if(AButton->Get())
	{
		SetPoint = 9700;
	}
	else if(BButton->Get())
	{
		SetPoint = 30000;
	}
//	else if(LButton->Get())
//	{
//		CommandBase::grabberLiftSubsystem->Reset();
//		Speed = -.5;
//		goto skip;
//	}
//	else if(RButton->Get())
//	{
//		CommandBase::grabberLiftSubsystem->Reset();
//		Speed = .5;
//		goto skip;
	else if(pOperatorStick->GetRawAxis(1) != 0)
	{
		Speed = pOperatorStick->GetRawAxis(1);
		if (Speed > 0)
		{
			Speed /= 7;
		}
		goto skip;
	}else{
		Speed = 0;
		SetPoint = CommandBase::grabberLiftSubsystem->GetDistance();
	}
	if(fabs(SetPoint - CommandBase::grabberLiftSubsystem->GetDistance()) < 1000)
	{
		if(SetPoint != CommandBase::grabberLiftSubsystem->GetDistance())
		{
			acceleration -= .01;
		}
		//std::cout << acceleration << std::endl;
	}else
	{
		acceleration = 1;
	}
	if(SetPoint > CommandBase::grabberLiftSubsystem->GetDistance())
	{
		Speed = -MAX_SPEED * acceleration;
	}
	else if(SetPoint < CommandBase::grabberLiftSubsystem->GetDistance())
	{
		Speed = MAX_SPEED * acceleration;
	}
	if((std::abs((SetPoint - CommandBase::grabberLiftSubsystem->GetDistance())) < 0))
	{
		Speed = 0;
	}
	skip:
	if(!CommandBase::grabberLiftSubsystem->GetUpper())
	{
		if(Speed < 0)
		{
			Speed = 0;
		}
	}
	if(!CommandBase::grabberLiftSubsystem->GetLower())
	{
		if(Speed > 0)
		{
			Speed = 0;
		}
	}
	CommandBase::grabberLiftSubsystem->SetSpeed(Speed);
}

// Make this return true when this Command no longer needs to run execute()
bool GrabberLift::IsFinished() {
	//if((abs(SetPoint - CommandBase::grabberLiftSubsystem->GetDistance()) > 10) & (abs(SetPoint - CommandBase::grabberLiftSubsystem->GetDistance()) < 10))
	//{
	//	return true;
	//}
	//else
	//{
	//	return false;
	//}
	return false;
}

// Called once after isFinished returns true
void GrabberLift::End() {
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void GrabberLift::Interrupted() {
}
