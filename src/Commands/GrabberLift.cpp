#include "GrabberLift.h"
#include"Subsystems/GrabberLiftSubsystem.h"
#include "RobotMap.h"
#include "WPILib.h"
#include "CommandBase.h"
#include <iostream>
GrabberLift::GrabberLift()
{

	Requires(CommandBase::grabberLiftSubsystem.get());
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void GrabberLift::Initialize()
{
}

// Called repeatedly when this Command is scheduled to run
void GrabberLift::Execute()
{
	Joystick* pOperatorStick;
	pOperatorStick = new Joystick(1);
	JoystickButton* AButton;
	JoystickButton* BButton;
	JoystickButton* XButton;
	JoystickButton* LButton;
	JoystickButton* RButton;
	AButton = new JoystickButton(pOperatorStick, 1);
	BButton = new JoystickButton(pOperatorStick, 2);
	XButton = new JoystickButton(pOperatorStick, 3);
	LButton = new JoystickButton(pOperatorStick, 9);
	RButton = new JoystickButton(pOperatorStick, 10);
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
		SetPoint = 28000;
	}
	else if(LButton->Get())
	{
		CommandBase::grabberLiftSubsystem->Reset();
		Speed = -.5;
		goto skip;
	}
	else if(RButton->Get())
	{
		CommandBase::grabberLiftSubsystem->Reset();
		Speed = .5;
		goto skip;
	}else{
		Speed = 0;
		SetPoint = CommandBase::grabberLiftSubsystem->GetDistance();
	}

	if(SetPoint > CommandBase::grabberLiftSubsystem->GetDistance())
	{
		Speed = -MAX_SPEED;
	}
	else if(SetPoint < CommandBase::grabberLiftSubsystem->GetDistance())
	{
		Speed = MAX_SPEED;
	}
	else if(SetPoint == 0)
	{
		Speed = 0;
	}
	if((std::abs((SetPoint - CommandBase::grabberLiftSubsystem->GetDistance())) < 00))
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
	return true;
}

// Called once after isFinished returns true
void GrabberLift::End() {
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void GrabberLift::Interrupted() {
}
