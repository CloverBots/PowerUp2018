#include "Grabber.h"
#include <WPILib.h>
#include "CommandBase.h"
#include <iostream>
Grabber::Grabber()
{
	speed = 0;
	Requires(CommandBase::grabber.get());
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	pOperatorStick = new Joystick(1);
	YButton = new JoystickButton(pOperatorStick, 4);
	BackButton = new JoystickButton(pOperatorStick, 7);
	StartButton = new JoystickButton(pOperatorStick, 1);
}

// Called just before this Command runs the first time
void Grabber::Initialize()
{
	//if(pDriveStick->GetRawAxis(4) != 0 & (!BackButton->Get() || !StartButton->Get()))
	//{

	//}
	//else
	//{
//		if(StartButton->Get())
//		{
//			speed = 1;
//		}
//		else if(BackButton->Get())
//		{
//			speed = -1;
//		}
	if(YButton->Get())
	{
		speed = -.2;
	}
	else
	{
		speed = 0;
	}
	if((pOperatorStick->GetRawAxis(3) - pOperatorStick->GetRawAxis(2)) != 0)
	{
		speed = pOperatorStick->GetRawAxis(3) - pOperatorStick->GetRawAxis(2);
	}
	//}
	CommandBase::grabber->SetGrabberSpeed(speed);
}

// Called repeatedly when this Command is scheduled to run
void Grabber::Execute() {

}

// Make this return true when this Command no longer needs to run execute()
bool Grabber::IsFinished() {
	return true;
}

// Called once after isFinished returns true
void Grabber::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void Grabber::Interrupted() {

}
