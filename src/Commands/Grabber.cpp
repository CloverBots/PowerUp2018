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
}

// Called just before this Command runs the first time
void Grabber::Initialize()
{
	Joystick* pDriveStick;
	pDriveStick = new Joystick(0);
	Joystick* pOperatorStick;
	pOperatorStick = new Joystick(1);
	JoystickButton* BackButton;
	JoystickButton* StartButton;
	BackButton = new JoystickButton(pOperatorStick, 7);
	StartButton = new JoystickButton(pOperatorStick, 8);
	//if(pDriveStick->GetRawAxis(4) != 0 & (!BackButton->Get() || !StartButton->Get()))
	//{

	//}
	//else
	//{
		if(StartButton->Get())
		{
			speed = 1;
		}
		else if(BackButton->Get())
		{
			speed = -1;
		}else
		{
			speed = 0;
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
