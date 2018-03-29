#include "Lift.h"
#include "CommandBase.h"
#include <iostream>
Lift::Lift(){
	liftspeed = 0;
	minispeed = 0;
	Requires(CommandBase::lift.get());
	pDriveStick = new Joystick(0);
	pOperatorStick = new Joystick(1);
	Rbumper = new JoystickButton(pOperatorStick, 6);
	Lbumper = new JoystickButton(pOperatorStick, 5);
	StartButton = new JoystickButton(pDriveStick, 8);
	XButton = new JoystickButton(pDriveStick, 3);
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void Lift::Initialize() {
	if(Rbumper->Get() && XButton->Get())
	{
		liftspeed = 1;
		minispeed = .3;
	}else if(Lbumper->Get())
	{
		liftspeed = 0;
		minispeed = -1;
	}else if(StartButton->Get())
	{
		liftspeed = -1;
		minispeed = 0;
	}else
	{
		liftspeed = 0;
		minispeed = 0;
	}
	/*
	if(!CommandBase::lift->GetLimit())
	{
		std::cout << "HIT" << std::endl;
		if(minispeed < 0)
		{
			minispeed = 0;
		}
	}
	*/
	CommandBase::lift->SetSpeed(liftspeed, minispeed);

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
