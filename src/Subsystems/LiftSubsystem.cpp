#include "LiftSubsystem.h"
#include "../RobotMap.h"
#include "WPILib.h"
#include <ctre/Phoenix.h>

LiftSubsystem::LiftSubsystem() : Subsystem("LiftSubsystem") {
	Lift_Motor_Right = new WPI_VictorSPX(RobotMap::LIFT_MOTOR_RIGHT);
	Lift_Motor_Left = new WPI_VictorSPX(RobotMap::LIFT_MOTOR_LEFT);
	Lift_Motor_Up = new WPI_TalonSRX(RobotMap::LIFT_MOTOR_UP);
	Limit = new DigitalInput(2);
}

void LiftSubsystem::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

void LiftSubsystem::SetSpeed(double speed) {
	Lift_Motor_Right->Set(speed);
	Lift_Motor_Right->Set(speed);
	/*
	if(speed > 0)
	{
		if(Limit->Get())
		{
			Lift_Motor_Right->Set(speed);
			Lift_Motor_Right->Set(speed);
			Lift_Motor_Up->Set(-speed / 10);
		}
		else
		{
			Lift_Motor_Right->Set(0);
			Lift_Motor_Right->Set(0);
			Lift_Motor_Up->Set(0);
		}
	}
	if(speed < 0)
	{
		if(Limit->Get())
		{
			Lift_Motor_Right->Set(0);
			Lift_Motor_Right->Set(0);
			Lift_Motor_Up->Set(speed);
		}
		else
		{
			Lift_Motor_Right->Set(0);
			Lift_Motor_Right->Set(0);
			Lift_Motor_Up->Set(0);
		}
	}
	if(speed == 0)
	{
		Lift_Motor_Right->Set(0);
		Lift_Motor_Right->Set(0);
		Lift_Motor_Up->Set(0);
	}
	*/
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
