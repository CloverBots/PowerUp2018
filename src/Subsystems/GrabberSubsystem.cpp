#include "GrabberSubsystem.h"
#include "../RobotMap.h"
#include <WPILib.h>
#include <ctre/Phoenix.h>
#include "Commands/Grabber.h"


GrabberSubsystem::GrabberSubsystem() : Subsystem("ExampleSubsystem") {
	Grabber_Motor_Right = new WPI_VictorSPX(RobotMap::GRABBER_MOTOR_RIGHT);
	Grabber_Motor_Left = new WPI_VictorSPX(RobotMap::GRABBER_MOTOR_LEFT);
}

void GrabberSubsystem::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
	SetDefaultCommand(new Grabber());
}
void GrabberSubsystem:: SetGrabberSpeed (double speed)
{
	Grabber_Motor_Right->Set(speed);
	Grabber_Motor_Left->Set(-speed);
}
// Put methods for controlling this subsystem
// here. Call these from Commands.
