#include "LiftSubsystem.h"
#include "../RobotMap.h"
#include "WPILib.h"

LiftSubsystem::LiftSubsystem() : Subsystem("LiftSubsystem") {
	Lift_Motor = new Talon(RobotMap::LIFT_MOTOR);
}

void LiftSubsystem::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

void LiftSubsystem::SetSpeed(double speed) {
	Lift_Motor->Set(speed);

}

// Put methods for controlling this subsystem
// here. Call these from Commands.
