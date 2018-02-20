#include "GrabberLiftSubsystem.h"
#include "../RobotMap.h"
#include "WPILib.h"
#include "DoublePIDController.h"
#include <ctre/Phoenix.h>
#include "Commands/GrabberLift.h"

GrabberLiftSubsystem::GrabberLiftSubsystem() : Subsystem("GrabberLiftSubsystem")
{
	GrabberMotor = new WPI_TalonSRX(RobotMap::GRABBER_LIFT_MOTOR);
	GrabberMotor->ConfigSelectedFeedbackSensor(phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0, 0);
	Upper = new DigitalInput(1);
	Lower = new DigitalInput(0);
}

void GrabberLiftSubsystem::InitDefaultCommand()
{
	// Set the default command for a subsystem here.
	SetDefaultCommand(new GrabberLift());
}

void GrabberLiftSubsystem::SetSpeed(double speed)
{
	GrabberMotor->Set(speed);
}

double GrabberLiftSubsystem::GetDistance()
{
	return GrabberMotor->GetSelectedSensorPosition(0);
}

bool GrabberLiftSubsystem::GetUpper()
{
	return Upper->Get();
}

bool GrabberLiftSubsystem::GetLower()
{
	return Lower->Get();
}

void GrabberLiftSubsystem::Reset()
{
	GrabberMotor->GetSensorCollection().SetQuadraturePosition(0, 10);
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
