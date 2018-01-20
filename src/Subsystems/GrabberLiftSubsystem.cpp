#include "GrabberLiftSubsystem.h"
#include "../RobotMap.h"
#include "WPILib.h"

GrabberLiftSubsystem::GrabberLiftSubsystem() : Subsystem("GrabberLiftSubsystem")
{
	GrabberMotor = new Talon(RobotMap::GRABBER_LIFT_MOTOR);
	GrabberLiftEncoder = new Encoder(0, 1, false, Encoder::EncodingType::k4X);
	GrabberLiftEncoder->SetMaxPeriod(.1);
	GrabberLiftEncoder->SetMinRate(10);
	GrabberLiftEncoder->SetDistancePerPulse(5);
	GrabberLiftEncoder->SetReverseDirection(true);
	GrabberLiftEncoder->SetSamplesToAverage(7);
	GrabberLiftEncoder->Reset();
	Upper = new DigitalInput(0);
	Lower = new DigitalInput(1);

}

void GrabberLiftSubsystem::InitDefaultCommand()
{
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

void GrabberLiftSubsystem::SetSpeed(double speed)
{
	GrabberMotor->Set(speed);
}

double GrabberLiftSubsystem::GetDistance()
{
	return GrabberLiftEncoder->GetDistance();
}

bool GrabberLiftSubsystem::GetUpper()
{
	return Upper->Get();
}

bool GrabberLiftSubsystem::GetLower()
{
	return Lower->Get();
}
// Put methods for controlling this subsystem
// here. Call these from Commands.
