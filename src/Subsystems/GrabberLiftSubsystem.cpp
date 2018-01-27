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
	GrabberLiftPID = new PIDController(m_P, m_I, m_D, GrabberLiftEncoder, GrabberMotor);

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

double GrabberLiftSubsystem::GetSetPoint()
{
	return GrabberLiftPID->GetSetpoint();
}

void GrabberLiftSubsystem::SetPIDEnabled(bool enabled)
{
	GrabberLiftPID->SetSetpoint(0.0f);
	SetSpeed(0);
	GrabberLiftPID->Reset();

	if (enabled)
		GrabberLiftPID->Enable();
}

void GrabberLiftSubsystem::SetPID(bool enabled, double setpoint)
{
	SetPIDEnabled(enabled);
	GrabberLiftPID->SetSetpoint(setpoint);
}
// Put methods for controlling this subsystem
// here. Call these from Commands.
