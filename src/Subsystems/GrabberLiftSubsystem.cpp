#include "GrabberLiftSubsystem.h"
#include "../RobotMap.h"
#include "WPILib.h"
#include "DoublePIDController.h"
#include <ctre/Phoenix.h>

GrabberLiftSubsystem::GrabberLiftSubsystem() : Subsystem("GrabberLiftSubsystem")
{
	speed = 0;
	distance = 0;
	GrabberMotor = new WPI_VictorSPX(RobotMap::GRABBER_LIFT_MOTOR);
	GrabberMotor->ConfigSelectedFeedbackSensor(phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0, 0);
	Upper = new DigitalInput(0);
	Lower = new DigitalInput(1);
	GrabberLiftPID = new DoublePIDController(m_P, m_I, m_D, speed, GrabberMotor);

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
	return 	distance += ((GrabberMotor->GetSelectedSensorPosition(0) * m_EncScaler)) / 360;
	return distance;
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
