#include <Commands/DriveFromInput.h>
#include "DriveSubsystem.h"
#include "../RobotMap.h"
#include <WPILib.h>
#include <iostream>

DriveSubsystem::DriveSubsystem() : Subsystem("driveSubsystem") {
		Front_Right_Motor = new Talon(RobotMap::FRONT_RIGHT_MOTOR);
		Front_Left_Motor = new Talon(RobotMap::FRONT_LEFT_MOTOR);
		Middle_Right_Motor = new Talon(RobotMap::MIDDLE_RIGHT_MOTOR);
		Middle_Left_Motor = new Talon(RobotMap::MIDDLE_LEFT_MOTOR);
		Back_Right_Motor = new Talon(RobotMap::BACK_RIGHT_MOTOR);
		Back_Left_Motor = new Talon(RobotMap::BACK_LEFT_MOTOR);
		Gear_Box = new DoubleSolenoid(0, 1);
		LeftEncoder = new Encoder(0, 1, false, Encoder::EncodingType::k4X);
		LeftEncoder->SetMaxPeriod(.1);
		LeftEncoder->SetMinRate(10);
		LeftEncoder->SetDistancePerPulse(5);
		LeftEncoder->SetReverseDirection(true);
		LeftEncoder->SetSamplesToAverage(7);
		LeftEncoder->Reset();
		RightEncoder = new Encoder(2, 3, false, Encoder::EncodingType::k4X);
		RightEncoder->SetMaxPeriod(.1);
		RightEncoder->SetMinRate(10);
		RightEncoder->SetDistancePerPulse(5);
		RightEncoder->SetReverseDirection(false);
		RightEncoder->SetSamplesToAverage(7);
		RightEncoder->Reset();
		m_gyro = new AnalogGyro(0);
		m_gyro->Calibrate();
		RightPID = new Motor3PIDController(m_DriveP, m_DriveI, m_DriveD, RightEncoder, Front_Right_Motor, Middle_Right_Motor, Back_Right_Motor);
		LeftPID = new Motor3PIDController(m_DriveP, m_DriveI, m_DriveD, LeftEncoder, Front_Left_Motor, Middle_Left_Motor, Back_Left_Motor);
		RotatePID = new RotatePIDController(m_RotateP, m_RotateI, m_RotateD, m_gyro, Front_Left_Motor, Middle_Left_Motor, Back_Left_Motor, Front_Right_Motor, Middle_Right_Motor, Back_Right_Motor);
}

void DriveSubsystem::InitDefaultCommand()
{
	SetDefaultCommand(new DriveFromInput());
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

void DriveSubsystem::Drive(double speed, double turn)
{
	Front_Right_Motor->Set(-(speed + (turn/3)));
	Front_Left_Motor->Set(speed - (turn/3));
	Middle_Right_Motor->Set(-(speed + (turn/3)));
	Middle_Left_Motor->Set(speed - (turn/3));
	Back_Right_Motor->Set(-(speed + (turn/3)));
	Back_Left_Motor->Set(speed - (turn/3));
}

void DriveSubsystem::Shift(DoubleSolenoid::Value value)
{
	Gear_Box->Set(value);
}

double DriveSubsystem::GetLeftDistance()
{
	return LeftEncoder->GetDistance();
}
double DriveSubsystem::GetRightDistance()
{
	return RightEncoder->GetDistance();
}

void DriveSubsystem::LeftReset()
{
	LeftEncoder->Reset();
}
void DriveSubsystem::RightReset()
{
	RightEncoder->Reset();
}

void DriveSubsystem::ResetGyro()
{
	m_gyro->Reset();
}

double DriveSubsystem::GetGyroAngle()
{
	return m_gyro->GetAngle();
}

void DriveSubsystem::SetDrivePIDEnabled(bool enabled)
{
	RightPID->SetSetpoint(0.0f);
	LeftPID->SetSetpoint(0.0f);
	Drive(0, 0);
	RightPID->Reset();
	LeftPID->Reset();

	if (enabled)
	{
		RightPID->Enable();
		LeftPID->Enable();
	}
}
void DriveSubsystem::SetRotatePIDEnabled(bool enabled)
{
	RotatePID->SetSetpoint(0.0f);
	Drive(0, 0);
	RotatePID->Reset();

	if (enabled)
		RotatePID->Enable();
}

void DriveSubsystem::SetDrive(bool enabled, double setpoint)
{
	SetDrivePIDEnabled(enabled);
	RightPID->SetSetpoint(setpoint);
	LeftPID->SetSetpoint(setpoint);
}
void DriveSubsystem::SetRotate(bool enabled, double setpoint)
{
	SetDrivePIDEnabled(enabled);
	RotatePID->SetSetpoint(setpoint);
}
// Put methods for controlling this subsystem
// here. Call these from Commands.
