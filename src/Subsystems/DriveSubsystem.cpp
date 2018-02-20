#include <Commands/DriveFromInput.h>
#include "DriveSubsystem.h"
#include "../RobotMap.h"
#include <WPILib.h>
#include <iostream>

DriveSubsystem::DriveSubsystem() : Subsystem("driveSubsystem") {
		Front_Right_Motor = new WPI_TalonSRX(RobotMap::FRONT_RIGHT_MOTOR);
		Front_Left_Motor = new WPI_TalonSRX(RobotMap::FRONT_LEFT_MOTOR);
		Middle_Right_Motor = new WPI_TalonSRX(RobotMap::MIDDLE_RIGHT_MOTOR);
		Middle_Left_Motor = new WPI_TalonSRX(RobotMap::MIDDLE_LEFT_MOTOR);
		Back_Right_Motor = new WPI_TalonSRX(RobotMap::BACK_RIGHT_MOTOR);
		Back_Left_Motor = new WPI_TalonSRX(RobotMap::BACK_LEFT_MOTOR);
		Middle_Right_Motor->ConfigSelectedFeedbackSensor(phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0, 0);
		Back_Left_Motor->ConfigSelectedFeedbackSensor(phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0, 0);
		Middle_Right_Motor->GetSensorCollection().SetQuadraturePosition(0, 10);
		Back_Left_Motor->GetSensorCollection().SetQuadraturePosition(0, 10);
		Front_Left_Motor->SetInverted(true);
		Middle_Left_Motor->SetInverted(true);
		Back_Left_Motor->SetInverted(true);
		Gear_Box = new DoubleSolenoid(0, 1);
		m_gyro = new ADXRS450_Gyro(SPI::kOnboardCS0);
		Output = new EncPIDSource(Middle_Right_Motor, Back_Left_Motor);
	 	PID = new Motor3PIDController(m_DriveP, m_DriveI, m_DriveD, Output, Front_Right_Motor, Middle_Right_Motor, Back_Right_Motor, Front_Left_Motor, Middle_Left_Motor, Back_Left_Motor);
	 	RotatePID = new RotatePIDController(m_RotateP, m_RotateI, m_RotateD, m_gyro, Front_Left_Motor, Middle_Left_Motor, Back_Left_Motor, Front_Right_Motor, Middle_Right_Motor, Back_Right_Motor);
		PID->SetOutputRange(-1,1);
		PID->SetPIDSourceType(PIDSourceType::kDisplacement);
		PID->SetTolerance(1);
}

void DriveSubsystem::InitDefaultCommand()
{
	SetDefaultCommand(new DriveFromInput());
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

void DriveSubsystem::Drive(double speed, double turn)
{
	Front_Right_Motor->Set(-(speed + (turn/2)));
	Front_Left_Motor->Set(-speed + (turn/2));
	Middle_Right_Motor->Set(-(speed + (turn/2)));
	Middle_Left_Motor->Set(-speed + (turn/2));
	Back_Right_Motor->Set(-(speed + (turn/2)));
	Back_Left_Motor->Set(-speed + (turn/2));
}

void DriveSubsystem::Shift(DoubleSolenoid::Value value)
{
	Gear_Box->Set(value);
}

void DriveSubsystem::ResetGyro()
{
	m_gyro->Reset();
}

void DriveSubsystem::ResetDrive()
{
	Output->Reset();
}

double DriveSubsystem::GetGyroAngle()
{
	return m_gyro->GetAngle();
}

void DriveSubsystem::SetDrivePIDEnabled(bool enabled)
{

	PID->SetSetpoint(0.0f);
	Drive(0, 0);
	PID->Reset();

	if (enabled)
	{
		PID->Enable();
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
	PID->SetSetpoint(setpoint);
}
void DriveSubsystem::SetRotate(bool enabled, double setpoint)
{
	SetDrivePIDEnabled(enabled);
	RotatePID->SetSetpoint(setpoint);
}

bool DriveSubsystem::OnTarget()
{
	return PID->OnTarget();
}

void DriveSubsystem::SetPID(double P, double I, double D)
{
	m_DriveP = P;
	m_DriveI = I;
	m_DriveD = D;
	PID->SetPID(P, I, D);
}

void DriveSubsystem::UpdateFromSmartDashboard()
{
	SetPID(
			(float)SmartDashboard::GetNumber("Drive P", 0.0),
			(float)SmartDashboard::GetNumber("Drive I", 0.0),
			(float)SmartDashboard::GetNumber("Drive D", 0.0));

}

double DriveSubsystem::GetDistance()
{
	DistanceRight += (Middle_Right_Motor->GetSelectedSensorPosition(0) / 54.3702 / 21.6 - DistanceOldRight);
	DistanceOldRight = DistanceRight;
	DistanceLeft += (Back_Left_Motor->GetSelectedSensorPosition(0) / 54.3702 / 21.6 - DistanceOldLeft);
	DistanceOldLeft = DistanceLeft;
	return (DistanceRight + DistanceLeft) / 2;
}
