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

		Front_Right_Motor->ConfigOpenloopRamp(0.25, 0);
		Front_Left_Motor->ConfigOpenloopRamp(0.25, 0);
		Middle_Right_Motor->ConfigOpenloopRamp(0.25, 0);
		Middle_Left_Motor->ConfigOpenloopRamp(0.25, 0);
		Back_Right_Motor->ConfigOpenloopRamp(0.25, 0);
		Back_Left_Motor->ConfigOpenloopRamp(0.25, 0);

//		Front_Right_Motor->ConfigContinuousCurrentLimit(30, 10);
//		Front_Left_Motor->ConfigContinuousCurrentLimit(30, 10);
//		Middle_Right_Motor->ConfigContinuousCurrentLimit(30, 10);
//		Middle_Left_Motor->ConfigContinuousCurrentLimit(30, 10);
//		Back_Right_Motor->ConfigContinuousCurrentLimit(30, 10);
//		Back_Left_Motor->ConfigContinuousCurrentLimit(30, 10);
//
//		Front_Right_Motor->ConfigPeakCurrentLimit(0, 10);
//		Front_Left_Motor->ConfigPeakCurrentLimit(0, 10);
//		Middle_Right_Motor->ConfigPeakCurrentLimit(0, 10);
//		Middle_Left_Motor->ConfigPeakCurrentLimit(0, 10);
//		Back_Right_Motor->ConfigPeakCurrentLimit(0, 10);
//		Back_Left_Motor->ConfigPeakCurrentLimit(0, 10);


		Middle_Right_Motor->ConfigSelectedFeedbackSensor(phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0, 0);
		Back_Left_Motor->ConfigSelectedFeedbackSensor(phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0, 0);
		Middle_Right_Motor->GetSensorCollection().SetQuadraturePosition(0, 10);
		Back_Left_Motor->GetSensorCollection().SetQuadraturePosition(0, 10);
		Front_Left_Motor->SetInverted(true);
		Middle_Left_Motor->SetInverted(true);
		Back_Left_Motor->SetInverted(true);
		Gear_Box = new DoubleSolenoid(0, 1);
		m_gyro = new ADXRS450_Gyro(SPI::kOnboardCS0);
		Source = new EncPIDSource(Middle_Right_Motor, Back_Left_Motor);
		DriveOutput = new Motor6PIDOutput();
		RotateOutput = new Motor6PIDOutput();
		DrivePID = new PIDController(m_DriveP, m_DriveI, m_DriveD, Source, DriveOutput);
	 	RotatePID = new RotatePIDController(m_RotateP, m_RotateI, m_RotateD, m_gyro, Front_Left_Motor, Middle_Left_Motor, Back_Left_Motor, Front_Right_Motor, Middle_Right_Motor, Back_Right_Motor);
	 	DriveRotatePID = new PIDController(m_RotateP, m_RotateI, m_RotateD, m_gyro, RotateOutput);
	 	DrivePID->SetOutputRange(-1,1);
	 	DrivePID->SetPIDSourceType(PIDSourceType::kDisplacement);
	 	DrivePID->SetAbsoluteTolerance(5);
		RotatePID->SetOutputRange(-.6,.6);
		RotatePID->SetAbsoluteTolerance(4);
		RotatePID->SetPIDSourceType(PIDSourceType::kDisplacement);
		DriveRotatePID->SetOutputRange(-.6,.6);
		DriveRotatePID->SetAbsoluteTolerance(4);
		DriveRotatePID->SetPIDSourceType(PIDSourceType::kDisplacement);
}

void DriveSubsystem::InitDefaultCommand()
{
	SetDefaultCommand(new DriveFromInput());
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

void DriveSubsystem::Drive(double speed, double turn)
{
	Front_Right_Motor->Set(-(speed + (turn / 2)));
	Front_Left_Motor->Set(-speed + (turn / 2));
	Middle_Right_Motor->Set(-(speed + (turn / 2)));
	Middle_Left_Motor->Set(-speed + (turn / 2));
	Back_Right_Motor->Set(-(speed + (turn / 2)));
	Back_Left_Motor->Set(-speed + (turn / 2));
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
	Source->Reset();
}

double DriveSubsystem::GetGyroAngle()
{
	return m_gyro->GetAngle();
}

void DriveSubsystem::SetDrivePIDEnabled(bool enabled)
{

	DrivePID->SetSetpoint(0.0f);
	Drive(0, 0);
	DrivePID->Reset();

	if (enabled)
	{
		DrivePID->Enable();
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

void DriveSubsystem::SetDriveRotatePIDEnabled(bool enabled)
{
	DriveRotatePID->SetSetpoint(0.0f);
	Drive(0, 0);
	DriveRotatePID->Reset();

	if (enabled)
		DriveRotatePID->Enable();
}

void DriveSubsystem::SetDrive(bool enabled, double setpoint)
{
	SetRotatePIDEnabled(false);
	SetDrivePIDEnabled(enabled);
	DrivePID->SetSetpoint(setpoint);
}
void DriveSubsystem::SetRotate(bool enabled, double setpoint)
{
	SetDrivePIDEnabled(false);
	SetRotatePIDEnabled(enabled);
	RotatePID->SetSetpoint(setpoint);
}

bool DriveSubsystem::OnTarget()
{
	return DrivePID->OnTarget();
}

void DriveSubsystem::SetDriveRotate(bool enabled, double setpoint)
{
	SetDriveRotatePIDEnabled(enabled);
	DriveRotatePID->SetSetpoint(setpoint);
}

void DriveSubsystem::SetPID(double P, double I, double D)
{
	m_DriveP = P;
	m_DriveI = I;
	m_DriveD = D;
	DrivePID->SetPID(P, I, D);
}

void DriveSubsystem::SetRotatePID(double P, double I, double D)
{
	m_RotateP = P;
	m_RotateI = I;
	m_RotateD = D;
	RotatePID->SetPID(P, I, D);
}

void DriveSubsystem::UpdateFromSmartDashboard()
{
	SmartDashboard::PutNumber("Gyro", m_gyro->GetAngle());
}

double DriveSubsystem::GetDistance()
{
	DistanceRight += (Middle_Right_Motor->GetSelectedSensorPosition(0) / 54.3702 / 21.6 - DistanceOldRight);
	DistanceOldRight = DistanceRight;
	DistanceLeft += (Back_Left_Motor->GetSelectedSensorPosition(0) / 54.3702 / 21.6 - DistanceOldLeft);
	DistanceOldLeft = DistanceLeft;
	return (DistanceRight + DistanceLeft) / 2;
}

bool DriveSubsystem::RotateOnTarget()
{
	return RotatePID->OnTarget();
}

void DriveSubsystem::DisableAllPID()
{
	RotatePID->Disable();
	DrivePID->Disable();
}

void DriveSubsystem::AutoDrivePID()
{
	std::cout << "Rotate: " << RotateOutput->GetValue() * 2 << std::endl;
	Front_Right_Motor->Set((DriveOutput->GetValue() + (RotateOutput->GetValue() * 2)));
	Front_Left_Motor->Set(DriveOutput->GetValue() + (RotateOutput->GetValue() * 2));
	Middle_Right_Motor->Set((DriveOutput->GetValue() + (RotateOutput->GetValue() * 2)));
	Middle_Left_Motor->Set(DriveOutput->GetValue() + (RotateOutput->GetValue() * 2));
	Back_Right_Motor->Set((DriveOutput->GetValue() + (RotateOutput->GetValue() * 2)));
	Back_Left_Motor->Set(DriveOutput->GetValue() + (RotateOutput->GetValue() * 2));
}
