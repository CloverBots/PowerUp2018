#ifndef DriveSubsystem_H
#define DriveSubsystem_H
#include <WPILib.h>
#include <Commands/Subsystem.h>
#include <Motor3PIDController.h>
#include <RotatePIDController.h>
#include <ctre/Phoenix.h>
#include "../EncPIDSource.h"

class DriveSubsystem : public Subsystem {
private:
	const float m_WheelDiameter = 6;
	const float m_WheelCircumference = m_WheelDiameter * M_PI;
	const float m_EncScaler = 2.8444;
	const float m_DriveP = 0.015f;
	const float m_DriveI = 0.000f;
	const float m_DriveD = 0.0f;
	const float m_RotateP = 0.0f;
	const float m_RotateI = 0.0f;
	const float m_RotateD = 0.0f;
	EncPIDSource* Output;
	WPI_TalonSRX* Front_Right_Motor;
	WPI_TalonSRX* Front_Left_Motor;
	WPI_TalonSRX* Middle_Right_Motor;
	WPI_TalonSRX* Middle_Left_Motor;
	WPI_TalonSRX* Back_Right_Motor;
	WPI_TalonSRX* Back_Left_Motor;
	DoubleSolenoid* Gear_Box;
	ADXRS450_Gyro* m_gyro;
	Motor3PIDController* PID;
	RotatePIDController* RotatePID;

public:
	DriveSubsystem();
	void Drive(double speed, double turn);
	void Shift(DoubleSolenoid::Value value);
	void InitDefaultCommand();
	void LeftReset();
	void RightReset();
	void ResetGyro();
	double GetGyroAngle();
	void SetDrivePIDEnabled(bool enabled);
	void SetRotatePIDEnabled(bool enabled);
	void SetDrive(bool enabled, double setpoint);
	void SetRotate(bool enabled, double setpoint);
	bool OnTarget();
};


#endif  // DriveSubsystem_H
