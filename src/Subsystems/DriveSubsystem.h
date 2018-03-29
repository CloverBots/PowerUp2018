#ifndef DriveSubsystem_H
#define DriveSubsystem_H
#include <WPILib.h>
#include <Commands/Subsystem.h>
#include <Motor3PIDController.h>
#include <RotatePIDController.h>
#include <ctre/Phoenix.h>
#include "../EncPIDSource.h"
#include "../Motor6PIDOutput.h"

class DriveSubsystem : public Subsystem {
private:
	float DistanceRight = 0;
	float DistanceLeft = 0;
	float DistanceOldRight = 0;
	float DistanceOldLeft = 0;
	const float m_WheelDiameter = 6;
	const float m_WheelCircumference = m_WheelDiameter * M_PI;
	const float m_EncScaler = 2.8444;
	float m_DriveP = 0.015f;
	float m_DriveI = 0.0f;
	float m_DriveD = 0.1f;
	float m_RotateP = 0.02f;
	float m_RotateI = 0.0f;
	float m_RotateD = 0.01f;
	EncPIDSource* Source;
	Motor6PIDOutput* DriveOutput;
	Motor6PIDOutput* RotateOutput;
	WPI_TalonSRX* Front_Right_Motor;
	WPI_TalonSRX* Front_Left_Motor;
	WPI_TalonSRX* Middle_Right_Motor;
	WPI_TalonSRX* Middle_Left_Motor;
	WPI_TalonSRX* Back_Right_Motor;
	WPI_TalonSRX* Back_Left_Motor;
	DoubleSolenoid* Gear_Box;
	ADXRS450_Gyro* m_gyro;
	PIDController* DrivePID;
	PIDController* DriveRotatePID;
	RotatePIDController* RotatePID;
public:
	DriveSubsystem();
	void Drive(double speed, double turn);
	void Shift(DoubleSolenoid::Value value);
	void InitDefaultCommand();
	void ResetGyro();
	void ResetDrive();
	double GetDistance();
	double GetGyroAngle();
	void SetDrivePIDEnabled(bool enabled);
	void SetRotatePIDEnabled(bool enabled);
	void SetDriveRotatePIDEnabled(bool enabled);
	void SetDriveRotate(bool enabled, double setpoint);
	void SetDrive(bool enabled, double setpoint);
	void SetRotate(bool enabled, double setpoint);
	bool OnTarget();
	bool RotateOnTarget();
	void SetPID(double P, double I, double D);
	void SetRotatePID(double P, double I, double D);
	void UpdateFromSmartDashboard();
	void DisableAllPID();
	void AutoDrivePID();
};


#endif  // DriveSubsystem_H
