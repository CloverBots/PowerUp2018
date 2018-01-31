#ifndef DriveSubsystem_H
#define DriveSubsystem_H
#include <WPILib.h>
#include <Commands/Subsystem.h>
#include <Motor3PIDController.h>
#include <RotatePIDController.h>

class DriveSubsystem : public Subsystem {
private:
	const float m_DriveP = 0.75f;
	const float m_DriveI = 0.175f;
	const float m_DriveD = 0.0f;
	const float m_RotateP = 0.75f;
	const float m_RotateI = 0.175f;
	const float m_RotateD = 0.0f;
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
	Talon* Front_Right_Motor;
	Talon* Front_Left_Motor;
	Talon* Middle_Right_Motor;
	Talon* Middle_Left_Motor;
	Talon* Back_Right_Motor;
	Talon* Back_Left_Motor;
	DoubleSolenoid* Gear_Box;
	Encoder* LeftEncoder;
	Encoder* RightEncoder;
	AnalogGyro* m_gyro;
	Motor3PIDController* RightPID;
	Motor3PIDController* LeftPID;
	RotatePIDController* RotatePID;

public:
	DriveSubsystem();
	void Drive(double speed, double turn);
	void Shift(DoubleSolenoid::Value value);
	void InitDefaultCommand();
	double GetLeftDistance();
	double GetRightDistance();
	void LeftReset();
	void RightReset();
	void ResetGyro();
	double GetGyroAngle();
	void SetDrivePIDEnabled(bool enabled);
	void SetRotatePIDEnabled(bool enabled);
	void SetDrive(bool enabled, double setpoint);
	void SetRotate(bool enabled, double setpoint);
};


#endif  // DriveSubsystem_H
