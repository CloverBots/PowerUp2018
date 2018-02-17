
#ifndef GrabberLiftSubsystem_H
#define GrabberLiftSubsystem_H

#include <Commands/Subsystem.h>
#include "WPILib.h"
#include <ctre/Phoenix.h>
#include "../DoublePIDController.h"
class GrabberLiftSubsystem : public Subsystem {
private:
	const float m_EncScaler = 555.555;
	double speed;
	double distance;
	DigitalInput* Upper;
	DigitalInput* Lower;
	WPI_VictorSPX* GrabberMotor;
	DoublePIDController* GrabberLiftPID;
	const int m_P = 1, m_I = 0, m_D = 0;
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

public:
	GrabberLiftSubsystem();
	void InitDefaultCommand();
	void SetSpeed(double speed);
	double GetDistance();
	bool GetUpper();
	bool GetLower();
	double GetSetPoint();
	void SetPIDEnabled(bool enabled);
	void SetPID(bool enabled, double setpoint);
};

#endif  // GrabberLiftSubsystem_H
