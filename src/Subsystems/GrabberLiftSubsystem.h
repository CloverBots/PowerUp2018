
#ifndef GrabberLiftSubsystem_H
#define GrabberLiftSubsystem_H

#include <Commands/Subsystem.h>
#include "WPILib.h"
class GrabberLiftSubsystem : public Subsystem {
private:
	DigitalInput* Upper;
	DigitalInput* Lower;
	Encoder* GrabberLiftEncoder;
	Talon* GrabberMotor;
	PIDController* GrabberLiftPID;
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
