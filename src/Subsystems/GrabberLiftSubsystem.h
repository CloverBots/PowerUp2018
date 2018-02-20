
#ifndef GrabberLiftSubsystem_H
#define GrabberLiftSubsystem_H

#include <Commands/Subsystem.h>
#include "WPILib.h"
#include <ctre/Phoenix.h>
#include "../DoublePIDController.h"
class GrabberLiftSubsystem : public Subsystem {
private:
	DigitalInput* Upper;
	DigitalInput* Lower;
	WPI_TalonSRX* GrabberMotor;

public:
	GrabberLiftSubsystem();
	void InitDefaultCommand();
	void SetSpeed(double speed);
	void Reset();
	double GetDistance();
	bool GetUpper();
	bool GetLower();
};

#endif  // GrabberLiftSubsystem_H
