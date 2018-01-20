
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
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

public:
	GrabberLiftSubsystem();
	void InitDefaultCommand();
	void SetSpeed(double speed);
	double GetDistance();
	bool GetUpper();
	bool GetLower();
};

#endif  // GrabberLiftSubsystem_H
