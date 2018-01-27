#ifndef LiftSubsystem_H
#define LiftSubsystem_H

#include <Commands/Subsystem.h>
#include <WPILib.h>

class LiftSubsystem : public Subsystem {
private:
	Talon* Lift_Motor;
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

public:
	LiftSubsystem();
	void SetSpeed(double speed);
	void InitDefaultCommand();
};

#endif  // LiftSubsystem_H
