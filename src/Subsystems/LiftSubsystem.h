#ifndef LiftSubsystem_H
#define LiftSubsystem_H

#include <Commands/Subsystem.h>
#include <WPILib.h>
#include <ctre/Phoenix.h>

class LiftSubsystem : public Subsystem {
private:
	WPI_VictorSPX* Lift_Motor_Left;
	WPI_VictorSPX* Lift_Motor_Right;
	WPI_TalonSRX* Lift_Motor_Up;
	DigitalInput* Limit;
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
public:
	LiftSubsystem();
	void SetSpeed(double liftspeed, double minispeed);
	void InitDefaultCommand();
	bool GetLimit();
};

#endif  // LiftSubsystem_H
