#ifndef GrabberSubsystem_H
#define GrabberSubsystem_H

#include <WPILib.h>
#include <Commands/Subsystem.h>
#include <ctre/Phoenix.h>

class GrabberSubsystem : public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
	WPI_VictorSPX* Grabber_Motor_Right;
	WPI_VictorSPX* Grabber_Motor_Left;
public:
	GrabberSubsystem();
	void SetGrabberSpeed (double speed);
	void InitDefaultCommand();
};

#endif  // GrabberSubsystem_H
