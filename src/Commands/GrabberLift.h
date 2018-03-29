#ifndef GrabberLift_H
#define GrabberLift_H

#include "../CommandBase.h"
#include "WPILib.h"
class GrabberLift : public CommandBase {
private:
	Joystick* pOperatorStick;
	JoystickButton* AButton;
	JoystickButton* BButton;
	JoystickButton* XButton;
	JoystickButton* LButton;
	JoystickButton* RButton;
public:
	double Speed = 0;
	double SetPoint = 0;
	const double MAX_SPEED = .8;
	double acceleration = 1;
	GrabberLift();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
};

#endif  // GrabberLift_H
