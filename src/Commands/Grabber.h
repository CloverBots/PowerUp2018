#ifndef Grabber_H
#define Grabber_H

#include "../CommandBase.h"

class Grabber : public CommandBase {
private:
	Joystick* pOperatorStick;
	JoystickButton* YButton;
	JoystickButton* BackButton;
	JoystickButton* StartButton;
	double speed;
public:
	Grabber();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // Grabber_H
