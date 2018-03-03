#pragma once
#include "../CommandBase.h"
#include <WPILib.h>

class Lift : public CommandBase {
private:
	Joystick* pDriveStick;
	Joystick* pOperatorStick;
	JoystickButton* Rbumper;
	JoystickButton* Lbumper;
	JoystickButton* StartButton;
public:
	Lift();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	double liftspeed;
	double minispeed;
};
