/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include <WPILib.h>
#pragma once

class OI {
public:
	const static int CAMERA_X_RES = 160;
	const static int CAMERA_Y_RES = 120;
	//cs::UsbCamera m_Camera;
	Joystick* pDriveStick;
	Joystick* pOperatorStick;
	JoystickButton* Rbumper;
	JoystickButton* Lbumper;
	JoystickButton* AButton;
	JoystickButton* BButton;
	JoystickButton* XButton;
	Joystick* GetDriveStick();
	OI();
};
