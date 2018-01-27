/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "OI.h"

#include <WPILib.h>
#include "Commands/Grabber.h"
#include "Commands/GrabberLift.h"
#include "Commands/Lift.h"

OI::OI() {
	pDriveStick = new Joystick(0);
	pOperatorStick = new Joystick(1);
	m_Camera = CameraServer::GetInstance()->StartAutomaticCapture(0);
	m_Camera.SetResolution(CAMERA_X_RES, CAMERA_Y_RES);
	m_Camera.SetFPS(30);
	Rbumper = new JoystickButton(pOperatorStick, 0);
	Lbumper = new JoystickButton(pOperatorStick, 1);
	AButton = new JoystickButton(pOperatorStick, 0);
	BButton = new JoystickButton(pOperatorStick, 1);
	XButton = new JoystickButton(pOperatorStick, 2);
	Rbumper->WhenPressed(new Grabber(1));
	Rbumper->WhenReleased(new Grabber(0));
	Lbumper->WhenPressed(new Grabber(-1));
	Lbumper->WhenReleased(new Grabber(0));
	AButton->WhenPressed(new GrabberLift(0));//lower limit
	BButton->WhenPressed(new GrabberLift(50));//upper limit
	XButton->WhenPressed(new GrabberLift(25));//middle limit
	// Process operator interface input here.
}

Joystick* OI::GetDriveStick()
{
	return pDriveStick;
}
