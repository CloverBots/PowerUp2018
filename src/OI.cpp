/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "OI.h"

#include <WPILib.h>
#include "Commands/GrabberLift.h"
#include "Commands/Lift.h"

OI::OI() {
	pDriveStick = new Joystick(0);
	pOperatorStick = new Joystick(1);
	m_Camera = CameraServer::GetInstance()->StartAutomaticCapture(0);
	m_Camera.SetResolution(CAMERA_X_RES, CAMERA_Y_RES);
	m_Camera.SetFPS(30);

//	AButton->WhenPressed(new GrabberLift(0));//lower
//	BButton->WhenPressed(new GrabberLift(10));//middle
//	XButton->WhenPressed(new GrabberLift(100));//upper

}

Joystick* OI::GetDriveStick()
{
	return pDriveStick;
}
