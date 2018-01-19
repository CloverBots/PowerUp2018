/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "OI.h"
#include "Lift.h"
#include <WPILib.h>

OI::OI() {
	driver_joystick = new Joystick(0);
	AButton = new JoystickButton(driver_joystick, 0);
	// Process operator interface input here.
}
