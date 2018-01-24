/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "OI.h"

#include <WPILib.h>
#include <Commands/Grabber.h>
OI::OI() {
	driver_joystick = new Joystick(0);
	Abutton = new JoystickButton(driver_joystick,0);
	Bbutton = new JoystickButton(driver_joystick,1);
	Abutton ->WhenPressed(new Grabber(1));
	Abutton ->WhenPressed(new Grabber(0));
	Bbutton ->WhenPressed(new Grabber(-1));
	Bbutton ->WhenPressed(new Grabber(0));
	// Process operator interface input here.
}
