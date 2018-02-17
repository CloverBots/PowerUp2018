/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once



/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */

// For example to map the left and right motors, you could define the
// following variables to use with your drivetrain subsystem.
// constexpr int kLeftMotor = 1;
// constexpr int kRightMotor = 2;

// If you are using multiple modules, make sure to define both the port
// number and the module. For example you with a rangefinder:
// constexpr int kRangeFinderPort = 1;
// constexpr int kRangeFinderModule = 1;
class RobotMap
{
public:
	const static unsigned int
		FRONT_RIGHT_MOTOR = 8,
		FRONT_LEFT_MOTOR = 5,
		MIDDLE_RIGHT_MOTOR = 9,
		MIDDLE_LEFT_MOTOR = 6,
		BACK_RIGHT_MOTOR = 11,
		BACK_LEFT_MOTOR = 7,
		GRABBER_LIFT_MOTOR = 1,
		LIFT_MOTOR_RIGHT = 2,
		LIFT_MOTOR_LEFT = 12,
		LIFT_MOTOR_UP = 10,
		GRABBER_MOTOR_RIGHT = 3,
		GRABBER_MOTOR_LEFT = 4;
};
