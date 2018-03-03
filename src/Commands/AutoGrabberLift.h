/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <Commands/Command.h>
#include "../CommandBase.h"
#include "WPILib.h"
class AutoGrabberLift : public frc::Command {
	bool Done = false;
	double Speed = 0;
	double SetPoint = 0;
	const double MAX_SPEED = .5;
public:
	AutoGrabberLift(double SetPoint);
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
};

