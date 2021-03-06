/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <Commands/CommandGroup.h>
#include "DriveDistance.h"
#include "Rotate.h"
#include "AutoGrabberLift.h"
#include "AutoGrabber.h"
#include "DelayCommand.h"
#include "AutoDrive.h"
class DoubleCenterRight : public frc::CommandGroup {
public:
	DoubleCenterRight();
};

