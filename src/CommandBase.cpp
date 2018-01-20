#include "CommandBase.h"
#include <Commands/Scheduler.h>
#include "Subsystems/GrabberLiftSubsystem.h"
#include <iostream>
#include "WPILib.h"
// Initialize a single static instance of all of your subsystems. The following
// line should be repeated for each subsystem in the project.

std::unique_ptr<GrabberLiftSubsystem> CommandBase::grabberLiftSubsystem;


std::unique_ptr<OI> CommandBase::oi;

CommandBase::CommandBase(const std::string &name) : frc::Command(name)
{

}

void CommandBase::Init()
{
	grabberLiftSubsystem.reset(new GrabberLiftSubsystem);
	oi.reset(new OI);
}
