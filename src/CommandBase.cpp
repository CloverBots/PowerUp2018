#include "CommandBase.h"

#include <Commands/Scheduler.h>
#include "Subsystems/GrabberSubsystem.h"
#include <iostream>
// Initialize a single static instance of all of your subsystems. The following
// line should be repeated for each subsystem in the project.

std::unique_ptr<OI> CommandBase::oi;
std::unique_ptr<GrabberSubsystem> Grab;
CommandBase::CommandBase(const std::string &name) :
		frc::Command(name) {

}

void CommandBase::Init()
{
	grabber.reset (new GrabberSubsystem);
	oi.reset(new OI);
}
