#include "CommandBase.h"
#include <WPILib.h>
#include <Commands/Scheduler.h>

#include <iostream>
// Initialize a single static instance of all of your subsystems. The following
// line should be repeated for each subsystem in the project.



std::unique_ptr<OI> CommandBase::oi;
std::unique_ptr<LiftSubsystem> CommandBase::lift;

CommandBase::CommandBase(const std::string &name) :
		frc::Command(name) {

}

void CommandBase::Init()
{
	lift.reset(new LiftSubsystem);
	oi.reset(new OI);
}
