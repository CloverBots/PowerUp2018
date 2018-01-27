#include "CommandBase.h"

#include <Commands/Scheduler.h>

#include "Subsystems/DriveSubsystem.h"
#include <iostream>
// Initialize a single static instance of all of your subsystems. The following
// line should be repeated for each subsystem in the project.

std::unique_ptr<DriveSubsystem> CommandBase::driveSubsystem;
std::unique_ptr<GrabberSubsystem> CommandBase::grabber;
std::unique_ptr<LiftSubsystem> CommandBase::lift;
std::unique_ptr<GrabberLiftSubsystem> CommandBase::grabberLiftSubsystem;
std::unique_ptr<OI> CommandBase::oi;

CommandBase::CommandBase(const std::string &name) :
		frc::Command(name) {

}

void CommandBase::Init()
{
	grabber.reset (new GrabberSubsystem);
	driveSubsystem.reset(new DriveSubsystem);
	lift.reset(new LiftSubsystem);
	grabberLiftSubsystem.reset(new GrabberLiftSubsystem);
	oi.reset(new OI);
}
