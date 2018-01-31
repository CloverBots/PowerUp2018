/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include <wpilib.h>
#include <Commands/Command.h>
#include <Commands/Scheduler.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <TimedRobot.h>
#include "Commands/DriveToSwitchAutoLeft.h"
#include "Commands/DriveToSwitchAutoRight.h"
#include "Commands/DriveToSwitchForwardLeft.h"
#include "Commands/DriveToSwitchForwardRight.h"
#include "Commands/DriveForwardAuto.h"

class Robot : public frc::TimedRobot {
public:
	Compressor *c = new Compressor(0);
	void RobotInit() override {
		CommandBase::Init();
		c->Start();
		c->SetClosedLoopControl(true);
		m_chooser.AddDefault("DriveForward", new DriveForwardAuto);
		m_chooser.AddObject("DriveToSwitch", new DriveForwardAuto);
		m_chooser.AddObject("DriveToSwtichForwardRight", new DriveForwardAuto);
		m_chooser.AddObject("DriveToSwtichForwardLeft", new DriveForwardAuto);
	}

	/**
	 * This function is called once each time the robot enters Disabled
	 * mode.
	 * You can use it to reset any subsystem information you want to clear
	 * when
	 * the robot is disabled.
	 */
	void DisabledInit() override {}

	void DisabledPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to
	 * select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the DIE->Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, (hi this was not saposed to be here) remove all of the chooser code and uncomment the
	 * GetString code to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional commands to
	 * the
	 * chooser code above (like the commented example) or additional
	 * comparisons
	 * to the if-else structure below with additional strings & commands.
	 */
	void AutonomousInit() override {
		autonomousCommand.reset(m_chooser.GetSelected());
		if(m_chooser.GetSelected()->GetName() == "DriveToSwitch")
		{
			std::string gameData;
			gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
			if(gameData[0] == 'L')
			{
				autonomousCommand.reset(new DriveToSwitchAutoLeft);
			}else if(gameData[0] == 'R')
			{
				autonomousCommand.reset(new DriveToSwitchAutoRight);
			}else{

			}
		}else if(m_chooser.GetSelected()->GetName() == "DriveToSwitchForwardRight")
		{
			std::string gameData;
			gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
			if(gameData[0] == 'L')
			{
				autonomousCommand.reset(new DriveForwardAuto);
			}else if(gameData[0] == 'R')
			{
				autonomousCommand.reset(new DriveToSwitchForwardRight);
			}else{

			}

		}else if(m_chooser.GetSelected()->GetName() == "DriveToSwitchForwardLeft"){
			std::string gameData;
			gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
			if(gameData[0] == 'R')
			{
				autonomousCommand.reset(new DriveForwardAuto);
			}else if(gameData[0] == 'L')
			{
				autonomousCommand.reset(new DriveToSwitchForwardLeft);
			}else{

			}
		}else{
			if (autonomousCommand.get() != nullptr) {
				autonomousCommand->Start();
			}
		}
	}

	void AutonomousPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
	}

	void TeleopInit() override {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != nullptr) {
			autonomousCommand->Cancel();
			autonomousCommand = nullptr;
		}
	}

	void TeleopPeriodic() override { frc::Scheduler::GetInstance()->Run(); }

	void TestPeriodic() override {}

private:
	// Have it null by default so that if testing teleop it
	// doesn't have undefined behavior and potentially crash.
	std::unique_ptr<frc::Command> autonomousCommand = nullptr;	//ExampleCommand m_defaultAuto;
	//MyAutoCommand m_myAuto;
	frc::SendableChooser<frc::Command*> m_chooser;
};

START_ROBOT_CLASS(Robot)
