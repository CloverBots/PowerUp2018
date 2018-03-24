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
#include "Commands/DriveForwardAuto.h"
#include "Commands/Rotate.h"
#include "Commands/Left90SwitchAuto.h"
#include "Commands/Right90SwitchAuto.h"
#include "Commands/LeftAroundSwitch.h"
#include "Commands/RightAroundSwitch.h"
#include "Commands/ScaleLeft.h"
#include "Commands/ScaleRight.h"
#include "Commands/CenterLeft.h"
#include "Commands/CenterRight.h"
#include "Commands/DoubleCubeAutoLeft.h"
#include "Commands/DoubleCubeAutoRight.h"
#include <iostream>

class Robot : public frc::TimedRobot {
public:
	Compressor *c = new Compressor(0);
	void RobotInit() override {
		CommandBase::Init();
		c->Start();
		c->SetClosedLoopControl(true);
//		m_chooser.AddDefault("DriveForward", new DriveForwardAuto);
//		m_chooser.AddObject("Center", new RunCenter);
//		m_chooser.AddObject("Right90Switch", new Right90SwitchAuto);
//		m_chooser.AddObject("Left90Switch", new Left90SwitchAuto);
//		m_chooser.AddObject("ScaleRight", new ScaleRight);
//		m_chooser.AddObject("ScaleLeft", new ScaleLeft);
//		m_chooser.AddObject("Rotate", new Rotate(90));
		m_chooser.AddDefault("DriveForward", "DriveForwardAuto");
		m_chooser.AddObject("Center", "CenterAuto");
		m_chooser.AddObject("Right90Switch", "Right90SwitchAuto");
		m_chooser.AddObject("Left90Switch", "Left90SwitchAuto");
		m_chooser.AddObject("ScaleRight", "ScaleRight");
		m_chooser.AddObject("ScaleLeft", "ScaleLeft");
		m_chooser.AddObject("DoubleCubeAutoRight", "DoubleCubeAutoRight");
		m_chooser.AddObject("DoubleCubeAutoLeft", "DoubleCubeAutoLeft");
		frc::SmartDashboard::PutData("Auto Chooser" , &m_chooser);
		frc::SmartDashboard::PutNumber("Gyro", 0.0);
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
		std::cout << "RUNNING AUTO YAY!" << std::endl;
		CommandBase::grabberLiftSubsystem->Reset();
		CommandBase::driveSubsystem->ResetDrive();
		CommandBase::driveSubsystem->ResetGyro();
		c->Start();
		std::string gameData;
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		if(m_chooser.GetSelected() == "DriveForwardAuto")
		{
			autonomousCommand.reset(new DriveForwardAuto);
		}
		autonomousCommand.reset(new DriveForwardAuto);
		if(gameData[1] == 'R')
		{
			if(m_chooser.GetSelected() == "ScaleRight")
			{
				autonomousCommand.reset(new ScaleRight);
				goto skip_switch;
			}
		}
		if(gameData[1] == 'L')
		{
			if(m_chooser.GetSelected() == "ScaleLeft")
			{
				autonomousCommand.reset(new ScaleLeft);
				goto skip_switch;
			}
		}
		if(gameData[0] == 'R')
		{
			if(m_chooser.GetSelected() == "ScaleRight")
			{
				autonomousCommand.reset(new Right90SwitchAuto);
			}
			if(m_chooser.GetSelected() == "CenterAuto")
			{
				autonomousCommand.reset(new CenterRight);
			}
			if(m_chooser.GetSelected() == "Right90SwitchAuto")
			{
				std::cout << "Right90SwitchAuto" << std::endl;
				autonomousCommand.reset(new Right90SwitchAuto);
			}
			if(m_chooser.GetSelected() == "Left90SwitchAuto")
			{
				autonomousCommand.reset(new LeftAroundSwitch);
			}
			if(m_chooser.GetSelected() == "DoubleCubeAutoRight")
			{
				autonomousCommand.reset(new DoubleCubeAutoRight);
			}
			if(m_chooser.GetSelected() == "DoubleCubeAutoLeft")
			{
				autonomousCommand.reset(new DriveForwardAuto);
			}
		}
		else if(gameData[0] == 'L')
		{
			if(m_chooser.GetSelected() == "ScaleRight")
			{
				autonomousCommand.reset(new Left90SwitchAuto);
			}
			if(m_chooser.GetSelected() == "CenterAuto")
			{
				autonomousCommand.reset(new CenterLeft);
			}
			if(m_chooser.GetSelected() == "Right90SwitchAuto")
			{
				autonomousCommand.reset(new RightAroundSwitch);
			}
			if(m_chooser.GetSelected() == "Left90SwitchAuto")
			{
				autonomousCommand.reset(new Left90SwitchAuto);
			}
			if(m_chooser.GetSelected() == "DoubleCubeAutoLeft")
			{
				autonomousCommand.reset(new DoubleCubeAutoLeft);
			}
			if(m_chooser.GetSelected() == "DoubleCubeAutoRight")
			{
				autonomousCommand.reset(new DriveForwardAuto);
			}
		}
		skip_switch:
		if (autonomousCommand.get() != nullptr) {
			autonomousCommand->Start();
		}
	}

	void AutonomousPeriodic() override {
		CommandBase::driveSubsystem->UpdateFromSmartDashboard();
		frc::Scheduler::GetInstance()->Run();
	}

	void TeleopInit() override {

		CommandBase::driveSubsystem->DisableAllPID();
		CommandBase::grabberLiftSubsystem->Reset();
		CommandBase::driveSubsystem->ResetDrive();
		CommandBase::driveSubsystem->SetDrivePIDEnabled(false);
		CommandBase::driveSubsystem->SetRotatePIDEnabled(false);
		c->Start();
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != nullptr) {
			autonomousCommand->Cancel();
			autonomousCommand = nullptr;
		}
	}

	void TeleopPeriodic() override
	{
		CommandBase::driveSubsystem->UpdateFromSmartDashboard();
		frc::Scheduler::GetInstance()->Run();
	}

	void TestPeriodic() override {}

private:
	// Have it null by default so that if testing teleop it
	// doesn't have undefined behavior and potentially crash.
	std::unique_ptr<frc::Command> autonomousCommand = nullptr;	//ExampleCommand m_defaultAuto;
	//MyAutoCommand m_myAuto;
	frc::SendableChooser<std::string> m_chooser;
};

START_ROBOT_CLASS(Robot)
