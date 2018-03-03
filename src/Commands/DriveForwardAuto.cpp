#include "DriveForwardAuto.h"
#include "CommandBase.h"
#include <iostream>
DriveForwardAuto::DriveForwardAuto() {
	std::string gameData;
	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	std::cout << "Side: " << gameData << std::endl;
	AddSequential(new DriveDistance(140));
	std::cout << "Side: " << gameData << std::endl;
}
