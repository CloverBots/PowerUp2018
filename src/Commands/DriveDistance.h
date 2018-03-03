#ifndef DriveDistance_H
#define DriveDistance_H

#include "../CommandBase.h"
#include "WPILib.h"
class DriveDistance : public CommandBase {
	double Distance;
	float m_P = 0.011f;
	float m_I = 0.0f;
	float m_D = 0.1f;
public:
	DriveDistance(double distance, float P = 0.011, float I = 0.0, float D = 0.1); //inches
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveDistance_H
