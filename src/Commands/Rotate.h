#ifndef Rotate_H
#define Rotate_H

#include "../CommandBase.h"

class Rotate : public CommandBase {
	double m_targetAngle;
	float m_P = 0.012f;
	float m_I = 0.0f;
	float m_D = 0.01f;
public:
	Rotate(double angle, float P = 0.008f, float I = 0.0, float D = 0.15);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // Rotate_H
