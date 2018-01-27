#ifndef Rotate_H
#define Rotate_H

#include "../CommandBase.h"

class Rotate : public CommandBase {
	double m_targetAngle;

public:
	Rotate(double angle);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // Rotate_H
