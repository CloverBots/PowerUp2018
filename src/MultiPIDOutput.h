#ifndef SRC_MULTIPIDOUTPUT_H_
#define SRC_MULTIPIDOUTPUT_H_

#include <vector>

#include "WPILib.h"

class MultiPIDOutput : public PIDOutput {
private:
	std::vector<Talon*> m_talons;
public:

	MultiPIDOutput() { }
	virtual ~MultiPIDOutput() { }

	virtual void PIDWrite(float output);

	void AddTalon(uint8_t port, bool inverted = false);
	void AddTalon(Talon* pTalon);
};

#endif /* SRC_MULTIPIDOUTPUT_H_ */
