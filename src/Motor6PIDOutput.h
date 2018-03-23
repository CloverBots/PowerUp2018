/*
 * Motor6PIDOutput.h
 *
 *  Created on: Mar 17, 2018
 *      Author: conso
 */

#ifndef Motor6PIDOutput_H_
#define Motor6PIDOutput_H_
#include <WPILib.h>
#include <ctre/Phoenix.h>

class Motor6PIDOutput : public PIDOutput{
private:
	double m_value;
public:
	Motor6PIDOutput();
	virtual ~Motor6PIDOutput();
	virtual void PIDWrite(double value);
	double GetValue();
};

#endif /* Motor6PIDOutput_H_ */
