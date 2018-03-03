/*
 * EncPIDSource.h
 *
 *  Created on: Feb 16, 2018
 *      Author: conso
 */

#ifndef SRC_ENCPIDSOURCE_H_
#define SRC_ENCPIDSOURCE_H_

#include "WPILib.h"
#include <ctre/Phoenix.h>

class EncPIDSource : public PIDSource
{
private:

	WPI_TalonSRX* m_pTalonRight;
	WPI_TalonSRX* m_pTalonLeft;
	float DistanceRight = 0;
	float DistanceLeft = 0;
	float DistanceOldRight = 0;
	float DistanceOldLeft = 0;
public:
	EncPIDSource(WPI_TalonSRX* pTalonRight, WPI_TalonSRX* pTalonLeft);
	void Reset();
	virtual ~EncPIDSource();
	virtual double PIDGet();
};

#endif /* SRC_ENCPIDSOURCE_H_ */
