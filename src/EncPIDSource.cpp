/*
 * EncPIDSource.cpp
 *
 *  Created on: Feb 16, 2018
 *      Author: conso
 */

#include "EncPIDSource.h"
#include <ctre/Phoenix.h>
#include <iostream>

EncPIDSource::EncPIDSource(WPI_TalonSRX* pTalonRight, WPI_TalonSRX* pTalonLeft)
			: m_pTalonRight(pTalonRight), m_pTalonLeft(pTalonLeft)
{

}

EncPIDSource::~EncPIDSource()
{
}

double EncPIDSource::PIDGet()
{
	DistanceRight += (m_pTalonRight->GetSelectedSensorPosition(0) / 54.3702 / 21.6 - DistanceOldRight);
	DistanceOldRight = DistanceRight;
	DistanceLeft += (m_pTalonLeft->GetSelectedSensorPosition(0) / 54.3702 / 21.6 - DistanceOldLeft);
	DistanceOldLeft = DistanceLeft;
	std::cout << (DistanceRight + DistanceLeft) / 2 << std::endl;
	return (DistanceRight + DistanceLeft) / 2;
}
