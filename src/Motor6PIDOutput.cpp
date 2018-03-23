/*
 * Motor6PIDOutput.cpp
 *
 *  Created on: Mar 17, 2018
 *      Author: conso
 */

#include <Motor6PIDOutput.h>

Motor6PIDOutput::Motor6PIDOutput()
{
	m_value = 0;
}

Motor6PIDOutput::~Motor6PIDOutput() {

}

void Motor6PIDOutput::PIDWrite(double value)
{
	m_value = value;
}

double Motor6PIDOutput::GetValue()
{
	return m_value;
}
