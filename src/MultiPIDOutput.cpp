#include "MultiPIDOutput.h"
#include "WPILib.h"
void MultiPIDOutput::AddTalon(uint8_t port, bool inverted)
{
	m_talons.push_back(new Talon(port));
	m_talons.back()->SetInverted(inverted);
}

void MultiPIDOutput::AddTalon(Talon* pTalon)
{
	m_talons.push_back(pTalon);
}

void MultiPIDOutput::PIDWrite(float output)
{
	for (auto talon : m_talons)
		talon->Set(output);
}
