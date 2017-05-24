#include "srStateJoint.h"


srStateJoint::srStateJoint(void)
:srJoint()
{

}


srStateJoint::~srStateJoint(void)
{
}

bool srStateJoint::IsPostionLimited()
{
	return m_IsPosLimited;
}

void srStateJoint::MakePositionLimit(bool v)
{
	m_IsPosLimited = v;
}

double srStateJoint::GetPositionLowerLimit()
{
	return m_PosLimit[0];
}

double srStateJoint::GetPositionUpperLimit()
{
	return m_PosLimit[1];
}

void srStateJoint::SetPositionLimit(double lowerlimit, double upperlimit)
{
	if (lowerlimit < upperlimit)
	{
		m_PosLimit[0] = lowerlimit;
		m_PosLimit[1] = upperlimit;
	}
}
double srStateJoint::GetTorqueLowerLimit()
{
	return m_TorqueLimit[0];
}

double srStateJoint::GetTorqueUpperLimit()
{
	return m_TorqueLimit[1];
}

void srStateJoint::SetTorqueLimit(double lowerlimit, double upperlimit)
{
	if (lowerlimit < upperlimit)
	{
		m_TorqueLimit[0] = lowerlimit;
		m_TorqueLimit[1] = upperlimit;
	}
}

double srStateJoint::GetOffset()
{
	return m_rOffset;
}

void srStateJoint::SetOffset(double v)
{
	m_rOffset = v;
}

double srStateJoint::GetSpringCoeff()
{
	return m_rK;
}

void srStateJoint::SetSpringCoeff(double v)
{
	if (v >= 0.0)
		m_rK = v;
}

double srStateJoint::GetDampingCoeff()
{
	return m_rC;
}

void srStateJoint::SetDampingCoeff(double v)
{
	if (v >= 0.0)
		m_rC = v;
}
