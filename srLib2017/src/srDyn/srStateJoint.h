#pragma once

#include "srjoint.h"

class srStateJoint : public srJoint
{
public:
	srStateJoint(void);
	~srStateJoint(void);



	se3		m_Axis;


	se3		m_FS_Screw;
	dse3	m_FS_AIS;
	double	m_FS_K;
	double	m_FS_T;
	se3		m_FS_W;

	/*!
	Limits of revolute joint angle. m_PosLimit[0] is lower limit and m_PosLimit[1] is upper limit.
	Upper limit must be greater than lower limit. This should be specified by user.
	Defaults are (-60, 60). Unit is degree.
	*/
	double	m_PosLimit[2];
	bool	m_IsPosLimited;
	//Prismatic joint state.

	srPrismaticState	m_State;



	/*!

	Limits of revolute joint torque. Valid when actuator type is VELOCITY.
	m_TorqueLimit[0] is lower limit and m_TorqueLimit[1] is upper limit.
	It is recommended that upper limit is set to be finite positive value and lower limit is set to be finite negative value.
	This should be specified by user.
	Default is (-100, 100).
	*/
	double	m_TorqueLimit[2];
	/*!
	Offset angle of revolute joint. Spring force is zero when angle of joint equals offset angle.
	Valid when actuator type is PASSIVE. This should be specified by user.
	Default is 0.0.
	*/
	double	m_rOffset;
	/*!
	Spring coefficient of revolute joint.
	Valid when actuator type is PASSIVE. This should be specified by user.
	This must be greater than or equal to zero.
	Default is 0.0.
	*/
	double	m_rK;
	/*!
	Damping coefficient of revolute joint.
	Valid when actuator type is PASSIVE. This should be specified by user.
	This must be greater than or equal to zero.
	Default is 0.01.
	*/
	double	m_rC;
	/*!
	Get position limit boolean value.
	*/
	bool	IsPostionLimited();
	/*!
	Set position limit boolean value.
	*/
	void	MakePositionLimit(bool v = true);
	/*!
	Get lower limit of joint angle.
	*/
	double	GetPositionLowerLimit();
	/*!
	Get upper limit of joint angle.
	*/
	double	GetPositionUpperLimit();
	/*!
	Set limits of joint angle . upper limit must be greater than lower limit.
	*/
	void	SetPositionLimit(double lowerlimit, double upperlimit);
	/*!
	Get lower limit of joint torque.
	*/
	double	GetTorqueLowerLimit();
	/*!
	Get upper limit of joint torque.
	*/
	double	GetTorqueUpperLimit();
	/*!
	Set limits of joint torque . Upper limit must be greater than lower limit.
	*/
	void	SetTorqueLimit(double lowerlimit, double upperlimit);
	/*!
	Get offset.
	*/
	double	GetOffset();
	/*!
	Set offset.
	*/
	void	SetOffset(double v);
	/*!
	Get spring coefficient.
	*/
	double	GetSpringCoeff();
	/*!
	Set spring coefficient. This must be greater than or equal to zero.
	*/
	void	SetSpringCoeff(double v);
	/*!
	Get damping coefficient.
	*/
	double	GetDampingCoeff();
	/*!
	Set damping coefficient. This must be greater than or equal to zero.
	*/
	void	SetDampingCoeff(double v);




};

