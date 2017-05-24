#ifndef	SRLIB_REVOLUTE_JOINT
#define SRLIB_REVOLUTE_JOINT

#include "srDyn/srJoint.h"
#include "srDyn/srStateJoint.h"
/*!
\class srRevoluteJoint
\brief Class represents revolute joint.
*/
class srRevoluteJoint : public srStateJoint
{
public:
	/*!
	Boolean value for limit of angle of revolute joint. This should be specified by user.
	Valid when actuator type is PASSIVE, TORQUE or VELOCITY.
	Default is true.
	*/
	void srRevoluteJoint::SetDeviceOnOff(bool onoff);

	/*!
	Get revolute joint state.
	*/
	srRevoluteState& GetRevoluteJointState();


public:
	/*!
	Constructor.
	*/
	srRevoluteJoint();
	/*!
	Destructor.
	*/
	virtual ~srRevoluteJoint();
	/*!
	Revolute joint state.
	*/
	//srRevoluteState	m_State;
	/*!
	Axis of revolute joint. This is pre-fixed as z-axis.
	*/
	//se3		m_Axis;


	//se3		m_FS_Screw;
	//dse3	m_FS_AIS;
	//double	m_FS_K;
	//double	m_FS_T;
	//se3		m_FS_W;

	virtual srState*	GetStatePtr();

	virtual void Initialize();
	virtual void FS_UpdateForce(const dse3& F);
	virtual void FS_UpdateAIS_K(const AInertia& AI);
	virtual void FS_UpdateAIS_K_P(AInertia& AIjari, const AInertia& AI);
	virtual void FS_UpdateBiasImp(dse3& Cias, const dse3& Bias);
	virtual void FS_UpdateBiasforce(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V);
	virtual void FS_UpdateLocalDelVel(se3& jari, const se3& DV);
	virtual void FS_UpdateLocalAcc(se3& jari, const se3& DV);

	virtual SE3&	FS_Transform(void);
	virtual se3&	FS_UpdateLocalVelocity(void);
	virtual se3&	FS_UpdatePosErrorLocalVelocity(void);
	virtual void	FS_SetScrew(int i);
	virtual void	FS_ResetT(void);

protected:
	// Actuator function
	void	(srRevoluteJoint::*m_pfnFS_UpdateForce)(const dse3& F);
	void	(srRevoluteJoint::*m_pfnFS_UpdateAIS_K)(const AInertia& AI);
	void	(srRevoluteJoint::*m_pfnFS_UpdateAIS_K_P)(AInertia& AIjari, const AInertia& AI);
	void	(srRevoluteJoint::*m_pfnFS_UpdateBiasImp)(dse3& Cias, const dse3& Bias);
	void	(srRevoluteJoint::*m_pfnFS_UpdateBiasforce)(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V);
	void	(srRevoluteJoint::*m_pfnFS_UpdateLocalDelVel)(se3& jari, const se3& DV);
	void	(srRevoluteJoint::*m_pfnFS_UpdateLocalAcc)(se3& jari, const se3& DV);

	void	_FS_UpdateForce_Passive(const dse3& F);
	void	_FS_UpdateAIS_K_Passive(const AInertia& AI);
	void	_FS_UpdateAIS_K_P_Passive(AInertia& AIjari, const AInertia& AI);
	void	_FS_UpdateBiasImp_Passive(dse3& Cias, const dse3& Bias);
	void	_FS_UpdateBiasforce_Passive(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V);
	void	_FS_UpdateLocalDelVel_Passive(se3& jari, const se3& DV);
	void	_FS_UpdateLocalAcc_Passive(se3& jari, const se3& DV);

	void	_FS_UpdateForce_Torque(const dse3& F);
	void	_FS_UpdateAIS_K_Torque(const AInertia& AI);
	void	_FS_UpdateAIS_K_P_Torque(AInertia& AIjari, const AInertia& AI);
	void	_FS_UpdateBiasImp_Torque(dse3& Cias, const dse3& Bias);
	void	_FS_UpdateBiasforce_Torque(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V);
	void	_FS_UpdateLocalDelVel_Torque(se3& jari, const se3& DV);
	void	_FS_UpdateLocalAcc_Torque(se3& jari, const se3& DV);

	void	_FS_UpdateForce_Servo(const dse3& F);
	void	_FS_UpdateAIS_K_Servo(const AInertia& AI);
	void	_FS_UpdateAIS_K_P_Servo(AInertia& AIjari, const AInertia& AI);
	void	_FS_UpdateBiasImp_Servo(dse3& Cias, const dse3& Bias);
	void	_FS_UpdateBiasforce_Servo(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V);
	void	_FS_UpdateLocalDelVel_Servo(se3& jari, const se3& DV);
	void	_FS_UpdateLocalAcc_Servo(se3& jari, const se3& DV);

	void	_FS_UpdateForce_Hybrid(const dse3& F);
	void	_FS_UpdateAIS_K_Hybrid(const AInertia& AI);
	void	_FS_UpdateAIS_K_P_Hybrid(AInertia& AIjari, const AInertia& AI);
	void	_FS_UpdateBiasImp_Hybrid(dse3& Cias, const dse3& Bias);
	void	_FS_UpdateBiasforce_Hybrid(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V);
	void	_FS_UpdateLocalDelVel_Hybrid(se3& jari, const se3& DV);
	void	_FS_UpdateLocalAcc_Hybrid(se3& jari, const se3& DV);
};

#endif
