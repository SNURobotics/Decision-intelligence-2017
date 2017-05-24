#pragma once
#include <srDyn/srSpace.h>
#include "../Eigen/core"
#include "../Math/mathOperator.h"
#include "../Math/Spline.h"
#include "../robotManager/robotManager.h"
#include <vector>

class impedanceCtrlManager
{
public:

	enum Metric
	{
		DoubleGeodesic = 0, Logarithm = 1
	};
	enum Mode
	{
		Robot = 0, Object = 1
	};
	impedanceCtrlManager();
	~impedanceCtrlManager();

	// object impedance control setting
	bool					setSystem(robotManager* _robotManager, vector<srLink*> _endeffector, vector<SE3> _offset, vector<SE3> _robotToObject);
	bool					setSystem(vector<robotManager*> _robotManagers, vector<vector<srLink*>> _endeffectors, vector<vector<SE3>> _offsets, vector<vector<SE3>> _robotToObject);
	void					setTimeStep(double _timeStep);
	void					setObject(srSystem* _object);
	void					setObjectLinks(srLink* _objectBaseLink, vector<srLink*> _objectLinks);
	void					setObjectRefFrame(SE3 T);
	void					setCenterofCompliance(SE3 T);
	void					setAttachCond(bool _isAttached);
	void					setGravity(Vec3 g);
	void					setRobotPostureAmendThreshold(double _postureThreshold);
	void					setGraspConstraint(vector<SE3> _endeffectorToObject);

	// set impedance parameters
	void					setInertial(Eigen::MatrixXd _inertial);
	void					setResistive(Eigen::MatrixXd _resistive);
	void					setCapacitive(Eigen::MatrixXd _capacitive);
	void					setIntegralGain(Eigen::MatrixXd _integral);

	// position based impedance control
	void					setDesiredTrajectory(SE3 _Tdes);			// const Tdes
	void					setDesiredTrajectory(SE3 _Tdes, se3 _Vdes, se3 _dotVdes);
	void					setDesiredTrajectory(vector<SE3> _Tdes_trj, vector<se3> _Vdes_trj, vector<se3> _dotVdes_trj);
	void					setDesiredTrajectory(vector<SE3> _Tdes_trj);
	void					setDesiredTrajectory(SE3Spline* _SE3spline);
	void					impedanceControl(srJoint::ACTTYPE actType = srJoint::ACTTYPE::TORQUE, bool useFTsensor = false, bool simulateInteraction = true, int metric = DoubleGeodesic);
	void					impedanceControlWeld(int metric = DoubleGeodesic);
	void					objectImpedanceControl(int metric = DoubleGeodesic);
	dse3					getGravitationalForce() const;
	

private:
	se3						getDesiredObjectAcc(dse3 Fext, SE3 T, se3 V, SE3 Tdes, se3 Vdes, se3 dotVdes, bool useIntegralGain = false, int metric = DoubleGeodesic);				// Vdes: desired spatial velocity
	dse3					getExternalForce(dse3 Fgen) const;				// get external generalized force in object ref frame
	dse3					getTotalExternalForce() const;
	dse3					getContactForce() const;
	void					updateIntegralError(SE3 e, int metric = DoubleGeodesic);

	dse3					generateImpedanceForce(dse3 Fext, SE3 T, se3 V, SE3 Tdes, se3 Vdes, se3 dotVdes, bool useIntegralGain = false, int metric = DoubleGeodesic);
	Inertia					getGeneralizedInertia() const;
	Eigen::VectorXd			getDesiredJointAcc(Eigen::VectorXd jointVal, Eigen::VectorXd jointVel, unsigned int robotIdx, se3 objectAcc);
	void					amendJointValVel(vector<Eigen::VectorXd> jointVal);		// amend error from numerical integrations
	double					getKinematicsError(const SE3& Tcur, const vector< Eigen::VectorXd>& jointVal);	// calculate kinematics (position) error from original constraint


public:
	vector<robotManager*>	m_robotManagers;				// consider multi robot case
	vector<vector<srLink*>> m_endeffectors;					// link contains end-effector frame,		first vector: # of robot, second vector: # of end-effector of each robot
	vector<vector<SE3>>     m_offsets;						// end-effector frame from link,			first vector: # of robot, second vector: # of end-effector of each robot
	vector<vector<SE3>>		m_robotToObject;				// SE(3) from end-effector to object,		first vector: # of robot, second vector: # of end-effector of each robot
	double					timeStep;
	//srSystem*				object;
	srLink*					objectBaseLink;
	vector<srLink*>			objectLinks;
	SE3						objectRefFrame;					// SE(3) from link base to RefFrame
	SE3						cocFrame;						// SE(3) from link base to center of compliance
	Inertia					objectInertia;
	Eigen::MatrixXd			inertial;
	Eigen::MatrixXd			resistive;
	Eigen::MatrixXd			capacitive;
	Eigen::MatrixXd			integralGain;
	bool					m_useIntegralGain;
	static Eigen::VectorXd  m_integral_error;
	//vector<SE3>				endeffectorToObject;		//////////////////////////////////// to remove
	vector<SE3>				Tdes_trj;
	vector<se3>				Vdes_trj;
	vector<se3>				dotVdes_trj;
	static SE3				Tbf;
	static dse3				Fgen;
	static dse3				Fctrlbf;
	static se3				Vbf;
	static unsigned int		controlStep;
	SE3Spline*				SE3spline;
	// gravity
	Vec3					g;
	se3						g6;
	// posture amend threshold
	double					postureThreshold;
	// for debugging
	static se3				desAcc;

	///////////////////////////////////// for this simulation
	bool					isAttached;
};

