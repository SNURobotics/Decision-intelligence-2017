#pragma once
#include <srDyn/srSpace.h>
#include <srGamasot/srRobot.h>
#include <vector>
#include <string>
#include <algorithm>
#include "Eigen/Dense"
#include <Math\mathOperator.h>

#define		INVERSEKIN_TOL						1.0E-8

class activeArmInfo
{
	// consider only revolute and prismatic joints of the robot
public:
	activeArmInfo();
	~activeArmInfo();

	static vector<srJoint*>		goToRoot(srSystem* robot, srLink* lastLink);
	static vector<srJoint*>		excludeJoint(vector<srJoint*> sourceJoints, vector<srJoint*> excludingJoints);
	void					setActiveArmInfo(gamasot::srRobot* robot, string lastLinkName);
	void					setActiveArmInfo(srSystem* robot, srLink* lastLink);
	void					setActiveArmInfoExclude(srSystem* robot, srLink* lastLink, vector<srJoint*> excludingJoints);

	// for multi end-effector case
	void					setActiveArmInfo(gamasot::srRobot* robot, vector<string> lastLinkName);
	void					setActiveArmInfo(srSystem* robot, vector<srLink*> lastLink);
	void					setActiveArmInfoExclude(srSystem* robot, vector<srLink*> lastLink, vector<srJoint*> excludingJoints);
	void					setActiveArmInfo(srSystem* robot, vector<srLink*> lastLink, vector<srJoint*> activeJoint);


	int						getActiveJointIdx(srJoint* joint);

	vector<srStateJoint*>   getActiveStateJoints() const;

	vector<srJoint*>		m_activeJoint;			// actuation type of joints should be set before dyn_pre_step, i.e., during the modeling of robot
	vector<srLink*>			m_endeffector;
	int						m_numJoint;
	vector<vector<srJoint*>> m_activeJointSet;		// active joints for each end-effector in multi end-effector case
};

class gripperInfo
{
public:
	gripperInfo();
	~gripperInfo();

	void					loadGripperLink(srLink* gripperBase);
	void					setGripperJoint(vector<srJoint*> joints);		// actuation type of gripper should be set before dyn_pre_step, i.e., during the modelling of robot
	void					setGripperDummyJoint(vector<srJoint*> joints);

	vector<srJoint*>		m_gripJoint;
	vector<srJoint*>		m_gripDummyJoint;
	vector<srLink*>			m_gripLink;
	srLink*					m_gripBase;
};

class ftSensorInfo
{
public:
	ftSensorInfo();
	~ftSensorInfo();

	void					setSensorLocation(srWeldJoint* wJoint, SE3 offset = SE3());
	dse3					readSensorValue();
	vector<srLink*>			getChildLinksOfAJoint(srJoint* Joint);
	dse3					getInertialForceOfDistalLinks();
	dse3					getGravityForceOfDistalLinks(const se3& g);
	dse3					getExtForce(const se3& g);

	srWeldJoint*			m_sensorLocJoint;
	vector<srLink*>			m_linksAfterSensor;
	SE3						m_offset;
};

class robotManager
{
public:
	enum invKinAlg
	{
		NR /*Newton-Raphson*/, QP /*Quadratic Programming*/
	};

	enum invKinMet
	{
		DG /*double geodesic (pos), (ori)*/, LOG /*logarithm (pos & ori)*/
	};

	enum invKinFlag
	{
		SOLVED, SOLVED_BUT_EXCEED_JOINT_LIM, EXCEED_MAX_ITER, SINGULARITY
		// EXCEED_MAX_ITER usually due to impossible end-effector SE(3) for given robot
	};

	enum manipKind
	{
		INVCOND, VOL, MIN
	};

public:
	robotManager();
	~robotManager();

	void						setRobot(srSystem* robot);
	void						setSpace(srSpace* space);
	
	void						setEndeffector(srLink* lastLink);
	void						setEndeffector(vector<srLink*> lastLink);
	void						setEndeffector(string lastLinkName);
	void						setEndeffector(vector<string> lastLinkName);

	// assume single gripper
	void						setGripper(vector<srJoint*> gripperJoint);
	void						setGripper(vector<srJoint*> gripperJoint, srLink* gripperBase);
	void						setGripper(vector<srJoint*> gripperJoint, vector<srJoint*> gripperDummyJoint);
	
	void						setGripperInput(Eigen::VectorXd input);
	void						setGripperPosition(Eigen::VectorXd posInput);
	Eigen::VectorXd				getGripperPosition() const;
	dse3						getForceOnGripperBase();

	// sensor info
	void						setFTSensor(srWeldJoint* wJoint, SE3 offset = SE3());
	void						setFTSensor(vector<srWeldJoint*> wJoint, vector<SE3> offset);
	dse3						readSensorValue(int idx = 0);

	int							getLinkIdx(srLink* link);
	bool						containLink(srLink* link);
	
	// Joints
	void						setJointVal(const Eigen::VectorXd& jointVal);
	void						setJointValVel(const Eigen::VectorXd& jointVal, const Eigen::VectorXd& jointVel);
	void						setJointValVelAcc(const Eigen::VectorXd& jointVal, const Eigen::VectorXd& jointVel, const Eigen::VectorXd& jointAcc);


	Eigen::VectorXd				getJointVal() const;
	Eigen::VectorXd				getJointVel() const;
	Eigen::VectorXd				getJointAcc() const;
	Eigen::VectorXd				getJointTorque() const;
	Eigen::VectorXd				getJointCommand() const;
	
	void						loadJointLimit();
	void						loadTorqueLimit();
	void						setVelLimit(const vector<double>& upperVelLimit, const vector<double>& lowerVelLimit);
	void						setAccLimit(const vector<double>& upperAccLimit, const vector<double>& lowerAccLimit);

	bool						checkJointLimit(const Eigen::VectorXd& q);
	bool						checkJointLimit(const Eigen::VectorXd& q, vector<unsigned int>& exceedIdx, bool print = false);
	bool						checkJointLimit(const Eigen::VectorXd& q, vector<unsigned int>& upperIdx, vector<unsigned int>& lowerIdx);
	bool						moveIntoJointLimit(Eigen::VectorXd& q, bool print = false);

	void						controlJoint(const Eigen::VectorXd& controlInput);
	void						controlJointTorque(const Eigen::VectorXd& torque);
	void						controlJointAcc(const Eigen::VectorXd& jointAcc);

	// Kinematics
	vector<SE3>					forwardKin(const Eigen::VectorXd& jointVal);			// forward kin sol of all the end-effectors
	SE3							forwardKin(const Eigen::VectorXd& jointVal, srLink* link, SE3 offset = SE3());
	vector<SE3>					forwardKin(const Eigen::VectorXd& jointVal, vector<srLink*> link, vector<SE3> offset);
	
	/////////////////// jacobian for arbitrary joints
	Eigen::MatrixXd				getSpaceJacobian(const Eigen::VectorXd& jointVal, srLink* link, vector<srJoint*> joint);
	Eigen::MatrixXd				getBodyJacobian(const Eigen::VectorXd& jointVal, srLink* link, vector<srJoint*> joint, SE3 offset = SE3());
	Eigen::MatrixXd				getBodyJacobianDot(const Eigen::VectorXd& jointVal, const Eigen::VectorXd& jointVel, srLink* link, vector<srJoint*> joint, SE3 offset = SE3());
	Eigen::MatrixXd				getAnalyticJacobian(const Eigen::VectorXd& jointVal, srLink* link, vector<srJoint*> joint, bool includeOri = false, SE3 offset = SE3());
	
	/////////////////// jacobian for all active joints
	Eigen::MatrixXd				getSpaceJacobian(const Eigen::VectorXd& jointVal, srLink* link);
	Eigen::MatrixXd				getBodyJacobian(const Eigen::VectorXd& jointVal, srLink* link, SE3 offset = SE3());
	Eigen::MatrixXd				getBodyJacobianDot(const Eigen::VectorXd& jointVal, const Eigen::VectorXd& jointVel, srLink* link, SE3 offset = SE3());
	Eigen::MatrixXd				getAnalyticJacobian(const Eigen::VectorXd& jointVal, srLink* link, bool includeOri = false, SE3 offset = SE3());

	/////////////////// jacobian for all active joints of multi end-effector
	Eigen::MatrixXd				getSpaceJacobian(const Eigen::VectorXd& jointVal, vector<srLink*> link);
	Eigen::MatrixXd				getBodyJacobian(const Eigen::VectorXd& jointVal, vector<srLink*> link, vector<SE3> offset);
	Eigen::MatrixXd				getBodyJacobianDot(const Eigen::VectorXd& jointVal, const Eigen::VectorXd& jointVel, vector<srLink*> link, vector<SE3> offset);
	Eigen::MatrixXd				getAnalyticJacobian(const Eigen::VectorXd& jointVal, vector<srLink*> link, vector<bool> includeOri, vector<SE3> offset);

	/////////////////// numerical inverse kinematics
	Eigen::VectorXd				getInverseKinUpdateQP(const Eigen::VectorXd& q, const Eigen::VectorXd& error, const Eigen::MatrixXd& J, double step_size);
	Eigen::VectorXd				inverseKin(const vector<SE3>& T, vector<srLink*> link, vector<bool> includeOri, vector<SE3> offset, int& flag, Eigen::VectorXd initGuess = (Eigen::VectorXd()), int maxIter = (500), invKinAlg alg = (invKinAlg::NR), invKinMet metric = (invKinMet::DG));
	Eigen::VectorXd				inverseKin(const SE3& T, srLink* link, bool includeOri, SE3 offset, int& flag, Eigen::VectorXd initGuess = (Eigen::VectorXd()), int maxIter = (500), invKinAlg alg = (invKinAlg::NR), invKinMet metric = (invKinMet::DG));
	
	double						manipulability(const Eigen::VectorXd& jointVal, srLink* link, manipKind kind = manipKind::MIN, Eigen::MatrixXd Select = Eigen::MatrixXd());
	Eigen::VectorXd				manipulabilityGradient(const Eigen::VectorXd& jointVal, srLink* link, manipKind kind = manipKind::MIN);
	Eigen::VectorXd				manipulabilityGradient(const Eigen::VectorXd& jointVal, srLink* link, double& manipulability, manipKind kind = manipKind::MIN);

	// Collision
	bool						checkCollision();

	// Dynamics
	Eigen::VectorXd				inverseDyn(const Eigen::VectorXd& jointVal, const Eigen::VectorXd& jointVel, const Eigen::VectorXd& jointAcc);
	Eigen::VectorXd				getBiasTerm(const Eigen::VectorXd& jointVal, const Eigen::VectorXd& jointVel);
	Eigen::MatrixXd				getMassMatrix(const Eigen::VectorXd& jointVal);
	double						getTotalMass() const;	

	// Simulation
	void						exertExternalForceToLink(const dse3& Fext, srLink* link, const SE3& offset);

public:
	srSystem*					m_robot;
	srSpace*					m_space;
	activeArmInfo*				m_activeArmInfo;
	gripperInfo*				m_gripperInfo;
	vector<ftSensorInfo*>		m_ftSensorInfo;

	vector<double>				m_upperJointLimit;
	vector<double>				m_lowerJointLimit;
	vector<double>				m_upperVelLimit;
	vector<double>				m_lowerVelLimit;
	vector<double>				m_upperAccLimit;
	vector<double>				m_lowerAccLimit;
	vector<double>				m_upperTorqueLimit;
	vector<double>				m_lowerTorqueLimit;

	static se3					rot_se3;
	static se3					trans_se3;
};

