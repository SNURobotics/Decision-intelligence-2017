#pragma once
#include <srDyn/srSpace.h>
#include <srGamasot/srRobot.h>
#include <vector>
#include <string>
#include <algorithm>
#include "Eigen/Dense"
#include <Math\mathOperator.h>

#define		INVERSEKIN_TOL						1.0E-5

class activeArmInfo
{
	// consider revolute and prismatic joints of the system
public:
	activeArmInfo();
	~activeArmInfo();

	vector<srJoint*>		goToRoot(srSystem* robot, srLink* lastLink);

	void					setActiveArmInfo(gamasot::srRobot* robot, string lastLinkName);
	void					setActiveArmInfo(srSystem* robot, srLink* lastLink);
	void					setHybrid();

	// for multi end-effector case
	void					setActiveArmInfo(gamasot::srRobot* robot, vector<string> lastLinkName);
	void					setActiveArmInfo(srSystem* robot, vector<srLink*> lastLink);
	void					setActiveArmInfo(srSystem* robot, vector<srLink*> lastLink, vector<srJoint*> activeJoint);

	int						getActiveJointIdx(srJoint* joint);

	vector<srJoint*>		m_activeJoint;
	vector<srLink*>			m_endeffector;
	int						m_numJoint;
	vector<vector<srJoint*>> m_activeJointSet;		// active joints for each end-effector in multi end-effector case
};

class robotManager
{

	enum invKinFlag
	{
		SOLVED, EXCEED_MAX_ITER, EXCEED_JOINT_LIM
	};

public:
	robotManager();
	~robotManager();

	void						setRobot(srSystem* robot, srSystem* robotcopy);
	void						setSpace(srSpace* space, srSpace* spacecopy);
	
	void						setEndeffector(srLink* lastLink, srLink* lastLinkCopy);
	void						setEndeffector(vector<srLink*> lastLink, vector<srLink*> lastLinkCopy);
	void						setEndeffector(string lastLinkName, string lastLinkNameCopy);
	void						setEndeffector(vector<string> lastLinkName, vector<string> lastLinkNameCopy);

	bool						containLink(srLink* link);

	// Joints
	void						setJointVal(const Eigen::VectorXd& jointVal);
	void						setJointVal(const Eigen::VectorXd& jointVal, srSystem* robot);
	void						setJointValVel(const Eigen::VectorXd& jointVal, const Eigen::VectorXd& jointVel);
	void						setJointValVel(const Eigen::VectorXd& jointVal, const Eigen::VectorXd& jointVel, srSystem* robot);
	void						setJointValVelAcc(const Eigen::VectorXd& jointVal, const Eigen::VectorXd& jointVel, const Eigen::VectorXd& jointAcc);
	void						setJointValVelAcc(const Eigen::VectorXd& jointVal, const Eigen::VectorXd& jointVel, const Eigen::VectorXd& jointAcc, srSystem* robot);


	Eigen::VectorXd				getJointVal() const;
	Eigen::VectorXd				getJointVel() const;
	Eigen::VectorXd				getJointAcc() const;
	void						setJointLimit(const Eigen::VectorXd& upper, const Eigen::VectorXd& lower);

	bool						checkJointLimit(const Eigen::VectorXd& q);
	bool						checkJointLimit(const Eigen::VectorXd& q, vector<int>& exceedIdx, bool print = false);
	bool						checkJointLimit(const Eigen::VectorXd& q, vector<int>& upperIdx, vector<int>& lowerIdx);
	bool						moveIntoJointLimit(Eigen::VectorXd& q, bool print = false);

	void						controlJointTorque(const Eigen::VectorXd& torque);

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

	Eigen::VectorXd				inverseKin(const vector<SE3>& T, vector<srLink*> link, vector<bool> includeOri, vector<SE3> offset, int& flag, Eigen::VectorXd initGuess = (Eigen::VectorXd()), int maxIter = (500));


	// Collision
	bool						checkCollision();

	// Dynamics
	Eigen::VectorXd				inverseDyn(const Eigen::VectorXd& jointVal, const Eigen::VectorXd& jointVel, const Eigen::VectorXd& jointAcc);
	Eigen::VectorXd				inverseDyn2(const Eigen::VectorXd& jointVal, const Eigen::VectorXd& jointVel, const Eigen::VectorXd& jointAcc);
	Eigen::VectorXd				getBiasTerm(const Eigen::VectorXd& jointVal, const Eigen::VectorXd& jointVel);
	Eigen::MatrixXd				getMassMatrix(const Eigen::VectorXd& jointVal);
	double						getTotalMass() const;	

	// Simulation
	void						exertExternalForceToLink(const dse3& Fext, srLink* link, const SE3& offset);

public:
	srSystem*					m_robot;
	srSpace*					m_space;
	activeArmInfo*				m_activeArmInfo;

	vector<double>				m_upperJointLimit;
	vector<double>				m_lowerJointLimit;

	// Copy for dynamics calculation (may be deleted later)
	srSystem*					m_robotCopy;
	srSpace*					m_spaceCopy;
	activeArmInfo*				m_activeArmInfoCopy;

	static se3					rot_se3;
	static se3					trans_se3;
};

