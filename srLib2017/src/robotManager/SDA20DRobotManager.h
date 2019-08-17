#ifndef _SDA20D_MANAGER_H_
#define _SDA20D_MANAGER_H_

#include "srDyn/srSpace.h"
#include "SDA20DRobot.h"
#include "../Eigen/Dense"
#include <vector>
#include <algorithm>
#include "robotManager.h"
#include "RRTmanager\rrtManager.h"
#include "RRTmanager/TBrrtManager.h"
#include "Math\mathOperator.h"
#include "Math\QuadraticProgramming.h"
//
//#define		NUM_OF_END_EFFECTOR_RIGHTARM		2
//#define		NUM_OF_END_EFFECTOR_LEFTARM			2
//
//class SDA20DactiveArminfo : public activeArmInfo
//{
//public:
//	void					setActiveMode(int ch);
//
//public:
//	int						m_activeMode;
//	vector<int>				m_activeJointIdx;
//	int						m_numJoint;
//	int						m_numFeaturePoint;
//	vector<int>				m_featurePointIdx;
//	bool					m_useWaist;
//};

class SDA20DManager : public robotManager
{
public:
	SDA20DManager(SDA20D* robot, srSpace* space, int ch, vector<srJoint*> excludeJoints = vector<srJoint*>());
	~SDA20DManager();

	enum
	{
		MoveRightArmOnly, MoveLeftArmOnly, MoveBothArmOnly,
		MoveRightArmWaist, MoveLeftArmWaist, MoveWholeBody
	};

	// Kinematics
	//void						flipLeftShoulder(vector<double>& jointVal);
	//void						flipRightShoulder(vector<double>& jointVal);
	//void						flipLeftElbow(vector<double>& jointVal);
	//void						flipRightElbow(vector<double>& jointVal);
	//void						flipLeftWrist(vector<double>& jointVal);
	//void						flipRightWrist(vector<double>& jointVal);

public:
	Eigen::VectorXd homePosActiveJoint;
	Eigen::VectorXd qInvKinInitActiveJoint;
public:
	int mode;
};


class SDA20DDualArmClosedLoopConstraint : public rrtConstraint
{
public:
	SDA20DDualArmClosedLoopConstraint(SDA20DManager* robotManager, const SE3 Tright2left);

	virtual		Eigen::VectorXd			getConstraintVector(const Eigen::VectorXd& jointVal);
	virtual		Eigen::MatrixXd			getConstraintJacobian(const Eigen::VectorXd& jointVal);
	virtual		Eigen::VectorXd			getConstraintHessian(const Eigen::VectorXd& jointVal, unsigned int i, unsigned int j);
	virtual		int						project2ConstraintManifold(Eigen::VectorXd& jointVal, int max_iter = 1000);

public:
	SDA20DManager*						_robotManager;
	
	SE3									_Tright2left;
	srLink*								left_link;
	srLink*								right_link;
	unsigned int						numEffectiveArmJoints;	// number of moving joints
	vector<unsigned int>				commonJointIdx;		// common joints from right and left end-effectors to root
	
	//int									numEffectiveLeftArmJoint;
	//int									numEffectiveRightArmJoint;
	//vector<int>							effectiveArmJointIdx;
};

#endif