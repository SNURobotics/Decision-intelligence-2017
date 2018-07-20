#ifndef _SDA20D_MANAGER_H_
#define _SDA20D_MANAGER_H_

#include "srDyn/srSpace.h"
#include "SDA20DRobot.h"
#include "../Eigen/Dense"
#include <vector>
#include <algorithm>
#include "robotManager.h"
#include "RRTmanager\rrtManager.h"
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
};


//class DualArmClosedLoopConstraint : public rrtConstraint
//{
//public:
//	DualArmClosedLoopConstraint(SDA20DManager* robotManager) { _robotManager = robotManager; };
//
//	void								setConstraintProblem(const SE3 _constraintFrame, srLink* right_link, srLink* left_link);
//	virtual		Eigen::VectorXd			getConstraintVector(const Eigen::VectorXd& jointVal);
//	virtual		Eigen::MatrixXd			getConstraintJacobian(const Eigen::VectorXd& jointVal);
//	virtual		void					project2ConstraintManifold(Eigen::VectorXd& jointVal);
//
//public:
//	SDA20DManager*						_robotManager;
//	
//	SE3									constraintFrame1to2;
//	srLink*								left_link;
//	srLink*								right_link;
//	
//	int									numEffectiveLeftArmJoint;
//	int									numEffectiveRightArmJoint;
//	int									numEffectiveArmJoints;
//	vector<int>							effectiveArmJointIdx;
//};

#endif