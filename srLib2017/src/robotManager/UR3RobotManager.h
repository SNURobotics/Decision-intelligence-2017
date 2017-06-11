#pragma once
#include "robotManager.h"
#include "UR3Robot.h"
class UR3RobotManager : public robotManager
{
public:
	UR3RobotManager(UR3Robot* robot, srSpace* space);
	~UR3RobotManager();

	Eigen::VectorXd				indyInverseKin(const SE3& T, srLink* link, bool includeOri, SE3 offset, int& flag, Eigen::VectorXd initGuess = (Eigen::VectorXd()), int maxIter = (500), invKinAlg alg = (invKinAlg::NR), invKinMet metric = (invKinMet::DG));
	//void						flipElbowJoint(Eigen::VectorXd& jointVal);

};