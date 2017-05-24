#pragma once
#include "robotManager.h"
#include "IndyRobot.h"
class indyRobotManager : public robotManager
{
public:
	indyRobotManager(IndyRobot* robot, srSpace* space);
	~indyRobotManager();

	Eigen::VectorXd				indyInverseKin(const SE3& T, srLink* link, bool includeOri, SE3 offset, int& flag, Eigen::VectorXd initGuess = (Eigen::VectorXd()), int maxIter = (500), invKinAlg alg = (invKinAlg::NR), invKinMet metric = (invKinMet::DG));
	//void						flipElbowJoint(Eigen::VectorXd& jointVal);

};