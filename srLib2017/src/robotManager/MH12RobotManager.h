#pragma once
#include "robotManager.h"
#include "MH12Robot.h"
class MH12RobotManager : public robotManager
{
public:
	MH12RobotManager(MH12Robot* robot, srSpace* space);
	~MH12RobotManager();

	void setGripperDistance(double dist);

	Eigen::VectorXd				indyInverseKin(const SE3& T, srLink* link, bool includeOri, SE3 offset, int& flag, Eigen::VectorXd initGuess = (Eigen::VectorXd()), int maxIter = (500), invKinAlg alg = (invKinAlg::NR), invKinMet metric = (invKinMet::DG));
	//void						flipElbowJoint(Eigen::VectorXd& jointVal);

};