#include "UR3RobotManager.h"

UR3RobotManager::UR3RobotManager(UR3Robot * robot, srSpace * space)
{
	this->setRobot((srSystem*)robot);
	this->setSpace(space);
	this->setEndeffector(&robot->gMarkerLink[UR3_Index::MLINK_GRIP]);



	// sensor setting
	this->setFTSensor(robot->gWeldJoint[UR3_Index::WELDJOINT_GRIPPER]);
}

UR3RobotManager::~UR3RobotManager()
{
}

Eigen::VectorXd UR3RobotManager::indyInverseKin(const SE3& T, srLink* link, bool includeOri, SE3 offset, int& flag, Eigen::VectorXd initGuess /*= (Eigen::VectorXd())*/, int maxIter /*= (500)*/, invKinAlg alg/* = (invKinAlg::NR)*/, invKinMet metric /*= (invKinMet::DG)*/)
{
	Eigen::VectorXd	jointVal = inverseKin(T, link, includeOri, offset, flag, initGuess, maxIter, alg, metric);
	
	//if (jointVal())
	//flipElbowJoint(jointVal);
	return Eigen::VectorXd();
}
