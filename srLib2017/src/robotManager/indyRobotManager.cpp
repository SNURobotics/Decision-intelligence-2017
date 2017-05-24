#include "indyRobotManager.h"

indyRobotManager::indyRobotManager(IndyRobot * robot, srSpace * space)
{
	this->setRobot((srSystem*)robot);
	this->setSpace(space);
	this->setEndeffector(&robot->gMarkerLink[Indy_Index::MLINK_GRIP]);

	// gripper setting
	vector<srJoint*> gripperJoint(2);
	gripperJoint[0] = robot->gPjoint[Indy_Index::GRIPJOINT_L];
	gripperJoint[1] = robot->gPjoint[Indy_Index::GRIPJOINT_U];
	vector<srJoint*> gripperDummyJoint(2);
	gripperDummyJoint[0] = robot->gPjoint[Indy_Index::GRIPJOINT_L_DUMMY];
	gripperDummyJoint[1] = robot->gPjoint[Indy_Index::GRIPJOINT_U_DUMMY];
	this->setGripper(gripperJoint, gripperDummyJoint);

	// sensor setting
	this->setFTSensor(robot->gWeldJoint[Indy_Index::WELDJOINT_GRIPPER]);
}

indyRobotManager::~indyRobotManager()
{
}

Eigen::VectorXd indyRobotManager::indyInverseKin(const SE3& T, srLink* link, bool includeOri, SE3 offset, int& flag, Eigen::VectorXd initGuess /*= (Eigen::VectorXd())*/, int maxIter /*= (500)*/, invKinAlg alg/* = (invKinAlg::NR)*/, invKinMet metric /*= (invKinMet::DG)*/)
{
	Eigen::VectorXd	jointVal = inverseKin(T, link, includeOri, offset, flag, initGuess, maxIter, alg, metric);
	
	//if (jointVal())
	//flipElbowJoint(jointVal);
	return Eigen::VectorXd();
}
