#include "MH12RobotManager.h"

MH12RobotManager::MH12RobotManager(MH12Robot * robot, srSpace * space)
{
	this->setRobot((srSystem*)robot);
	this->setSpace(space);
	this->setEndeffector(&robot->gMarkerLink[MH12_Index::MLINK_GRIP]);

	// gripper setting
	//vector<srJoint*> gripperJoint(3);
	//gripperJoint[0] = robot->gGripJoint[MH12_Index::WELDJOINT_GRIPJOINT_2];
	//gripperJoint[1] = robot->gGripJoint[MH12_Index::WELDJOINT_GRIPJOINT_2];
	//gripperJoint[2] = robot->gGripJoint[MH12_Index::WELDJOINT_GRIPJOINT_2];
	//this->setGripper(gripperJoint);

	// sensor setting
	//this->setFTSensor(robot->gWeldJoint[MH12_Index::WELDJOINT_COUPLING]);
}

MH12RobotManager::~MH12RobotManager()
{
}

void MH12RobotManager::setGripperDistance(double dist)
{
	if (dist < 0.0)
		dist = 0.0;
	else if (dist > 0.0877)
		dist = 0.0877;
	double q0 = acos((0.0877 - 0.0134) / 0.1143);
	double q = acos((dist - 0.0134) / 0.1143) - q0;

	Eigen::VectorXd gripVal(6);
	gripVal[0] = -q; gripVal[1] = -q; gripVal[2] = q; gripVal[3] = q; gripVal[4] = q; gripVal[5] = -q;
	this->setGripperPosition(gripVal);
}

Eigen::VectorXd MH12RobotManager::indyInverseKin(const SE3& T, srLink* link, bool includeOri, SE3 offset, int& flag, Eigen::VectorXd initGuess /*= (Eigen::VectorXd())*/, int maxIter /*= (500)*/, invKinAlg alg/* = (invKinAlg::NR)*/, invKinMet metric /*= (invKinMet::DG)*/)
{
	Eigen::VectorXd	jointVal = inverseKin(T, link, includeOri, offset, flag, initGuess, maxIter, alg, metric);
	
	//if (jointVal())
	//flipElbowJoint(jointVal);
	return Eigen::VectorXd();
}
