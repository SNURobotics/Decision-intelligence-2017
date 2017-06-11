#include "UR3RobotManager.h"

UR3RobotManager::UR3RobotManager(UR3Robot * robot, srSpace * space)
{
	this->setRobot((srSystem*)robot);
	this->setSpace(space);
	this->setEndeffector(&robot->gMarkerLink[UR3_Index::MLINK_GRIP]);

	// gripper setting
	vector<srJoint*> gripperJoint(6);
	gripperJoint[0] = robot->gGripJoint[UR3_Index::GRIPJOINT_1_M];
	gripperJoint[1] = robot->gGripJoint[UR3_Index::GRIPJOINT_2_M];
	gripperJoint[2] = robot->gGripJoint[UR3_Index::GRIPJOINT_F_M];
	gripperJoint[3] = robot->gGripJoint[UR3_Index::GRIPJOINT_1_P];
	gripperJoint[4] = robot->gGripJoint[UR3_Index::GRIPJOINT_2_P];
	gripperJoint[5] = robot->gGripJoint[UR3_Index::GRIPJOINT_F_P];
	this->setGripper(gripperJoint);

	// sensor setting
	this->setFTSensor(robot->gWeldJoint[UR3_Index::WELDJOINT_COUPLING]);
}

UR3RobotManager::~UR3RobotManager()
{
}

void UR3RobotManager::setGripperDistance(double dist)
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

Eigen::VectorXd UR3RobotManager::indyInverseKin(const SE3& T, srLink* link, bool includeOri, SE3 offset, int& flag, Eigen::VectorXd initGuess /*= (Eigen::VectorXd())*/, int maxIter /*= (500)*/, invKinAlg alg/* = (invKinAlg::NR)*/, invKinMet metric /*= (invKinMet::DG)*/)
{
	Eigen::VectorXd	jointVal = inverseKin(T, link, includeOri, offset, flag, initGuess, maxIter, alg, metric);
	
	//if (jointVal())
	//flipElbowJoint(jointVal);
	return Eigen::VectorXd();
}
