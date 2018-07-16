#include "UR5RobotManager.h"

UR5RobotManager::UR5RobotManager(UR5Robot * robot, srSpace * space)
{
	this->setRobot((srSystem*)robot);
	this->setSpace(space);
	this->setEndeffector(&robot->gMarkerLink[UR5_Index::MLINK_GRIP]);

	// gripper setting
	//vector<srJoint*> gripperJoint(3);
	//gripperJoint[0] = robot->gGripJoint[UR5_Index::WELDJOINT_GRIPJOINT_2];
	//gripperJoint[1] = robot->gGripJoint[UR5_Index::WELDJOINT_GRIPJOINT_2];
	//gripperJoint[2] = robot->gGripJoint[UR5_Index::WELDJOINT_GRIPJOINT_2];
	//this->setGripper(gripperJoint);

	// sensor setting
	//this->setFTSensor(robot->gWeldJoint[UR5_Index::WELDJOINT_COUPLING]);
}

UR5RobotManager::~UR5RobotManager()
{
}


