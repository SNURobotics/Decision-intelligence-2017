#include "UR5RobotManager.h"

UR5RobotManager::UR5RobotManager(UR5Robot * robot, srSpace * space)
{
	this->setRobot((srSystem*)robot);
	this->setSpace(space);
	this->setEndeffector(&robot->gMarkerLink[UR5_Index::MLINK_GRIP]);

	// gripper setting
	vector<srJoint*> gripperJoint(6);
	gripperJoint[0] = robot->gGripJoint[UR5_Index::GRIPJOINT_B_1];
	gripperJoint[1] = robot->gGripJoint[UR5_Index::GRIPJOINT_B_2];
	gripperJoint[2] = robot->gGripJoint[UR5_Index::GRIPJOINT_B_3];
	gripperJoint[3] = robot->gGripJoint[UR5_Index::GRIPJOINT_M_1];
	gripperJoint[4] = robot->gGripJoint[UR5_Index::GRIPJOINT_M_2];
	gripperJoint[5] = robot->gGripJoint[UR5_Index::GRIPJOINT_M_3];
	this->setGripper(gripperJoint);

	// sensor setting
	//this->setFTSensor(robot->gWeldJoint[UR5_Index::WELDJOINT_COUPLING]);
}

UR5RobotManager::~UR5RobotManager()
{
}


