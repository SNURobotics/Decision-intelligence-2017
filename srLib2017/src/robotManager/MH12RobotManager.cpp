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


