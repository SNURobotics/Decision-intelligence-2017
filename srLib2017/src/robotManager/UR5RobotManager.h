#pragma once
#include "robotManager.h"
#include "UR5Robot.h"
class UR5RobotManager : public robotManager
{
public:
	UR5RobotManager(UR5Robot* robot, srSpace* space);
	~UR5RobotManager();

};