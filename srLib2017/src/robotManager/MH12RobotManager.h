#pragma once
#include "robotManager.h"
#include "MH12Robot.h"
class MH12RobotManager : public robotManager
{
public:
	MH12RobotManager(MH12Robot* robot, srSpace* space);
	~MH12RobotManager();

};