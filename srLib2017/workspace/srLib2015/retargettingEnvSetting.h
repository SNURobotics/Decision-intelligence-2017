#pragma once
#include "common\dataIO.h"
#include "robotManager\environment_4th.h"
#include <Windows.h>
#include "robotManager\UR5Robot.h"
#include "robotManager\UR5RobotManager.h"

#include <string>
#include <cstring>
#include <sstream>
#include <ctime>

class retargetEnvironment
{
public:
	retargetEnvironment();
	~retargetEnvironment();

	void setEnvironmentInSrSpace(srSpace* space);
public:
	SE3 Tworld2camera;
	SE3 Tworld2tableCenter;
	SE3 Trobotbase;
	TableRetarget* m_table;
	BlueFemaleConnector* m_blueFemaleConnetor;
	BlueMaleConnector* m_blueMmaleConnetor;
	RedFemaleConnector* m_redFemaleConnetor;
	RedMaleConnector* m_redMmaleConnetor;
};
