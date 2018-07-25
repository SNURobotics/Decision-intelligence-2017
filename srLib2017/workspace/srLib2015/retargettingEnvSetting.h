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
	SE3 Trobotbase_connector_right;
	SE3 Trobotbase_connector_left;
	SE3 TblueFemale2robotEE;
	SE3 TblueMale2robotEE;
	SE3 TredFemale2robotEE;
	SE3 TredMale2robotEE;

	SE3 Trobotbase_busbar_right;
	SE3 Trobotbase_busbar_left;

	TableRetarget* m_table;
	BlueFemaleConnector* m_blueFemaleConnetor;
	BlueMaleConnector* m_blueMaleConnetor;
	RedFemaleConnector* m_redFemaleConnetor;
	RedMaleConnector* m_redMaleConnetor;
};
