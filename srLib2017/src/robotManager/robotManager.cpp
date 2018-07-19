#include "robotManager.h"
#include "Math\QuadraticProgramming.h"

activeArmInfo::activeArmInfo()
{
	m_activeJoint.resize(0);
	m_endeffector.resize(0);
	m_numJoint = 0;
	m_activeJointSet.resize(0);
}

activeArmInfo::~activeArmInfo()
{
}

vector<srJoint*> activeArmInfo::goToRoot(srSystem* robot, srLink * lastLink)
{
	vector<srJoint*> tempJointSet(0);

	// check if the link is included in the robot
	bool isIncluded = false;
	for (int i = 0; i < robot->m_KIN_Links.get_size(); i++)
		if (robot->m_KIN_Links[i] == lastLink)
			isIncluded = true;
	if (!isIncluded)
	{
		printf("link is not included in the robot\n");
		return tempJointSet;
	}

	srLink* tempLink = lastLink;
	srJoint* tempJoint;

	while (tempLink != robot->GetBaseLink())
	{
		tempJoint = tempLink->m_ParentJoint;
		if (tempJoint->GetType() == srJoint::REVOLUTE || tempJoint->GetType() == srJoint::PRISMATIC)
			tempJointSet.push_back(tempJoint);
		tempLink = tempJoint->m_ParentLink;
	}

	reverse(tempJointSet.begin(), tempJointSet.end());

	return tempJointSet;
}

vector<srJoint*> activeArmInfo::excludeJoint(vector<srJoint*> sourceJoints, vector<srJoint*> excludingJoints)
{
	vector<srJoint*> returnJoints(0);
	for (unsigned int i = 0; i < sourceJoints.size(); i++)
	{
		bool exclude = false;
		for (unsigned int j = 0; j < excludingJoints.size(); j++)
		{
			if (sourceJoints[i] == excludingJoints[j])
			{
				exclude = true;
				break;
			}
		}
		if (!exclude)
			returnJoints.push_back(sourceJoints[i]);
	}
	return returnJoints;
}

void activeArmInfo::setActiveArmInfo(gamasot::srRobot* robot, string lastLinkName)
{
	srLink* tempLink = robot->getLink(lastLinkName);
	setActiveArmInfo((srSystem*)robot, tempLink);
}

void activeArmInfo::setActiveArmInfo(srSystem* robot, srLink * lastLink)
{
	m_activeJoint.resize(0);
	m_activeJoint = goToRoot(robot, lastLink);
	m_endeffector.resize(0);
	m_endeffector.push_back(lastLink);
	m_numJoint = m_activeJoint.size();
	m_activeJointSet.resize(0);
	m_activeJointSet.push_back(m_activeJoint);
}

void activeArmInfo::setActiveArmInfoExclude(srSystem * robot, srLink * lastLink, vector<srJoint*> excludingJoints)
{
	m_activeJoint.resize(0);
	vector<srJoint*> tempJoints = goToRoot(robot, lastLink);
	m_activeJoint = excludeJoint(tempJoints, excludingJoints);
	m_endeffector.resize(0);
	m_endeffector.push_back(lastLink);
	m_numJoint = m_activeJoint.size();
	m_activeJointSet.resize(0);
	m_activeJointSet.push_back(m_activeJoint);
}

void activeArmInfo::setActiveArmInfo(gamasot::srRobot * robot, vector<string> lastLinkName)
{
	vector<srLink*> lastLink(0);
	for (unsigned int i = 0; i < lastLinkName.size(); i++)
		lastLink.push_back(robot->getLink(lastLinkName[i]));
	setActiveArmInfo((srSystem*)robot, lastLink);
}

void activeArmInfo::setActiveArmInfo(srSystem * robot, vector<srLink*> lastLink)
{
	m_endeffector = lastLink;
	m_activeJointSet.resize(0);
	m_activeJoint.resize(0);
	int maxJ = 0, maxJbf = 0;
	for (unsigned int i = 0; i < lastLink.size(); i++)
	{
		m_activeJointSet.push_back(goToRoot(robot, lastLink[i]));
		for (unsigned int k = 0; k < i; k++)
		{
			for (unsigned int j = 0; j < min(m_activeJointSet[i].size(), m_activeJointSet[k].size()); j++)
			{
				if (m_activeJointSet[i][j] != m_activeJointSet[k][j])
				{
					maxJ = j;
					break;
				}
			}
			if (maxJ > maxJbf)
				maxJbf = maxJ;
		}
		for (unsigned int k = maxJbf; k < m_activeJointSet[i].size(); k++)
			m_activeJoint.push_back(m_activeJointSet[i][k]);
		
	}
	m_numJoint = m_activeJoint.size();
}

void activeArmInfo::setActiveArmInfoExclude(srSystem * robot, vector<srLink*> lastLink, vector<srJoint*> excludingJoints)
{
	m_endeffector = lastLink;
	m_activeJointSet.resize(0);
	m_activeJoint.resize(0);
	int maxJ = 0, maxJbf = 0;
	for (unsigned int i = 0; i < lastLink.size(); i++)
	{
		vector<srJoint*> tempJoints = goToRoot(robot, lastLink[i]);
		m_activeJointSet.push_back(excludeJoint(tempJoints, excludingJoints));
		for (unsigned int k = 0; k < i; k++)
		{
			for (unsigned int j = 0; j < min(m_activeJointSet[i].size(), m_activeJointSet[k].size()); j++)
			{
				if (m_activeJointSet[i][j] != m_activeJointSet[k][j])
				{
					maxJ = j;
					break;
				}
			}
			if (maxJ > maxJbf)
				maxJbf = maxJ;
		}
		for (unsigned int k = maxJbf; k < m_activeJointSet[i].size(); k++)
			m_activeJoint.push_back(m_activeJointSet[i][k]);

	}
	m_numJoint = m_activeJoint.size();
}

void activeArmInfo::setActiveArmInfo(srSystem * robot, vector<srLink*> lastLink, vector<srJoint*> activeJoint)
{
	m_endeffector = lastLink;
	m_activeJoint = activeJoint;
	m_numJoint = activeJoint.size();
}

//void activeArmInfo::setActType(srJoint::ACTTYPE actType)
//{
//	for (unsigned int i = 0; i < m_activeJoint.size(); i++)
//	{
//		m_activeJoint[i]->SetActType(actType);
//		m_activeJoint[i]->Initialize();
//	}
//}

int activeArmInfo::getActiveJointIdx(srJoint * joint)
{
	int idx = -1;
	for (int i = 0; i < m_numJoint; i++)
		if (joint == m_activeJoint[i])
		{
			idx = i;
			break;
		}
	return idx;
}

vector<srStateJoint*> activeArmInfo::getActiveStateJoints() const
{
	vector<srStateJoint*> stateJoints;
	stateJoints.resize(m_activeJoint.size());

	for (unsigned int i = 0; i < m_activeJoint.size(); i++)
		stateJoints[i] = (srStateJoint*)m_activeJoint[i];
		
	return stateJoints;
}

robotManager::robotManager()
{
	m_activeArmInfo = new activeArmInfo();
	m_gripperInfo = new gripperInfo();
	m_ftSensorInfo.resize(0);
}


robotManager::~robotManager()
{
	delete m_activeArmInfo;
	delete m_gripperInfo;
	m_ftSensorInfo.resize(0);
}

void robotManager::setRobot(srSystem * robot)
{
	m_robot = robot;
}

void robotManager::setSpace(srSpace * space)
{
	m_space = space;
}

void robotManager::setEndeffector(srLink * lastLink)
{
	m_activeArmInfo->setActiveArmInfo(m_robot, lastLink);
	loadJointLimit();
}

void robotManager::setEndeffector(vector<srLink*> lastLink)
{
	m_activeArmInfo->setActiveArmInfo(m_robot, lastLink);
	loadJointLimit();
}

void robotManager::setEndeffector(string lastLinkName)
{
	m_activeArmInfo->setActiveArmInfo(static_cast<gamasot::srRobot*>(m_robot), lastLinkName);
	loadJointLimit();
}

void robotManager::setEndeffector(vector<string> lastLinkName)
{
	m_activeArmInfo->setActiveArmInfo(static_cast<gamasot::srRobot*>(m_robot), lastLinkName);
	loadJointLimit();
}

void robotManager::setGripper(vector<srJoint*> gripperJoint)
{
	m_gripperInfo->setGripperJoint(gripperJoint);
}

void robotManager::setGripper(vector<srJoint*> gripperJoint, srLink* gripperBase)
{
	m_gripperInfo->setGripperJoint(gripperJoint);
	m_gripperInfo->loadGripperLink(gripperBase);
	//m_gripperInfo->setActType(actType);
}

void robotManager::setGripper(vector<srJoint*> gripperJoint, vector<srJoint*> gripperDummyJoint)
{
	m_gripperInfo->setGripperJoint(gripperJoint);
	m_gripperInfo->setGripperDummyJoint(gripperDummyJoint);
}

void robotManager::setGripperInput(Eigen::VectorXd input)
{
	for (unsigned int i = 0; i < m_gripperInfo->m_gripJoint.size(); i++)
	{
			((srStateJoint*)m_gripperInfo->m_gripJoint[i])->m_State.m_rCommand = input(i);
	}

}

void robotManager::setGripperPosition(Eigen::VectorXd posInput)
{
	for (unsigned int i = 0; i < m_gripperInfo->m_gripJoint.size(); i++)
		((srStateJoint*)m_gripperInfo->m_gripJoint[i])->m_State.m_rValue[0] = posInput(i);

	if (m_gripperInfo->m_gripDummyJoint.size() > 0)
		for (unsigned int i = 0; i < m_gripperInfo->m_gripDummyJoint.size(); i++)
			((srStateJoint*)m_gripperInfo->m_gripDummyJoint[i])->m_State.m_rValue[0] = posInput(i);
}

Eigen::VectorXd robotManager::getGripperPosition() const
{
	Eigen::VectorXd posInput(m_gripperInfo->m_gripJoint.size());
	for (unsigned int i = 0; i < m_gripperInfo->m_gripJoint.size(); i++)
		posInput(i) = ((srStateJoint*)m_gripperInfo->m_gripJoint[i])->m_State.m_rValue[0];
	return posInput;
}

dse3 robotManager::getForceOnGripperBase()
{
	dse3 temp;
	dse3 forceongripper(0.0);
	for (unsigned int i = 0; i < m_gripperInfo->m_gripLink.size(); i++)
	{
		temp = m_gripperInfo->m_gripLink[i]->m_Inertia * m_gripperInfo->m_gripLink[i]->m_Acc - dad(m_gripperInfo->m_gripLink[i]->m_Vel, m_gripperInfo->m_gripLink[i]->m_Inertia * m_gripperInfo->m_gripLink[i]->m_Vel) - m_gripperInfo->m_gripLink[i]->m_ExtForce - m_gripperInfo->m_gripLink[i]->m_ConstraintImpulse * (1.0 / m_space->m_Timestep_dyn_fixed);
		forceongripper += InvdAd(m_gripperInfo->m_gripBase->m_Frame % m_gripperInfo->m_gripLink[i]->m_Frame, temp);
	}
	return forceongripper;
}

void robotManager::setFTSensor(srWeldJoint * wJoint, SE3 offset /*= SE3()*/)
{
	ftSensorInfo* tempSensorInfo = new ftSensorInfo;
	tempSensorInfo->setSensorLocation(wJoint, offset);
	m_ftSensorInfo.push_back(tempSensorInfo);
}

void robotManager::setFTSensor(vector<srWeldJoint*> wJoint, vector<SE3> offset)
{
	ftSensorInfo* tempSensorInfo = new ftSensorInfo;
	tempSensorInfo->setSensorLocation(wJoint[0], offset[0]);
	m_ftSensorInfo.push_back(tempSensorInfo);
	for (unsigned int i = 1; i < wJoint.size(); i++)
	{
		tempSensorInfo = new ftSensorInfo;
		tempSensorInfo->setSensorLocation(wJoint[i], offset[i]);
		m_ftSensorInfo.push_back(tempSensorInfo);
	}
}

dse3 robotManager::readSensorValue(int idx /*= 0*/)
{
	return m_ftSensorInfo[idx]->readSensorValue();
}

int robotManager::getLinkIdx(srLink * link)
{
	int linkIdx = -1;
	for (int i = 0; i < m_robot->m_KIN_Links.get_size(); i++)
		if (link == m_robot->m_KIN_Links[i])
			return i;
	return linkIdx;
}

bool robotManager::containLink(srLink * link)
{
	if (getLinkIdx(link) == -1)
		return false;
	else
		return true;
}

void robotManager::setJointVal(const Eigen::VectorXd& jointVal)
{
	if (jointVal.size() != m_activeArmInfo->m_numJoint)
		std::cout << "check number of joint values!!! (function: setJointVal)" << std::endl;

	for (int i = 0; i < m_activeArmInfo->m_numJoint; i++)
		((srStateJoint*)m_activeArmInfo->m_activeJoint[i])->m_State.m_rValue[0] = jointVal[i];

	m_robot->KIN_UpdateFrame_All_The_Entity();
}

void robotManager::setJointValVel(const Eigen::VectorXd & jointVal, const Eigen::VectorXd & jointVel)
{
	if (jointVal.size() != m_activeArmInfo->m_numJoint || jointVel.size() != m_activeArmInfo->m_numJoint)
		std::cout << "check number of joint values!!! (function: setJointValVel)" << std::endl;

	for (int i = 0; i < m_activeArmInfo->m_numJoint; i++)
	{
		((srStateJoint*)m_activeArmInfo->m_activeJoint[i])->m_State.m_rValue[0] = jointVal[i];
		((srStateJoint*)m_activeArmInfo->m_activeJoint[i])->m_State.m_rValue[1] = jointVel[i];
	}
	
	m_robot->KIN_UpdateFrame_All_The_Entity();
}

void robotManager::setJointValVelAcc(const Eigen::VectorXd & jointVal, const Eigen::VectorXd & jointVel, const Eigen::VectorXd & jointAcc)
{
	srStateJoint* curJoint;
	
	if (jointVal.size() != m_activeArmInfo->m_numJoint || jointVel.size() != m_activeArmInfo->m_numJoint || jointAcc.size() != m_activeArmInfo->m_numJoint)
		cout << "Check number of joints !!! (function: setJointValVelAcc)" << endl;

	for (int i = 0; i < m_activeArmInfo->m_numJoint; i++) {
		curJoint = (srStateJoint*)m_activeArmInfo->m_activeJoint[i];
		curJoint->m_State.m_rValue[0] = jointVal[i];
		curJoint->m_State.m_rValue[1] = jointVel[i];
		//curJoint->m_State.m_rValue[2] = 0.0;
		curJoint->m_State.m_rValue[3] = 0.0;
		curJoint->m_State.m_rCommand = jointAcc[i];
	}
	m_robot->KIN_UpdateFrame_All_The_Entity();
	m_robot->DIFFKIN_LinkVelocityPropagation();
}

Eigen::VectorXd robotManager::getJointVal() const
{
	Eigen::VectorXd rJointVal(m_activeArmInfo->m_numJoint);
	for (int i = 0; i < rJointVal.size(); i++)
		rJointVal[i] = ((srStateJoint*)m_activeArmInfo->m_activeJoint[i])->m_State.m_rValue[0];
	return rJointVal;
}

Eigen::VectorXd robotManager::getJointVel() const
{
	Eigen::VectorXd rJointVel(m_activeArmInfo->m_numJoint);
	for (int i = 0; i < rJointVel.size(); i++)
		rJointVel[i] = ((srStateJoint*)m_activeArmInfo->m_activeJoint[i])->m_State.m_rValue[1];
	return rJointVel;
}

Eigen::VectorXd robotManager::getJointAcc() const
{
	Eigen::VectorXd dotdotq(m_activeArmInfo->m_numJoint);
	for (int i = 0; i < m_activeArmInfo->m_numJoint; i++)
		dotdotq(i) = ((srStateJoint*)m_activeArmInfo->m_activeJoint[i])->m_State.m_rValue[2];
	return dotdotq;
}

Eigen::VectorXd robotManager::getJointTorque() const
{
	Eigen::VectorXd tau(m_activeArmInfo->m_numJoint);
	for (int i = 0; i < m_activeArmInfo->m_numJoint; i++)
		tau(i) = ((srStateJoint*)m_activeArmInfo->m_activeJoint[i])->m_State.m_rValue[3];
	return tau;
}

Eigen::VectorXd robotManager::getJointCommand() const
{
	Eigen::VectorXd command(m_activeArmInfo->m_numJoint);
	for (int i = 0; i < m_activeArmInfo->m_numJoint; i++)
		command(i) = ((srStateJoint*)m_activeArmInfo->m_activeJoint[i])->m_State.m_rCommand;
	return command;
}

void robotManager::loadJointLimit()
{
	m_upperJointLimit.resize(m_activeArmInfo->m_numJoint);
	m_lowerJointLimit.resize(m_activeArmInfo->m_numJoint);
	for (int i = 0; i < m_activeArmInfo->m_numJoint; i++)
	{
		m_upperJointLimit[i] = DEG2RAD(((srStateJoint*)m_activeArmInfo->m_activeJoint[i])->m_PosLimit[1]);
		m_lowerJointLimit[i] = DEG2RAD(((srStateJoint*)m_activeArmInfo->m_activeJoint[i])->m_PosLimit[0]);
	}
}

void robotManager::loadTorqueLimit()
{
	m_upperTorqueLimit.resize(m_activeArmInfo->m_numJoint);
	m_lowerTorqueLimit.resize(m_activeArmInfo->m_numJoint);
	for (int i = 0; i < m_activeArmInfo->m_numJoint; i++)
	{
		m_upperTorqueLimit[i] = DEG2RAD(((srStateJoint*)m_activeArmInfo->m_activeJoint[i])->m_TorqueLimit[1]);
		m_lowerTorqueLimit[i] = DEG2RAD(((srStateJoint*)m_activeArmInfo->m_activeJoint[i])->m_TorqueLimit[0]);
	}
}

void robotManager::setVelLimit(const vector<double>& upperVelLimit, const vector<double>& lowerVelLimit)
{
	if (upperVelLimit.size() == m_activeArmInfo->m_numJoint)
		m_upperVelLimit = upperVelLimit;
	if (lowerVelLimit.size() == m_activeArmInfo->m_numJoint)
		m_lowerVelLimit = lowerVelLimit;
}

void robotManager::setAccLimit(const vector<double>& upperAccLimit, const vector<double>& lowerAccLimit)
{
	if (upperAccLimit.size() == m_activeArmInfo->m_numJoint)
		m_upperAccLimit = upperAccLimit;
	if (lowerAccLimit.size() == m_activeArmInfo->m_numJoint)
		m_lowerAccLimit = lowerAccLimit;
}

bool robotManager::checkJointLimit(const Eigen::VectorXd & q)
{
	bool exceedJointLimit = false;
	for (int i = 0; i < q.size(); i++)
	{
		if (q(i) > m_upperJointLimit[i])
		{
			exceedJointLimit = true;
			break;
		}
		if (q(i) < m_lowerJointLimit[i])
		{
			exceedJointLimit = true;
			break;
		}
	}

	return !exceedJointLimit;
}

bool robotManager::checkJointLimit(const Eigen::VectorXd & q, vector<unsigned int>& exceedIdx, bool print)
{
	exceedIdx.resize(0);
	bool exceedJointLimit = false;
	for (int i = 0; i < q.size(); i++)
	{
		if (q(i) > m_upperJointLimit[i])
		{
			exceedJointLimit = true;
			exceedIdx.push_back(i);
		}
		if (q(i) < m_lowerJointLimit[i])
		{
			exceedJointLimit = true;
			exceedIdx.push_back(i);
		}
	}
	if (print && exceedIdx.size() > 0)
	{
		printf("exceed idx: ");
		for (unsigned int j = 0; j < exceedIdx.size() - 1; j++)
			printf("%d, ", exceedIdx[j]);
		printf("%d\n", exceedIdx[exceedIdx.size() - 1]);
	}
	return !exceedJointLimit;
}

bool robotManager::checkJointLimit(const Eigen::VectorXd & q, vector<unsigned int>& upperIdx, vector<unsigned int>& lowerIdx)
{
	upperIdx.resize(0);
	lowerIdx.resize(0);
	double offset = 0.01;
	bool exceedJointLimit = false;
	for (int i = 0; i < q.size(); i++)
	{
		if (q(i) > m_upperJointLimit[i] - offset)
		{
			exceedJointLimit = true;
			upperIdx.push_back(i);
		}
		if (q(i) < m_lowerJointLimit[i] + offset)
		{
			exceedJointLimit = true;
			lowerIdx.push_back(i);
		}
	}

	return !exceedJointLimit;
}

bool robotManager::moveIntoJointLimit(Eigen::VectorXd & q, bool print)
{
	// move jointval to be inside joint limit
	double qi;
	double tmp1;
	double tmp2;
	double tmpint;
	
	for (int i = 0; i < q.size(); i++)
	{
		qi = q(i);
		tmp1 = (m_upperJointLimit[i] - qi) / SR_TWO_PI;
		tmp2 = (m_lowerJointLimit[i] - qi) / SR_TWO_PI;
		tmpint = std::round(0.5*(tmp1 + tmp2));

		q(i) += SR_TWO_PI * tmpint;
	}

	if (print)
	{
		vector<unsigned int> exceedIdx(0);
		return checkJointLimit(q, exceedIdx, print);
	}
	else
		return checkJointLimit(q);
}

void robotManager::controlJoint(const Eigen::VectorXd & controlInput)
{
	for (int i = 0; i < m_activeArmInfo->m_numJoint; i++)
		((srStateJoint*)m_activeArmInfo->m_activeJoint[i])->m_State.m_rCommand = controlInput(i);
}

void robotManager::controlJointTorque(const Eigen::VectorXd & torque)
{
	bool isTorque = true;
	for (int i = 0; i < m_activeArmInfo->m_numJoint; i++) 
	{
		if (m_activeArmInfo->m_activeJoint[i]->GetActType() != srJoint::TORQUE) 
		{
			printf("check acttype!!! (function: controlJointTorque)\n");
			isTorque = false;
			break;
		}
	}
	if (isTorque == true) 
	{
		for (int i = 0; i < m_activeArmInfo->m_numJoint; i++) 
			((srStateJoint*)m_activeArmInfo->m_activeJoint[i])->m_State.m_rCommand = torque(i);
		
	}
}

void robotManager::controlJointAcc(const Eigen::VectorXd & jointAcc)
{
	bool isHybrid = true;
	for (int i = 0; i < m_activeArmInfo->m_numJoint; i++)
	{
		if (m_activeArmInfo->m_activeJoint[i]->GetActType() != srJoint::HYBRID)
		{
			printf("check acttype!!! (function: controlJointAcc)\n");
			isHybrid = false;
			break;
		}
	}
	if (isHybrid == true)
	{
		for (int i = 0; i < m_activeArmInfo->m_numJoint; i++)
		{
			((srStateJoint*)m_activeArmInfo->m_activeJoint[i])->m_State.m_rCommand = jointAcc(i);
			//((srStateJoint*)m_activeArmInfo->m_activeJoint[i])->m_State.m_rValue[3] = 0.0;
		}
			

	}
}

vector<SE3> robotManager::forwardKin(const Eigen::VectorXd& jointVal)
{
	vector<SE3> forKinSol(m_activeArmInfo->m_endeffector.size());
	setJointVal(jointVal);
	for (unsigned int i = 0; i < forKinSol.size(); i++)
		forKinSol[i] = m_activeArmInfo->m_endeffector[i]->GetFrame();
	return forKinSol;
}

SE3 robotManager::forwardKin(const Eigen::VectorXd& jointVal, srLink * link, SE3 offset)
{
	setJointVal(jointVal);

	return link->GetFrame()*offset;
}

vector<SE3> robotManager::forwardKin(const Eigen::VectorXd & jointVal, vector<srLink*> link, vector<SE3> offset)
{
	vector<SE3> forKinSol(link.size());
	for (unsigned int i = 0; i < link.size(); i++)
		forKinSol[i] = forwardKin(jointVal, link[i])*offset[i];
	return forKinSol;
}

Eigen::MatrixXd robotManager::getSpaceJacobian(const Eigen::VectorXd & jointVal, srLink * link, vector<srJoint*> joint)
{
	setJointVal(jointVal);
	vector<se3> temp(joint.size());
	for (unsigned int i = 0; i < joint.size(); i++)
		if (joint[i]->GetType() == srJoint::PRISMATIC)
			temp[i] = Ad(joint[i]->GetFrame(), trans_se3);
		else
			temp[i] = Ad(joint[i]->GetFrame(), rot_se3);
	Eigen::MatrixXd Js = Eigen::MatrixXd::Zero(6, joint.size());

	vector<srJoint*> tempJoint = m_activeArmInfo->goToRoot(m_robot, link);
	for (unsigned int i = 0; i < tempJoint.size(); i++)
	{
		for (unsigned int j = 0; j < joint.size(); j++)
		{
			if (tempJoint[i] == joint[j])
			{
				for (int k = 0; k < 6; k++)
					Js(k,j) = temp[j][k];
				break;
			}
		}
	}
	return Js;
}

Eigen::MatrixXd robotManager::getBodyJacobian(const Eigen::VectorXd & jointVal, srLink * link, vector<srJoint*> joint, SE3 offset)
{
	setJointVal(jointVal);
	vector<se3> temp(joint.size());
	SE3 SE3_endeffector = link->GetFrame()*offset;
	for (unsigned int i = 0; i < joint.size(); i++)
		if (joint[i]->GetType() == srJoint::PRISMATIC)
			temp[i] = Ad(SE3_endeffector % joint[i]->GetFrame(), trans_se3);
		else
			temp[i] = Ad(SE3_endeffector % joint[i]->GetFrame(), rot_se3);
	Eigen::MatrixXd Jb = Eigen::MatrixXd::Zero(6, joint.size());

	vector<srJoint*> tempJoint = m_activeArmInfo->goToRoot(m_robot, link);
	for (unsigned int i = 0; i < tempJoint.size(); i++)
	{
		for (unsigned int j = 0; j < joint.size(); j++)
		{
			if (tempJoint[i] == joint[j])
			{
				for (int k = 0; k < 6; k++)
					Jb(k,j) = temp[j][k];
				break;
			}
		}
	}
	////////////////////
	temp.resize(0);
	tempJoint.resize(0);
	////////////////////
	return Jb;
}

Eigen::MatrixXd robotManager::getBodyJacobianDot(const Eigen::VectorXd & jointVal, const Eigen::VectorXd & jointVel, srLink * link, vector<srJoint*> joint, SE3 offset)
{
	setJointVal(jointVal);
	Eigen::VectorXd Vi = Eigen::VectorXd::Zero(6);
	vector<srJoint*> tempJoint = m_activeArmInfo->goToRoot(m_robot, link);
	vector<se3> temp(tempJoint.size());
	SE3 SE3_endeffector = link->GetFrame()*offset;
	se3 curScrew;
	int jIdx;
	for (int i = tempJoint.size() - 1; i > -1; i--)
	{
		jIdx = m_activeArmInfo->getActiveJointIdx(tempJoint[i]);
		if (tempJoint[i]->GetType() == srJoint::REVOLUTE)
			curScrew = Ad(SE3_endeffector % (tempJoint[i]->GetFrame()), rot_se3);

		else if (tempJoint[i]->GetType() == srJoint::PRISMATIC)
			curScrew = Ad(SE3_endeffector % (tempJoint[i]->GetFrame()), trans_se3);
		temp[i] = ad(Vectortose3(-Vi), curScrew);
		Vi += jointVel(jIdx)*se3toVector(curScrew);
	}
	
	Eigen::MatrixXd Jb_dot = Eigen::MatrixXd::Zero(6, joint.size());
	
	for (unsigned int i = 0; i < tempJoint.size(); i++)
	{
		for (unsigned int j = 0; j < joint.size(); j++)
		{
			if (tempJoint[i] == joint[j])
			{
				for (int k = 0; k < 6; k++)
					Jb_dot(k, j) = temp[i][k];
				break;
			}
		}
	}
	return Jb_dot;
}

Eigen::MatrixXd robotManager::getAnalyticJacobian(const Eigen::VectorXd & jointVal, srLink * link, vector<srJoint*> joint, bool includeOri, SE3 offset)
{
	Eigen::MatrixXd analyticJac = getBodyJacobian(jointVal, link, joint, offset);

	Eigen::MatrixXd tmpOrientation_Mtx(3, 3);
	SO3 tmpOrientation = link->GetFrame().GetOrientation()*offset.GetOrientation();


	// transform SO3 to Eigen::MatrixXd
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			tmpOrientation_Mtx(i, j) = tmpOrientation(i, j);

	if (includeOri)
	{
		for (int j = 0; j < 2; j++)
			analyticJac.block(3 * j, 0, 3, analyticJac.cols()) = tmpOrientation_Mtx*analyticJac.block(3 * j, 0, 3, analyticJac.cols());
		return analyticJac;
	}
	else
		return tmpOrientation_Mtx*analyticJac.block(3, 0, 3, analyticJac.cols());
}

Eigen::MatrixXd robotManager::getSpaceJacobian(const Eigen::VectorXd & jointVal, srLink * link)
{
	return getSpaceJacobian(jointVal, link, m_activeArmInfo->m_activeJoint);
}

Eigen::MatrixXd robotManager::getBodyJacobian(const Eigen::VectorXd & jointVal, srLink * link, SE3 offset)
{
	return getBodyJacobian(jointVal, link, m_activeArmInfo->m_activeJoint, offset);
}

Eigen::MatrixXd robotManager::getBodyJacobianDot(const Eigen::VectorXd & jointVal, const Eigen::VectorXd & jointVel, srLink * link, SE3 offset)
{
	return getBodyJacobianDot(jointVal, jointVel, link, m_activeArmInfo->m_activeJoint, offset);
}

Eigen::MatrixXd robotManager::getAnalyticJacobian(const Eigen::VectorXd & jointVal, srLink * link, bool includeOri, SE3 offset)
{
	return getAnalyticJacobian(jointVal, link, m_activeArmInfo->m_activeJoint, includeOri, offset);
}

Eigen::MatrixXd robotManager::getSpaceJacobian(const Eigen::VectorXd & jointVal, vector<srLink*> link)
{
	Eigen::MatrixXd Js(6 * link.size(), m_activeArmInfo->m_numJoint);
	int idx = 0;
	for (unsigned int i = 0; i < link.size(); i++)
	{
		Js.block(idx, 0, 6, m_activeArmInfo->m_numJoint) = getSpaceJacobian(jointVal, link[i]);
		idx += 6;
	}
	return Js;
}

Eigen::MatrixXd robotManager::getBodyJacobian(const Eigen::VectorXd & jointVal, vector<srLink*> link, vector<SE3> offset)
{
	Eigen::MatrixXd Jb(6 * link.size(), m_activeArmInfo->m_numJoint);
	int idx = 0;
	for (unsigned int i = 0; i < link.size(); i++)
	{
		Jb.block(idx, 0, 6, m_activeArmInfo->m_numJoint) = getBodyJacobian(jointVal, link[i], offset[i]);
		idx += 6;
	}
	return Jb;
}

Eigen::MatrixXd robotManager::getBodyJacobianDot(const Eigen::VectorXd & jointVal, const Eigen::VectorXd & jointVel, vector<srLink*> link, vector<SE3> offset)
{
	Eigen::MatrixXd Jb_dot(6 * link.size(), m_activeArmInfo->m_numJoint);
	int idx = 0;
	for (unsigned int i = 0; i < link.size(); i++)
	{
		Jb_dot.block(idx, 0, 6, m_activeArmInfo->m_numJoint) = getBodyJacobianDot(jointVal, jointVel, link[i], offset[i]);
		idx += 6;
	}
	return Jb_dot;
}

Eigen::MatrixXd robotManager::getAnalyticJacobian(const Eigen::VectorXd & jointVal, vector<srLink*> link, vector<bool> includeOri, vector<SE3> offset)
{
	int nRow = 0;
	for (unsigned int i = 0; i < includeOri.size(); i++)
		if (includeOri[i])
			nRow += 6;
		else
			nRow += 3;
	Eigen::MatrixXd J(nRow, m_activeArmInfo->m_numJoint);
	int idx = 0;
	for (unsigned int i = 0; i < link.size(); i++)
	{
		if (includeOri[i])
			nRow = 6;
		else
			nRow = 3;
		J.block(idx, 0, nRow, m_activeArmInfo->m_numJoint) = getAnalyticJacobian(jointVal, link[i], includeOri[i], offset[i]);
		idx += nRow;
	}
	return J;
}

Eigen::VectorXd robotManager::getInverseKinUpdateQP(const Eigen::VectorXd& q, const Eigen::VectorXd& error, const Eigen::MatrixXd& J, double step_size)
{
	// solve quadratic programming
	QuadraticProgramming quadprog;
	double lambda = 0.01;
	vector<unsigned int> upperIdx(0);
	vector<unsigned int> lowerIdx(0);
	checkJointLimit(q, upperIdx, lowerIdx);
	unsigned int nDim = q.size();
	bool isRedundant = false;
	if (!isRedundant)
	{
		//Eigen::MatrixXd M = Eigen::MatrixXd::Identity(6, 6);
		//for (int i = 0; i < 3; i++)
		//	M(i, i) = 0.01;
		quadprog._P = J.transpose()*J + 0.001*lambda * Eigen::MatrixXd::Identity(q.size(), q.size());
		quadprog._q = -J.transpose()*error*step_size;
		//cout << error.transpose() << endl;
		quadprog._A = Eigen::MatrixXd();
		quadprog._b = Eigen::VectorXd();
	}
	else
	{
		quadprog._P = lambda * Eigen::MatrixXd::Identity(q.size(), q.size());
		quadprog._q = Eigen::VectorXd::Zero(q.size());
		quadprog._A = J;
		quadprog._b = error*step_size;
	}



	Eigen::MatrixXd G = Eigen::MatrixXd::Zero(upperIdx.size() + lowerIdx.size(), q.size());
	Eigen::VectorXd h = -1e-5*Eigen::VectorXd::Ones(G.rows());
	for (unsigned int i = 0, j = 0, k = 0; i < nDim; i++)
	{
		if (j < upperIdx.size())
		{
			if (i == upperIdx[j])
			{
				G(j + k, i) = 1;
				h(j + k) = m_upperJointLimit[i] - q(i);
				j++;
			}
		}
		if (k < lowerIdx.size())
		{
			if (i == lowerIdx[k])
			{
				G(j + k, i) = -1;
				h(j + k) = q(i) - m_lowerJointLimit[i];
				k++;
			}
		}
	}

	quadprog._G = G;
	quadprog._h = h;

	return quadprog.solve(Eigen::VectorXd::Zero(m_activeArmInfo->m_numJoint));
}

Eigen::VectorXd robotManager::inverseKin(const vector<SE3>& T, vector<srLink*> link, vector<bool> includeOri, vector<SE3> offset, int & flag, Eigen::VectorXd initGuess, int maxIter, invKinAlg alg /*= (invKinAlg::NR)*/, invKinMet metric /*= (invKinMet::DG)*/)
{
	// solve inverse kinematics numerically
	if (T.size() != link.size() || T.size() != includeOri.size() || T.size() != offset.size())
		printf("check number of input T, feature index, and includeOri\n");
	//vector<Eigen::VectorXd> qTrj(0);
	Eigen::VectorXd q(m_activeArmInfo->m_numJoint);
	if (initGuess.size() == m_activeArmInfo->m_numJoint)
		q = initGuess;
	else
		for (int i = 0; i < m_activeArmInfo->m_numJoint; i++)
			q(i) = (double)rand() / RAND_MAX * (m_upperJointLimit[i] - m_lowerJointLimit[i]) + m_lowerJointLimit[i];
	//cout << q.transpose() << endl;
	int Jrows = 0;
	for (unsigned int i = 0; i < includeOri.size(); i++)
	{
		if (includeOri[i])
			Jrows += 6;
		else
			Jrows += 3;
	}

	Eigen::MatrixXd J(Jrows, q.size());
	Eigen::VectorXd error(Jrows);
	Eigen::VectorXd delta_q(m_activeArmInfo->m_numJoint);
	Eigen::VectorXd rJointVal(delta_q.size());
	vector<SE3> tmpForKinSol;
	int iter = 0;
	int row_num;
	Vec3 tmpError;
	se3 tmpError_se3;
	double step_size = 0.1;
	while (iter < maxIter)
	{
		iter++;
		for (int i = 0; i < q.size(); i++)
			rJointVal[i] = q(i);
		tmpForKinSol = forwardKin(rJointVal, link, offset);
		// calculate jacobian & error
		row_num = 0;
		for (unsigned int i = 0; i < T.size(); i++)
		{
			if (includeOri[i])
			{
				if (metric == invKinMet::DG)
				{
					J.block(row_num, 0, 6, q.size()) = getAnalyticJacobian(rJointVal, link[i], true, offset[i]);
					tmpError = T[i].GetPosition() - tmpForKinSol[i].GetPosition();
					for (int j = 0; j < 3; j++)
						error(row_num + 3 + j) = tmpError[j];
					tmpError = 0.5*Log(T[i].GetOrientation()*Inv(tmpForKinSol[i].GetOrientation()));
					//tmpError = Cross(tmpForKinSol[featureIdx[i]].GetOrientation().GetZ(), T[i].GetZ()).Normalize() * acos(std::min(std::max(Inner(tmpForKinSol[featureIdx[i]].GetOrientation().GetZ(), T[i].GetZ()), -1.0), 1.0));
					for (int j = 0; j < 3; j++)
						error(row_num + j) = tmpError[j];
					row_num += 6;
				}
				else
				{
					J.block(row_num, 0, 6, q.size()) = getBodyJacobian(rJointVal, link[i], offset[i]);
					tmpError_se3 = Log(Inv(tmpForKinSol[i])*T[i]);
					for (int j = 0; j < 6; j++)
						error(row_num + j) = tmpError_se3[j];
					row_num += 6;
				}
			}
			else
			{
				J.block(row_num, 0, 3, q.size()) = getAnalyticJacobian(rJointVal, link[i], false, offset[i]);
				tmpError = T[i].GetPosition() - tmpForKinSol[i].GetPosition();
				for (int j = 0; j < 3; j++)
					error(row_num + j) = tmpError[j];
				row_num += 3;
			}
		}
		if (alg == invKinAlg::NR)
			delta_q = step_size * pinv(J)*error;
		else
			delta_q = getInverseKinUpdateQP(q, error, J, step_size);
		//cout << J << endl;
		//cout << pinv(J) << endl;
		//cout << q << endl;
		q += delta_q;
		if (delta_q.norm() < INVERSEKIN_TOL)
		{
			//moveIntoJointLimit(q, false);
			//printf("converged\n");
			flag = invKinFlag::SOLVED;
			if (alg == invKinAlg::NR)
				break;
			else if (error.norm() < 1e-3)
				break;
		}
	}
	if (iter == maxIter)
		flag = invKinFlag::EXCEED_MAX_ITER;
	bool insideJointLimit = moveIntoJointLimit(q, false);
	if (!insideJointLimit)
	{
		//printf("exceed joint limit !!!\n");
		if (iter == maxIter)
			flag = invKinFlag::EXCEED_MAX_ITER;
		else
			flag = invKinFlag::SOLVED_BUT_EXCEED_JOINT_LIM;
	}

	//if ((J*J.transpose).determinant() < 1e-5)
	//	flag = invKinFlag::SINGULARITY;

	/////////////////////////////
	//J.resize(0, 0);
	//error.resize(0);
	//delta_q.resize(0);
	//rJointVal.resize(0);
	/////////////////////////////
	return q;
}

Eigen::VectorXd robotManager::inverseKin(const SE3 & T, srLink * link, bool includeOri, SE3 offset, int & flag, Eigen::VectorXd initGuess, int maxIter, invKinAlg alg /*= (invKinAlg::NR)*/, invKinMet metric /*= (invKinMet::DG)*/)
{
	vector<SE3> Ts(1, T);
	vector<srLink*> links(1, link);
	vector<bool> includeOris(1, includeOri);
	vector<SE3> offsets(1, offset);
	return inverseKin(Ts, links, includeOris, offsets, flag, initGuess, maxIter, alg, metric);
}

double robotManager::manipulability(const Eigen::VectorXd & jointVal, srLink* link, manipKind kind /*= manipKind::INVCOND*/, Eigen::MatrixXd Select /*= Eigen::MatrixXd()*/)
{
	Eigen::MatrixXd Jb = getBodyJacobian(jointVal, link);
	if (Jb.cols() == Select.rows())
		Jb = Jb * Select;
	if (kind == manipKind::VOL)
	{
		if (Jb.rows() <= Jb.cols())
			return sqrt(max((Jb*Jb.transpose()).determinant(), 0.0));
		else
			return 0.0;
	}
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(Jb);
	svd.singularValues();
	double manip = 0.0;
	switch (kind)
	{
	case manipKind::INVCOND:
		manip = svd.singularValues()[min(Jb.rows(), Jb.cols()) - 1]/ svd.singularValues()[0];
		break;
	case manipKind::MIN:
		manip = svd.singularValues()[min(Jb.rows(), Jb.cols()) - 1];
		break;
	default:
		break;
	}
	return manip;
}

Eigen::VectorXd robotManager::manipulabilityGradient(const Eigen::VectorXd & jointVal, srLink * link, manipKind kind)
{
	double manip;
	return manipulabilityGradient(jointVal, link, manip, kind);
}

Eigen::VectorXd robotManager::manipulabilityGradient(const Eigen::VectorXd & jointVal, srLink * link, double & manipulability, manipKind kind)
{
	Eigen::MatrixXd Jb = getBodyJacobian(jointVal, link);
	Eigen::VectorXd manipGrad = Eigen::VectorXd::Zero(jointVal.size());
	switch (kind)
	{
	case manipKind::INVCOND:
		// TO DO
		break;
	case manipKind::MIN:
	{
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(Jb, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::VectorXd um, vm;
		um = svd.matrixU().col(min(Jb.rows(), Jb.cols()) - 1);
		vm = svd.matrixV().col(min(Jb.rows(), Jb.cols()) - 1);
		Eigen::VectorXd temp = Eigen::VectorXd::Zero(jointVal.size());
		Eigen::MatrixXd dJdq_i = Eigen::MatrixXd::Zero(Jb.rows(), Jb.cols());
		for (int i = 0; i < jointVal.size(); i++)
		{
			if (i > 0)
				temp[i - 1] = 0.0;
			temp[i] = 1.0;
			dJdq_i = getBodyJacobianDot(jointVal, temp, link);
			manipGrad[i] = um.dot(dJdq_i*vm);
		}
		manipulability = svd.singularValues()[min(Jb.rows(), Jb.cols()) - 1];
		break;
	}
	case manipKind::VOL:
		// TO DO
		break;
	default:
		break;
	}
	return manipGrad;
}

bool robotManager::checkCollision()
{
	// true: collision occurs
	return m_space->m_srDYN.RUNTIME_MARKKH();
}

Eigen::VectorXd robotManager::inverseDyn(const Eigen::VectorXd & jointVal, const Eigen::VectorXd & jointVel, const Eigen::VectorXd & jointAcc)
{
	// inverse dynamics of tree structure without consideration of external force except gravity
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(jointVal.size());
	if (jointVal.size() != m_activeArmInfo->m_numJoint || jointVel.size() != m_activeArmInfo->m_numJoint || jointAcc.size() != m_activeArmInfo->m_numJoint)
		cout << "Check number of joints !!!" << endl;

	srJoint* tempJoint;
	srLink* tempLink;
	int plinkIdx, ajointIdx, clinkIdx;
	se3 zero_se3 = se3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	dse3 zero_dse3 = dse3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	vector<SE3> T(m_robot->m_KIN_Links.get_size(), SE3());
	vector<se3> V(m_robot->m_KIN_Links.get_size(), zero_se3);
	vector<se3> Vdot(m_robot->m_KIN_Links.get_size(), zero_se3);
	vector<se3> S(m_robot->m_KIN_Links.get_size(), zero_se3);
	vector<dse3> F(m_robot->m_KIN_Links.get_size(), zero_dse3);
	double qi, qdoti, qddoti;
	
	for (int i = 0; i < 3; i++)
		Vdot[0][3 + i] = -m_space->m_Gravity[i];
	Vdot[0] = InvAd(m_robot->GetBaseLink()->GetFrame(), Vdot[0]);

	// forward iteration
	for (int i = 1; i < m_robot->m_KIN_Links.get_size(); i++)
	{
		// get current active joint idx
		tempJoint = m_robot->m_KIN_Links[i]->m_ParentJoint;
		ajointIdx = m_activeArmInfo->getActiveJointIdx(tempJoint);
		// get parent link idx
		plinkIdx = getLinkIdx(tempJoint->m_ParentLink);
		
		if (tempJoint->GetType() == srJoint::WELD)
		{
			T[i] = tempJoint->m_ParentLinkToJoint / tempJoint->m_ChildLinkToJoint;
			V[i] = InvAd(T[i], V[plinkIdx]);
			Vdot[i] = InvAd(T[i], Vdot[plinkIdx]);
		}
		else
		{
			if (tempJoint->GetType() == srJoint::REVOLUTE)
				S[i] = rot_se3;
			else
				S[i] = trans_se3;
			if (ajointIdx == -1)
			{
				// use current values for joints which is not chosen as active joints
				qi = ((srStateJoint*)tempJoint)->m_State.m_rValue[0];
				qdoti = ((srStateJoint*)tempJoint)->m_State.m_rValue[1];
				qddoti = ((srStateJoint*)tempJoint)->m_State.m_rValue[2];
			}
			else
			{
				qi = jointVal[ajointIdx];
				qdoti = jointVel[ajointIdx];
				qddoti = jointAcc[ajointIdx];
			}
			T[i] = tempJoint->m_ParentLinkToJoint * Exp(S[i], qi) / tempJoint->m_ChildLinkToJoint;
			S[i] = Ad(tempJoint->m_ChildLinkToJoint, S[i]);
			V[i] = InvAd(T[i], V[plinkIdx]) + S[i]*qdoti;
			Vdot[i] = InvAd(T[i], Vdot[plinkIdx]) + ad(V[i], S[i]*qdoti) + S[i]*qddoti;
		}
	}

	// backward iteration
	for (int i = m_robot->m_KIN_Links.get_size() - 1; i > -1; i--)
	{
		tempLink = m_robot->m_KIN_Links[i];
		F[i] = tempLink->m_Inertia*Vdot[i] - dad(V[i], tempLink->m_Inertia*V[i]);
		if (tempLink->m_ChildLinks.get_size() > 0)
		{
			// find child links
			for (int j = 0; j < tempLink->m_ChildLinks.get_size(); j++)
			{
				clinkIdx = getLinkIdx(tempLink->m_ChildLinks[j]);
				F[i] += InvdAd(T[clinkIdx], F[clinkIdx]);
			}
		}
		tempJoint = tempLink->m_ParentJoint;
		if (tempJoint != NULL)
		{
			ajointIdx = m_activeArmInfo->getActiveJointIdx(tempJoint);
			if (ajointIdx != -1)
				tau(ajointIdx) = F[i] * S[i];
		}
	}
	return tau;
}

Eigen::VectorXd robotManager::getBiasTerm(const Eigen::VectorXd & jointVal, const Eigen::VectorXd & jointVel)
{
	return inverseDyn(jointVal, jointVel, Eigen::VectorXd::Zero(m_activeArmInfo->m_numJoint));;
}

Eigen::MatrixXd robotManager::getMassMatrix(const Eigen::VectorXd & jointVal)
{
	Eigen::MatrixXd M(m_activeArmInfo->m_numJoint, m_activeArmInfo->m_numJoint);
	Eigen::VectorXd zeroVec = Eigen::VectorXd::Zero(m_activeArmInfo->m_numJoint);
	Eigen::VectorXd biasTerm = getBiasTerm(jointVal, zeroVec);
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(m_activeArmInfo->m_numJoint, m_activeArmInfo->m_numJoint);

	for (int i = 0; i < m_activeArmInfo->m_numJoint; i++)
		M.col(i) = inverseDyn(jointVal, zeroVec, I.col(i)) - biasTerm;

	// construct symmetric mass matrix
	//M = (M + M.transpose()) * 0.5;

	return M;
}

double robotManager::getTotalMass() const
{
	double mass = 0.0;
	for (int i = 0; i < m_robot->m_KIN_Links.get_size(); i++) 
		mass += (m_robot->m_KIN_Links[i])->GetMass();
	
	return mass;
}

void robotManager::exertExternalForceToLink(const dse3 & Fext, srLink* link, const SE3 & offset)
{
	link->AddUserExternalForce(InvdAd(offset, Fext));
}

se3 robotManager::rot_se3(0, 0, 1, 0, 0, 0);
se3 robotManager::trans_se3(0, 0, 0, 0, 0, 1);

gripperInfo::gripperInfo()
{
	m_gripJoint.resize(0);
	m_gripDummyJoint.resize(0);
	m_gripLink.resize(0);
}

gripperInfo::~gripperInfo()
{
}

void gripperInfo::loadGripperLink(srLink * gripperBase)
{
	m_gripBase = gripperBase;

	srLink* temp;
	srLink* curLink;
	for (int i = 0; i < gripperBase->m_pSystem->m_KIN_Links.get_size(); i++)
	{
		curLink = gripperBase->m_pSystem->m_KIN_Links[i];
		temp = curLink;
		while (1)
		{
			if (temp->m_ParentLink == gripperBase)
			{
				m_gripLink.push_back(curLink);
				break;
			}
			if (temp == gripperBase->m_pSystem->GetBaseLink())
				break;
			temp = temp->m_ParentLink;
		}
	}
	
	// second version
	//m_gripLink.resize(0);
	//temp = gripperBase;
	//for (int j = 0; j < temp->m_ChildLinks.get_size(); j++)
	//	m_gripLink.push_back(temp->m_ChildLinks[j]);
	//int num_add = 0;
	//int num_bf = 0;
	//int num_add_bf = temp->m_ChildLinks.get_size();
	//while (num_add_bf != 0)
	//{
	//	num_add = 0;
	//	for (int j = 0; j < num_add_bf; j++)
	//	{
	//		temp = m_gripLink[num_bf + j];
	//		for (int i = 0; i < temp->m_ChildLinks.get_size(); i++)
	//			m_gripLink.push_back(temp->m_ChildLinks[i]);
	//		num_add += temp->m_ChildLinks.get_size();
	//	}
	//	num_bf += num_add_bf;
	//	num_add_bf = num_add;
	//}
}

void gripperInfo::setGripperJoint(vector<srJoint*> joints)
{
	m_gripJoint.resize(joints.size());
	for (unsigned int i = 0; i < joints.size(); i++)
	{
		m_gripJoint[i] = joints[i];
	}
}

void gripperInfo::setGripperDummyJoint(vector<srJoint*> joints)
{
	m_gripDummyJoint.resize(joints.size());
	for (unsigned int i = 0; i < joints.size(); i++)
	{
		m_gripDummyJoint[i] = joints[i];
	}
}

ftSensorInfo::ftSensorInfo()
{
	m_linksAfterSensor.resize(0);
}

ftSensorInfo::~ftSensorInfo()
{
	m_linksAfterSensor.resize(0);
}

void ftSensorInfo::setSensorLocation(srWeldJoint * wJoint, SE3 offset /*= SE3()*/)
{
	m_sensorLocJoint = wJoint;
	m_offset = offset;
	m_linksAfterSensor = getChildLinksOfAJoint((srJoint*)wJoint);
}

dse3 ftSensorInfo::readSensorValue()
{
	return InvdAd((m_sensorLocJoint->m_Frame*m_offset) % m_sensorLocJoint->m_ChildLink->m_Frame, m_sensorLocJoint->m_FS_Force);
}

vector<srLink*> ftSensorInfo::getChildLinksOfAJoint(srJoint * Joint)
{
	srLink* tempLink = Joint->m_ChildLink;
	vector<srLink*> links(0);
	links.push_back(tempLink);
	unsigned int numBfLinks = 0;
	unsigned int numCurLinks = 1;
	int numCurChild = tempLink->m_ChildLinks.get_size();
	while (numCurChild > 0)
	{
		numCurChild = 0;
		for (unsigned int i = numBfLinks; i < numCurLinks; i++)
		{
			tempLink = links[i];
			for (int j = 0; j < tempLink->m_ChildLinks.get_size(); j++)
				links.push_back(tempLink->m_ChildLinks[j]);
			numCurChild += tempLink->m_ChildLinks.get_size();
		}
		numBfLinks = numCurLinks;
		numCurLinks = links.size();
	}
	return links;
}

dse3 ftSensorInfo::getInertialForceOfDistalLinks()
{
	/// inertial force represented at sensor frame
	dse3 Finertial(0.0);
	for (unsigned int i = 0; i < m_linksAfterSensor.size(); i++)
	{
		Finertial += InvdAd(m_sensorLocJoint->GetFrame() % m_linksAfterSensor[i]->GetFrame(), 
			m_linksAfterSensor[i]->m_Inertia * m_linksAfterSensor[i]->m_Acc - dad(m_linksAfterSensor[i]->m_Vel, m_linksAfterSensor[i]->m_Inertia * m_linksAfterSensor[i]->m_Vel));
	}
	return Finertial;
}

dse3 ftSensorInfo::getGravityForceOfDistalLinks(const se3& g)
{
	/// gravitational force represented at sensor frame
	//g: gravity expressed in global frame
	dse3 Fg(0.0);
	for (unsigned int i = 0; i < m_linksAfterSensor.size(); i++)
	{
		Fg += InvdAd(m_sensorLocJoint->GetFrame() % m_linksAfterSensor[i]->GetFrame(), m_linksAfterSensor[i]->m_Inertia*InvAd(m_linksAfterSensor[i]->GetFrame(), g));
	}
	return Fg;
}

dse3 ftSensorInfo::getExtForce(const se3& g)
{
	return getInertialForceOfDistalLinks() - getGravityForceOfDistalLinks(g) - readSensorValue();
}
