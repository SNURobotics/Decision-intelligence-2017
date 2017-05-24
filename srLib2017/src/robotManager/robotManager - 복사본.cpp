#include "robotManager.h"

activeArmInfo::activeArmInfo()
{

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
	int maxJ, maxJbf = 0;
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

void activeArmInfo::setActiveArmInfo(srSystem * robot, vector<srLink*> lastLink, vector<srJoint*> activeJoint)
{
	m_endeffector = lastLink;
	m_activeJoint = activeJoint;
	m_numJoint = activeJoint.size();
}

void activeArmInfo::setHybrid()
{
	for (unsigned int i = 0; i < m_activeJoint.size(); i++)
		m_activeJoint[i]->SetActType(srJoint::HYBRID);
}

int activeArmInfo::getActiveJointIdx(srJoint * joint)
{
	int idx = -1;
	for (int i = 0; i < m_numJoint; i++)
	{
		if (joint == m_activeJoint[i])
		{
			idx = i;
			break;
		}
	}
	return idx;
}

robotManager::robotManager()
{
	m_activeArmInfo = new activeArmInfo();
	m_activeArmInfoCopy = new activeArmInfo();
}


robotManager::~robotManager()
{
	delete m_activeArmInfo;
	delete m_activeArmInfoCopy;
}

void robotManager::setRobot(srSystem * robot, srSystem * robotcopy)
{
	m_robot = robot;
	m_robotCopy = robotcopy;
}

void robotManager::setSpace(srSpace * space, srSpace * spacecopy)
{
	m_space = space;
	m_spaceCopy = spacecopy;
}

void robotManager::setEndeffector(srLink * lastLink, srLink * lastLinkCopy)
{
	m_activeArmInfo->setActiveArmInfo(m_robot, lastLink);
	m_activeArmInfo->setHybrid();
	m_activeArmInfoCopy->setActiveArmInfo(m_robotCopy, lastLinkCopy);
	m_activeArmInfoCopy->setHybrid();
}

void robotManager::setEndeffector(vector<srLink*> lastLink, vector<srLink*> lastLinkCopy)
{
	m_activeArmInfo->setActiveArmInfo(m_robot, lastLink);
	m_activeArmInfo->setHybrid();
	m_activeArmInfoCopy->setActiveArmInfo(m_robotCopy, lastLinkCopy);
	m_activeArmInfoCopy->setHybrid();
}

void robotManager::setEndeffector(string lastLinkName, string lastLinkNameCopy)
{
	m_activeArmInfo->setActiveArmInfo(static_cast<gamasot::srRobot*>(m_robot), lastLinkName);
	m_activeArmInfo->setHybrid();
	m_activeArmInfoCopy->setActiveArmInfo(static_cast<gamasot::srRobot*>(m_robotCopy), lastLinkNameCopy);
	m_activeArmInfoCopy->setHybrid();
}

void robotManager::setEndeffector(vector<string> lastLinkName, vector<string> lastLinkNameCopy)
{
	m_activeArmInfo->setActiveArmInfo(static_cast<gamasot::srRobot*>(m_robot), lastLinkName);
	m_activeArmInfo->setHybrid();
	m_activeArmInfoCopy->setActiveArmInfo(static_cast<gamasot::srRobot*>(m_robotCopy), lastLinkNameCopy);
	m_activeArmInfoCopy->setHybrid();
}

bool robotManager::containLink(srLink * link)
{
	bool isContained = false;
	for (int i = 0; i < m_robot->m_KIN_Links.get_size(); i++)
		if (link == m_robot->m_KIN_Links[i])
		{
			isContained = true;
			break;
		}
	return isContained;
}

void robotManager::setJointVal(const Eigen::VectorXd & jointVal)
{
	setJointVal(jointVal, m_robot);
	if (m_robotCopy != NULL)
		setJointVal(jointVal, m_robotCopy);
}

void robotManager::setJointVal(const Eigen::VectorXd& jointVal, srSystem* robot)
{
	activeArmInfo* _activeArmInfo = new activeArmInfo;
	if (robot == m_robot)
		_activeArmInfo = m_activeArmInfo;
	else if (robot == m_robotCopy)
		_activeArmInfo = m_activeArmInfoCopy;
	if (jointVal.size() != _activeArmInfo->m_numJoint)
		std::cout << "check number of joint values!!! (function: setJointVal)" << std::endl;

	for (int i = 0; i < _activeArmInfo->m_numJoint; i++)
		((srStateJoint*)_activeArmInfo->m_activeJoint[i])->m_State.m_rValue[0] = jointVal[i];

	robot->KIN_UpdateFrame_All_The_Entity();
}

void robotManager::setJointValVel(const Eigen::VectorXd & jointVal, const Eigen::VectorXd & jointVel)
{
	setJointValVel(jointVal, jointVel, m_robot);
	if (m_robotCopy != NULL)
		setJointValVel(jointVal, jointVel, m_robotCopy);
}

void robotManager::setJointValVel(const Eigen::VectorXd & jointVal, const Eigen::VectorXd & jointVel, srSystem* robot)
{
	activeArmInfo* _activeArmInfo = new activeArmInfo;
	if (robot == m_robot)
		_activeArmInfo = m_activeArmInfo;
	else if (robot == m_robotCopy)
		_activeArmInfo = m_activeArmInfoCopy;
	if (jointVal.size() != m_activeArmInfo->m_numJoint || jointVel.size() != _activeArmInfo->m_numJoint)
		std::cout << "check number of joint values!!! (function: setJointValVel)" << std::endl;

	for (int i = 0; i < m_activeArmInfo->m_numJoint; i++)
	{
		((srStateJoint*)m_activeArmInfo->m_activeJoint[i])->m_State.m_rValue[0] = jointVal[i];
		((srStateJoint*)m_activeArmInfo->m_activeJoint[i])->m_State.m_rValue[1] = jointVel[i];
	}
	
	robot->KIN_UpdateFrame_All_The_Entity();
}

void robotManager::setJointValVelAcc(const Eigen::VectorXd & jointVal, const Eigen::VectorXd & jointVel, const Eigen::VectorXd & jointAcc)
{
	setJointValVelAcc(jointVal, jointVel, jointAcc, m_robot);
	if (m_robotCopy != NULL)
		setJointValVelAcc(jointVal, jointVel, jointAcc, m_robotCopy);
}

void robotManager::setJointValVelAcc(const Eigen::VectorXd & jointVal, const Eigen::VectorXd & jointVel, const Eigen::VectorXd & jointAcc, srSystem * robot)
{
	srStateJoint* curJoint;
	activeArmInfo* _activeArmInfo = new activeArmInfo;
	if (robot == m_robot)
		_activeArmInfo = m_activeArmInfo;
	else if (robot == m_robotCopy)
		_activeArmInfo = m_activeArmInfoCopy;

	if (jointVal.size() != _activeArmInfo->m_numJoint || jointVel.size() != _activeArmInfo->m_numJoint || jointAcc.size() != _activeArmInfo->m_numJoint)
		cout << "Check number of joints !!! (function: setJointValVelAcc)" << endl;


	for (int i = 0; i < _activeArmInfo->m_numJoint; i++) {
		curJoint = (srStateJoint*)_activeArmInfo->m_activeJoint[i];
		curJoint->m_State.m_rValue[0] = jointVal[i];
		curJoint->m_State.m_rValue[1] = jointVel[i];
		//curJoint->m_State.m_rValue[2] = 0.0;
		curJoint->m_State.m_rValue[3] = 0.0;
		curJoint->m_State.m_rCommand = jointAcc[i];
	}
	robot->KIN_UpdateFrame_All_The_Entity();
	robot->DIFFKIN_LinkVelocityPropagation();
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

void robotManager::setJointLimit(const Eigen::VectorXd & upper, const Eigen::VectorXd & lower)
{
	if (upper.size() != m_activeArmInfo->m_numJoint || upper.size() != m_activeArmInfo->m_numJoint)
		printf("check joint limit dimension\n");
	m_upperJointLimit.resize(m_activeArmInfo->m_numJoint);
	m_lowerJointLimit.resize(m_activeArmInfo->m_numJoint);
	for (int i = 0; i < m_activeArmInfo->m_numJoint; i++)
	{
		m_upperJointLimit[i] = upper(i);
		m_lowerJointLimit[i] = lower(i);
	}
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

bool robotManager::checkJointLimit(const Eigen::VectorXd & q, vector<int>& exceedIdx, bool print)
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

bool robotManager::checkJointLimit(const Eigen::VectorXd & q, vector<int>& upperIdx, vector<int>& lowerIdx)
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
	double dist1;
	double dist2;
	double tmp1;
	double tmp2;
	for (int i = 0; i < q.size(); i++)
	{
		tmp1 = q(i);
		tmp2 = q(i);
		dist1 = 0.0;
		dist2 = 0.0;
		while (tmp1 > m_upperJointLimit[i])
		{
			tmp1 -= SR_PI * 2.0;
			dist1 = min(dist1, tmp1 - m_lowerJointLimit[i]);
		}
		while (tmp2 < m_lowerJointLimit[i])
		{
			tmp2 += SR_PI*2.0;
			dist2 = min(dist2, -tmp2 + m_upperJointLimit[i]);
		}

		if (abs(dist1) > abs(dist2))
			q(i) = tmp2;
		else
			q(i) = tmp1;
	}

	if (print)
	{
		vector<int> exceedIdx(0);
		return checkJointLimit(q, exceedIdx, print);
	}
	else
		return checkJointLimit(q);
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
	setJointVal(jointVal);
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

Eigen::VectorXd robotManager::inverseKin(const vector<SE3>& T, vector<srLink*> link, vector<bool> includeOri, vector<SE3> offset, int & flag, Eigen::VectorXd initGuess, int maxIter)
{
	// solve inverse kinematics numerically
	if (T.size() != link.size() || T.size() != includeOri.size() || T.size() != offset.size())
		printf("check number of input T, feature index, and includeOri\n");
	vector<Eigen::VectorXd> qTrj(0);
	Eigen::VectorXd q(m_activeArmInfo->m_numJoint);
	if (initGuess.size() == m_activeArmInfo->m_numJoint)
		q = initGuess;
	else
		for (int i = 0; i < m_activeArmInfo->m_numJoint; i++)
			q(i) = (double)rand() / RAND_MAX * (m_upperJointLimit[i] - m_lowerJointLimit[i]) + m_lowerJointLimit[i];

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
				J.block(row_num, 0, 3, q.size()) = getAnalyticJacobian(rJointVal, link[i], false, offset[i]);
				tmpError = T[i].GetPosition() - tmpForKinSol[i].GetPosition();
				for (int j = 0; j < 3; j++)
					error(row_num + j) = tmpError[j];
				row_num += 3;
			}
		}
		delta_q = step_size * pinv(J)*error;
		//cout << J << endl;
		//cout << pinv(J) << endl;
		q += delta_q;
		if (delta_q.norm() < INVERSEKIN_TOL)
		{
			printf("converged\n");
			flag = invKinFlag::SOLVED;
			break;
		}
	}
	if (iter == maxIter)
		flag = invKinFlag::EXCEED_MAX_ITER;
	bool insideJointLimit = moveIntoJointLimit(q, true);
	if (!insideJointLimit)
	{
		//printf("exceed joint limit !!!\n");
		flag = invKinFlag::EXCEED_JOINT_LIM;
	}
	return q;
}

bool robotManager::checkCollision()
{
	// true: collision occurs
	return m_space->_KIN_COLLISION_RUNTIME_SIMULATION_LOOP();
}

Eigen::VectorXd robotManager::inverseDyn2(const Eigen::VectorXd & jointVal, const Eigen::VectorXd & jointVel, const Eigen::VectorXd & jointAcc)
{
	// use copy space
	bool isHybrid = true;
	for (int i = 0; i < m_activeArmInfoCopy->m_numJoint; i++) 
	{
		if (m_activeArmInfoCopy->m_activeJoint[i]->GetActType() != srJoint::HYBRID) 
		{
			isHybrid = false;
			cout << "check joint acttype for inverse dynamics!!!" << endl;
		}
	}

	Eigen::VectorXd jointTorque = Eigen::VectorXd::Zero(m_activeArmInfoCopy->m_numJoint);
	
	if (isHybrid == true) 
	{
		srStateJoint* curJoint;
		if (jointVal.size() != m_activeArmInfoCopy->m_numJoint || jointVel.size() != m_activeArmInfoCopy->m_numJoint || jointAcc.size() != m_activeArmInfoCopy->m_numJoint)
			cout << "Check number of joints !!!" << endl;

		for (int i = 0; i < m_activeArmInfoCopy->m_numJoint; i++) 
		{
			curJoint = (srStateJoint*) m_activeArmInfoCopy->m_activeJoint[i];
			curJoint->m_State.m_rValue[0] = jointVal[i];
			curJoint->m_State.m_rValue[1] = jointVel[i];
			//curJoint->m_State.m_rValue[2] = 0.0;
			curJoint->m_State.m_rValue[3] = 0.0;
			curJoint->m_State.m_rCommand = jointAcc[i];
		}
		m_robotCopy->KIN_UpdateFrame_All_The_Entity();
		m_robotCopy->DIFFKIN_LinkVelocityPropagation();
		m_spaceCopy->DYN_MODE_RUNTIME_SIMULATION_LOOP();
		//cout << "q = " << getJointVal(m_robotCopy) << endl;
		//cout << "dotq = " << getJointVel(m_robotCopy) << endl;
		//cout << "dotdotq = " << getJointAcc(m_robotCopy) << endl;
		for (int i = 0; i < m_activeArmInfoCopy->m_numJoint; i++) 
		{
			curJoint = (srStateJoint*)m_activeArmInfoCopy->m_activeJoint[i];
			jointTorque(i) = curJoint->m_State.m_rValue[3];
		}
	}
	return jointTorque;
}

Eigen::VectorXd robotManager::inverseDyn(const Eigen::VectorXd & jointVal, const Eigen::VectorXd & jointVel, const Eigen::VectorXd & jointAcc)
{
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(jointVal.size());
	// inverse dynamics of tree structure without consideration of external force except gravity
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
		tempLink = tempJoint->m_ParentLink;
		for (int j = 0; j < m_robot->m_KIN_Links.get_size(); j++)
			if (m_robot->m_KIN_Links[j] == tempLink)
			{
				plinkIdx = j;
				break;
			}
		
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

	//vector<SE3> curT(T.size(), SE3());
	//vector<se3> curVdot(m_robot->m_KIN_Links.get_size(), zero_se3);
	//vector<dse3> Fg(m_robot->m_KIN_Links.get_size(), zero_dse3);
	//for (unsigned int i = 0; i < T.size(); i++)
	//{
	//	if (i > 0)
	//	{
	//		curT[i] = curT[i - 1] * T[i];
	//		curVdot[i] = Vdot[i] - InvAd(curT[i], Vdot[0]);
	//	}		
	//	Fg[i] = m_robot->m_KIN_Links[i]->m_Inertia * InvAd(curT[i], - Vdot[0]);
	//}

	// backward iteration
	for (int i = m_robot->m_KIN_Links.get_size() - 1; i > -1; i--)
	{
		tempLink = m_robot->m_KIN_Links[i];
		F[i] = tempLink->m_Inertia*Vdot[i] - dad(V[i], tempLink->m_Inertia*V[i]);
		if (tempLink->m_ChildLinks.get_size() > 0)
		{
			for (int j = 0; j < tempLink->m_ChildLinks.get_size(); j++)
			{
				for (int k = 0; k < m_robot->m_KIN_Links.get_size(); k++)
					if (m_robot->m_KIN_Links[k] == tempLink->m_ChildLinks[j])
					{
						clinkIdx = k;
						break;
					}
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