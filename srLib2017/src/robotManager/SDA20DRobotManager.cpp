#include "SDA20DRobotManager.h"


SDA20DManager::SDA20DManager(SDA20D * robot, srSpace * space, int ch, vector<srJoint*> excludeJoints /*= vector<srJoint*>()*/)
{
	this->setRobot((srSystem*)robot);
	this->setSpace(space);
	vector<srLink*> tempLinks(2);
	tempLinks[0] = &robot->gMarkerLink[SDA20D_Index::MLINK_RIGHT_T];
	tempLinks[1] = &robot->gMarkerLink[SDA20D_Index::MLINK_LEFT_T];
	srJoint* waistJoint = robot->gJoint[SDA20D_Index::JOINT_WAIST];
	switch (ch)
	{
	case SDA20DManager::MoveRightArmOnly:
		excludeJoints.push_back(waistJoint);
		this->m_activeArmInfo->setActiveArmInfoExclude((srSystem*)robot, tempLinks[0], excludeJoints);
		this->loadJointLimit();
		break;
	case SDA20DManager::MoveLeftArmOnly:
		excludeJoints.push_back(waistJoint);
		this->m_activeArmInfo->setActiveArmInfoExclude((srSystem*)robot, tempLinks[1], excludeJoints);
		this->loadJointLimit();
		break;
	case SDA20DManager::MoveBothArmOnly:
		excludeJoints.push_back(waistJoint);
		this->m_activeArmInfo->setActiveArmInfoExclude((srSystem*)robot, tempLinks, excludeJoints);
		this->loadJointLimit();
		break;
	case SDA20DManager::MoveRightArmWaist:
		this->m_activeArmInfo->setActiveArmInfoExclude((srSystem*)robot, tempLinks[0], excludeJoints);
		this->loadJointLimit();
		break;
	case SDA20DManager::MoveLeftArmWaist:
		this->m_activeArmInfo->setActiveArmInfoExclude((srSystem*)robot, tempLinks[1], excludeJoints);
		this->loadJointLimit();
		break;
	case SDA20DManager::MoveWholeBody:
		this->m_activeArmInfo->setActiveArmInfoExclude((srSystem*)robot, tempLinks, excludeJoints);
		this->loadJointLimit();
		break;
	}
	homePosActiveJoint.resize(this->m_activeArmInfo->m_numJoint);
	qInvKinInitActiveJoint.resize(this->m_activeArmInfo->m_numJoint);
	for (int i = 0; i < this->m_activeArmInfo->m_numJoint; i++)
	{
		for (int j = 0; j < NUM_OF_RJOINT_SDA20D; j++)
		{
			if (this->m_activeArmInfo->m_activeJoint[i] == robot->gJoint[j])
			{
				homePosActiveJoint[i] = robot->homePos[j];
				qInvKinInitActiveJoint[i] = robot->qInvKinInit[j];
				break;
			}	
		}
	}
	mode = ch;
}

SDA20DManager::~SDA20DManager()
{
}


//
//void SDA20DManager::flipLeftShoulder(vector<double>& jointVal)
//{
//	for (int i = 0; i < m_activearminfo->m_numJoint; i++)
//	{
//		if (m_activearminfo->m_activeJointIdx[i] == SDA20D_Index::JOINT_LEFT_S || m_activearminfo->m_activeJointIdx[i] == SDA20D_Index::JOINT_LEFT_E)
//		{
//			if (jointVal[i] < 0)
//				jointVal[i] += SR_PI;
//			else
//				jointVal[i] -= SR_PI;
//		}
//		if (m_activearminfo->m_activeJointIdx[i] == SDA20D_Index::JOINT_LEFT_L)
//			jointVal[i] *= -1.0;
//	}
//}
//
//void SDA20DManager::flipRightShoulder(vector<double>& jointVal)
//{
//	for (int i = 0; i < m_activearminfo->m_numJoint; i++)
//	{
//		if (m_activearminfo->m_activeJointIdx[i] == SDA20D_Index::JOINT_RIGHT_S || m_activearminfo->m_activeJointIdx[i] == SDA20D_Index::JOINT_RIGHT_E)
//		{
//			if (jointVal[i] < 0)
//				jointVal[i] += SR_PI;
//			else
//				jointVal[i] -= SR_PI;
//		}
//		if (m_activearminfo->m_activeJointIdx[i] == SDA20D_Index::JOINT_RIGHT_L)
//			jointVal[i] *= -1.0;
//	}
//}
//
//void SDA20DManager::flipLeftElbow(vector<double>& jointVal)
//{
//	for (int i = 0; i < m_activearminfo->m_numJoint; i++)
//	{
//		if (m_activearminfo->m_activeJointIdx[i] == SDA20D_Index::JOINT_LEFT_E || m_activearminfo->m_activeJointIdx[i] == SDA20D_Index::JOINT_LEFT_R)
//		{
//			if (jointVal[i] < 0)
//				jointVal[i] += SR_PI;
//			else
//				jointVal[i] -= SR_PI;
//		}
//		if (m_activearminfo->m_activeJointIdx[i] == SDA20D_Index::JOINT_LEFT_U)
//			jointVal[i] *= -1.0;		
//	}
//}
//
//void SDA20DManager::flipRightElbow(vector<double>& jointVal)
//{
//	for (int i = 0; i < m_activearminfo->m_numJoint; i++)
//	{
//		if (m_activearminfo->m_activeJointIdx[i] == SDA20D_Index::JOINT_RIGHT_E || m_activearminfo->m_activeJointIdx[i] == SDA20D_Index::JOINT_RIGHT_R)
//		{
//			if (jointVal[i] < 0)
//				jointVal[i] += SR_PI;
//			else
//				jointVal[i] -= SR_PI;
//		}
//		if (m_activearminfo->m_activeJointIdx[i] == SDA20D_Index::JOINT_RIGHT_U)
//			jointVal[i] *= -1.0;
//	}
//}
//
//void SDA20DManager::flipLeftWrist(vector<double>& jointVal)
//{
//	for (int i = 0; i < m_activearminfo->m_numJoint; i++)
//	{
//		if (m_activearminfo->m_activeJointIdx[i] == SDA20D_Index::JOINT_LEFT_R || m_activearminfo->m_activeJointIdx[i] == SDA20D_Index::JOINT_LEFT_T)
//		{
//			if (jointVal[i] < 0)
//				jointVal[i] += SR_PI;
//			else
//				jointVal[i] -= SR_PI;
//		}
//		if (m_activearminfo->m_activeJointIdx[i] == SDA20D_Index::JOINT_LEFT_B)
//			jointVal[i] *= -1.0;
//	}
//}
//
//void SDA20DManager::flipRightWrist(vector<double>& jointVal)
//{
//	for (int i = 0; i < m_activearminfo->m_numJoint; i++)
//	{
//		if (m_activearminfo->m_activeJointIdx[i] == SDA20D_Index::JOINT_RIGHT_R || m_activearminfo->m_activeJointIdx[i] == SDA20D_Index::JOINT_RIGHT_T)
//		{
//			if (jointVal[i] < 0)
//				jointVal[i] += SR_PI;
//			else
//				jointVal[i] -= SR_PI;
//		}
//		if (m_activearminfo->m_activeJointIdx[i] == SDA20D_Index::JOINT_RIGHT_B)
//			jointVal[i] *= -1.0;
//	}
//}


SDA20DDualArmClosedLoopConstraint::SDA20DDualArmClosedLoopConstraint(SDA20DManager * robotManager, const SE3 Tright2left)
{
	if (robotManager->mode == SDA20DManager::MoveBothArmOnly || robotManager->mode == SDA20DManager::MoveWholeBody)
	{
		_robotManager = robotManager;
		activeArmInfo* curArmInfo = _robotManager->m_activeArmInfo;
		for (unsigned int i = 0; i < min(curArmInfo->m_activeJointSet[0].size(), curArmInfo->m_activeJointSet[1].size()); i++)
		{
			if (curArmInfo->m_activeJointSet[0][i] == curArmInfo->m_activeJointSet[1][i])
				commonJointIdx.push_back(i);
		}
		_Tright2left = Tright2left;
		right_link = &((SDA20D*)_robotManager->m_robot)->gMarkerLink[SDA20D_Index::MLINK_RIGHT_T];
		left_link = &((SDA20D*)_robotManager->m_robot)->gMarkerLink[SDA20D_Index::MLINK_LEFT_T];
		numEffectiveArmJoints = curArmInfo->m_numJoint;
	}		
	else
	{
		// wrong robot mode for dual arm motion
		_robotManager = NULL;
	}
}

//void DualArmClosedLoopConstraint::setConstraintProblem(const SE3 _right2left)
//{
//	constraintFrame1to2 = _right2left;
//	
//
//	// extract effective joints
//	srJoint*	tempJoint;
//	srJoint*	tempCommonJoint;
//	srLink*		tempLink;
//	
//
//	activeArminfo* _activearminfo = _robotManager->getActiveArmInfo();
//	vector<int> leftArmJointIdx(0);
//	vector<int> rightArmJointIdx(0);
//	vector<int> commonJointIdx(0);
//	// find left joints
//	tempLink = left_link;
//	while (tempLink != _robotManager->getRobot()->GetBaseLink())
//	{
//		tempJoint = tempLink->m_ParentJoint;
//		for (int i = 0; i < _activearminfo->m_numJoint; i++) 
//		{
//			if (_robotManager->getRobot()->gJoint[_activearminfo->m_activeJointIdx[i]] == tempJoint) 
//			{
//				leftArmJointIdx.push_back(i);
//				break;
//			}
//		}
//		tempLink = tempJoint->m_ParentLink;
//	}
//	
//	// find right joints
//	tempLink = right_link;
//	while (tempLink != _robotManager->getRobot()->GetBaseLink())
//	{
//		tempJoint = tempLink->m_ParentJoint;
//		for (int i = 0; i < _activearminfo->m_numJoint; i++)
//		{
//			if (_robotManager->getRobot()->gJoint[_activearminfo->m_activeJointIdx[i]] == tempJoint)
//			{
//				rightArmJointIdx.push_back(i);
//				break;
//			}
//		}
//		tempLink = tempJoint->m_ParentLink;
//	}
//	
//	// find common joints
//	for (unsigned int i = 0; i < min(leftArmJointIdx.size(), rightArmJointIdx.size()); i++)
//	{
//		if (leftArmJointIdx[leftArmJointIdx.size() - 1 - i] == rightArmJointIdx[rightArmJointIdx.size() - 1 - i])
//			commonJointIdx.push_back(leftArmJointIdx[leftArmJointIdx.size() - 1 - i]);
//	}
//	numEffectiveLeftArmJoint = leftArmJointIdx.size() - commonJointIdx.size();
//	numEffectiveRightArmJoint = rightArmJointIdx.size() - commonJointIdx.size();
//	numEffectiveArmJoints = numEffectiveLeftArmJoint + numEffectiveRightArmJoint;
//	effectiveArmJointIdx.resize(numEffectiveArmJoints);
//
//	for (int i = 0; i < numEffectiveRightArmJoint; i++)
//		effectiveArmJointIdx[i] = rightArmJointIdx[rightArmJointIdx.size() - 1 - i - commonJointIdx.size()];
//	for (int i = 0; i < numEffectiveLeftArmJoint; i++)
//		effectiveArmJointIdx[i + numEffectiveRightArmJoint] = leftArmJointIdx[leftArmJointIdx.size() -  1 - i - commonJointIdx.size()];
//	
//}

Eigen::VectorXd SDA20DDualArmClosedLoopConstraint::getConstraintVector(const Eigen::VectorXd & jointVal)
{
	SE3 SE3diff;
	se3 logSE3diff;
	_robotManager->setJointVal(jointVal);
	SE3diff = (right_link->GetFrame() % left_link->GetFrame()) % _Tright2left;
	logSE3diff = Log(SE3diff);
	return -se3toVector(logSE3diff);
}

Eigen::MatrixXd SDA20DDualArmClosedLoopConstraint::getConstraintJacobian(const Eigen::VectorXd& jointVal)
{
	_robotManager->setJointVal(jointVal);
	Eigen::MatrixXd constraintJacobian = Eigen::MatrixXd::Zero(6, numEffectiveArmJoints);
	vector<se3> screw(numEffectiveArmJoints);

	srJoint* curJoint;
	SE3 SE3_endeffector = left_link->GetFrame();		// left end-effector is set as link2

	for (unsigned int i = 0; i < numEffectiveArmJoints; i++) {
		curJoint = _robotManager->m_activeArmInfo->m_activeJoint[i];
		if (curJoint->GetType() == srJoint::REVOLUTE)
			screw[i].Ad(SE3_endeffector % (curJoint->GetFrame()), se3(0, 0, 1, 0, 0, 0));
		else if (curJoint->GetType() == srJoint::PRISMATIC)
			screw[i].Ad(SE3_endeffector % (curJoint->GetFrame()), se3(0, 0, 0, 0, 0, 1));
	}
	for (int i = 0; i < 6; i++) {
		// joint order should follow (common joints, right arm joints, left arm joints)
		for (unsigned int j = commonJointIdx.size(); j < numEffectiveArmJoints; j++) {
			if (j < _robotManager->m_activeArmInfo->m_activeJointSet[0].size())
				constraintJacobian(i, j) = -screw[j][i];
			else
				constraintJacobian(i, j) = screw[j][i];
		}
	}

	return constraintJacobian;
}

Eigen::VectorXd SDA20DDualArmClosedLoopConstraint::getConstraintHessian(const Eigen::VectorXd & jointVal, unsigned int i, unsigned int j)
{
	return Eigen::VectorXd();
}

int SDA20DDualArmClosedLoopConstraint::project2ConstraintManifold(Eigen::VectorXd& jointVal, int max_iter /*= 1000*/)
{
	Eigen::MatrixXd Jc;
	Eigen::VectorXd delta_q(numEffectiveArmJoints);
	Eigen::MatrixXd Q(numEffectiveArmJoints, numEffectiveArmJoints);
	Q.setIdentity();
	Eigen::VectorXd f;
	double norm_delta_q = 1;
	double f_norm = 1;
	int iter = 0;
	while (iter < max_iter) {
		Jc = getConstraintJacobian(jointVal);
		f = -getConstraintVector(jointVal);
		f_norm = f.norm();
		delta_q = Q.inverse()*Jc.transpose()*(Jc*Q.inverse()*Jc.transpose()).inverse()*f;
		for (unsigned int i = 0; i < numEffectiveArmJoints; i++)
			jointVal(i) += 0.5*delta_q(i);
		norm_delta_q = delta_q.norm();
		//cout << "normq: " << norm_delta_q << endl;
		if ((norm_delta_q < INVERSEKIN_TOL))
		{
			// implement quadratic programming...??
			Eigen::VectorXd temp = jointVal;
			_robotManager->moveIntoJointLimit(jointVal);
			if ((temp-jointVal).norm() < 1E-3)
				return Success;
			return Exeeded_Joint_Limit;
		}
		iter++;
	}
	return Exeeded_Max_Iter;
}
