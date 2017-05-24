#include "impedanceCtrlManager.h"
#include <windows.h>

impedanceCtrlManager::impedanceCtrlManager()
{
	Tdes_trj.resize(0);
	Vdes_trj.resize(0);
	dotVdes_trj.resize(0);
	SE3spline = NULL;
	cocFrame = EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0));
}

impedanceCtrlManager::~impedanceCtrlManager()
{
}

bool impedanceCtrlManager::setSystem(robotManager * _robotManager, vector<srLink*> _endeffector, vector<SE3> _offset, vector<SE3> _robotToObject)
{
	if (_endeffector.size() != _offset.size())
		return false;
	if (_endeffector.size() != _robotToObject.size())
		return false;
	for (unsigned int j = 0; j < _endeffector.size(); j++)
	{
		if (!_robotManager->containLink(_endeffector[j]))
			return false;
	}
	m_robotManagers.resize(1);
	m_endeffectors.resize(1);
	m_offsets.resize(1);
	m_robotToObject.resize(1);
	m_robotManagers[0] = _robotManager;
	m_endeffectors[0] = _endeffector;
	m_offsets[0] = _offset;
	m_robotToObject[0] = _robotToObject;
	return true;
}

bool impedanceCtrlManager::setSystem(vector<robotManager*> _robotManagers, vector<vector<srLink*>> _endeffectors, vector<vector<SE3>> _offsets, vector<vector<SE3>> _robotToObject)
{
	if (_robotManagers.size() != _endeffectors.size())
		return false;
	if (_robotManagers.size() != _offsets.size())
		return false;
	if (_robotManagers.size() != _robotToObject.size())
		return false;
	for (unsigned int i = 0; i < _robotManagers.size(); i++)
	{
		if (_endeffectors[i].size() != _offsets[i].size())
			return false;
		if (_endeffectors[i].size() != _robotToObject[i].size())
			return false;
		for (unsigned int j = 0; j < _endeffectors[i].size(); j++)
		{
			if (!_robotManagers[i]->containLink(_endeffectors[i][j]))
				return false;
		}
	}
	m_robotManagers = _robotManagers;
	m_endeffectors = _endeffectors;
	m_offsets = _offsets;
	m_robotToObject = _robotToObject;
	return true;
}

void impedanceCtrlManager::setObject(srSystem* _object)
{
	//object = _object;
	objectBaseLink = _object->GetBaseLink();
	objectLinks.resize(0);
	objectRefFrame = SE3();
	for (int i = 0; i < _object->m_KIN_Links.get_size(); i++)
		objectLinks.push_back(_object->m_KIN_Links[i]);
	setObjectRefFrame(objectRefFrame);
}

void impedanceCtrlManager::setObjectLinks(srLink * _objectBaseLink, vector<srLink*> _objectLinks)
{
	objectBaseLink = _objectBaseLink;
	objectLinks = _objectLinks;
}

void impedanceCtrlManager::setObjectRefFrame(SE3 T)
{
	// project inertias to object reference frame (T is SE3 from objectBaseLink frame to ref frame)
	// check if the frame of all the child links are updated
	objectRefFrame = T;
	objectInertia.SetMass(0.0);
	objectInertia.SetAngularMoment(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	objectInertia.SetOffset(Vec3(0.0, 0.0, 0.0));
	for (unsigned int i = 0; i < objectLinks.size(); i++)
		objectInertia = objectInertia + objectLinks[i]->GetInertiaRef().Transform(objectLinks[i]->GetFrame() % objectBaseLink->GetFrame() * objectRefFrame);
}

void impedanceCtrlManager::setInertial(Eigen::MatrixXd _inertial)
{
	inertial = _inertial;
}

void impedanceCtrlManager::setResistive(Eigen::MatrixXd _resistive)
{
	resistive = _resistive;
}

void impedanceCtrlManager::setCapacitive(Eigen::MatrixXd _capacitive)
{
	capacitive = _capacitive;
}

void impedanceCtrlManager::setIntegralGain(Eigen::MatrixXd _integral)
{
	m_useIntegralGain = true;
	integralGain = _integral;
}

se3 impedanceCtrlManager::getDesiredObjectAcc(dse3 Fext, SE3 T, se3 V, SE3 Tdes, se3 Vdes, se3 dotVdes, bool useIntegralGain /*= false*/, int metric)
{
	Eigen::VectorXd Ve;
	Ve = se3toVector(V - InvAd(T, Vdes));
	SE3 e = Tdes%T;
	if (useIntegralGain)
		updateIntegralError(e, metric);
	Eigen::VectorXd Utr = inertial*se3toVector(InvAd(T, dotVdes) + ad(InvAd(T, Vdes), V));
	
	Eigen::VectorXd Z(6);
	if (metric == DoubleGeodesic){
		Z << - capacitive.block<3, 3>(0, 0)*Vec3toVector(Log(e.GetOrientation())), - SO3toMatrix(Inv(e.GetOrientation())) * capacitive.block<3, 3>(3, 3) * Vec3toVector(e.GetPosition());
		Z -= resistive*Ve;
		if (useIntegralGain)
			Z -= integralGain*m_integral_error;
	}
	else
	{
		Z = -resistive*Ve - capacitive*se3toVector(Log(e));
		if (useIntegralGain)
			Z -= integralGain*m_integral_error;
	}
		
	return Vectortose3(inertial.inverse()*(Utr + Z + dse3toVector(Fext)));
}

dse3 impedanceCtrlManager::getExternalForce(dse3 Fgen) const
{
	SE3 curFrame;
	dse3 Fcur;
	dse3 Fext(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	for (unsigned int i = 0; i < objectLinks.size(); i++){
		Fcur = objectLinks[i]->GetExtForce();
		curFrame = objectLinks[i]->GetFrame();
		Fext += InvdAd((objectBaseLink->GetFrame()*objectRefFrame)%curFrame, Fcur);
	}
	Fext -= Fgen;
	return Fext;
}

dse3 impedanceCtrlManager::getTotalExternalForce() const
{
	SE3 curFrame;
	dse3 Fcur;
	dse3 Fext(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	for (unsigned int i = 0; i < objectLinks.size(); i++){
		Fcur = objectLinks[i]->GetExtForce();
		curFrame = objectLinks[i]->GetFrame();
		//cout << i << "th link force = " << dse3toVector(Fcur) << endl;
		//cout << objectRefFrame % curFrame << endl;
		Fext += InvdAd((objectBaseLink->GetFrame()*objectRefFrame)%curFrame, Fcur);
	}
	return Fext;
}

dse3 impedanceCtrlManager::getContactForce() const
{
	dse3 Fcur;
	dse3 Fcont(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	for (unsigned int i = 0; i < objectLinks.size(); i++)
	{
		Fcur = objectLinks[i]->m_ConstraintImpulse * (1.0 / timeStep);
		//cout << i << "th link force = " << dse3toVector(Fcur) << endl;
		//cout << objectRefFrame % curFrame << endl;
		Fcont += InvdAd((objectBaseLink->GetFrame()*objectRefFrame) % objectLinks[i]->GetFrame(), Fcur);
	}
	return Fcont;
}

dse3 impedanceCtrlManager::generateImpedanceForce(dse3 Fext, SE3 T, se3 V, SE3 Tdes, se3 Vdes, se3 dotVdes, bool useIntegralGain /*= false*/, int metric)
{
	se3 dotVctrl = getDesiredObjectAcc(Fext, T, V, Tdes, Vdes, dotVdes, useIntegralGain, metric);
	dse3 Fdes = objectInertia*dotVctrl - dad(V, objectInertia*V);
	return Fdes - Fext;
}

Inertia impedanceCtrlManager::getGeneralizedInertia() const
{
	return objectInertia;
}

//Eigen::VectorXd impedanceCtrlManager::getDesiredJointAcc(Eigen::VectorXd jointVal, Eigen::VectorXd jointVel, se3 objectAcc)
//{
//	Eigen::VectorXd jointAcc(jointVal.size());
//	if (m_robotManager->getPlanningMode() == DualArmRobotManager::MoveLeftArmOnly || m_robotManager->getPlanningMode() == DualArmRobotManager::MoveRightArmOnly || m_robotManager->getPlanningMode() == DualArmRobotManager::MoveLeftArmWaist || m_robotManager->getPlanningMode() == DualArmRobotManager::MoveRightArmWaist){
//		se3 robotEndEffectorAcc = Ad(endeffectorToObject[0], objectAcc);
//		Eigen::MatrixXd Jb = m_robotManager->getBodyJacobian(jointVal);
//
//		Eigen::MatrixXd JJ = Jb*Jb.transpose();
//		Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(JJ);
//		
//		Eigen::VectorXd v_JJ = eigensolver.eigenvalues();
//		double condNum = v_JJ(0) / v_JJ(v_JJ.size()-1);
//		//cout << "Condition Number: " << condNum << endl;
//
//		if (Jb.cols() < 6)
//			cout << "deficient manipulator!!!" << endl;
//		else if (Jb.cols() == 6)
//			jointAcc = Jb.inverse()*(se3toVector(robotEndEffectorAcc) - m_robotManager->getBodyJacobianDot(jointVal, jointVel)*jointVel);
//		else{
//			Eigen::MatrixXd Q = m_robotManager->generateWeightMatrix(true);
//			jointAcc = Q.inverse()*Jb.transpose()*(Jb*Q.inverse()*Jb.transpose()).inverse()*(se3toVector(objectAcc) - m_robotManager->getBodyJacobianDot(jointVal, jointVel)*jointVel);
//		}
//	}
//	else if (m_robotManager->getPlanningMode() == DualArmRobotManager::MoveBothArmOnly || m_robotManager->getPlanningMode() == DualArmRobotManager::MoveWholeBody){
//		se3 leftEndEffectorAcc = Ad(endeffectorToObject[0], objectAcc);
//		se3 rightEndEffectorAcc = Ad(endeffectorToObject[1], objectAcc);
//		Eigen::MatrixXd Jb = m_robotManager->generateAugmentedJacobian(jointVal, false);
//		Eigen::VectorXd robotEndEffectorAcc(12);
//		robotEndEffectorAcc << se3toVector(leftEndEffectorAcc), se3toVector(rightEndEffectorAcc);
//		Eigen::MatrixXd dotJb = m_robotManager->generateAugmentedJacobianDot(jointVal, jointVel, false);
//		Eigen::MatrixXd Q = m_robotManager->generateWeightMatrix(false);
//		if (Jb.cols() < 12)
//			cout << "deficient manipulator!!!" << endl;
//		else{
//			Eigen::MatrixXd Q = m_robotManager->generateWeightMatrix(true);
//			jointAcc = Q.inverse()*Jb.transpose()*(Jb*Q.inverse()*Jb.transpose()).inverse()*(robotEndEffectorAcc - dotJb*jointVel);
//		}
//	}
//	return jointAcc;
//}

Eigen::VectorXd impedanceCtrlManager::getDesiredJointAcc(Eigen::VectorXd jointVal, Eigen::VectorXd jointVel, unsigned int robotIdx, se3 objectAcc)
{
	Eigen::MatrixXd Jb = m_robotManagers[robotIdx]->getBodyJacobian(jointVal, m_endeffectors[robotIdx], m_offsets[robotIdx]);
	Eigen::MatrixXd Jb_dot = m_robotManagers[robotIdx]->getBodyJacobianDot(jointVal, jointVel, m_endeffectors[robotIdx], m_offsets[robotIdx]);
	Eigen::VectorXd robotEndEffectorAcc(6*m_endeffectors[robotIdx].size());
	int idx = 0;
	SE3 temp;
	for (unsigned int i = 0; i < m_endeffectors[robotIdx].size(); i++)
	{
		temp = (m_endeffectors[robotIdx][i]->GetFrame() * m_offsets[robotIdx][i]) % objectBaseLink->GetFrame()*objectRefFrame;
		robotEndEffectorAcc.segment(idx, 6) = se3toVector(Ad(temp/*m_robotToObject[robotIdx][i]*/, objectAcc));
		idx += 6;
	}
	return pinv(Jb)*(robotEndEffectorAcc - Jb_dot*jointVel);
}


void impedanceCtrlManager::amendJointValVel(vector<Eigen::VectorXd> jointVal)
{
	SE3 Tcur = objectBaseLink->GetFrame() * objectRefFrame;
	se3 Vcur = InvAd(objectRefFrame, objectBaseLink->GetVel());
	Eigen::VectorXd Vendeffector;
	Eigen::VectorXd jointVel;
	vector<SE3> Tendeffector;
	vector<bool> includeOri;
	int flag;
	int idx;
	for (unsigned int i = 0; i < m_robotManagers.size(); i++)
	{
		idx = 0;
		Tendeffector.resize(m_endeffectors[i].size());
		includeOri.resize(m_endeffectors[i].size());
		Vendeffector.resize(6 * m_endeffectors[i].size());
		for (unsigned int j = 0; j < m_endeffectors[i].size(); j++)
		{
			//printf("manipulability of robot %d, ee %d: %f\n", i, j, m_robotManagers[i]->manipulability(jointVal[i], m_endeffectors[i][j]));
			Tendeffector[j] = Tcur / m_robotToObject[i][j];
			includeOri[j] = true;
			Vendeffector.segment(idx, 6) = se3toVector(Ad(m_robotToObject[i][j], Vcur));
			idx += 6;
		}
		jointVal[i] = m_robotManagers[i]->inverseKin(Tendeffector, m_endeffectors[i], includeOri, m_offsets[i], flag, jointVal[i]);
		jointVel = pinv(m_robotManagers[i]->getBodyJacobian(jointVal[i], m_endeffectors[i], m_offsets[i])) * Vendeffector;
		m_robotManagers[i]->setJointValVel(jointVal[i], jointVel);
	}
	
}

double impedanceCtrlManager::getKinematicsError(const SE3& Tcur, const vector<Eigen::VectorXd>& jointVal)
{
	double maxError = 0.0;
	SE3 Tdes;
	SE3 Trobot;
	for (unsigned int i = 0; i < m_robotManagers.size(); i++)
	{
		for (unsigned int j = 0; j < m_endeffectors[i].size(); j++)
		{
			Tdes = Tcur / m_robotToObject[i][j];
			Trobot = m_robotManagers[i]->forwardKin(jointVal[i], m_endeffectors[i][j], m_offsets[i][j]);
			maxError = max(maxError, Norm((Trobot % Tdes).GetPosition()));
		}
	}
	return maxError;
}

void impedanceCtrlManager::impedanceControl(srJoint::ACTTYPE actType/* = srJoint::ACTTYPE::TORQUE*/, bool useFTsensor /* = false*/, bool simulateInteraction /* = true*/, int metric /* = DoubleGeodesic*/)
{
	// get current T, V of object
	SE3 Tcur = objectBaseLink->GetFrame() * objectRefFrame;
	se3 Vcur = InvAd(objectRefFrame, objectBaseLink->GetVel());
	se3 Acur = InvAd(objectRefFrame, objectBaseLink->GetAcc());

	dse3 Fcont(0.0);
	dse3 Fext(0.0);
	dse3 Fr2o(0.0);
	if (!useFTsensor && simulateInteraction)
	{
		//////////////////////////////////////// here we do not consider FTsensor and applying interaction force as external force
		// get external force
		Fcont = getContactForce();
		//cout << Fcont << endl;
		Fext = Fcont + getGravitationalForce();
		////////////////////////////////////////
	}
	else
	{
		for (unsigned int i = 0; i < m_robotManagers.size(); i++)
		{
			for (unsigned int j = 0; j < m_robotManagers[i]->m_ftSensorInfo.size(); j++)
			{
				dse3 FsensorExt = m_robotManagers[i]->m_ftSensorInfo[j]->getExtForce(g6);
				Fr2o -= InvdAd(Tcur % m_robotManagers[i]->m_ftSensorInfo[j]->m_sensorLocJoint->m_Frame * m_robotManagers[i]->m_ftSensorInfo[j]->m_offset,
					m_robotManagers[i]->m_ftSensorInfo[j]->getExtForce(g6));
			}
		}
		dse3 Fobjinert = objectInertia*Acur - dad(Vcur, objectInertia*Vcur);
		Fext = objectInertia*Acur - dad(Vcur, objectInertia*Vcur) - Fr2o;
		Fcont = Fext - getGravitationalForce();
		Fctrlbf;
		int stopstop = 1;
		//cout << "Fcont_s: " << Fcont << endl;
		//cout << "Fext_s:  " << Fext << endl;
		//cout << "Fcont:   " << getContactForce() << endl;
		//cout << "Fext:    " << getContactForce() + getGravitationalForce() << endl;

	}

	//////////////////////////////////////// to check the difference between des and mes of obj acc
	static dse3 Fcont_bf = dse3(0.0);
	static se3 objacc_bf_des = se3(0.0);
	////////////////////////////////////////


	// set desired trajectory
	SE3 Tdes;
	se3 Vdes(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	se3 dotVdes(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

	if (SE3spline == NULL)
	{
		// from real trajectory
		if (controlStep > Tdes_trj.size() - 1)
			Tdes = Tdes_trj[Tdes_trj.size() - 1];
		else
		{
			Tdes = Tdes_trj[controlStep];
			Vdes = Vdes_trj[controlStep];
			dotVdes = dotVdes_trj[controlStep];
		}
	}
	else
	{
		// from spline
		double t = controlStep*timeStep;
		Tdes = SE3spline->getSE3(t);
		Vdes = SE3spline->getSpaceVelocity(t);
		dotVdes = SE3spline->getSpaceAcceleration(t);
	}
	// generate desired object acceleration
	se3 objectAcc = getDesiredObjectAcc(Fext, Tcur, Vcur, Tdes, Vdes, dotVdes, m_useIntegralGain && Tdes_trj.size() == 1, metric);
	
	////////////////////////////////////
	se3 acc = Ad(Tcur, objectAcc);
	Vec3 f(Fext[3], Fext[4], Fext[5]);
	f = Tcur.GetOrientation()*f;
	////////////////////////////////////

	// generate robot joint acc for desired object acceleration
	vector<Eigen::VectorXd> jointVal(m_robotManagers.size());
	for (unsigned int i = 0; i < m_robotManagers.size(); i++)
		jointVal[i] = m_robotManagers[i]->getJointVal();
	// amend robot posture when position error is larger than threshold
	if (getKinematicsError(Tcur, jointVal) > postureThreshold)
	{
		cout << m_endeffectors[0][0]->GetFrame() << endl;
		amendJointValVel(jointVal);
		for (unsigned int i = 0; i < m_robotManagers.size(); i++)
			jointVal[i] = m_robotManagers[i]->getJointVal();
		cout << m_endeffectors[0][0]->GetFrame() << endl;
	}
	vector<Eigen::VectorXd> jointVel(m_robotManagers.size());
	for (unsigned int i = 0; i < m_robotManagers.size(); i++)
		jointVel[i] = m_robotManagers[i]->getJointVel();
	vector<Eigen::VectorXd> jointAcc(m_robotManagers.size());
	for (unsigned int i = 0; i < m_robotManagers.size(); i++)
		jointAcc[i] = getDesiredJointAcc(jointVal[i], jointVel[i], i, objectAcc);
	
	// control force which should be exerted to the object
	dse3 Fctrl_ = objectInertia*objectAcc - dad(Vcur, objectInertia*Vcur) - Fext;
	//cout << "Fctrl:   " << Fctrlbf << endl;
	Eigen::VectorXd Fctrl = dse3toVector(Fctrl_);

	double normF = 0.0;
	for (int i = 0; i < 6; i++)
		normF += Fcont[i] * Fcont[i];
	
	normF = sqrt(normF);
	bool contactOccur = false;
	if (normF > 1e-5)
		contactOccur = true;
	bool printContact = false;
	if (contactOccur & printContact)
	{
		printf("contact!!!\n");
		//cout << "Fcont: " << Fcont;
		//dse3 dF1 = Fcont - Fcont_bf;
		//dse3 dF2 = objectInertia*(object->GetBaseLink()->m_Acc - objacc_bf_des);
		//cout << "dF1: " << dF1;
		//cout << "dF2: " << dF2;
		//objacc_bf_des = objectAcc;
		//Fcont_bf = Fcont;
	}
		
	
	//////////////////////////////////////////////////////// acc control
	if (actType == srJoint::ACTTYPE::HYBRID)
	{
		for (unsigned int i = 0; i < m_robotManagers.size(); i++)
			m_robotManagers[i]->controlJointAcc(jointAcc[i]);
		//cout << jointAcc[0].transpose() << endl;
		cout << "Ades: " << objectAcc;
		//cout << "Arob: " << InvAd(m_robotToObject[0][0],Vectortose3(m_robotManagers[0]->getBodyJacobian(jointVal[0], m_endeffectors[0][0]) * jointAcc[0] 
		//	+ m_robotManagers[0]->getBodyJacobianDot(jointVal[0], jointVel[0], m_endeffectors[0][0]) * jointVel[0]));
	}
	
	//////////////////////////////////////////////////////// torque control
	if (actType == srJoint::ACTTYPE::TORQUE)
	{
		vector<Eigen::VectorXd> jointTorque(m_robotManagers.size());
		for (unsigned int i = 0; i < m_robotManagers.size(); i++)
			jointTorque[i] = m_robotManagers[i]->inverseDyn(jointVal[i], jointVel[i], jointAcc[i]);    // joint torque without robot-object force ......... may need to change to hybridDyn

		int numF = 0;
		for (unsigned int i = 0; i < m_robotManagers.size(); i++)
			for (unsigned int j = 0; j < m_endeffectors[i].size(); j++)
				numF += 1;
		Eigen::MatrixXd W(6 * numF, 6 * numF);
		numF = 0;
		for (unsigned int i = 0; i < m_robotManagers.size(); i++)
		{
			for (unsigned int j = 0; j < m_endeffectors[i].size(); j++)
			{
				W.block(numF * 6, numF * 6, 6, 6) = InvdAdtoMatrix(Tcur % m_endeffectors[i][j]->GetFrame() * m_offsets[i][j]  /*Inv(m_robotToObject[i][j])*/);
				numF += 1;
			}
		}
		Eigen::VectorXd F = pinv(W)*Fctrl;
		numF = 0;
		if (simulateInteraction)
		{
			// simulate interaction force
			// exert virtual force from object to robot
			for (unsigned int i = 0; i < m_robotManagers.size(); i++)
			{
				///////////////////////////
				//cout << "F: " << F << endl;
				//cout << "Jb: " << m_robotManagers[i]->getBodyJacobian(jointVal[i], m_endeffectors[i], m_offsets[i]) << endl;
				///////////////////////////
				jointTorque[i] += m_robotManagers[i]->getBodyJacobian(jointVal[i], m_endeffectors[i], m_offsets[i]).transpose()*F.segment(6 * numF, 6 * m_endeffectors[i].size());

				m_robotManagers[i]->setJointValVel(jointVal[i], jointVel[i]);
				m_robotManagers[i]->controlJointTorque(jointTorque[i]);
				for (unsigned int j = 0; j < m_endeffectors[i].size(); j++)
					m_robotManagers[i]->exertExternalForceToLink(Vectortodse3(-F.segment(6 * (numF + j), 6)), m_endeffectors[i][j], m_offsets[i][j]);
				numF += m_endeffectors[i].size();
			}

			cout << "Ades: " << objectAcc;
			cout << "Arob: " << InvAd(Inv(Tcur % m_endeffectors[0][0]->GetFrame() * m_offsets[0][0])/*m_robotToObject[0][0]*/, Vectortose3(m_robotManagers[0]->getBodyJacobian(jointVal[0], m_endeffectors[0][0]) * jointAcc[0]
				+ m_robotManagers[0]->getBodyJacobianDot(jointVal[0], jointVel[0], m_endeffectors[0][0]) * jointVel[0]));
			//cout << "torq: " << jointTorque[0].transpose() << endl;
			//cout << endl;
		}
		else
		{
			for (unsigned int i = 0; i < m_robotManagers.size(); i++)
			{
				///////////////////////////
				//cout << "F: " << F << endl;
				//cout << "Jb: " << m_robotManagers[i]->getBodyJacobian(jointVal[i], m_endeffectors[i], m_offsets[i]) << endl;
				///////////////////////////
				jointTorque[i] += m_robotManagers[i]->getBodyJacobian(jointVal[i], m_endeffectors[i], m_offsets[i]).transpose()*F.segment(6 * numF, 6 * m_endeffectors[i].size());

				m_robotManagers[i]->setJointValVel(jointVal[i], jointVel[i]);
				m_robotManagers[i]->controlJointTorque(jointTorque[i]);
				numF += m_endeffectors[i].size();
			}
		}
	}
	if (simulateInteraction)
	{
		// exert virtual force to object
		objectBaseLink->AddUserExternalForce(InvdAd(objectRefFrame, Vectortodse3(Fctrl)));
	}
	
	controlStep++;
	// save data of former step
	Fgen = Fext;
	Vbf = Vcur;
	Fctrlbf = Vectortodse3(Fctrl);
}

void impedanceCtrlManager::impedanceControlWeld(int metric /* = DoubleGeodesic*/)
{
	// assume object is connected to robot with weld joint. we only consider single robotManager and single endeffector here.
	// get current T, V of object
	SE3 Tcur = objectBaseLink->GetFrame() * objectRefFrame;
	se3 Vcur = InvAd(objectRefFrame, objectBaseLink->GetVel());
	se3 Acur = InvAd(objectRefFrame, objectBaseLink->GetAcc());

	dse3 Fcont(0.0);
	dse3 Fext(0.0);
	dse3 Fr2o(0.0);
	//////////////////////////////////////// here we do not use FTsensor and applying interaction force as external force
	// get external force
	Fcont = getContactForce();
	//cout << Fcont << endl;
	Fext = Fcont + getGravitationalForce();
	////////////////////////////////////////
	

	//////////////////////////////////////// to check the difference between des and mes of obj acc
	static dse3 Fcont_bf = dse3(0.0);
	static se3 objacc_bf_des = se3(0.0);
	////////////////////////////////////////


	// set desired trajectory
	SE3 Tdes;
	se3 Vdes(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	se3 dotVdes(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

	if (SE3spline == NULL)
	{
		// from real trajectory
		if (controlStep > Tdes_trj.size() - 1)
			Tdes = Tdes_trj[Tdes_trj.size() - 1];
		else
		{
			Tdes = Tdes_trj[controlStep];
			Vdes = Vdes_trj[controlStep];
			dotVdes = dotVdes_trj[controlStep];
		}
	}
	else
	{
		// from spline
		double t = controlStep*timeStep;
		Tdes = SE3spline->getSE3(t);
		Vdes = SE3spline->getSpaceVelocity(t);
		dotVdes = SE3spline->getSpaceAcceleration(t);
	}
	// generate desired object acceleration
	se3 objectAcc = getDesiredObjectAcc(Fext, Tcur, Vcur, Tdes, Vdes, dotVdes, m_useIntegralGain && Tdes_trj.size() == 1, metric);

	////////////////////////////////////
	se3 acc = Ad(Tcur, objectAcc);
	Vec3 f(Fext[3], Fext[4], Fext[5]);
	f = Tcur.GetOrientation()*f;
	////////////////////////////////////

	// generate robot joint acc for desired object acceleration
	vector<Eigen::VectorXd> jointVal(m_robotManagers.size());
	for (unsigned int i = 0; i < m_robotManagers.size(); i++)
		jointVal[i] = m_robotManagers[i]->getJointVal();
	vector<Eigen::VectorXd> jointVel(m_robotManagers.size());
	for (unsigned int i = 0; i < m_robotManagers.size(); i++)
		jointVel[i] = m_robotManagers[i]->getJointVel();
	vector<Eigen::VectorXd> jointAcc(m_robotManagers.size());
	for (unsigned int i = 0; i < m_robotManagers.size(); i++)
		jointAcc[i] = getDesiredJointAcc(jointVal[i], jointVel[i], i, objectAcc);


	double normF = 0.0;
	for (int i = 0; i < 6; i++)
		normF += Fcont[i] * Fcont[i];

	normF = sqrt(normF);
	bool contactOccur = false;
	if (normF > 1e-5)
		contactOccur = true;
	bool printContact = false;
	if (contactOccur & printContact)
	{
		printf("contact!!!\n");
		//cout << "Fcont: " << Fcont;
		//dse3 dF1 = Fcont - Fcont_bf;
		//dse3 dF2 = objectInertia*(object->GetBaseLink()->m_Acc - objacc_bf_des);
		//cout << "dF1: " << dF1;
		//cout << "dF2: " << dF2;
		//objacc_bf_des = objectAcc;
		//Fcont_bf = Fcont;
	}


	//////////////////////////////////////////////////////// torque control
	vector<Eigen::VectorXd> jointTorque(m_robotManagers.size());
	for (unsigned int i = 0; i < m_robotManagers.size(); i++)
		jointTorque[i] = m_robotManagers[i]->inverseDyn(jointVal[i], jointVel[i], jointAcc[i]);    // joint torque without robot-object force ......... may need to change to hybridDyn

	int numF = 0;
	for (unsigned int i = 0; i < m_robotManagers.size(); i++)
		for (unsigned int j = 0; j < m_endeffectors[i].size(); j++)
			numF += 1;
	Eigen::MatrixXd W(6 * numF, 6 * numF);
	numF = 0;
	for (unsigned int i = 0; i < m_robotManagers.size(); i++)
	{
		for (unsigned int j = 0; j < m_endeffectors[i].size(); j++)
		{
			W.block(numF * 6, numF * 6, 6, 6) = InvdAdtoMatrix(Tcur % m_endeffectors[i][j]->GetFrame() * m_offsets[i][j]  /*Inv(m_robotToObject[i][j])*/);
			numF += 1;
		}
	}
	Eigen::VectorXd Fctrl = -dse3toVector(Fcont);
	Eigen::VectorXd F = pinv(W)*Fctrl;
	numF = 0;
	for (unsigned int i = 0; i < m_robotManagers.size(); i++)
	{
		///////////////////////////
		//cout << "F: " << F << endl;
		//cout << "Jb: " << m_robotManagers[i]->getBodyJacobian(jointVal[i], m_endeffectors[i], m_offsets[i]) << endl;
		///////////////////////////
		jointTorque[i] += m_robotManagers[i]->getBodyJacobian(jointVal[i], m_endeffectors[i], m_offsets[i]).transpose()*F.segment(6 * numF, 6 * m_endeffectors[i].size());

		m_robotManagers[i]->setJointValVel(jointVal[i], jointVel[i]);
		m_robotManagers[i]->controlJointTorque(jointTorque[i]);
		numF += m_endeffectors[i].size();
	}

	//cout << "Ades: " << objectAcc;
	//cout << "Arob: " << InvAd(Inv(Tcur % m_endeffectors[0][0]->GetFrame() * m_offsets[0][0])/*m_robotToObject[0][0]*/, Vectortose3(m_robotManagers[0]->getBodyJacobian(jointVal[0], m_endeffectors[0][0]) * jointAcc[0]
	//	+ m_robotManagers[0]->getBodyJacobianDot(jointVal[0], jointVel[0], m_endeffectors[0][0]) * jointVel[0]));
	//cout << "torq: " << jointTorque[0].transpose() << endl;
	//cout << endl;

	controlStep++;
	// save data of former step
	Fgen = Fext;
	Vbf = Vcur;
	Fctrlbf = Vectortodse3(Fctrl);
}

void impedanceCtrlManager::objectImpedanceControl(int metric /* = DoubleGeodesic*/)
{
	SE3 Tcur = objectBaseLink->GetFrame() * objectRefFrame;
	se3 Vcur = InvAd(objectRefFrame, objectBaseLink->GetVel());
	SE3 Tdes;
	se3 Vdes;
	se3 dotVdes;
	if (controlStep > Tdes_trj.size() - 1)
	{
		Tdes = Tdes_trj[Tdes_trj.size() - 1];
		Vdes = Vdes_trj[Vdes_trj.size() - 1];
		dotVdes = dotVdes_trj[dotVdes_trj.size() - 1];
	}
	else
	{
		Tdes = Tdes_trj[controlStep];
		Vdes = Vdes_trj[controlStep];
		dotVdes = dotVdes_trj[controlStep];
	}

	dse3 Fext = getContactForce() + getGravitationalForce();
	dse3 Fext2 = getTotalExternalForce();
	dse3 Fimp = generateImpedanceForce(Fext, Tcur, Vcur, Tdes, Vdes, dotVdes, m_useIntegralGain && Tdes_trj.size() == 1, metric);
	Fimp = InvdAd(objectRefFrame, Fimp);		// transform to the base link frame
	Eigen::MatrixXd G = InertiatoMatrix(getGeneralizedInertia());

	//dse3 Ftest(0.0, 0.0, 0.0, 0.00*G(3, 3), 0.00*G(3, 3), 0.01*G(3, 3));
	//Fimp = Ftest;
	//dse3 Fzero(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	//static int i = 0;
	//if (i < 1)
	//	Slot->GetBaseLink()->AddUserExternalForce(Fimp);
	//else
	//	Slot->GetBaseLink()->AddUserExternalForce(Fzero);
	//i++;

	//if (se3toVector(Vcur)[0] > 1){
	//	cout << i << endl;
	//	cout << "e = " << Tdes % Tcur << endl;
	//	cout << "ee = " << Tbf % Tcur << endl;
	//	cout << "Fimp = " << fimp << endl;
	//	cout << "a expected = " << a << endl;
	//	cout << "external force = " << dse3toVector(impede->getExternalForce(Fgen)) << endl;
	//	cout << endl << "inertia = " << G << endl << endl;
	//	gSpace.SET_USER_CONTROL_FUNCTION(stop);
	//}

	// robot joint acc control

	//Eigen::VectorXd dotdotq_ctrl = impede->getDesiredJointAcc(jointVal, jointVel, objectAcc);

	// object impedance control
	objectBaseLink->AddUserExternalForce(Fimp);
	bool printValues = false;
	if (printValues == true)
	{
		Eigen::VectorXd fimp = dse3toVector(Fimp);
		Eigen::VectorXd a(6);
		a = G.inverse()*(fimp + dse3toVector(Fext)) + G.inverse()*dse3toVector(dad(Vcur, getGeneralizedInertia()*Vcur));
		//cout << "e = " << Tdes % Tcur << endl;
		//cout << "ee = " << Tbf % Tcur << endl;
		cout << "========================================" << endl;
		cout << "Fimp = " << fimp.transpose() << endl;
		cout << "external force = " << getTotalExternalForce() << endl;
		cout << "a current  = " << objectBaseLink->GetAcc() << endl;
		cout << "a expected = " << a.transpose() << endl;
		cout << "a command  = " << getDesiredObjectAcc(Fext, Tcur, Vcur, Tdes, Vdes, dotVdes) << endl;
		//cout << endl << "inertia = " << G << endl << endl;
	}


	Fgen = Fimp;
	Tbf = Tcur;
	///////////////////////////////////////////////////////////////////////
	controlStep++;
}

void impedanceCtrlManager::setDesiredTrajectory(SE3 _Tdes)
{
	Tdes_trj.resize(1);
	Tdes_trj[0] = _Tdes;
	Vdes_trj.resize(1);
	Vdes_trj[0] = se3(0.0);
	dotVdes_trj.resize(1);
	dotVdes_trj[0] = se3(0.0);
}

void impedanceCtrlManager::setDesiredTrajectory(SE3 _Tdes, se3 _Vdes, se3 _dotVdes)
{
	Tdes_trj.push_back(_Tdes);
	Vdes_trj.push_back(_Vdes);
	dotVdes_trj.push_back(_dotVdes);
}

void impedanceCtrlManager::setDesiredTrajectory(vector<SE3> _Tdes_trj, vector<se3> _Vdes_trj, vector<se3> _dotVdes_trj)
{
	Tdes_trj = _Tdes_trj;
	Vdes_trj = _Vdes_trj;
	dotVdes_trj = _dotVdes_trj;
}

void impedanceCtrlManager::setDesiredTrajectory(vector<SE3> _Tdes_trj)
{
	Tdes_trj = _Tdes_trj;
	Vdes_trj.resize(0);
	dotVdes_trj.resize(0);
	double inv_dt = 1. / timeStep;
	// Set Vdes_trj from Tdes_trj by numerical differentiation
	for (unsigned int i = 0; i < _Tdes_trj.size() - 1; i++)
		Vdes_trj.push_back(Log(_Tdes_trj[i + 1] / _Tdes_trj[i]) * inv_dt);
	Vdes_trj.push_back(Vdes_trj[Vdes_trj.size() - 1]);

	// Set dotVdes_trj from Vdes_trj by numerical differentiation
	for (unsigned int i = 0; i < _Tdes_trj.size() - 1; i++)
		dotVdes_trj.push_back( (Vdes_trj[i + 1]  - Vdes_trj[i]) * inv_dt);
	dotVdes_trj.push_back(dotVdes_trj[dotVdes_trj.size() - 1]);
}

void impedanceCtrlManager::setDesiredTrajectory(SE3Spline* _SE3spline)
{
	SE3spline = _SE3spline;
}

void impedanceCtrlManager::setTimeStep(double _timeStep)
{
	timeStep = _timeStep;
}

void impedanceCtrlManager::setCenterofCompliance(SE3 T)
{
	cocFrame = T;
}

//void impedanceCtrlManager::amendJointValueVelocity()
//{
//	SE3 Tcur = objectBaseLink->GetFrame() * objectRefFrame;
//	se3 Vcur = InvAd(objectRefFrame, objectBaseLink->GetVel());
//	if (endeffectorToObject.size() == 1){
//		se3 Vendeffector = Ad(endeffectorToObject[0], Vcur);
//		vector<SE3> Tendeffector;
//		Tendeffector.resize(0);
//		Tendeffector.push_back(Tcur / endeffectorToObject[0]);
//		Eigen::VectorXd jointVal = m_robotManager->inverseKin(Tendeffector);
//		Eigen::MatrixXd Jb = m_robotManager->getBodyJacobian(jointVal);
//		Eigen::VectorXd jointVel = Jb.inverse()*se3toVector(Vendeffector);
//		m_robotManager->setJointValVel(jointVal, jointVel);
//	}
//	else{
//		vector<SE3> Tendeffector;
//		Tendeffector.resize(0);
//		Tendeffector.push_back(Tcur / endeffectorToObject[0]);
//		Tendeffector.push_back(Tcur / endeffectorToObject[1]);
//		se3 Vendeffector1 = Ad(endeffectorToObject[0], Vcur);
//		se3 Vendeffector2 = Ad(endeffectorToObject[1], Vcur);
//		Eigen::VectorXd jointVal = m_robotManager->inverseKin(Tendeffector, m_robotManager->getJointVal());
//		Eigen::MatrixXd Jb = m_robotManager->generateAugmentedJacobian(jointVal, false);
//		Eigen::VectorXd Vendeffector;
//		Vendeffector.resize(12, 1);
//		Vendeffector << se3toVector(Vendeffector1), se3toVector(Vendeffector2);
//		Eigen::VectorXd jointVel = pinv(Jb)*Vendeffector;
//		m_robotManager->setJointValVel(jointVal, jointVel);
//	}
//}


void impedanceCtrlManager::setAttachCond(bool _isAttached)
{
	isAttached = _isAttached;
}

void impedanceCtrlManager::setGravity(Vec3 _g)
{
	g = _g;
	g6 = se3(0.0, 0.0, 0.0, g[0], g[1], g[2]);
}

dse3 impedanceCtrlManager::getGravitationalForce() const
{
	dse3 Fg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	Fg = objectInertia * InvAd((objectBaseLink->GetFrame()*objectRefFrame), g6);
	/*for (unsigned int i = 0; i < objectLinks.size(); i++)
		Fg += InvdAd((objectBaseLink->GetFrame()*objectRefFrame) % objectLinks[i]->GetFrame(), objectLinks[i]->GetMass()*g6);*/
	return Fg;
}

void impedanceCtrlManager::updateIntegralError(SE3 e, int metric)
{
	if (metric == Metric::DoubleGeodesic)
	{
		m_integral_error.head(3) += Vec3toVector(Log(e.GetOrientation())) * timeStep;
		m_integral_error.tail(3) += Vec3toVector(e.GetPosition()) * timeStep;
	}
	else
		m_integral_error += se3toVector(Log(e)) * timeStep;
}

void impedanceCtrlManager::setRobotPostureAmendThreshold(double _postureThreshold)
{
	postureThreshold = _postureThreshold;
}

unsigned int impedanceCtrlManager::controlStep = 0;

SE3 impedanceCtrlManager::Tbf = EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0));
se3 impedanceCtrlManager::Vbf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
dse3 impedanceCtrlManager::Fgen(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
dse3 impedanceCtrlManager::Fctrlbf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
Eigen::VectorXd impedanceCtrlManager::m_integral_error = Eigen::VectorXd::Zero(6);