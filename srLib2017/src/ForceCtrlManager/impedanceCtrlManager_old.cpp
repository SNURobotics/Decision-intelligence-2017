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

void impedanceCtrlManager::setSystem(DualArmRobotManager* _robotManager)
{
	robotManager = _robotManager;
}

void impedanceCtrlManager::setObject(srSystem* _object)
{
	setObjectBaseLink(_object->GetBaseLink());
}

void impedanceCtrlManager::setObjectBaseLink(srLink* _objectBaseLink)
{
	objectLinks.resize(0);
	objectBaseLink = _objectBaseLink;
	objectRefFrame = EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0));
	objectLinks.push_back(_objectBaseLink);
	int beforeLinkNum = 0;
	while (true){
		int addedLinkNum = objectLinks.size() - beforeLinkNum;
		if (addedLinkNum == 0)
			break;
		int beforebeforeLinkNum = beforeLinkNum;
		beforeLinkNum = objectLinks.size();
		for (int i = 0; i < addedLinkNum; i++){
			int childLinks = (objectLinks[beforebeforeLinkNum + i]->m_ChildLinks).get_size();
			for (int j = 0; j < (objectLinks[beforebeforeLinkNum + i]->m_ChildLinks).get_size(); j++){
				objectLinks.push_back(objectLinks[beforebeforeLinkNum + i]->m_ChildLinks[j]);
			}
		}
	}
}

void impedanceCtrlManager::setObjectRefFrame(SE3 T)
{
	objectRefFrame = T;
	objectInertia.SetMass(0.0);
	objectInertia.SetAngularMoment(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	objectInertia.SetOffset(Vec3(0.0, 0.0, 0.0));
	SE3 curFrame;
	for (int i = 0; i < objectLinks.size(); i++){
		curFrame = objectLinks[i]->GetFrame();
		objectInertia = objectInertia + objectLinks[i]->GetInertiaRef().Transform(curFrame%objectBaseLink->GetFrame()*objectRefFrame);
		//cout << curFrame%objectRefFrame << endl;
		//cout << InertiatoMatrix(object->m_KIN_Links[i]->GetInertiaRef().Transform(curFrame%objectRefFrame));
	}
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

void impedanceCtrlManager::setEndeffectorToObject(vector<SE3> _endeffectorToObject)
{
	endeffectorToObject = _endeffectorToObject;
}

se3 impedanceCtrlManager::getDesiredObjectAcc(dse3 Fext, SE3 T, se3 V, SE3 Tdes, se3 Vdes, se3 dotVdes, int metric) const
{
	Eigen::VectorXd Ve;
	Ve = se3toVector(V - InvAd(T, Vdes));
	SE3 e = Tdes%T;

	Eigen::VectorXd Utr = inertial*se3toVector(InvAd(T, dotVdes) + ad(InvAd(T, Vdes), V));
	
	Eigen::VectorXd Z(6);
	if (metric == DoubleGeodesic){
		Z << - capacitive.block<3, 3>(0, 0)*Vec3toVector(Log(e.GetOrientation())), - SO3toMatrix(Inv(e.GetOrientation())) * capacitive.block<3, 3>(3, 3) * Vec3toVector(e.GetPosition());
		Z -= resistive*Ve;
	}
	else
		Z = -resistive*Ve - capacitive*se3toVector(Log(e));
	se3 dotV = Vectortose3(inertial.inverse()*( Utr + Z + dse3toVector(Fext) ));
	return dotV;
}

dse3 impedanceCtrlManager::getExternalForce(dse3 Fgen) const
{
	SE3 curFrame;
	dse3 Fcur;
	dse3 Fext(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	for (int i = 0; i < objectLinks.size(); i++){
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
	for (int i = 0; i < objectLinks.size(); i++){
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
	SE3 curFrame;
	dse3 Fcur;
	dse3 Fcont(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	for (int i = 0; i < objectLinks.size(); i++){
		Fcur = Vectortodse3(dse3toVector(objectLinks[i]->m_ConstraintImpulse) / timeStep);
		curFrame = objectLinks[i]->GetFrame();
		//cout << i << "th link force = " << dse3toVector(Fcur) << endl;
		//cout << objectRefFrame % curFrame << endl;
		Fcont += InvdAd((objectBaseLink->GetFrame()*objectRefFrame) % curFrame, Fcur);
	}
	return Fcont;
}

dse3 impedanceCtrlManager::generateImpedanceForce(dse3 Fext, SE3 T, se3 V, SE3 Tdes, se3 Vdes, se3 dotVdes, int metric) const
{
	se3 dotVctrl = getDesiredObjectAcc(Fext, T, V, Tdes, Vdes, dotVdes, metric);
	dse3 Fdes = objectInertia*dotVctrl - dad(V, objectInertia*V);
	dse3 Fctrl = Fdes - Fext;
	return Fctrl;
}

Inertia impedanceCtrlManager::getGeneralizedInertia() const
{
	return objectInertia;
}

Eigen::VectorXd impedanceCtrlManager::getDesiredJointAcc(Eigen::VectorXd jointVal, Eigen::VectorXd jointVel, se3 objectAcc)
{
	Eigen::VectorXd jointAcc(jointVal.size());
	if (robotManager->getPlanningMode() == DualArmRobotManager::MoveLeftArmOnly || robotManager->getPlanningMode() == DualArmRobotManager::MoveRightArmOnly || robotManager->getPlanningMode() == DualArmRobotManager::MoveLeftArmWaist || robotManager->getPlanningMode() == DualArmRobotManager::MoveRightArmWaist){
		se3 robotEndEffectorAcc = Ad(endeffectorToObject[0], objectAcc);
		Eigen::MatrixXd Jb = robotManager->getBodyJacobian(jointVal);

		Eigen::MatrixXd JJ = Jb*Jb.transpose();
		Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(JJ);
		
		Eigen::VectorXd v_JJ = eigensolver.eigenvalues();
		double condNum = v_JJ(0) / v_JJ(v_JJ.size()-1);
		//cout << "Condition Number: " << condNum << endl;

		if (Jb.cols() < 6)
			cout << "deficient manipulator!!!" << endl;
		else if (Jb.cols() == 6)
			jointAcc = Jb.inverse()*(se3toVector(robotEndEffectorAcc) - robotManager->getBodyJacobianDot(jointVal, jointVel)*jointVel);
		else{
			Eigen::MatrixXd Q = robotManager->generateWeightMatrix(true);
			jointAcc = Q.inverse()*Jb.transpose()*(Jb*Q.inverse()*Jb.transpose()).inverse()*(se3toVector(objectAcc) - robotManager->getBodyJacobianDot(jointVal, jointVel)*jointVel);
		}
	}
	else if (robotManager->getPlanningMode() == DualArmRobotManager::MoveBothArmOnly || robotManager->getPlanningMode() == DualArmRobotManager::MoveWholeBody){
		se3 leftEndEffectorAcc = Ad(endeffectorToObject[0], objectAcc);
		se3 rightEndEffectorAcc = Ad(endeffectorToObject[1], objectAcc);
		Eigen::MatrixXd Jb = robotManager->generateAugmentedJacobian(jointVal, false);
		Eigen::VectorXd robotEndEffectorAcc(12);
		robotEndEffectorAcc << se3toVector(leftEndEffectorAcc), se3toVector(rightEndEffectorAcc);
		Eigen::MatrixXd dotJb = robotManager->generateAugmentedJacobianDot(jointVal, jointVel, false);
		Eigen::MatrixXd Q = robotManager->generateWeightMatrix(false);
		if (Jb.cols() < 12)
			cout << "deficient manipulator!!!" << endl;
		else{
			Eigen::MatrixXd Q = robotManager->generateWeightMatrix(true);
			jointAcc = Q.inverse()*Jb.transpose()*(Jb*Q.inverse()*Jb.transpose()).inverse()*(robotEndEffectorAcc - dotJb*jointVel);
		}
	}
	return jointAcc;
}

void impedanceCtrlManager::impedanceControl()
{
	
	// generate desired object acceleration
	SE3 Tcur = objectBaseLink->GetFrame() * objectRefFrame;
	se3 Vcur = InvAd(objectRefFrame, objectBaseLink->GetVel());
	dse3 Fgen(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);		// for attached robot and object
	//dse3 Fext = getExternalForce(Fgen);
	dse3 Fext = getContactForce();					// for zero gravity case

	// set desired trajectory
	SE3 Tdes;
	se3 Vdes(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	se3 dotVdes(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

	if (SE3spline == NULL){
		// from real trajectory
		if (controlStep > Tdes_trj.size() - 1){
			Tdes = Tdes_trj[Tdes_trj.size() - 1];
		}
		else{
			Tdes = Tdes_trj[controlStep];
			Vdes = Vdes_trj[controlStep];
			dotVdes = dotVdes_trj[controlStep];
		}
	}
	else{
		// from spline
		double t = controlStep*timeStep;
		Tdes = SE3spline->getSE3(t);
		Vdes = SE3spline->getSpaceVelocity(t);
		dotVdes = SE3spline->getSpaceAcceleration(t);
	}

	se3 objectAcc = getDesiredObjectAcc(Fext, Tcur, Vcur, Tdes, Vdes, dotVdes);

	////////////////////////////////////
	se3 acc = Ad(Tcur, objectAcc);
	Vec3 f(Fext[3], Fext[4], Fext[5]);
	f = Tcur.GetOrientation()*f;
	////////////////////////////////////

	// generate robot joint torque for desired object acceleration
	Eigen::VectorXd jointVal = robotManager->getJointValue();
	Eigen::VectorXd jointVel = robotManager->getJointVel();
	Eigen::VectorXd dotdotq = getDesiredJointAcc(jointVal, jointVel, objectAcc);
	
	

	//////////////////////////////////////////////////////// torque control
	Eigen::VectorXd jointTorque = robotManager->inverseDyn(jointVal, jointVel, dotdotq);    // joint torque without robot-object force
	//Eigen::VectorXd jointTorque2 = robotManager->getMassMatrix(jointVal)*dotdotq + robotManager->getBiasTerm(jointVal, jointVel);

	se3 Acur = InvAd(objectRefFrame, objectBaseLink->GetAcc());
	bool printValues = false;

	if (printValues == true){
		//cout << "joint torque: " << jointTorque << endl;
		//cout << "joint torque2: " << jointTorque2 << endl << endl;
		cout << "cur acc: " << robotManager->getJointAcc() << endl;
		cout << "cur object acc: " << se3toVector(Acur) << endl;
		cout << "des acc: " << dotdotq << endl;
		cout << "des object acc: " << se3toVector(objectAcc) << endl;
	}

	//Sleep(500);

	vector<dse3> Frobot;
	Frobot.resize(0);

	// find force from robot to object (in real case Fext is unknown, but in this case Frobot is unknown)
	if (endeffectorToObject.size() == 1){
		Frobot.push_back(InvdAd(endeffectorToObject[0], objectInertia*Acur - dad(Vcur, objectInertia*Vcur) - Fext));
		Eigen::MatrixXd Jb = robotManager->getBodyJacobian(jointVal);
		jointTorque += Jb.transpose()*dse3toVector(Frobot[0]);
	}
	else{
		Eigen::MatrixXd Q(12, 12);
		Q.setIdentity();
		Eigen::MatrixXd A(6, 12);
		A << InvdAdtoMatrix(Inv(endeffectorToObject[0])), InvdAdtoMatrix(Inv(endeffectorToObject[1]));
		Eigen::VectorXd Fctrl(6);

		Fctrl = dse3toVector(objectInertia*objectAcc - dad(Vcur, objectInertia*Vcur) - Fext);	// fixed 15.06.14
		//Fctrl = dse3toVector(objectInertia*Acur - dad(Vcur, objectInertia*Vcur) - Fext);		// former code
		Eigen::VectorXd F = Q.inverse()*A.transpose()*(A*Q.inverse()*A.transpose()).inverse()*Fctrl;
		jointTorque += robotManager->generateAugmentedJacobian(jointVal, false).transpose()*F;

		// exert virtual force from second arm to object
		dse3 Fobj = dAd(endeffectorToObject[1], Vectortodse3(F.tail(6)));
		objectBaseLink->AddUserExternalForce(Fobj);

		// exert virtual force from object to second arm
		dse3 Farm = Vectortodse3(-F.tail(6));
		robotManager->exertExternalForceToEndEffector(Farm, false);

		////////////////////////////////////////////////

	}
	robotManager->setJointValueVelocity(jointVal, jointVel);
	robotManager->controlJointTorque(jointTorque);

	///////////////////////////////////////////////////////////// acc control
	//robotManager->controlJointAcc(dotdotq);


	///////////////////////////////////////////////
	//cout << "Tcur = " << Tcur << endl;
	//cout << "objectAcc = " << objectAcc << endl;
	///////////////////////////////////////////////
	controlStep++;
}

void impedanceCtrlManager::objectImpedanceControl()
{
	bool printValues = false;
	if (printValues == true){
		cout << "a cur = " << se3toVector(objectBaseLink->GetAcc()) << endl;
		cout << "V cur = " << se3toVector(objectBaseLink->GetVel()) << endl;
	}
	///////////////////////////////////////////////////////////////////////
	SE3 Tcur = objectBaseLink->GetFrame() * objectRefFrame;
	se3 Vcur = InvAd(objectRefFrame, objectBaseLink->GetVel());
	SE3 Tdes;
	se3 Vdes;
	se3 dotVdes;
	if (controlStep > Tdes_trj.size() - 1){
		Tdes = Tdes_trj[Tdes_trj.size() - 1];
		Vdes = Vdes_trj[Vdes_trj.size() - 1];
		dotVdes = dotVdes_trj[dotVdes_trj.size() - 1];
	}
	else{
		Tdes = Tdes_trj[controlStep];
		Vdes = Vdes_trj[controlStep];
		dotVdes = dotVdes_trj[controlStep];
	}

	//SE3 Tdes = TslotInit*EulerZYX(Vec3(0.1, 0.1, 0.0), Vec3(0.0, 0.05, 0.0));
	//SE3 Tdes = EulerZYX(Vec3(-0.5*SR_PI_HALF, 0.0, 0.0), Vec3(1.0, 0.0, mockup_base_h*0.5)) * EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.1 -1.5*cl - mockup_extend_d - 0.5*mockup_base_d, mockup_base_h*0.5 - mockup_extend_h*0.5));
	Fgen = InvdAd(Tcur%Tbf, Fgen);
	dse3 Fext = getExternalForce(Fgen);

	dse3 Fimp = generateImpedanceForce(Fext, Tcur, Vcur, Tdes, Vdes, dotVdes);
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
	Eigen::VectorXd fimp = dse3toVector(Fimp);
	Eigen::VectorXd a(6);
	a = G.inverse()*fimp + G.inverse()*dse3toVector(dad(Vcur, getGeneralizedInertia()*Vcur));

	if (printValues == true){
		cout << "e = " << Tdes % Tcur << endl;
		cout << "ee = " << Tbf % Tcur << endl;
		cout << "Fimp = " << fimp << endl;
		cout << "a expected = " << a << endl;
		cout << "external force = " << dse3toVector(getTotalExternalForce()) << endl;
		//cout << endl << "inertia = " << G << endl << endl;
	}


	////////////////////////////
	//////////////////////////// reactive force to robot
	// generate robot joint torque for desired object acceleration
	se3 Acur = InvAd(objectRefFrame, objectBaseLink->GetAcc());
	se3 objectAcc = getDesiredObjectAcc(Fext, Tcur, Vcur, Tdes, Vdes, dotVdes);
	Eigen::VectorXd jointVal = robotManager->getJointValue();
	Eigen::VectorXd jointVel = robotManager->getJointVel();
	Eigen::VectorXd dotdotq = getDesiredJointAcc(jointVal, jointVel, objectAcc);

	Eigen::VectorXd jointTorque = robotManager->inverseDyn(jointVal, jointVel, dotdotq);    // joint torque without robot-object force
	vector<dse3> Frobot;
	Frobot.resize(0);

	// find force from robot to object (in real case Fext is unknown, but in this case Frobot is unknown)
	if (endeffectorToObject.size() == 1){
		Frobot.push_back(InvdAd(endeffectorToObject[0], objectInertia*Acur - dad(Vcur, objectInertia*Vcur) - Fext));
		Eigen::MatrixXd Jb = robotManager->getBodyJacobian(jointVal);
		jointTorque += Jb.transpose()*dse3toVector(Frobot[0]);
	}
	else{
		Eigen::MatrixXd Q(12, 12);
		Q.setIdentity();
		Eigen::MatrixXd A(6, 12);
		A << InvdAdtoMatrix(Inv(endeffectorToObject[0])), InvdAdtoMatrix(Inv(endeffectorToObject[1]));
		Eigen::VectorXd Fctrl(6);

		Fctrl = dse3toVector(objectInertia*Acur - dad(Vcur, objectInertia*Vcur) - Fext);
		Eigen::VectorXd F = Q.inverse()*A.transpose()*(A*Q.inverse()*A.transpose()).inverse()*Fctrl;
		jointTorque += robotManager->generateAugmentedJacobian(jointVal, false).transpose()*F;

		// exert virtual force from object to first arm
		dse3 Farm1 = Vectortodse3(-F.head(6));
		robotManager->exertExternalForceToEndEffector(Farm1, true);

		// exert virtual force from object to second arm
		dse3 Farm2 = Vectortodse3(-F.tail(6));
		robotManager->exertExternalForceToEndEffector(Farm2, false);

		////////////////////////////////////////////////

	}
	robotManager->setJointValueVelocity(jointVal, jointVel);
	robotManager->controlJointTorque(jointTorque);


	///////////////////////////////////////////////////
	///////////////////////////////////////////////////

	Fgen = Fimp;
	Tbf = Tcur;
	///////////////////////////////////////////////////////////////////////
	controlStep++;
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

	// Set Vdes_trj from Tdes_trj by numerical differentiation
	for (int i = 0; i < _Tdes_trj.size() - 1; i++){
		Vdes_trj.push_back(Log(_Tdes_trj[i + 1] / _Tdes_trj[i]) * (1 / timeStep));
	}
	Vdes_trj.push_back(Vdes_trj[Vdes_trj.size() - 1]);

	// Set dotVdes_trj from Vdes_trj by numerical differentiation
	for (int i = 0; i < _Tdes_trj.size() - 1; i++){
		dotVdes_trj.push_back( (Vdes_trj[i + 1]  - Vdes_trj[i]) * (1 / timeStep) );
	}
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

int impedanceCtrlManager::controlStep = 0;

SE3 impedanceCtrlManager::Tbf = EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0));

dse3 impedanceCtrlManager::Fgen(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
