#include "hybridPFCtrlManager_old.h"



hybridPFCtrlManager::hybridPFCtrlManager()
{
	T_des_trj.resize(0);
	V_des_trj.resize(0);
	Vdot_des_trj.resize(0);
	Fext_des_trj.resize(0);

	isDesTrjSet = false;
	isContactOriSet = false;
	isSystemSet = false;
	isICSet = false;
}


hybridPFCtrlManager::~hybridPFCtrlManager()
{
}

bool hybridPFCtrlManager::setSystem(robotManager * _robotManager, srLink * endeffector, srLink* contactLink /* = NULL*/)
{
	m_robotManager = _robotManager;
	for (unsigned int i = 0; i < m_robotManager->m_activeArmInfo->m_endeffector.size(); i++)
		if (endeffector = m_robotManager->m_activeArmInfo->m_endeffector[i])
		{
			m_endeffector = endeffector;
			if (contactLink == NULL)
				m_contactLink = endeffector;
			else
				m_contactLink = contactLink;
			return true;
		}
	return false;
}

bool hybridPFCtrlManager::setTimeStep(double _timeStep)
{
	m_timeStep = _timeStep;
	return true;
}

bool hybridPFCtrlManager::setDesiredTraj(vector<SE3> T_trj, vector<dse3> Fext_trj)
{
	Fext_des_trj = Fext_trj;
	T_des_trj = T_trj;
	if (T_trj.size() == 1)
	{
		V_des_trj.push_back(se3(0.0));
		Vdot_des_trj.push_back(se3(0.0));
		return true;
	}
	else
	{
		if (isICSet && isContactOriSet)
		{
			double dt_inv = 1.0 / m_timeStep;
			for (unsigned int i = 0; i < T_trj.size() - 1; i++)
				V_des_trj.push_back(InvAd(getEndeffectorToContactFrame(T_trj[i]), Log(T_trj[i] % T_trj[i + 1]))*dt_inv);
			V_des_trj.push_back(V_des_trj[V_des_trj.size() - 1]);
			for (unsigned int i = 0; i < V_des_trj.size() - 1; i++)
				Vdot_des_trj.push_back((V_des_trj[i + 1] - V_des_trj[i])*dt_inv);
			Vdot_des_trj.push_back(Vdot_des_trj[Vdot_des_trj.size() - 1]);
			return true;
		}
	}
	return false;
}

bool hybridPFCtrlManager::setDesiredJointVal(const Eigen::VectorXd& q0)
{
	if (isDesTrjSet && isSystemSet)
	{
		int flag = 0;
		q_des = m_robotManager->inverseKin(T_des_trj[T_des_trj.size() - 1], m_endeffector, true, SE3(), flag, q0);
		return true;
	}
	return false;
}

void hybridPFCtrlManager::hybridPFControl(int metric /*= DoubleGeodesic*/)
{
	Eigen::VectorXd jointVal = m_robotManager->getJointVal();
	Eigen::VectorXd jointVel = m_robotManager->getJointVel();
	Eigen::VectorXd tau = m_robotManager->getBiasTerm(jointVal, jointVel);
	Eigen::MatrixXd M = m_robotManager->getMassMatrix(jointVal);

	// selection matrices
	Eigen::MatrixXd Sv = velSelectionMtx(jointVal);
	Eigen::MatrixXd Sv_pinv = velSelectionMtxPinv(jointVal);
	Eigen::MatrixXd Sv_dot = velSelectionMtxDot(jointVal, jointVel);
	Eigen::MatrixXd Sf = forceSelectionMtx(jointVal);
	Eigen::MatrixXd Sf_pinv = forceSelectionMtxPinv(jointVal);

	// get current state
	SE3 Tec = getEndeffectorToContactFrame(jointVal);
	Eigen::MatrixXd J = getJacobianofContactFrame(jointVal, &Tec);
	Eigen::MatrixXd Jdot = getJacobianDotofContactFrame(jointVal, jointVel, &Tec, J);
	SE3 T = m_endeffector->GetFrame();
	se3 Vc = InvAd(Tec, m_endeffector->GetVel());				// end-effector body velocity expressed in contact frame
	dse3 Fext = dAd(Tec, InvdAd(T % m_contactLink->GetFrame(), m_contactLink->m_ConstraintImpulse)) * (1.0 / m_timeStep);
	//dse3 Fext = m_robotManager->readSensorValue();
	dse3 Fcont = dAd(Tec, InvdAd(T % m_contactLink->GetFrame(), m_contactLink->m_ConstraintImpulse)) * (1.0 / m_timeStep);
	if (controlStep == 0)
		del_lambda_int = Eigen::VectorXd::Zero(Sf.cols());
		
	// calculate norm of external force
	double Fnorm = 0.0;
	double Fcont_norm = 0.0;
	for (int i = 0; i < 6; i++)
	{
		Fnorm += Fext[i] * Fext[i];
		Fcont_norm += Fcont[i] * Fcont[i];
	}
		
	bool print = true;
	// print state
	if (controlStep % 10 == 0 && print)
	{
		cout << T << endl;
		cout << "Fext: " << Fext << endl;
		printf("%d\n", Fnorm > 0);
	}

	// get desired state
	unsigned int t = min(controlStep, T_des_trj.size() - 1);
	unsigned int f_t = min(controlStep, Fext_des_trj.size() - 1);
	SE3 T_des = T_des_trj[t];

	// calculate error
	se3 error_se3 = se3(0.0);
	if (metric == DoubleGeodesic)
	{
		Vec3 temp = Log(Inv(T.GetOrientation()) * T_des.GetOrientation());
		for (int i = 0; i < 3; i++)
			error_se3[i] = temp[i];
		temp = Inv(T.GetOrientation()) * (T_des.GetPosition() - T.GetPosition());
		for (int i = 0; i < 3; i++)
			error_se3[i + 3] = temp[i];
	}
	else
		error_se3 = Log(T%T_des);
	error_se3 = InvAd(getEndeffectorToContactFrame(T), error_se3);

	// calculate torque input
	if (Fcont_norm > -1.0 || controlStep == 0)
	{
		// hybrid position/force control (when contact occurs)
		se3 Vec = getBodyVelofContactFramefromEE(jointVal, jointVel, &Tec, J);

		///////////////////////////// for test /////////////////////////
		//Eigen::VectorXd Vc_test = J*jointVel;
		//double del = 0.0;
		//for (int i = 0; i < 6; i++)
		//	del += (Vc_test(i) - Vc[i])*(Vc_test(i) - Vc[i]);
		//cout << del << endl;
		////////////////////////////////////////////////////////////////

		Eigen::VectorXd v = Sv_pinv * se3toVector(Vc);
		Eigen::VectorXd lambda = Sf_pinv*dse3toVector(Fext);

		//cout << (J*jointVel).transpose() << endl;
		//cout << InvAd(Tec, m_robotManager->m_activeArmInfo->m_endeffector[m_num_endeffector]->GetVel());
		//cout << "q:     " << jointVal.transpose() << endl;
		//cout << "qdes:  " << q_des.transpose() << endl;

		// get desired state
		Eigen::VectorXd v_des = Sv_pinv*se3toVector(V_des_trj[t]);
		Eigen::VectorXd vdot_des = Sv_pinv*(se3toVector(Vdot_des_trj[t]) - Sv_dot*v);
		Eigen::VectorXd lambda_des = Sf_pinv*dse3toVector(Fext_des_trj[f_t]);

		// control input
		Eigen::VectorXd alpha = vdot_des + Kv_v * (v_des - v) + Kp_v * (Sv_pinv * se3toVector(error_se3));
		cout << "error: " << error_se3;
		cout << "Fext:  " << InvdAd(SE3((m_endeffector->GetFrame() * Tec).GetOrientation(), Vec3(0.0,0.0,0.0)), Fext);

		//////////////////////////////////////// for test ////////////////////////////////////////
		static Eigen::VectorXd v_bf = Eigen::VectorXd::Zero(v.size());
		cout << "alp_m: " << (v - v_bf).transpose() / m_timeStep << endl << endl;
		cout << "alp_d: " << alpha.transpose() << endl;
		v_bf = v;
		cout << "alp_v: " << (Kv_v * (v_des - v)).transpose() << endl;
		cout << "alp_p: " << (Kp_v * (Sv_pinv * se3toVector(error_se3))).transpose() << endl;
		//////////////////////////////////////////////////////////////////////////////////////////

		if (!isPFctrlled_bf)// || Fnorm < DBL_EPSILON)
			del_lambda_int.setZero();
		del_lambda_int += (lambda_des - lambda)*m_timeStep;
		Eigen::VectorXd f_lambda = lambda_des + Ki_f*del_lambda_int;

		////////////////////// for test ///////////////////////
		//Eigen::VectorXd f_lambda = Eigen::VectorXd::Zero(lambda.size());
		//////////////////////////////////////////////////////


		cout << "f_lam: " << f_lambda.transpose() << endl;
		cout << "lam:   " << lambda.transpose() << endl;
		cout << "lamdes:" << lambda_des.transpose() << endl;
		cout << "lamint:" << (Ki_f*del_lambda_int).transpose() << endl;

		///////////////////// for test ///////////////////////
		//Eigen::VectorXd aa = Sv_dot*v - Jdot*jointVel + Sv*alpha;
		//Eigen::VectorXd a1 = J.ldlt().solve(aa);
		//Eigen::VectorXd a2 = J.inverse()*(Sv_dot*v - Jdot*jointVel + Sv*alpha);
		//cout << (a1 - a2).norm() << endl;
		//cout << (aa - J*a1).norm() << endl;
		//cout << (aa - J*a2).norm() << endl;
		//////////////////////////////////////////////////////


		if (J.cols() == 6)
		{
			Eigen::MatrixXd Jinv = J.inverse();
			tau += M * (Jinv*(Sv_dot*v - Jdot*jointVel + Sv*alpha)) - Jinv.transpose()*(Sf*f_lambda);
		}
		else if (J.cols() > 6)
			tau += M * (J.transpose() * ((J*J.transpose()).ldlt().solve(Sv_dot*v - Jdot*jointVel + Sv*alpha - J*M.ldlt().solve(J.transpose()*(Sf*f_lambda)))));

		//cout << "tau:  " << tau.transpose() << endl << endl;
		isPFctrlled_bf = true;
	}
	else
	{
		double kv = 10.0;
		double kp = 10.0;
		// computed torque control (when contact doesn't occur)
		Eigen::MatrixXd Jinv = J.inverse();
		Eigen::VectorXd dq_des = Jinv*se3toVector(V_des_trj[t]);
		Eigen::VectorXd ddq_des = Jinv*(se3toVector(Vdot_des_trj[t]) - Jdot*jointVel);
		Eigen::VectorXd jointAcc = ddq_des + kv*(dq_des - jointVel) + kp*(q_des - jointVal);
		tau += M*jointAcc;
		if ((q_des - jointVal).norm() < 1.0e-4)
			tau -= J.transpose()*dse3toVector(Fext_des_trj[f_t]);

		// cartesian space position control (when contact doesn't occur)
		//Eigen::VectorXd dV = se3toVector(Vdot_des_trj[t] - kv*(Vc - V_des_trj[t]) - kp*(- error_se3));
		//if (J.cols() == 6)
		//	tau += M * (J.inverse()*(dV - Jdot*jointVel));
		//else if (J.cols() > 6)
		//	tau += M * (J.transpose() * ((J*J.transpose()).ldlt().solve(dV)));
		isPFctrlled_bf = false;
	}
	
	m_robotManager->controlJointTorque(tau);

	controlStep++;
}

Eigen::VectorXd hybridPFCtrlManager::del_lambda_int = Eigen::VectorXd::Zero(0);
unsigned int hybridPFCtrlManager::controlStep = 0;
bool hybridPFCtrlManager::isPFctrlled_bf = false;

bool hybridPFCtrlManagerPlane::setContactFrameOri(srSystem * env, Vec3 contactNormal)
{
	return setContactFrameOri(env->GetBaseLink(), contactNormal);
}

bool hybridPFCtrlManagerPlane::setContactFrameOri(srLink * link, Vec3 contactNormal)
{
	// align contact normal as z-axis of contact frame orientation
	vector<Vec3> dirs(3);
	for (unsigned int i = 0; i < dirs.size(); i++)
		for (unsigned int j = 0; j < 3; j++)
			dirs[i][j] = link->GetFrame()[3 * i + j];

	for (unsigned int i = 0; i < dirs.size(); i++)
	{
		double sign = 1.0;
		if (abs(Cosine(dirs[i], contactNormal) - 1.0) < 1.e-4 || abs(Cosine(dirs[i], contactNormal) + 1.0) < 1.e-4)
		{
			if (abs(Cosine(dirs[i], contactNormal) - 1.0) < 1.e-4)
				sign = 1.0;
			else
				sign = -1.0;
			for (unsigned int k = 0; k < 3; k++)
				contactFrameOri[6 + k] = sign*dirs[i][k];
			for (unsigned int k = 0; k < 3; k++)
				contactFrameOri[k] = sign*dirs[(i + 1) % 3][k];
			for (unsigned int k = 0; k < 3; k++)
				contactFrameOri[3 + k] = dirs[(i + 2) % 3][k];
		}
	}

	Sv = Eigen::MatrixXd::Zero(6, 3);
	Sv(2, 0) = 1.0;
	Sv(3, 1) = 1.0;
	Sv(4, 2) = 1.0;
	Sv_pinv = pinv(Sv);
	Sv_dot = Eigen::MatrixXd::Zero(6, 3);
	Sf = Eigen::MatrixXd::Zero(6, 3);
	Sf(0, 0) = 1.0;
	Sf(1, 1) = 1.0;
	Sf(5, 2) = 1.0;
	Sf_pinv = pinv(Sf);
	return true;
}

bool hybridPFCtrlManagerPlane::setInitialConfig(const Eigen::VectorXd & jointVal, Vec3 contactPoint)
{
	// set initial configuration to have proper contact with the environment
	if (isSystemSet)
	{
		m_robotManager->setJointVal(jointVal);
		SE3 Tinit = m_endeffector->GetFrame();
		SE3 contactTemp = SE3(contactPoint);
		relPos = (Tinit % contactTemp).GetPosition();
		return true;
	}
	return false;
}

Eigen::MatrixXd hybridPFCtrlManagerPlane::forceSelectionMtx(const Eigen::VectorXd & jointVal)
{
	return Sf;
}

Eigen::MatrixXd hybridPFCtrlManagerPlane::forceSelectionMtxPinv(const Eigen::VectorXd & jointVal)
{
	return Sf_pinv;
}

Eigen::MatrixXd hybridPFCtrlManagerPlane::velSelectionMtx(const Eigen::VectorXd & jointVal)
{
	return Sv;
}

Eigen::MatrixXd hybridPFCtrlManagerPlane::velSelectionMtxPinv(const Eigen::VectorXd & jointVal)
{
	return Sv_pinv;
}

Eigen::MatrixXd hybridPFCtrlManagerPlane::velSelectionMtxDot(const Eigen::VectorXd & jointVal, const Eigen::VectorXd & jointVel)
{
	return Sv_dot;
}

SE3 hybridPFCtrlManagerPlane::getEndeffectorToContactFrame(const SE3 & Te)
{
	SE3 Tcontact;
	Tcontact.SetOrientation(contactFrameOri);
	Tcontact.SetPosition(Te.GetPosition() + Te.GetOrientation()*relPos);
	return Te % Tcontact;
}

SE3 hybridPFCtrlManagerPlane::getEndeffectorToContactFrame(const Eigen::VectorXd & jointVal)
{
	m_robotManager->setJointVal(jointVal);
	return getEndeffectorToContactFrame(m_endeffector->GetFrame());
}

se3 hybridPFCtrlManagerPlane::getBodyVelofContactFramefromEE(const SE3 & Te, const se3 & Ve, SE3 * Tec /*= NULL*/)
{
	SE3 Tec_temp;
	if (Tec == NULL)
	{
		Tec_temp = getEndeffectorToContactFrame(Te);
		Tec = &Tec_temp;
	}

	se3 Vec = -InvAd(*Tec, Ve);
	for (int i = 0; i < 3; i++)
		Vec[i + 3] = 0.0;
	return Vec;
}

se3 hybridPFCtrlManagerPlane::getBodyVelofContactFramefromEE(const Eigen::VectorXd & jointVal, const Eigen::VectorXd & jointVel, SE3* Tec /*= NULL*/, Eigen::MatrixXd& Jc /*= Eigen::MatrixXd()*/)
{
	SE3 Tec_temp;
	if (Tec == NULL)
	{
		Tec_temp = getEndeffectorToContactFrame(jointVal);
		Tec = &Tec_temp;
	}
	if (Jc.rows() == 0)
		Jc = getJacobianofContactFrame(jointVal, Tec);

	se3 Vec = -Vectortose3(Jc*jointVel);
	for (int i = 0; i < 3; i++)
		Vec[i + 3] = 0.0;
	return Vec;
}

Eigen::MatrixXd hybridPFCtrlManagerPlane::getJacobianofContactFrame(const Eigen::VectorXd & jointVal, SE3* Tec /*= NULL*/)
{
	SE3 Tec_temp;
	if (Tec == NULL)
	{
		Tec_temp = getEndeffectorToContactFrame(jointVal);
		Tec = &Tec_temp;
	}
	Eigen::MatrixXd Jc = m_robotManager->getBodyJacobian(jointVal, m_endeffector, *Tec);
	//Jc.block(0, 0, 3, Jc.cols()) = Eigen::MatrixXd::Zero(3, Jc.cols());
	return Jc;
}

Eigen::MatrixXd hybridPFCtrlManagerPlane::getJacobianDotofContactFrame(const Eigen::VectorXd & jointVal, const Eigen::VectorXd & jointVel, SE3* Tec /*= NULL*/, Eigen::MatrixXd& Jc /*= Eigen::MatrixXd()*/)
{
	SE3 Tec_temp;
	if (Tec == NULL)
	{
		Tec_temp = getEndeffectorToContactFrame(jointVal);
		Tec = &Tec_temp;
	}
	if (Jc.rows() == 0)
		Jc = getJacobianofContactFrame(jointVal, Tec);
	
	Eigen::MatrixXd Jcdot = m_robotManager->getBodyJacobianDot(jointVal, jointVel, m_endeffector, *Tec);
	
	se3 Vce = Vectortose3(Jc*jointVel);
	for (int i = 0; i < 3; i++)
		Vce[i + 3] = 0.0;

	for (int i = 0; i < Jcdot.cols(); i++)
		Jcdot.col(i) += se3toVector(ad(Vce, Vectortose3(Jc.col(i))));
	//Jcdot.block(0, 0, 3, Jcdot.cols()) = Eigen::MatrixXd::Zero(3, Jcdot.cols());
	return Jcdot;
}


