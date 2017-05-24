#include "hybridPFCtrlManager.h"



hybridPFCtrlManager_6dof::hybridPFCtrlManager_6dof()
{
	T_des_trj.resize(0);
	V_des_trj.resize(0);
	Vdot_des_trj.resize(0);
	Fext_des_trj.resize(0);

	isDesTrjSet = false;
	isSystemSet = false;
	isICSet = false;
}


hybridPFCtrlManager_6dof::~hybridPFCtrlManager_6dof()
{
}

bool hybridPFCtrlManager_6dof::setSystem(robotManager * _robotManager, srLink * endeffector, SE3 offset, srLink * contactLink)
{
	m_robotManager = _robotManager;
	m_offset = offset;
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

bool hybridPFCtrlManager_6dof::setTimeStep(double _timeStep)
{
	m_timeStep = _timeStep;
	return true;
}

bool hybridPFCtrlManager_6dof::setDesiredTraj(vector<SE3> T_trj, vector<dse3> Fext_trj)
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
		double dt_inv = 1.0 / m_timeStep;
		for (unsigned int i = 0; i < T_trj.size() - 1; i++)
			V_des_trj.push_back(Log(T_trj[i] % T_trj[i + 1])*dt_inv);
		V_des_trj.push_back(V_des_trj[V_des_trj.size() - 1]);
		for (unsigned int i = 0; i < V_des_trj.size() - 1; i++)
			Vdot_des_trj.push_back((V_des_trj[i + 1] - V_des_trj[i])*dt_inv);
		Vdot_des_trj.push_back(Vdot_des_trj[Vdot_des_trj.size() - 1]);
		return true;
	}
	return false;
}

bool hybridPFCtrlManager_6dof::setDesiredJointVal(const Eigen::VectorXd& q0)
{
	if (isDesTrjSet && isSystemSet)
	{
		int flag = 0;
		q_des = m_robotManager->inverseKin(T_des_trj[T_des_trj.size() - 1], m_endeffector, true, SE3(), flag, q0);
		return true;
	}
	return false;
}

void hybridPFCtrlManager_6dof::setSelectionMatrix(const Eigen::MatrixXd & SelectionMtx)
{
	// SelectMtx * Vb = 0 should be satisfied from constraint
	SelectMtx = SelectionMtx;
}

void hybridPFCtrlManager_6dof::setGain(const double kv_v, const double kp_v, const double ki_v, const double kp_f, const double ki_f)
{
	Kv_v = kv_v * Eigen::MatrixXd::Identity(6, 6);
	Kp_v = kp_v * Eigen::MatrixXd::Identity(6, 6);
	Ki_v = ki_v * Eigen::MatrixXd::Identity(6, 6);
	Kp_f = kp_f * Eigen::MatrixXd::Identity(6, 6);
	Ki_f = ki_f * Eigen::MatrixXd::Identity(6, 6);
}

void hybridPFCtrlManager_6dof::resetIntegralError()
{
	F_int = dse3(0.0);
}

void hybridPFCtrlManager_6dof::hybridPFControl(int metric)
{
	Eigen::VectorXd jointVal = m_robotManager->getJointVal();
	Eigen::VectorXd jointVel = m_robotManager->getJointVel();
	Eigen::VectorXd tau = m_robotManager->getBiasTerm(jointVal, jointVel);
	Eigen::MatrixXd M = m_robotManager->getMassMatrix(jointVal);

	Eigen::MatrixXd J = m_robotManager->getBodyJacobian(jointVal, m_endeffector, m_offset);
	Eigen::MatrixXd Jinv = J.inverse();
	
	Eigen::MatrixXd Jdot = m_robotManager->getBodyJacobianDot(jointVal, jointVel, m_endeffector, m_offset);

	Eigen::MatrixXd Lambda = Jinv.transpose()*M*Jinv;
	Eigen::MatrixXd Lambda_inv = Lambda.inverse();
	
	Eigen::MatrixXd P;
	if (SelectMtx.rows() == 0)
		P = Eigen::MatrixXd::Identity(6, 6);
	else if (SelectMtx.rows() == 6)
		P = Eigen::MatrixXd::Zero(6, 6);
	else
	{
		Eigen::MatrixXd Lambda_f = SelectMtx*Lambda_inv*SelectMtx.transpose();
		Eigen::MatrixXd Lambda_f_inv = Lambda_f.inverse();
		P = Eigen::MatrixXd::Identity(6, 6) - SelectMtx.transpose()*Lambda_f_inv*SelectMtx*Lambda_inv;
	}
		
	//Eigen::VectorXd eta = Jinv.transpose()*tau - Lambda*Jdot*jointVel;
	tau -= J.transpose()*Lambda*Jdot*jointVel;

	// get current state
	SE3 T = m_endeffector->GetFrame() * m_offset;
	se3 V = InvAd(m_offset, m_endeffector->GetVel());
	dse3 Fext = - InvdAd(T % m_contactLink->GetFrame(), m_contactLink->m_ConstraintImpulse) * (1.0 / m_timeStep);

	// get desired state
	unsigned int t = min(controlStep, T_des_trj.size() - 1);
	unsigned int f_t = min(controlStep, Fext_des_trj.size() - 1);
	SE3 T_des = T_des_trj[t];


	// calculate error
	se3 error_se3 = se3(0.0);
	SE3 Te = T%T_des;
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

	// hybrid motion and force control
	se3 AdVd = Ad(Te, V_des_trj[t]);
	se3 Ve = AdVd - V;
	se3 Z = ad(Ve, AdVd) + Ad(Te, Vdot_des_trj[t]);
	X_int += error_se3 * m_timeStep;
	Eigen::VectorXd curFerror = dse3toVector(Fext_des_trj[f_t] - Fext);

	// remove other directions except z-dir force
	//for (int i = 0; i < 5; i++)
	//	curFerror[i] = 0.0;
	if (SelectMtx.rows() > 0)
		F_int += Vectortodse3(curFerror) * m_timeStep;
	tau += J.transpose()*(P*Lambda*(se3toVector(Z) + Kp_v*se3toVector(error_se3) + Ki_v*se3toVector(X_int) + Kv_v*se3toVector(Ve))
		+ (Eigen::MatrixXd::Identity(6, 6) - P)*(curFerror + dse3toVector(Fext) + Kp_f*curFerror + Ki_f*dse3toVector(F_int)));

	m_robotManager->controlJointTorque(tau);

	controlStep++;
}

dse3 hybridPFCtrlManager_6dof::F_int = dse3(0.0);
se3 hybridPFCtrlManager_6dof::X_int = se3(0.0);
unsigned int hybridPFCtrlManager_6dof::controlStep = 0;
bool hybridPFCtrlManager_6dof::isPFctrlled_bf = false;