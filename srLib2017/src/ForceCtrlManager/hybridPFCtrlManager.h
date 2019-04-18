#pragma once
#include "robotManager\robotManager.h"
class hybridPFCtrlManager_6dof
{
	// abstract class for hybrid position / force controller of single end-effector 
public:
	hybridPFCtrlManager_6dof();
	~hybridPFCtrlManager_6dof();

	enum Metric
	{
		DoubleGeodesic = 0, Logarithm = 1
	};

	bool								setSystem(robotManager* _robotManager, srLink* endeffector, SE3 offset = SE3(), srLink* contactLink = NULL);
	bool								setSystem(robotManager* _robotManager, srLink* endeffector, SE3 offset, vector<srLink*> contactLinks);
	bool								setTimeStep(double _timeStep);
	bool								setDesiredTraj(vector<SE3> T_trj, vector<dse3> Fext_trj);
	bool								setDesiredJointVal(const Eigen::VectorXd& q0);
	void								setSelectionMatrix(const Eigen::MatrixXd& SelectionMtx);
	void								setGain(const double kv_v, const double kp_v, const double ki_v, const double kp_f, const double ki_f);
	void								resetIntegralError();
	// hybrid position / force control
	void								hybridPFControl(int metric = DoubleGeodesic);

public:
	robotManager*						m_robotManager;
	srLink*								m_endeffector;
	vector<srLink*>						m_contactLinks;
	SE3									m_offset;
	Eigen::MatrixXd						Kv_v;				// velocity feedback gain for position tracking
	Eigen::MatrixXd						Kp_v;				// position feedback gain for position tracking
	Eigen::MatrixXd						Ki_v;				// integral feedback gain for position tracking
	Eigen::MatrixXd						Kp_f;				// feedback gain for force tracking
	Eigen::MatrixXd						Ki_f;				// integral feedback gain for force tracking
	vector<SE3>							T_des_trj;			// desired trajectory of contact frame expressed in global fixed frame
	Eigen::VectorXd						q_des;				// desired joint value earned from T_des_trj[end]
	vector<se3>							V_des_trj;
	vector<se3>							Vdot_des_trj;
	vector<dse3>						Fext_des_trj;		// desired trajectory of contact force expressed in contact frame
	Eigen::MatrixXd						SelectMtx;			// SelectMtx*V = 0 should be satisfied in contact frame
	
	static dse3							F_int;		// integration of (F_d - F)
	static se3							X_int;		// integration of (X_e)
	static unsigned int					controlStep;
	static bool							isPFctrlled_bf;		// true: hybridPF controlled at before step, false: position controlled at before step
	double								m_timeStep;

	bool								isDesTrjSet;
	bool								isSystemSet;
	bool								isICSet;

	static se3							Ve_bf;
	static se3							temp2;
	static se3							temp3;
};
