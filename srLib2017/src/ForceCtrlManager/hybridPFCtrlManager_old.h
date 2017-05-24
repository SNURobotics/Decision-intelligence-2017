#pragma once
#include "robotManager\robotManager.h"
class hybridPFCtrlManager
{
	// abstract class for hybrid position / force controller of single end-effector 
public:
	hybridPFCtrlManager();
	~hybridPFCtrlManager();

	enum Metric
	{
		DoubleGeodesic = 0, Logarithm = 1
	};

	bool								setSystem(robotManager* _robotManager, srLink* endeffector, srLink* contactLink = NULL);
	bool								setTimeStep(double _timeStep);
	bool								setDesiredTraj(vector<SE3> T_trj, vector<dse3> Fext_trj);
	bool								setDesiredJointVal(const Eigen::VectorXd& q0);

	// get selection matrix and others according to the contact status
	virtual Eigen::MatrixXd				forceSelectionMtx(const Eigen::VectorXd& jointVal) = 0;
	virtual Eigen::MatrixXd				forceSelectionMtxPinv(const Eigen::VectorXd& jointVal) = 0;
	virtual Eigen::MatrixXd				velSelectionMtx(const Eigen::VectorXd& jointVal) = 0;
	virtual Eigen::MatrixXd				velSelectionMtxPinv(const Eigen::VectorXd& jointVal) = 0;
	virtual Eigen::MatrixXd				velSelectionMtxDot(const Eigen::VectorXd& jointVal, const Eigen::VectorXd& jointVel) = 0;
	virtual SE3							getEndeffectorToEEContactFrame() = 0;		// SE(3) from ee link to another ee fixed frame where contact should occur
	virtual SE3							getEndeffectorToContactFrame(const SE3& Te) = 0;
	virtual SE3							getEndeffectorToContactFrame(const Eigen::VectorXd& jointVal) = 0;
	virtual se3							getBodyVelofContactFramefromEE(const SE3& Te, const se3& Ve, SE3* Tec = NULL) = 0;
	virtual se3							getBodyVelofContactFramefromEE(const Eigen::VectorXd& jointVal, const Eigen::VectorXd& jointVel, SE3* Tec = NULL, Eigen::MatrixXd& Jc = Eigen::MatrixXd()) = 0;
	virtual Eigen::MatrixXd				getJacobianofContactFrame(const Eigen::VectorXd& jointVal, SE3* Tec = NULL) = 0;
	virtual Eigen::MatrixXd				getJacobianDotofContactFrame(const Eigen::VectorXd& jointVal, const Eigen::VectorXd& jointVel, SE3* Tec = NULL, Eigen::MatrixXd& Jc = Eigen::MatrixXd()) = 0;
	
	// hybrid position / force control
	void								hybridPFControl(int metric = DoubleGeodesic);

public:
	robotManager*						m_robotManager;
	srLink*								m_endeffector;
	srLink*								m_contactLink;
	Eigen::MatrixXd						Kv_v;				// velocity feedback gain for position tracking
	Eigen::MatrixXd						Kp_v;				// position feedback gain for position tracking
	Eigen::MatrixXd						Ki_f;				// integral feedback gain for force tracking
	vector<SE3>							T_des_trj;			// desired trajectory of contact frame expressed in global fixed frame
	Eigen::VectorXd						q_des;				// desired joint value earned from T_des_trj[end]
	vector<se3>							V_des_trj;
	vector<se3>							Vdot_des_trj;
	vector<dse3>						Fext_des_trj;		// desired trajectory of contact force expressed in contact frame
	
	static Eigen::VectorXd				del_lambda_int;		// integration of (lambda_d - lambda)
	static unsigned int					controlStep;
	static bool							isPFctrlled_bf;		// true: hybridPF controlled at before step, false: position controlled at before step
	double								m_timeStep;

	bool								isDesTrjSet;
	bool								isContactOriSet;
	bool								isSystemSet;
	bool								isICSet;
};

class hybridPFCtrlManagerPlane : public hybridPFCtrlManager
{
	// sliding end-effector on a plane
public:
	bool								setContactFrameOri(srSystem* env, Vec3 contactNormal);			// contact normal should be outside direction of the box & parallel to principal axes
	bool								setContactFrameOri(srLink* link, Vec3 contactNormal);
	bool								setInitialConfig(const Eigen::VectorXd& jointVal, Vec3 contactPoint);			// should be called after system setting
	virtual Eigen::MatrixXd				forceSelectionMtx(const Eigen::VectorXd& jointVal);
	virtual Eigen::MatrixXd				forceSelectionMtxPinv(const Eigen::VectorXd& jointVal);
	virtual Eigen::MatrixXd				velSelectionMtx(const Eigen::VectorXd& jointVal);
	virtual Eigen::MatrixXd				velSelectionMtxPinv(const Eigen::VectorXd& jointVal);
	virtual Eigen::MatrixXd				velSelectionMtxDot(const Eigen::VectorXd& jointVal, const Eigen::VectorXd& jointVel);
	virtual SE3							getEndeffectorToEEContactFrame() { return SE3(relPos); };
	virtual SE3							getEndeffectorToContactFrame(const SE3& Te);
	virtual SE3							getEndeffectorToContactFrame(const Eigen::VectorXd& jointVal);
	virtual se3							getBodyVelofContactFramefromEE(const SE3& Te, const se3& Ve, SE3* Tec = NULL);
	virtual se3							getBodyVelofContactFramefromEE(const Eigen::VectorXd& jointVal, const Eigen::VectorXd& jointVel, SE3* Tec = NULL, Eigen::MatrixXd& Jc = Eigen::MatrixXd());
	virtual Eigen::MatrixXd				getJacobianofContactFrame(const Eigen::VectorXd& jointVal, SE3* Tec = NULL);
	virtual Eigen::MatrixXd				getJacobianDotofContactFrame(const Eigen::VectorXd& jointVal, const Eigen::VectorXd& jointVel, SE3* Tec = NULL, Eigen::MatrixXd& Jc = Eigen::MatrixXd());

public:
	Eigen::MatrixXd						Sv;
	Eigen::MatrixXd						Sv_pinv;
	Eigen::MatrixXd						Sv_dot;
	Eigen::MatrixXd						Sf;
	Eigen::MatrixXd						Sf_pinv;
	SO3									contactFrameOri;		// fixed orientation of contact frame (only the contact point can be changed)
	Vec3								relPos;					// fixed relative position of contact frame with respect to endeffector frame
};