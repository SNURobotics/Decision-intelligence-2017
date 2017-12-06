#pragma once
#include "../RRTmanager/rrtManager.h"
#include "robotManager.h"

class trajFollowVectorField : public rrtVectorField
{
public:
	trajFollowVectorField();
	~trajFollowVectorField();

	void								setRefTraj(const vector<Eigen::VectorXd>& refTraj);
	virtual Eigen::VectorXd				getVectorField(const Eigen::VectorXd& pos1);
	virtual	void						checkFeasibility(int nDim);
public:
	vector<Eigen::VectorXd>				_refTraj;
};

class river2dofVectorField : public rrtVectorField
{
public:
	river2dofVectorField();
	~river2dofVectorField();
	
	void								setBound(const Eigen::VectorXd& lowerBound, const Eigen::VectorXd& upperBound);
	virtual Eigen::VectorXd				getVectorField(const Eigen::VectorXd& pos1);
	virtual	void						checkFeasibility(int nDim);
public:
	Eigen::VectorXd						_lowerBound;
	Eigen::VectorXd						_upperBound;
};

class singularityAvoidanceVectorField : public rrtVectorField
{
public:
	singularityAvoidanceVectorField();
	~singularityAvoidanceVectorField();

	void								setRobotEndeffector(robotManager* rManager, srLink* link);
	void								setManipulabilityKind(robotManager::manipKind kind);
	virtual Eigen::VectorXd				getVectorField(const Eigen::VectorXd& pos1);
	virtual void						checkFeasibility(int nDim);
public:
	robotManager*						_rManager;
	srLink*								_link;
	robotManager::manipKind				_kind;
	double								_eps;		// parameter for numerical stability in potential function
};

class workspaceConstantVectorField : public rrtVectorField
{
public:
	workspaceConstantVectorField();
	~workspaceConstantVectorField();

	virtual Eigen::VectorXd				getVectorField(const Eigen::VectorXd& pos1);
	virtual void						checkFeasibility(int nDim);
public:
	robotManager*						_rManager;
	srLink*								_link;
	Eigen::VectorXd						_workspaceVector;
};

class objectClearanceVectorField : public rrtVectorField
{

};