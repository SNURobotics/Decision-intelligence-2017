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

class robotRRTVectorField : public rrtVectorField
{
public:
	robotRRTVectorField();
	~robotRRTVectorField();

	void								setRobotEndeffector(robotManager* rManager, srLink* link);
	virtual Eigen::VectorXd				getVectorField(const Eigen::VectorXd& pos1) = 0;
	virtual void						checkFeasibility(int nDim);
public:
	robotManager*						_rManager;
	srLink*								_link;
};

class singularityAvoidanceVectorField : public robotRRTVectorField
{
public:
	singularityAvoidanceVectorField();
	~singularityAvoidanceVectorField();

	void								setManipulabilityKind(robotManager::manipKind kind);
	virtual Eigen::VectorXd				getVectorField(const Eigen::VectorXd& pos1);

public:
	robotManager::manipKind				_kind;
	double								_eps;		// parameter for numerical stability in potential function
};

class workspaceConstantPositionVectorField : public robotRRTVectorField
{
public:
	workspaceConstantPositionVectorField();
	~workspaceConstantPositionVectorField();

	void								setWorkspaceVector(const Eigen::VectorXd& vec);
	virtual Eigen::VectorXd				getVectorField(const Eigen::VectorXd& pos1);
	virtual void						checkFeasibility(int nDim);

public:
	Eigen::VectorXd						_workspaceVector;		// vector to follow in workspace expressed in global coordinate
	bool								_fixOri;				// true: generate vector field to perserve orientation
};

class objectClearanceVectorField : public rrtVectorField
{

};