#pragma once
#include "../RRTmanager/vfrrtManager.h"
#include "robotManager.h"

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
	Eigen::VectorXd						_centerPoint;			// point where potential is maximum
	bool								_fixOri;				// true: generate vector field to perserve orientation
};

class objectClearanceVectorField : public robotRRTVectorField
{
public:
	objectClearanceVectorField();
	~objectClearanceVectorField();

	void								setObjects(vector<Vec3> objectLoc);
	void								setWeights(vector<double> weight);
	virtual Eigen::VectorXd				getVectorField(const Eigen::VectorXd& pos1);
	virtual void						checkFeasibility(int nDim);

public:
	vector<Vec3>						_objectCenters;			// location of point objects to avoid
	vector<double>						_objectWeights;			// coefficient for vectors from each object
	SE3									_endeffectorOffset;		// offset for end-effector
};