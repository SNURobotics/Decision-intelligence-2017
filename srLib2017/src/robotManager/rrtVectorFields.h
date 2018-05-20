#pragma once
#include "../RRTmanager/vfrrtManager.h"
#include "robotManager.h"

class robotRRTVectorField : public rrtVectorField
{
public:
	robotRRTVectorField();
	robotRRTVectorField(robotManager* rManager, srLink* link);
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
	singularityAvoidanceVectorField(robotManager* rManager, srLink* link);
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
	workspaceConstantPositionVectorField(robotManager* rManager, srLink* link);
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
	objectClearanceVectorField(robotManager* rManager, srLink* link);
	~objectClearanceVectorField();

	void								setObjectLocation(Vec3 objectLoc, double size = 0.0);
	void								setWeights(vector<double> weight);
	void								setLinks(vector<srLink*> links);
	void								setOffsets(vector<SE3> offsets);
	virtual Eigen::VectorXd				getVectorField(const Eigen::VectorXd& pos1);
	virtual void						checkFeasibility(int nDim);

public:
	double								_eps;					// epsilon value to avoid zero denominator
	double								_size;					// assume sphere object of radius = size, and repulsive point is located at the nearest point to the links
	Vec3								_objectLoc;				// location of point object to avoid
	vector<srLink*>						_links;					// links which should also avoid object
	vector<SE3>							_offsets;				// offsets of the location of avoidance from link fixed frame
	vector<double>						_weights;				// coefficient for vectors from each link (end-effector weight is 1.0)
	SE3									_endeffectorOffset;		// offset for end-effector
};