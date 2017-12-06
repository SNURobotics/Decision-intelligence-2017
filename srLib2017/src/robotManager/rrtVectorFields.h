#pragma once
#include "../RRTmanager/rrtManager.h"

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

};

class objectClearanceVectorField : public rrtVectorField
{

};