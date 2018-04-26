#pragma once
#include "rrtManager.h"

using namespace std;

class rrtVectorField;

class vfrrtManager : public rrtManager
{
	vfrrtManager();
	~vfrrtManager();

public:
	enum MODE
	{
		SIMPLE = 0, GENUINE = 1
	};
	virtual bool							isProblemFeasible();
	virtual void							connectParentAndChild(rrtVertex* parentVertex, rrtVertex* childVertex);

	// vector field
	bool									checkVectorFieldFeasibility();
	void									addVectorField(rrtVectorField* vectorField);
	void									clearVectorField();
	Eigen::VectorXd							getVectorField(const Eigen::VectorXd& pos1);
	void									setVectorFieldWeight(double weight);

	double									getUpstreamCost(const Eigen::VectorXd& vertPos1, const Eigen::VectorXd& vertPos2, int n = 10);

protected:
	virtual Eigen::VectorXd					extendStepSize(const Eigen::VectorXd& vertPos1, const Eigen::VectorXd& vertPos2, double criterion, TARGET_TREE tree = TARGET_TREE::TREE1);
	Eigen::VectorXd							extendStepSize_noVectorField(const Eigen::VectorXd& vertPos1, const Eigen::VectorXd& vertPos2, double criterion);
	Eigen::VectorXd							extendStepSize_simple(const Eigen::VectorXd& vertPos1, const Eigen::VectorXd& vertPos2, double criterion, TARGET_TREE tree = TARGET_TREE::TREE1);
	Eigen::VectorXd							extendStepSize_genuine(const Eigen::VectorXd& vertPos1, const Eigen::VectorXd& vertPos2, double criterion, TARGET_TREE tree = TARGET_TREE::TREE1);

//protected:  //  for genuine algorithm
//	double									sampleInputWeight(const Eigen::VectorXd& v_field, const Eigen::VectorXd& v_rand);


protected:
	virtual double							getCost(rrtVertex* pos1, rrtVertex* pos2);
	virtual double							getRRTpathSmoothingCost(rrtVertex* pos1, rrtVertex* pos2, vector<rrtVertex*>& removedVertex);
	virtual double							getNewPathSmoothingCost(vector<rrtVertex*> vertices);

	bool									_vectorFieldExist;
	vector<rrtVectorField*>					_vectorFields;
	double									_vectorFieldWeight;
	double									_lambda;
	vfrrtManager::MODE						_algorithmMode;
};


// rrt vector field

class rrtVectorField
{
public:
	rrtVectorField();
	~rrtVectorField();

	virtual	Eigen::VectorXd			getVectorField(const Eigen::VectorXd& pos1) = 0;
	virtual	void					checkFeasibility(int nDim) = 0;
	bool							_isFeasible;
	double							_C;		// parameter to control the scale of vector field
};

// examples of rrt vector fields

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