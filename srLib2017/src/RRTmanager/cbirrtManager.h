#pragma once
#include "rrtManager.h"

using namespace std;

class rrtConstraint;


class cbirrtManager : public rrtManager
{
public:
	cbirrtManager();
	~cbirrtManager();

	// handle constraint
	void									setConstraint(rrtConstraint* constraint);
	void									clearConstraints();


protected:
	virtual Eigen::VectorXd					extendStepSize(const Eigen::VectorXd& vertPos1, const Eigen::VectorXd& vertPos2, double criterion, TARGET_TREE tree = TARGET_TREE::TREE1);
	
	// smoothing function
	vector<rrtVertex*>						getConstrainedPathConnectingTwoVertices(rrtVertex* vertex1, rrtVertex* vertex2, double eps, int maxIter = 10000);
	virtual vector<rrtVertex*>				getCandidateVertices(vector<rrtVertex*> vertices);
	virtual bool							replaceVertices(list<rrtVertex*>& path, vector<rrtVertex*>& tempVertices, vector<rrtVertex*>& removedVertex);


	rrtConstraint*							rrtConstraints;
};

class rrtConstraint
{
public:
	rrtConstraint();
	~rrtConstraint();

	int								nDim;

	virtual	Eigen::VectorXd			getConstraintVector(const Eigen::VectorXd& jointVal) = 0;
	virtual	Eigen::MatrixXd			getConstraintJacobian(const Eigen::VectorXd& jointVal) = 0;
	virtual	void					project2ConstraintManifold(Eigen::VectorXd& jointVal) = 0;
};