#pragma once
#include "rrtManager.h"

using namespace std;

class rrtConstraint;


class TBrrtManager : public rrtManager
{
public:
	TBrrtManager();
	~TBrrtManager();

	// handle constraint
	void									setConstraint(rrtConstraint* constraint);
	void									clearConstraints();

protected:
	virtual Eigen::VectorXd					extendStepSize(const Eigen::VectorXd& vertPos1, const Eigen::VectorXd& vertPos2, double criterion, TARGET_TREE tree = TARGET_TREE::TREE1,
														   double error_threshold = 0.1);
	// smoothing function
	vector<rrtVertex*>						getConstrainedPathConnectingTwoVertices(rrtVertex* vertex1, rrtVertex* vertex2, double eps, int maxIter = 10000);
	virtual vector<rrtVertex*>				getCandidateVertices(vector<rrtVertex*> vertices);
	virtual bool							replaceVertices(list<rrtVertex*>& path, vector<rrtVertex*>& tempVertices, vector<rrtVertex*>& removedVertex);
	Eigen::VectorXd *						TBrandomSample(const Eigen::VectorXd qroot, double range);

protected:
	double									_error_threshold;
	void									setThreshold(double threshold);

protected:
	rrtConstraint*							rrtConstraints;

protected:
	// Tangent Space
	list<vector<Eigen::VectorXd *> *>		TangentSpaces;
	vector<Eigen::VectorXd *>				tangentBasis;
	void									getTangentBasis(const Eigen::VectorXd& vertPos1);
	void									project2TangentSpace(Eigen::VectorXd& jointval, Eigen::VectorXd& jointvalRef);
	bool									projectionNewtonRaphson(Eigen::VectorXd& jointval, double threshold = 0.1, int maxIter = 100);
	Eigen::MatrixXd							ProjectionMatrix;
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