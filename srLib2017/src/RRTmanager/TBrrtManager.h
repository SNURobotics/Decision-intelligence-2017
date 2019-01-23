#pragma once
#include "rrtManager.h"

using namespace std;

class rrtConstraint;
class tangentSpace;

class TBrrtManager : public rrtManager
{
public:
	TBrrtManager();
	~TBrrtManager();

	// handle constraint
	void									setConstraint(rrtConstraint* constraint);
	void									clearConstraints();

protected:
	virtual Eigen::VectorXd					extendStepSize(const Eigen::VectorXd& vertPos1, const Eigen::VectorXd& vertPos2, double criterion, TARGET_TREE tree = TARGET_TREE::TREE1);
	virtual Eigen::VectorXd					extendStepSizeSimple(const Eigen::VectorXd& vertPos1, const Eigen::VectorXd& vertPos2, double criterion, TARGET_TREE tree = TARGET_TREE::TREE1);
	// smoothing function
	vector<rrtVertex*>						getConstrainedPathConnectingTwoVertices(rrtVertex* vertex1, rrtVertex* vertex2, double eps, int maxIter = 10000);
	virtual vector<rrtVertex*>				getCandidateVertices(vector<rrtVertex*> vertices);
	virtual bool							replaceVertices(list<rrtVertex*>& path, vector<rrtVertex*>& tempVertices, vector<rrtVertex*>& removedVertex);

protected:
	double									_error_threshold;
	void									setThreshold(double threshold);

protected:
	rrtConstraint*							rrtConstraints;

protected:
	// Tangent Space
	list<tangentSpace *>					TangentSpaces;										// List of existing tangent space
	vector<Eigen::VectorXd *>				tangentBasis;										// current tangent space's basis
	list<Eigen::VectorXd *>					localCurvatures;									// List of existing tangent space curvature
	void									getTangentBasis(const Eigen::VectorXd& vertPos1);	// retrieve current node's TB basis form constraint manifold
	void									projectOntoTangentSpace(Eigen::VectorXd& jointval, const Eigen::VectorXd jointvalRef);
	bool									projectionNewtonRaphson(Eigen::VectorXd& jointval, double threshold = 0.1, int maxIter = 100);
	Eigen::MatrixXd							ProjectionMatrix;
	Eigen::VectorXd *						TBrandomSample(const Eigen::VectorXd qroot, double range);
	void									LazyProjection(vector<rrtVertex *>& path);
	unsigned int							selectTangentSpace();								// Heuristically select tangent space for extension
};

class rrtConstraint
{
public:
	rrtConstraint();
	~rrtConstraint();
	
	int								nDim;

	virtual	Eigen::VectorXd			getConstraintVector(const Eigen::VectorXd& jointVal) = 0;
	virtual	Eigen::MatrixXd			getConstraintJacobian(const Eigen::VectorXd& jointVal) = 0;
	virtual Eigen::VectorXd			getConstraintHessian(const Eigen::VectorXd& jointVal, unsigned int i, unsigned int j) = 0;
	virtual	void					project2ConstraintManifold(Eigen::VectorXd& jointVal) = 0;
};

class tangentSpace
{
public:
	tangentSpace();
	~tangentSpace();

	tangentSpace(Eigen::VectorXd Node, vector<Eigen::VectorXd *> Basis);

protected:
	vector<Eigen::VectorXd *>		tangentBasis;
	Eigen::VectorXd					rootNode;
	unsigned int					nodeNumber;

};