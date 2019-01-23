#pragma once
#include "rrtManager.h"

using namespace std;

class rrtConstraint;
class tangentSpace;

class TBrrtVertex : public rrtVertex
{
public:
	TBrrtVertex();
	~TBrrtVertex();
public:
	tangentSpace*		_tangentSpace;
};


class TBrrtManager : public rrtManager
{
public:
	TBrrtManager();
	~TBrrtManager();

	// handle constraint
	void									setConstraint(rrtConstraint* constraint);
	void									clearConstraints();

protected:
	// RRT functions
	bool									innerloop();			// TB-RRT-simple inner loop 


	// generate new vertex
	rrtVertex*								generateNewVertex(rrtVertex* pos1, const Eigen::VectorXd& pos2, double step_size_collision = 0.01);	// generate TBrrtVertex inside

	// extend step size
	virtual Eigen::VectorXd					extendStepSize(const Eigen::VectorXd& vertPos1, const Eigen::VectorXd& vertPos2, double criterion, TARGET_TREE tree = TARGET_TREE::TREE1);
	virtual Eigen::VectorXd					extendStepSizeSimple(const Eigen::VectorXd& vertPos1, const Eigen::VectorXd& vertPos2, double criterion, TARGET_TREE tree = TARGET_TREE::TREE1);
	// smoothing function
	vector<rrtVertex*>						getConstrainedPathConnectingTwoVertices(rrtVertex* vertex1, rrtVertex* vertex2, double eps, int maxIter = 10000);
	virtual vector<rrtVertex*>				getCandidateVertices(vector<rrtVertex*> vertices);
	virtual bool							replaceVertices(list<rrtVertex*>& path, vector<rrtVertex*>& tempVertices, vector<rrtVertex*>& removedVertex);



protected:
	// TB-RRT functions
	// tanget space
	void									getTangentBasis(const Eigen::VectorXd& vertPos1);	// retrieve current node's TB basis form constraint manifold
	void									projectOntoTangentSpace(Eigen::VectorXd& jointval, const Eigen::VectorXd jointvalRef);
	Eigen::VectorXd *						TBrandomSample(const Eigen::VectorXd qroot, double range);
	unsigned int							selectTangentSpace();								// Heuristically select tangent space for extension

	// projection function
	bool									projectionNewtonRaphson(Eigen::VectorXd& jointval, double threshold = 0.1, int maxIter = 100);
	void									setThreshold(double threshold);
	void									LazyProjection(vector<rrtVertex *>& path);

	

protected:
	rrtConstraint*							rrtConstraints;

protected:
	// Tangent Space
	list<tangentSpace *>					TangentSpaces;										// List of existing tangent space

	/////////////// move to tangent space class ////////////////////////////////
	vector<Eigen::VectorXd *>				tangentBasis;										// current tangent space's basis
	list<Eigen::VectorXd *>					localCurvatures;									// List of existing tangent space curvature
	Eigen::MatrixXd							ProjectionMatrix;
	/////////////////////////////////////////////////////////////////////////////
	// projection function
	double									_error_threshold;
	
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