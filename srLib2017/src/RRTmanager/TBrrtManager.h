#pragma once
#include "rrtManager.h"

using namespace std;

class rrtConstraint;
class tangentSpace;
class TBrrtVertex;

//#define RRTExtCon

class TBrrtManager : public rrtManager
{
public:
	TBrrtManager(rrtConstraint* constraint);
	~TBrrtManager();

	// handle constraint
	void									setConstraint(rrtConstraint* constraint);
	void									clearConstraints();

	virtual void							setStartandGoal(const Eigen::VectorXd& _start, const Eigen::VectorXd& _goal);
	virtual vector<Eigen::VectorXd>			extractPath(int smoothingNum = 200);

protected:
	// RRT functions
	virtual bool							innerloop();			// TB-RRT-simple inner loop 


	// generate new vertex
	virtual rrtVertex*						generateNewVertex(rrtVertex* pos1, const Eigen::VectorXd& pos2, double step_size_collision = 0.01);	// generate TBrrtVertex inside


	// extend step size
	//virtual Eigen::VectorXd					extendStepSize(const Eigen::VectorXd& vertPos1, const Eigen::VectorXd& vertPos2, double criterion, TARGET_TREE tree = TARGET_TREE::TREE1);
	//Eigen::VectorXd							extendStepSizeSimple(TBrrtVertex* nearestVertex, const Eigen::VectorXd& vertPos2, double criterion, TARGET_TREE tree = TARGET_TREE::TREE1);
	


	// smoothing function
	vector<rrtVertex*>						getConstrainedPathConnectingTwoVertices(rrtVertex* vertex1, rrtVertex* vertex2, double eps, int maxIter = 10000);
	virtual vector<rrtVertex*>				getCandidateVertices(vector<rrtVertex*> vertices);
	virtual bool							replaceVertices(list<rrtVertex*>& path, vector<rrtVertex*>& tempVertices, vector<rrtVertex*>& removedVertex);



protected:
	// TB-RRT functions
	// tangent space
	//void									getTangentBasis(const Eigen::VectorXd& vertPos1);	// retrieve current node's TB basis form constraint manifold
	//Eigen::VectorXd *						TBrandomSample(const Eigen::VectorXd qroot, double range);
	unsigned int							selectTangentSpace();								// Heuristically select tangent space for extension

	// projection function
	bool									projectionNewtonRaphson(Eigen::VectorXd& jointval, double threshold = 0.1, int maxIter = 100);
	void									setThreshold(double threshold);
	void									LazyProjection(list<rrtVertex *>& path);

	

protected:
	rrtConstraint*							rrtConstraints;

protected:
	// Tangent Space
	vector<tangentSpace *>					TangentSpaces;										// List of existing tangent space

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
	virtual	bool					project2ConstraintManifold(Eigen::VectorXd& jointVal, int max_iter = 1000) = 0;
};

class tangentSpace
{
public:
	tangentSpace();
	~tangentSpace();

	tangentSpace(Eigen::VectorXd vertex, rrtConstraint* constraints);

	void projectOntoTangentSpace(Eigen::VectorXd& jointval);

public:
	Eigen::MatrixXd					tangentBasis;
	
	Eigen::VectorXd					rootNode;
	//Eigen::VectorXd			    localCurvature;
	Eigen::MatrixXd					ProjectionMatrix;
	unsigned int					nodeNumber;
};

class TBrrtVertex : public rrtVertex
{
public:
	TBrrtVertex();
	~TBrrtVertex();
public:
	tangentSpace*		_tangentSpace;
};