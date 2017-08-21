#pragma once
#include "../Eigen/Core"
#include <srDyn/srSpace.h>

#include <set>
#include <list>
#include <vector>

using namespace std;

class rrtVertex
{
public:
	rrtVertex(){
		parentVertex = NULL;
	}
	//////////////////////////////////////////
	//~rrtVertex() {
	//	parentVertex = NULL;
	//	posState.resize(0);
	//	distance2parent = 0.0;
	//	cost_bw_parent = 0.0;
	//}
	//////////////////////////////////////////
	rrtVertex*					parentVertex;
	Eigen::VectorXd				posState;
	double						distance2parent;
	double						cost_bw_parent;
};

typedef set<rrtVertex*>		rrtTree;

class rrtConstraint;

class rrtManager
{
public:
	rrtManager();
	~rrtManager();
	enum TARGET_TREE {TREE1, TREE2};
	
	virtual void							setSystem(srSystem* _pSystem);
	virtual void							setSystem(vector<srStateJoint*> _pStateJoints);
	void									setSpace(srSpace* _pSpace) { pSpace = _pSpace; }
	void									setStartandGoal(const Eigen::VectorXd& _start, const Eigen::VectorXd& _goal);
	void									setStateBound(const Eigen::VectorXd& _lowerbound, const Eigen::VectorXd& _upperbound);
	virtual bool							setState(const Eigen::VectorXd& state);
	void									execute(double _step_size);
	vector<Eigen::VectorXd>					extractPath(int smoothingNum = 200);
	
	
	bool									collisionChecking(const Eigen::VectorXd& pos1, const Eigen::VectorXd& pos2, double step_size_collision = 0.01);


	// print tree
	void									printTree(TARGET_TREE tree);

	// constrained
	void									addConstraint(rrtConstraint* constraint);
	void									clearConstraints();

	// vector field
	Eigen::VectorXd							getVectorField(const Eigen::VectorXd& pos1);
	enum VECTOR_FIELD {TRAJFOLLOW, RIVER_2DOF};
	void									setTrajFollowVectorField(const vector<Eigen::VectorXd>& refTraj);
	void									setRiver2dofVectorField();
	Eigen::VectorXd							trajFollowVectorField(const Eigen::VectorXd& pos1, const vector<Eigen::VectorXd>& refTraj);
	double									getUpstreamCost(const Eigen::VectorXd& vertPos1, const Eigen::VectorXd& vertPos2, int n = 10);

protected:
	
	bool									innerloop();

	void									swapTree();
	int										randomInt(int LB, int UB);
	list<rrtVertex*>						smoothingPath(list<rrtVertex*>& path, int smoothingnum);
	virtual double							getDistance(const Eigen::VectorXd& vertPos1, const Eigen::VectorXd& vertPos2);
	list<Eigen::VectorXd>					fillingPath(list<rrtVertex*>& path);

protected: // innerloop function
	virtual Eigen::VectorXd					generateRandomVertex();
	double									randomDouble(double LB, double UB);
	virtual Eigen::VectorXd					extendStepSize(const Eigen::VectorXd& vertPos1, const Eigen::VectorXd& vertPos2, double criterion, TARGET_TREE tree = TARGET_TREE::TREE1);
	rrtVertex*								nearestVertex(const Eigen::VectorXd& _vertex, TARGET_TREE treeNum);
	rrtVertex*								generateNewVertex(rrtVertex* pos1, const Eigen::VectorXd& pos2, double step_size_collision = 0.01);
	virtual vector<Eigen::VectorXd>			generateIntermediateVertex(Eigen::VectorXd pos1, Eigen::VectorXd pos2, int numMidPoint);
	
protected:
	srSpace*								pSpace;
	srSystem*								pSystem;


	vector<srStateJoint*>					pState;
	Eigen::VectorXd							lowerBound;
	Eigen::VectorXd							upperBound;
	int										nDim;
	double									step_size;

protected:
	rrtTree									startTree;
	rrtTree									goalTree;

protected:
	rrtTree*								pTargetTree1;
	rrtTree*								pTargetTree2;
	bool									_isTreeSwaped;
	rrtVertex*								connectedVertex1;
	rrtVertex*								connectedVertex2;

	bool									_vectorFieldExist;
	VECTOR_FIELD							_vectorField;
	vector<Eigen::VectorXd>					_refTraj;
	rrtConstraint*							rrtConstraints;
};

class rrtConstraint
{
public:
	rrtConstraint();
	~rrtConstraint();

	int			nDim;

	virtual		Eigen::VectorXd			getConstraintVector(const Eigen::VectorXd& jointVal) = 0;
	virtual		Eigen::MatrixXd			getConstraintJacobian(const Eigen::VectorXd& jointVal) = 0;
	virtual		void					project2ConstraintManifold(Eigen::VectorXd& jointVal) = 0;
};

//// planning trajectory of object (Configuration space is SE(3))
//class ObjectrrtManager : public rrtManager
//{
//public:
//	virtual void							setSystem(srSystem* _pSystem);
//	virtual bool							setState(const Eigen::VectorXd& state);
//	void									setStartandGoalSE3(SE3 Tstart, SE3 Tgoal);
//	srSystem*								getObject() const;
//	static vector<SE3>						convertPathToSE3(list<Eigen::VectorXd>& path);
//
//private:
//	virtual Eigen::VectorXd					generateRandomVertex();
//	virtual double							getDistance(Eigen::VectorXd vertPos1, Eigen::VectorXd vertPos2);
//	virtual Eigen::VectorXd					extendStepSize(Eigen::VectorXd vertPos1, Eigen::VectorXd vertPos2, double criterion);
//	virtual vector<Eigen::VectorXd>			generateIntermediateVertex(Eigen::VectorXd pos1, Eigen::VectorXd pos2, int numMidPoint);
//	
//};