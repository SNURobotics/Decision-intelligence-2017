#pragma once
#include "../Eigen/Core"
#include <srDyn/srSpace.h>

#include <set>
#include <list>
#include <vector>
#include <utility>

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


class rrtManager
{
public:
	rrtManager();
	~rrtManager();
	enum TARGET_TREE {TREE1, TREE2};
	
	virtual void							setSystem(srSystem* _pSystem);
	virtual void							setSystem(vector<srStateJoint*> _pStateJoints);
	void									setSpace(srSpace* _pSpace) { pSpace = _pSpace; }
	virtual void							setStartandGoal(const Eigen::VectorXd& _start, const Eigen::VectorXd& _goal);
	void									setStateBound(const Eigen::VectorXd& _lowerbound, const Eigen::VectorXd& _upperbound);
	virtual bool							setState(const Eigen::VectorXd& state);
	
	bool									checkStartGoalFeasibility();
	virtual bool							isProblemFeasible();

	void									execute(double _step_size);
	bool									isExecuted();
	virtual vector<Eigen::VectorXd>			extractPath(int smoothingNum = 200);
	
	bool									collisionChecking(const Eigen::VectorXd& pos1, const Eigen::VectorXd& pos2, double step_size_collision = 0.01);
	

	// print tree
	void									printTree(TARGET_TREE tree);
	Eigen::VectorXd							getStart();
	Eigen::VectorXd							getGoal();


protected:
	
	virtual bool							innerloop();

	void									swapTree();
	virtual void							connectParentAndChild(rrtVertex* parentVertex, rrtVertex* childVertex);
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

protected: // smoothing function
	virtual vector<rrtVertex*>				getRandomVertices(list<rrtVertex*>& path);
	virtual vector<rrtVertex*>				getCandidateVertices(vector<rrtVertex*> vertices);
	virtual double							getCost(rrtVertex* pos1, rrtVertex* pos2);
	virtual double							getRRTpathSmoothingCost(rrtVertex* pos1, rrtVertex* pos2, vector<rrtVertex*>& removedVertex);
	virtual double							getNewPathSmoothingCost(vector<rrtVertex*> vertices);
	virtual bool							replaceVertices(list<rrtVertex*>& path, vector<rrtVertex*>& tempVertices, vector<rrtVertex*>& removedVertex);

protected:
	srSpace*								pSpace;
	srSystem*								pSystem;


	vector<srStateJoint*>					pState;
	Eigen::VectorXd							lowerBound;
	Eigen::VectorXd							upperBound;
	int										nDim;
	double									step_size;
	bool									_isExecuted;
protected:
	rrtTree									startTree;
	rrtTree									goalTree;
	Eigen::VectorXd							m_start;
	Eigen::VectorXd							m_goal;
protected:
	rrtTree*								pTargetTree1;
	rrtTree*								pTargetTree2;
	bool									_isTreeSwaped;
	rrtVertex*								connectedVertex1;
	rrtVertex*								connectedVertex2;

public: // print options
	bool									printIter;
	bool									printFinish;
	bool									printDist;
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