#include <time.h>
#include "rrtManager.h"
#include "common\dataIO.h"

rrtManager::rrtManager()
{
	_vectorFieldExist = false;
	rrtConstraints = NULL;
}


rrtManager::~rrtManager()
{
}

void rrtManager::setSystem(srSystem* _pSystem)
{
	for (int i = 0; i < _pSystem->m_KIN_Joints.get_size(); i++){
		srStateJoint* pJoint = dynamic_cast<srStateJoint*>(_pSystem->m_KIN_Joints[i]);
		if (pJoint != NULL) {
			pState.push_back(pJoint);
		}
	}

	nDim = pState.size();
}

void rrtManager::setSystem(vector<srStateJoint*> _pStateJoints)
{
	pState = _pStateJoints;
	nDim = pState.size();
}

bool rrtManager::setState(const Eigen::VectorXd& state)
{
	// set state
	for (int i = 0; i < nDim; i++)
		pState[i]->m_State.m_rValue[0] = state[i];

	// check collision
	return pSpace->_KIN_COLLISION_RUNTIME_SIMULATION_LOOP();

}

void rrtManager::setStartandGoal(const Eigen::VectorXd& _start, const Eigen::VectorXd& _goal)
{
	/////////////////////////////////////
	//rrtTree::iterator it;
	//for (it = startTree.begin(); it != startTree.end(); ++it)
	//	delete (*it);
	//for (it = goalTree.begin(); it != goalTree.end(); ++it)
	//	delete (*it);
	/////////////////////////////////////
	startTree.clear();
	goalTree.clear();

	rrtVertex* startVertex = new rrtVertex();
	startVertex->posState = _start;
	
	rrtVertex* goalVertex = new rrtVertex();
	goalVertex->posState = _goal;
	
	startTree.insert(startVertex);
	goalTree.insert(goalVertex);

	pTargetTree1 = &startTree;
	pTargetTree2 = &goalTree;
}


void rrtManager::execute(double _step_size)
{
	step_size = _step_size;
	_isTreeSwaped = false;
	// seed
	//srand((unsigned)time(NULL));

	int iter = 0;
	bool bConnected = false;

	// print options
	bool printIter = true;
	bool printFinish = true;
	///////////////////////////
	while (!bConnected)
	{
		bConnected = innerloop();
		swapTree();
		
		if (printIter == true)
			cout << "iteration # = " << iter++ << endl;
	}
	
	if (printFinish == true)
		cout << "end of RRT " << endl;

	//printTree(TARGET_TREE::TREE1);
	//printTree(TARGET_TREE::TREE2);
}

bool rrtManager::innerloop()
{
	const double eps = 1e-16;
	/* -------------------------------------------------------------  TREE 1 -------------------------------------------------------------*/
	rrtVertex* new_vertex = NULL;
	do{
		// 1. random node generation
		Eigen::VectorXd random_vertex_pos = generateRandomVertex();

		// 2. finding nearest vertex
		rrtVertex* nearest_vertex = nearestVertex(random_vertex_pos, TREE1);
		Eigen::VectorXd nearest_vertex_pos = nearest_vertex->posState;

		// 3. extend step size to random node (vector field is considered here)
		Eigen::VectorXd temp_vertex_pos = extendStepSize(nearest_vertex_pos, random_vertex_pos, eps, TREE1);
		
		// 4. (optional) constraint projection
		if (rrtConstraints != NULL)
			rrtConstraints->project2ConstraintManifold(temp_vertex_pos);

		// 5. collision checking 
		new_vertex = generateNewVertex(nearest_vertex, temp_vertex_pos, step_size*0.1);
	} while (new_vertex == NULL);
	
	pTargetTree1->insert(new_vertex);
	
	/* -------------------------------------------------------------  TREE 2 -------------------------------------------------------------*/

	rrtVertex* nearest_tree2_vertex = nearestVertex(new_vertex->posState, TREE2);


	//Eigen::VectorXd dir_2_tree1 = (new_vertex->posState - nearest_tree2_vertex->posState);
	//double distance_2_tree1 = dir_2_tree1.norm();

	Eigen::VectorXd temp_tree2_vertex_pos = extendStepSize(nearest_tree2_vertex->posState, new_vertex->posState, step_size, TREE2);
	// (optional) constraint projection
	if (rrtConstraints != NULL)
		rrtConstraints->project2ConstraintManifold(temp_tree2_vertex_pos);
	rrtVertex* new_tree2_vertex = NULL;

	// collision checking
	new_tree2_vertex = generateNewVertex(nearest_tree2_vertex, temp_tree2_vertex_pos, step_size*0.1);
	if (new_tree2_vertex != NULL)
		pTargetTree2->insert(new_tree2_vertex);

	/* -------------------------------------------------------------  CHECK CONNECTIVITY -------------------------------------------------------------*/
	bool printDist = false;
	if (new_tree2_vertex == NULL)
		return false;

	else 
	{
		double dist = getDistance(new_tree2_vertex->posState, new_vertex->posState);
		if (printDist == true)
			cout << "distance b/w trees: " << dist << endl;
		if (dist < step_size && !collisionChecking(new_vertex->posState, new_tree2_vertex->posState, step_size*0.1))
		{
			rrtVertex* new_tree1_last_vertex = new rrtVertex;
			new_tree1_last_vertex->posState = new_tree2_vertex->posState;
			new_tree1_last_vertex->parentVertex = new_vertex;
			pTargetTree1->insert(new_tree1_last_vertex);

			connectedVertex1 = new_tree1_last_vertex;
			connectedVertex2 = new_tree2_vertex;
			return true;
		}
		else
			return false;
	}


}

Eigen::VectorXd rrtManager::extendStepSize(const Eigen::VectorXd& vertPos1, const Eigen::VectorXd& vertPos2, double criterion, TARGET_TREE tree /* = TARGET_TREE::TREE1*/)
{
	Eigen::VectorXd dir_2_random = (vertPos2 - vertPos1);
	Eigen::VectorXd temp_vertex_pos;
	if (!_vectorFieldExist)
	{
		if (dir_2_random.norm() < criterion) 
			temp_vertex_pos = vertPos2;
		else 
		{
			dir_2_random.normalize();
			temp_vertex_pos = vertPos1 + step_size* dir_2_random;
		}
	}
	else
	{
		double weight = 0.7;
		Eigen::VectorXd vec = getVectorField(vertPos1);
		if (dir_2_random.norm() > 0)
			dir_2_random.normalize();
		if (vec.norm() > 0)
			vec.normalize();
		bool plusVec;
		if (_vectorField == VECTOR_FIELD::TRAJFOLLOW)
			plusVec = true;
		else
		{
			if (_isTreeSwaped)
			{
				if (tree == TARGET_TREE::TREE1)
					plusVec = false;
				else
					plusVec = true;
			}
			else
			{
				if (tree == TARGET_TREE::TREE1)
					plusVec = true;
				else
					plusVec = false;
			}
		}
		if (plusVec)
			vec = dir_2_random + weight * vec;
		else
			vec = dir_2_random - weight * vec;
		if (vec.norm() > 0)
			vec.normalize();
		temp_vertex_pos = vertPos1 + step_size * vec;
	}
	return temp_vertex_pos;
}

Eigen::VectorXd rrtManager::generateRandomVertex()
{
	Eigen::VectorXd randTemp(nDim);
	for (int i = 0; i < nDim; i++)
		randTemp(i) = randomDouble(lowerBound[i], upperBound[i]);

	return randTemp;
}
	

void rrtManager::setStateBound(const Eigen::VectorXd& _lowerbound, const Eigen::VectorXd& _upperbound)
{
	lowerBound = _lowerbound;
	upperBound = _upperbound;
}

double rrtManager::randomDouble(double LB, double UB)
{
	double randNum = (double)rand() / RAND_MAX;
	return (randNum*(UB - LB) + LB);
}

void rrtManager::swapTree()
{
	rrtTree* tempTree = pTargetTree1;
	pTargetTree1 = pTargetTree2;
	pTargetTree2 = tempTree;
	_isTreeSwaped = !_isTreeSwaped;
}


double rrtManager::getDistance(const Eigen::VectorXd& vertPos1, const Eigen::VectorXd& vertPos2)
{
	return (vertPos1 - vertPos2).norm();
}

rrtVertex* rrtManager::nearestVertex(const Eigen::VectorXd& _vertex, TARGET_TREE treeNum)
{

	rrtVertex*   minVertex = NULL;

	rrtTree* targetTree;
	if (treeNum == TREE1)
		targetTree = pTargetTree1;
	else
		targetTree = pTargetTree2;

	double minDist = 1e16;
	double dist;
	rrtTree::iterator iter = targetTree->begin();
	for (iter; iter != targetTree->end(); iter++){
		dist = getDistance((*iter)->posState, _vertex);
		if (dist < minDist){
			minVertex = (*iter);
			minDist = dist;
		}
	}

	return minVertex;
}

vector<Eigen::VectorXd> rrtManager::generateIntermediateVertex(Eigen::VectorXd pos1, Eigen::VectorXd pos2, int numMidPoint)
{
	vector<Eigen::VectorXd> checkSet(numMidPoint);
	for (int i = 0; i < numMidPoint; i++){
		checkSet[i] = (double)(i + 1) / numMidPoint * (pos2 - pos1) + pos1;
	}
	return checkSet;
}


rrtVertex* rrtManager::generateNewVertex(rrtVertex* nearest_vertex, const Eigen::VectorXd& pos2, double step_size_collision /*= 0.01*/)
{
	rrtVertex* new_vertex = new rrtVertex();
	
	Eigen::VectorXd pos1 = nearest_vertex->posState;
	double length = getDistance(pos1, pos2);

	int numMidPoint = (int)floor(length / step_size_collision) + 1;

	vector<Eigen::VectorXd> checkSet;
	checkSet = generateIntermediateVertex(pos1, pos2, numMidPoint);

	int indexCollision = -1;
	for (int i = 0; i < numMidPoint; i++){
		bool bCollision = setState(checkSet[i]);
		if (bCollision){
			indexCollision = i;
			break;
		}
	}

	if (indexCollision == 0)
		return NULL;
	else if (indexCollision > 0)
		new_vertex->posState = checkSet[indexCollision - 1];
	else
		new_vertex->posState = pos2;

	new_vertex->parentVertex = nearest_vertex;
	//new_vertex->distance2parent = getDistance(new_vertex->parentVertex->posState, new_vertex->posState);
	//if (_vectorFieldExist)
	//	new_vertex->cost2parent = getUpstreamCost(new_vertex->parentVertex->posState, new_vertex->posState);
	//else
	//	new_vertex->cost2parent = 0.0;
	return new_vertex;
}

bool rrtManager::collisionChecking(const Eigen::VectorXd& pos1, const Eigen::VectorXd& pos2, double step_size_collision /*= 0.01*/)
{
	Eigen::VectorXd dir = pos2 - pos1;
	double length = getDistance(pos1, pos2);

	int numMidPoint = (int) floor(length / step_size_collision) + 1;

	vector<Eigen::VectorXd> checkSet(numMidPoint);
	checkSet = generateIntermediateVertex(pos1, pos2, numMidPoint);

	int indexCollision = -1;
	for (int i = 0; i < numMidPoint; i++){
		bool bCollision = setState(checkSet[i]);
		if (bCollision){
			indexCollision = i;
			break;
		}
	}
	if (indexCollision < 0)
		return false;
	else
		return true;

}

vector<Eigen::VectorXd> rrtManager::extractPath(int smoothingNum /*= 200*/)
{
	list<rrtVertex*> path;

	rrtVertex*  pCurrent;
	if (startTree.find(connectedVertex1) == startTree.end())
	{
		rrtVertex* temp = connectedVertex1;
		connectedVertex1 = connectedVertex2;
		connectedVertex2 = temp;
	}
	pCurrent = connectedVertex1;
	path.push_front(pCurrent);
	pCurrent = pCurrent->parentVertex;
	while (pCurrent != NULL)
	{
		path.push_front(pCurrent);
		pCurrent = pCurrent->parentVertex;
	}
	pCurrent = connectedVertex2->parentVertex;
	while (pCurrent != NULL)
	{
		path.push_back(pCurrent);
		pCurrent = pCurrent->parentVertex;
	}

	list<rrtVertex*>::reverse_iterator iter = path.rbegin();
	list<rrtVertex*>::reverse_iterator iter_end = path.rend(); iter_end--;
	list<rrtVertex*>::reverse_iterator iter2;

	for (iter; iter != iter_end; iter++)
	{
		advance(iter, 1);
		iter2 = iter;
		advance(iter, -1);

		(*iter)->parentVertex = (*iter2);
		(*iter)->distance2parent = getDistance((*iter2)->posState, (*iter)->posState);
		if (_vectorFieldExist)
			(*iter)->cost_bw_parent = getUpstreamCost((*iter2)->posState, (*iter)->posState);
		else
			(*iter)->cost_bw_parent = 0.0;
	}

	path = smoothingPath(path, smoothingNum);
	list<Eigen::VectorXd> filledPath = fillingPath(path);
	list<Eigen::VectorXd>::iterator iter_path = filledPath.begin();
	vector<Eigen::VectorXd> outputPath(filledPath.size());
	for (unsigned int i = 0; i < outputPath.size(); i++, iter_path++)
	{
		outputPath[i] = *iter_path;
	}
	return outputPath;
}

list<rrtVertex*> rrtManager::smoothingPath(list<rrtVertex*>& path, int smoothingnum)
{
	list<rrtVertex*> tmpPath;
	for (int iterSmoothing = 0; iterSmoothing < smoothingnum; iterSmoothing++){
		int n = path.size();

		int idx1, idx2;
		while (1)
		{
			idx1 = randomInt(0, n - 1);
			idx2 = randomInt(0, n - 1);
			if (idx1 != idx2)
				break;
		}

		if (idx1 > idx2)
		{
			int temp = idx1;
			idx1 = idx2;
			idx2 = temp;
		}

		list<rrtVertex*>::iterator iter = path.begin();
		advance(iter, idx1);
		rrtVertex* vertex1 = *iter;

		iter = path.begin();
		advance(iter, idx2);
		rrtVertex* vertex2 = *iter;
		double dist_rrtpath = 0;
		double cost_rrtpath = 0;
		rrtVertex* currentVertex = vertex2;
		vector<rrtVertex*>		removedVertex;
		while (currentVertex != vertex1)
		{
			dist_rrtpath += currentVertex->distance2parent;
			cost_rrtpath += currentVertex->cost_bw_parent;
			currentVertex = currentVertex->parentVertex;
			removedVertex.push_back(currentVertex);
		}
		
		double cmp;
		double cmp_rrtpath;
		/////////////////////////////////////////////////////////////
		if (rrtConstraints == NULL)
		{
			//////////////////////////////////////// without constraint
			double dist = getDistance(vertex1->posState, vertex2->posState);
			double cost;
			
			if (!_vectorFieldExist)
			{
				cost = 0.0;
				cmp = dist;
				cmp_rrtpath = dist_rrtpath;
			}
			else
			{
				cost = getUpstreamCost(vertex1->posState, vertex2->posState);
				cmp = cost;
				cmp_rrtpath = cost_rrtpath;
			}

			if (cmp < cmp_rrtpath && !collisionChecking(vertex1->posState, vertex2->posState, step_size*0.1))
			{
				vertex2->parentVertex = vertex1;
				vertex2->distance2parent = dist;
				vertex2->cost_bw_parent = cost;
				for (unsigned int i = 0; i < removedVertex.size() - 1; i++)
					path.remove(removedVertex[i]);
			}
		}
		else
		{
			//////////////////////////////////////// with constraint
			vector<rrtVertex*> extendedVertex;
			rrtVertex* firstExtenedVertex = new rrtVertex;
			firstExtenedVertex->posState = vertex1->posState;
			extendedVertex.push_back(firstExtenedVertex);
			double eps = 0.5*step_size;
			while (true)
			{
				Eigen::VectorXd diff = vertex2->posState - extendedVertex[extendedVertex.size() - 1]->posState;
				double diffnorm = diff.norm();
				if (diffnorm < eps && !collisionChecking(vertex2->posState, extendedVertex[extendedVertex.size() - 1]->posState, step_size*0.1))
					break;
				if (extendedVertex.size() > 2 && diffnorm > (vertex2->posState - extendedVertex[extendedVertex.size() - 2]->posState).norm())
				{
					// if distance b/w neighboring vertices is bigger than distance b/w vertices apart, end the extension
					extendedVertex.resize(extendedVertex.size() - 1);
					break;
				}
				Eigen::VectorXd temp_vertex = extendedVertex[extendedVertex.size() - 1]->posState + min(step_size, diffnorm)*diff;
				rrtConstraints->project2ConstraintManifold(temp_vertex);
				if (!collisionChecking(temp_vertex, extendedVertex[extendedVertex.size() - 1]->posState, step_size*0.1))
				{
					rrtVertex* tempVertex = new rrtVertex;
					tempVertex->posState = temp_vertex;
					tempVertex->parentVertex = extendedVertex[extendedVertex.size() - 1];
					tempVertex->distance2parent = (extendedVertex[extendedVertex.size() - 1]->posState - temp_vertex).norm();
					if (!_vectorFieldExist)
						tempVertex->cost_bw_parent = 0.0;
					else
						tempVertex->cost_bw_parent = getUpstreamCost(extendedVertex[extendedVertex.size() - 1]->posState, temp_vertex);
					extendedVertex.push_back(tempVertex);
				}
				else
					break;
			}
			double dist_cur = (vertex2->posState - extendedVertex[extendedVertex.size() - 1]->posState).norm();
			if (dist_cur < eps)
			{
				double dist = 0;
				double cost = 0;
				double cost_cur = 0;
				if (_vectorFieldExist)
					cost_cur = getUpstreamCost(extendedVertex[extendedVertex.size() - 1]->posState, vertex2->posState);
				for (unsigned int i = 1; i < extendedVertex.size(); i++)
				{
					dist += extendedVertex[i]->distance2parent;
					cost += extendedVertex[i]->cost_bw_parent;
				}
				dist += dist_cur;
				cost += cost_cur;
				if (!_vectorFieldExist)
				{
					cmp = dist;
					cmp_rrtpath = dist_rrtpath;
				}
				else
				{
					cmp = cost;
					cmp_rrtpath = cost_rrtpath;
				}
				if (cmp < cmp_rrtpath)
				{
					if (extendedVertex.size() == 1)
					{
						vertex2->parentVertex = vertex1;
					}
					else
					{
						vertex2->parentVertex = extendedVertex[extendedVertex.size() - 1];
						extendedVertex[1]->parentVertex = vertex1;
					}
					
					vertex2->distance2parent = dist_cur;
					vertex2->cost_bw_parent = cost_cur;
					
					for (unsigned int i = 0; i < removedVertex.size() - 1; i++)
						path.remove(removedVertex[i]);
					// add new vertices
					n = path.size();
					tmpPath.resize(0);
					list<rrtVertex*>::reverse_iterator tmpriter = tmpPath.rbegin();
					list<rrtVertex*>::reverse_iterator riter = path.rbegin();
					tmpPath.push_back(*riter);
					
					for (unsigned int i = 1; i < n + extendedVertex.size() - 1; i++, tmpriter++)
						tmpPath.push_front((*tmpriter)->parentVertex);
					path = tmpPath;
					delete firstExtenedVertex;
				}
				else
				{
					// delete extended vertices
					for (unsigned int i = 0; i < extendedVertex.size(); i++)
						delete extendedVertex[i];
				}
			}
		}
		/////////////////////////////////////////////////////////////
	}
	
	return path;

}

int rrtManager::randomInt(int LB, int UB)
{
	return rand() % (UB - LB + 1) + LB;
}

list<Eigen::VectorXd> rrtManager::fillingPath(list<rrtVertex*>& path)
{
	list<Eigen::VectorXd>			filledPath(0);

	list<rrtVertex*>::iterator iter = path.begin(); iter++;
	list<rrtVertex*>::iterator end_iter = path.end(); 
	filledPath.push_back((*iter)->parentVertex->posState);
	for (iter; iter != end_iter; iter++){
		int numMidPoint = (int) ceil((*iter)->distance2parent / step_size);
		//Eigen::VectorXd diff = (*iter)->posState - (*iter)->parentVertex->posState ;
		vector<Eigen::VectorXd>	midPoints = generateIntermediateVertex((*iter)->parentVertex->posState, (*iter)->posState, numMidPoint);
		for (int j = 0; j < numMidPoint; j++){
			filledPath.push_back(midPoints[j]);
			//filledPath.push_back((*iter)->parentVertex->posState + (double) j / (double)numMidPoint*diff);
		}
	}
	//end_iter--;
	//filledPath.push_back((*end_iter)->posState);

	return filledPath;
	
}

void rrtManager::setTrajFollowVectorField(const vector<Eigen::VectorXd>& refTraj)
{
	bool isFeasible = true;
	for (unsigned int i = 0; i < refTraj.size(); i++)
	{
		if (refTraj[i].size() != nDim)
		{
			isFeasible = false;
		}
		if (!isFeasible)
			break;
	}
	if (isFeasible)
	{
		_vectorField = VECTOR_FIELD::TRAJFOLLOW;
		_refTraj = refTraj;
		_vectorFieldExist = true;
	}
	else
		printf("check reference trajectory !!!\n");
}

void rrtManager::setRiver2dofVectorField()
{
	if (nDim == 2)
	{
		_vectorField = VECTOR_FIELD::RIVER_2DOF;
		_vectorFieldExist = true;
	}
	else
		_vectorFieldExist = false;
}

void rrtManager::printTree(TARGET_TREE tree)
{
	vector<Eigen::VectorXd> treeData;
	set<rrtVertex*>::iterator iter;
	string fileName;
	if (tree == TARGET_TREE::TREE1)
	{
		treeData.resize(pTargetTree1->size());
		iter = pTargetTree1->begin();
		fileName = "rrttest_tree1.txt";
	}
	else
	{
		treeData.resize(pTargetTree2->size());
		iter = pTargetTree2->begin();
		fileName = "rrttest_tree2.txt";
	}
	
	for (unsigned int i = 0; i < treeData.size(); i++, iter++)
	{
		treeData[i].resize(2*nDim);
		treeData[i].setZero();
		treeData[i].head(nDim) = (*iter)->posState;
		if ((*iter)->parentVertex != NULL)
			treeData[i].tail(nDim) = (*iter)->parentVertex->posState;
	}
	saveDataToText(treeData, fileName);
}

Eigen::VectorXd rrtManager::getVectorField(const Eigen::VectorXd & pos1)
{
	if (_vectorField == VECTOR_FIELD::TRAJFOLLOW)
		return trajFollowVectorField(pos1, _refTraj);
	else if (_vectorField == VECTOR_FIELD::RIVER_2DOF)
	{
		Eigen::VectorXd vec = Eigen::VectorXd::Zero(2);
		if (pos1(1) < 0.6*(upperBound(1) - lowerBound(1)) + lowerBound(1) && pos1(1) > 0.4*(upperBound(1) - lowerBound(1)) + lowerBound(1))
		{
			vec(0) = 1.0;
		}
		else
		{
			vec(0) = 0.2;
		}
		return vec;
	}
	else
		return Eigen::VectorXd::Zero(pos1.size());
}

Eigen::VectorXd rrtManager::trajFollowVectorField(const Eigen::VectorXd & pos1, const vector<Eigen::VectorXd>& refTraj)
{
	Eigen::VectorXd vec(nDim);
	double dist;
	unsigned int minIdx = 0;
	double minDist;
	for (unsigned int i = 0; i < refTraj.size(); i++)
	{
		dist = (pos1 - refTraj[i]).norm();
		if (i == 0)
			minDist = dist;
		else
		{
			if (dist < minDist)
			{
				minDist = dist;
				minIdx = i;
			}
		}
	}
	vec = refTraj[minIdx] - pos1;
	//vec.normalize();
	return vec;
}

double rrtManager::getUpstreamCost(const Eigen::VectorXd & vertPos1, const Eigen::VectorXd & vertPos2, int n /* = 10*/)
{
	Eigen::VectorXd dir = vertPos2 - vertPos1;
	double dist = dir.norm() / (double) n;
	dir.normalize();
	Eigen::VectorXd tmpVec;
	double cost = 0.0;
	for (int i = 0; i < n + 1; i++)
	{
		tmpVec = vertPos1 + (double)i / (double)n * (vertPos2 - vertPos1);
		tmpVec = getVectorField(tmpVec);

		if (i == 0 || i == n)
			cost += 0.5*(tmpVec.norm() - tmpVec.transpose()*dir)*dist;
		else
			cost += (tmpVec.norm() - tmpVec.transpose()*dir)*dist;
	}
	return cost;
}

void rrtManager::addConstraint(rrtConstraint* constraint)
{
	rrtConstraints = constraint;
}

void rrtManager::clearConstraints()
{
	rrtConstraints = NULL;
}


rrtConstraint::rrtConstraint()
{
}


rrtConstraint::~rrtConstraint()
{
}


//void ObjectrrtManager::setSystem(srSystem* _pSystem)
//{
//	pSystem = _pSystem;
//	nDim = 3;		// number of position only (orientation doesn't have lower, upper bound)
//}
//
//bool ObjectrrtManager::setState(const Eigen::VectorXd& state)
//{
//	SE3 T = VectortoSE3(state);
//	pSystem->GetBaseLink()->SetFrame(T);
//	// check collision
//	return pSpace->_KIN_COLLISION_RUNTIME_SIMULATION_LOOP();
//}
//
//void ObjectrrtManager::setStartandGoalSE3(SE3 Tstart, SE3 Tgoal)
//{
//	Eigen::VectorXd _start = SE3toVector(Tstart);
//	Eigen::VectorXd _goal = SE3toVector(Tgoal);
//	setStartandGoal(_start, _goal);
//}
//
//Eigen::VectorXd ObjectrrtManager::generateRandomVertex()
//{
//	// random sampling SE(3)
//	double u1, u2, u3;
//	u1 = randomDouble(0.0, 1.0);
//	u2 = randomDouble(0.0, 1.0);
//	u3 = randomDouble(0.0, 1.0);
//
//	Vec3 p;
//	for (int i = 0; i < nDim; i++)
//		p[i] = randomDouble(lowerBound[i], upperBound[i]);
//
//	double q0, q1, q2, q3;
//	q0 = sqrt(1 - u1)*sin(2 * SR_PI*u2);
//	q1 = sqrt(1 - u1)*cos(2 * SR_PI*u2);
//	q2 = sqrt(u1)*sin(2 * SR_PI*u3);
//	q3 = sqrt(u1)*cos(2 * SR_PI*u3);
//	double theta = 2 * acos(q0);
//
//	Vec3 r;
//	if (theta < 1e-15){
//		r[0] = 0.0;
//		r[1] = 0.0;
//		r[2] = 0.0;
//	}
//	else{
//		r[0] = q1 / sin(theta * 0.5) * theta;
//		r[1] = q2 / sin(theta * 0.5) * theta;
//		r[2] = q3 / sin(theta * 0.5) * theta;
//	}
//	Eigen::VectorXd vec = concatenateVec3(r, p);
//	return vec;
//}
//
//double ObjectrrtManager::getDistance(Eigen::VectorXd vertPos1, Eigen::VectorXd vertPos2)
//{
//	SE3 T1 = VectortoSE3(vertPos1);
//	SE3 T2 = VectortoSE3(vertPos2);
//	SE3 Tdiff = T1%T2;
//	Eigen::VectorXd diff = SE3toVector(Tdiff);
//	Eigen::MatrixXd M(6, 6);
//	M.setZero();
//	double ori = 1.0;
//	double pos = 10.0;
//	for (int i = 0; i < 3; i++){
//		M(i, i) = ori;
//		M(i + 3, i + 3) = pos;
//	}
//	return sqrt(diff.transpose()*M*diff);
//}
//
//Eigen::VectorXd ObjectrrtManager::extendStepSize(Eigen::VectorXd vertPos1, Eigen::VectorXd vertPos2, double criterion)
//{
//	Eigen::VectorXd temp_vertex_pos;
//	if (getDistance(vertPos1, vertPos2) < criterion){
//		temp_vertex_pos = vertPos2;
//	}
//	else{
//		SE3 T1 = VectortoSE3(vertPos1);
//		SE3 T2 = VectortoSE3(vertPos2);
//		SE3 Tdiff = T1%T2;
//
//		Eigen::Vector3d angDisp = Vec3toVector(Log(Tdiff.GetOrientation()));
//		angDisp.normalize();
//		Eigen::Vector3d disp = Vec3toVector(Tdiff.GetPosition());
//		disp.normalize();
//		SO3 Rextend = Exp(VectortoVec3(angDisp*step_size));
//		Vec3 pextend = VectortoVec3(disp*step_size);
//		SE3 Textend(Rextend, pextend);
//		SE3 Ttemp = T1*Textend;
//		temp_vertex_pos = SE3toVector(Ttemp);
//	}
//	return temp_vertex_pos;
//}
//
//vector<Eigen::VectorXd> ObjectrrtManager::generateIntermediateVertex(Eigen::VectorXd pos1, Eigen::VectorXd pos2, int numMidPoint)
//{
//	SE3 T1 = VectortoSE3(pos1);
//	SE3 T2 = VectortoSE3(pos2);
//	SE3 Tdiff = T1%T2;
//	Eigen::Vector3d angDisp = Vec3toVector(Log(Tdiff.GetOrientation()));
//	Eigen::Vector3d disp = Vec3toVector(Tdiff.GetPosition());
//	vector<Eigen::VectorXd> checkSet(numMidPoint);
//	for (int i = 0; i < numMidPoint; i++){
//		SO3 Rextend = Exp(VectortoVec3(angDisp * (double)(i + 1) / numMidPoint));
//		Vec3 pextend = VectortoVec3(disp * (double)(i + 1) / numMidPoint);
//		SE3 Textend(Rextend, pextend);
//		SE3 Ttemp = T1*Textend;
//		checkSet[i] = SE3toVector(Ttemp);
//	}
//	return checkSet;
//}
//
//srSystem* ObjectrrtManager::getObject() const
//{
//	return pSystem;
//}
//
//vector<SE3> ObjectrrtManager::convertPathToSE3(list<Eigen::VectorXd>& path)
//{
//	list<Eigen::VectorXd>::iterator iter = path.begin();
//	list<Eigen::VectorXd>::iterator iter_end = path.end();
//	vector<SE3> SE3path(0);
//
//	for (iter; iter != iter_end; iter++)
//		SE3path.push_back(VectortoSE3(*iter));
//
//	return SE3path;
//}

