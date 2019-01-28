#include "TBrrtManager.h"
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

TBrrtManager::TBrrtManager(rrtConstraint* constraint) : _error_threshold(0.1), _max_smoothing_range(15)
{	
	rrtConstraints = constraint;
}


TBrrtManager::~TBrrtManager()
{
}



rrtVertex * TBrrtManager::generateNewVertex(rrtVertex * nearest_vertex, const Eigen::VectorXd & pos2, double step_size_collision)
{
	rrtVertex* new_vertex = new TBrrtVertex();

	//////////////////// project onto tangent space //////////////////
	Eigen::VectorXd pos1 = nearest_vertex->posState;
	Eigen::VectorXd temp_vertex_pos = pos2;
	((TBrrtVertex*)nearest_vertex)->_tangentSpace->projectOntoTangentSpace(temp_vertex_pos);

	double length = getDistance(pos1, temp_vertex_pos);


	/////////////////// check collision ///////////////////////////////
	int numMidPoint = (int)floor(length / step_size_collision) + 1;

	vector<Eigen::VectorXd> checkSet;
	checkSet = generateIntermediateVertex(pos1, temp_vertex_pos, numMidPoint);

	int indexCollision = -1;
	for (int i = 0; i < numMidPoint; i++) {
		bool bCollision = setState(checkSet[i]);
		if (bCollision) {
			indexCollision = i;
			break;
		}
	}

	if (indexCollision == 0)
		return NULL;
	else if (indexCollision > 0)
		new_vertex->posState = checkSet[indexCollision - 1];
	else
		new_vertex->posState = temp_vertex_pos;

	new_vertex->parentVertex = nearest_vertex;

	/////////////////////////// check distance to constraint manifold ////////////////////
	if (rrtConstraints->getConstraintVector(new_vertex->posState).norm() > _error_threshold)
	{
		if (projectionNewtonRaphson(new_vertex->posState))
		{
			if (setState(new_vertex->posState)) { return NULL; }
			// Create new Tangent Space and Add to TS list
			tangentSpace * TS = new tangentSpace(new_vertex->posState, rrtConstraints);
			TangentSpaces.push_back(TS);
			((TBrrtVertex*)new_vertex)->_tangentSpace = TS;
		}
		else
			return NULL;
	}
	else
		((TBrrtVertex*)new_vertex)->_tangentSpace = ((TBrrtVertex*)nearest_vertex)->_tangentSpace;
	//new_vertex->distance2parent = getDistance(new_vertex->parentVertex->posState, new_vertex->posState);
	//if (_vectorFieldExist)
	//	new_vertex->cost2parent = getUpstreamCost(new_vertex->parentVertex->posState, new_vertex->posState);
	//else
	//	new_vertex->cost2parent = 0.0;
	return new_vertex;
}

// Currently not using
//Eigen::VectorXd TBrrtManager::extendStepSize(const Eigen::VectorXd& vertPos1, const Eigen::VectorXd& vertPos2, double criterion, TARGET_TREE tree /* = TARGET_TREE::TREE1*/)
//{
//	Eigen::VectorXd dir_2_random = (vertPos2 - vertPos1);
//	Eigen::VectorXd temp_vertex_pos;
//	Eigen::VectorXd temp_vertex_pos_old;
//
//	#ifdef RRTExtCon
//		// extend one step
//		if (dir_2_random.norm() < criterion)
//			temp_vertex_pos = vertPos2;
//		else
//		{
//			dir_2_random.normalize();
//			temp_vertex_pos = vertPos1 + step_size * dir_2_random;
//		}
//	
//		if (rrtConstraints != NULL)
//		{
//			if (collisionChecking(vertPos1, temp_vertex_pos, step_size*0.1))
//				return Eigen::VectorXd();
//			if (rrtConstraints->getConstraintVector(temp_vertex_pos).norm() > _error_threshold)
//			{
//				if (projectionNewtonRaphson(temp_vertex_pos))
//				{
//					// Create new Tangent Space and Add to TS list
//					tangentSpace * TS = new tangentSpace(temp_vertex_pos, rrtConstraints);
//					TangentSpaces.push_back(TS);
//				}
//				else
//					return Eigen::VectorXd();
//			}
//		}
//		// Add vertex to tree
//		rrtVertex* new_vertex = NULL;
//		rrtVertex* nearest_vertex = nearestVertex(temp_vertex_pos, tree);
//		new_vertex = generateNewVertex(nearest_vertex, temp_vertex_pos, step_size*0.1);
//		pTargetTree1->insert(new_vertex);
//		return temp_vertex_pos;
//	#endif // RRTExtCon
//	
//	#ifdef RRTConCon
//		Eigen::VectorXd qr;
//		temp_vertex_pos_old = vertPos1; // qold
//		while (temp_vertex_pos != vertPos2)
//		{
//			// extend one step
//			if (dir_2_random.norm() < criterion)
//				qr = vertPos2;
//			else
//			{
//				dir_2_random.normalize();
//				qr = vertPos1 + step_size * dir_2_random;
//			}
//
//			if (rrtConstraints != NULL)
//			{
//				if (collisionChecking(vertPos1, qr, step_size*0.1))
//					return temp_vertex_pos;
//
//				if (rrtConstraints->getConstraintVector(qr).norm() > _error_threshold)
//				{
//					if (projectionNewtonRaphson(qr))
//					{
//						// Add vertex to tree
//						rrtVertex* new_vertex = NULL;
//						rrtVertex* nearest_vertex = nearestVertex(qr, tree);
//						new_vertex = generateNewVertex(nearest_vertex, qr, step_size*0.1);
//						pTargetTree1->insert(new_vertex);
//						// Create new Tangent Space and Add to TS list
//						tangentSpace * TS = new tangentSpace(qr, rrtConstraints);
//						TangentSpaces.push_back(TS);
//						return qr;
//					}
//					else
//						return temp_vertex_pos;
//				}
//				temp_vertex_pos = qr;
//				// Add vertex to tree
//				rrtVertex* new_vertex = NULL;
//				rrtVertex* nearest_vertex = nearestVertex(temp_vertex_pos, tree);
//				new_vertex = generateNewVertex(nearest_vertex, temp_vertex_pos, step_size*0.1);
//				pTargetTree1->insert(new_vertex);
//				temp_vertex_pos_old = temp_vertex_pos;
//			}
//			else
//				return qr;
//		} // end of while
//	#endif // RRTConCon	
//}

//Eigen::VectorXd TBrrtManager::extendStepSizeSimple(TBrrtVertex* nearVertex, const Eigen::VectorXd& vertPos2, double criterion, TARGET_TREE tree /* = TARGET_TREE::TREE1*/)
//{
//	Eigen::VectorXd vertPos1 = nearVertex->posState;
//	Eigen::VectorXd dir_2_random = (vertPos2 - vertPos1);
//	Eigen::VectorXd temp_vertex_pos;
//	Eigen::VectorXd temp_vertex_pos_old;
//	bool deviationFlag = 0;	// flag for excessive deviation from constraint manifold
//
//#ifdef RRTExtCon
//	// extend one step
//	if (dir_2_random.norm() < criterion)
//		temp_vertex_pos = vertPos2;
//	else
//	{
//		dir_2_random.normalize();
//		temp_vertex_pos = vertPos1 + step_size * dir_2_random;
//	}
//	nearVertex->_tangentSpace->projectOntoTangentSpace(temp_vertex_pos);
//
//	if (rrtConstraints != NULL)
//	{
//		if (collisionChecking(vertPos1, temp_vertex_pos, step_size*0.1))
//			return Eigen::VectorXd();
//		if (rrtConstraints->getConstraintVector(temp_vertex_pos).norm() > _error_threshold)
//		{
//			if (projectionNewtonRaphson(temp_vertex_pos))
//			{
//				deviationFlag = 1;
//				// Create new Tangent Space and Add to TS list
//				tangentSpace * TS = new tangentSpace(temp_vertex_pos, rrtConstraints);
//				TangentSpaces.push_back(TS);
//
//				return temp_vertex_pos;
//			}
//			else
//				return Eigen::VectorXd();
//		}
//	}
//	// Add new TBrrtVertex to tree1
//	rrtVertex* new_vertex = NULL;
//	new_vertex = generateNewVertex(nearVertex, temp_vertex_pos, step_size*0.1);
//	if (deviationFlag == 0)
//		((TBrrtVertex*) new_vertex)->_tangentSpace = nearVertex->_tangentSpace;
//	else
//		((TBrrtVertex*)new_vertex)->_tangentSpace = TangentSpaces.back();
//	pTargetTree1->insert(new_vertex);
//	return temp_vertex_pos;
//#endif // RRTExtCon
//
//#ifdef RRTConCon
//	Eigen::VectorXd qr;
//	temp_vertex_pos_old = vertPos1; // qold
//	tangentSpace* old_TS_ptr = nearVertex->_tangentSpace;
//	while (1)
//	{
//		// extend one step
//		if (dir_2_random.norm() < criterion)
//			qr = vertPos2;
//		else
//		{
//			dir_2_random.normalize();
//			qr = vertPos1 + step_size * dir_2_random;
//		}
//		old_TS_ptr->projectOntoTangentSpace(qr);
//
//		if (rrtConstraints != NULL)
//		{
//			if (collisionChecking(vertPos1, qr, step_size*0.1))
//				return temp_vertex_pos;
//
//			if (rrtConstraints->getConstraintVector(qr).norm() > _error_threshold)
//			{
//				if (projectionNewtonRaphson(qr))
//				{
//					deviationFlag = 1;
//					// Create new Tangent Space and Add to TS list
//					tangentSpace * TS = new tangentSpace(qr, rrtConstraints);
//					TangentSpaces.push_back(TS);
//				}
//				else
//					return temp_vertex_pos;
//			}
//			if ((vertPos2 - qr).norm() > (vertPos2 - temp_vertex_pos_old).norm())
//				return temp_vertex_pos;
//
//			temp_vertex_pos = qr;
//			// Add new TBrrtVertex to tree1
//			TBrrtVertex* new_vertex = NULL;
//			new_vertex = (TBrrtVertex*)generateNewVertex((rrtVertex*)nearVertex, temp_vertex_pos, step_size*0.1);
//			if (deviationFlag == 0)
//				new_vertex->_tangentSpace = nearVertex->_tangentSpace;
//			else
//				new_vertex->_tangentSpace = TangentSpaces.back();
//			pTargetTree1->insert(new_vertex);
//			
//			// Saving new vertex as old vertex for next iteration
//			temp_vertex_pos_old = temp_vertex_pos;
//			old_TS_ptr = new_vertex->_tangentSpace;
//		}
//		else
//			return qr;
//	} // end of while
//#endif // RRTConCon	
//}

void TBrrtManager::setStartandGoal(const Eigen::VectorXd & _start, const Eigen::VectorXd & _goal)
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

	rrtVertex* startVertex = new TBrrtVertex();
	startVertex->posState = _start;
	m_start = _start;

	rrtVertex* goalVertex = new TBrrtVertex();
	goalVertex->posState = _goal;
	m_goal = _goal;

	startTree.insert(startVertex);
	goalTree.insert(goalVertex);

	pTargetTree1 = &startTree;
	pTargetTree2 = &goalTree;

	tangentSpace* startTangentSpace = new tangentSpace(_start, rrtConstraints);
	tangentSpace* goalTangentSpace = new tangentSpace(_goal, rrtConstraints);
	((TBrrtVertex*)startVertex)->_tangentSpace = startTangentSpace;
	((TBrrtVertex*)goalVertex)->_tangentSpace = goalTangentSpace;
	TangentSpaces.push_back(startTangentSpace);
	TangentSpaces.push_back(goalTangentSpace);
}

vector<Eigen::VectorXd> TBrrtManager::extractPath(int smoothingNum)
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
		iter2 = iter;				// parent vertex
		advance(iter, -1);			// child vertex
		connectParentAndChild((*iter2), (*iter));
	}

	// check path connection
	iter = path.rbegin();
	int temp = 0;
	rrtVertex* curVertex = (*iter);
	while (curVertex != NULL)
	{
		curVertex = curVertex->parentVertex;
		temp++;
		
	}


	path = smoothingPath(path, smoothingNum);
	
	list<Eigen::VectorXd> filledPath = fillingPath(path);

	// project to constraint manifold
	LazyProjection(filledPath);
	list<Eigen::VectorXd>::iterator iter_path = filledPath.begin();
	vector<Eigen::VectorXd> outputPath(filledPath.size());
	for (unsigned int i = 0; i < outputPath.size(); i++, iter_path++)
	{
		outputPath[i] = *iter_path;
	}
	return outputPath;
}

vector<rrtVertex*> TBrrtManager::getRandomVertices(list<rrtVertex*>& path)
{
	// perform smoothing only locally
	int n = path.size();
	int _smoothing_range = randomInt(1, _max_smoothing_range);
	int idx = randomInt(0, n - 1);
	
	int idx1 = max(idx - _smoothing_range, 0);
	int idx2 = min(idx + _smoothing_range, n - 1);
	

	list<rrtVertex*>::iterator iter = path.begin();
	advance(iter, idx1);
	rrtVertex* vertex1 = *iter;

	iter = path.begin();
	advance(iter, idx2);
	rrtVertex* vertex2 = *iter;
	vector<rrtVertex*> randomVertices(2);
	randomVertices[0] = vertex1;
	randomVertices[1] = vertex2;
	return randomVertices;
}

//bool TBrrtManager::replaceVertices(list<rrtVertex*>& path, vector<rrtVertex*>& tempVertices, vector<rrtVertex*>& removedVertex)
//{
//	list<rrtVertex*> tmpPath;
//	rrtVertex* vertex1 = removedVertex[removedVertex.size() - 1]->parentVertex;
//	rrtVertex* vertex2 = tempVertices[tempVertices.size() - 1];
//	if (tempVertices.size() == 2)
//	{
//		vertex2->parentVertex = vertex1;
//		vertex2->distance2parent = getDistance(vertex1->posState, vertex2->posState);
//		vertex2->cost_bw_parent = getCost(vertex1, vertex2);
//	}
//	else
//	{
//		tempVertices[1]->parentVertex = vertex1;
//		tempVertices[1]->distance2parent = getDistance(vertex1->posState, tempVertices[1]->posState);
//		tempVertices[1]->cost_bw_parent = getCost(vertex1, tempVertices[1]);
//		for (unsigned int i = tempVertices.size() - 1; i > 1; i--)
//		{
//			tempVertices[i]->parentVertex = tempVertices[i - 1];
//			tempVertices[i]->distance2parent = getDistance(tempVertices[i - 1]->posState, tempVertices[i]->posState);
//			tempVertices[i]->cost_bw_parent = getCost(tempVertices[i - 1], tempVertices[i]);
//		}
//	}
//
//
//	for (unsigned int i = 0; i < removedVertex.size() - 1; i++)
//		path.remove(removedVertex[i]);
//
//	// add new vertices to the path
//	int n = path.size();
//	tmpPath.resize(0);
//	list<rrtVertex*>::reverse_iterator tmpriter = tmpPath.rbegin();
//	list<rrtVertex*>::reverse_iterator riter = path.rbegin();
//	tmpPath.push_back(*riter);
//	int added_num = tempVertices.size() - 1;
//	for (int i = 1; i < n + added_num - 1; i++, tmpriter++)
//		tmpPath.push_front((*tmpriter)->parentVertex);
//	path = tmpPath;
//	delete tempVertices[0];
//
//	return true;
//}

/*void TBrrtManager::getTangentBasis(const Eigen::VectorXd& vertPos1) {

	//// %%%% Basis formulation using principle directions %%%%
	//Eigen::MatrixXd J = rrtConstraints->getConstraintJacobian(vertPos1);
	//Eigen::MatrixXd G = J.transpose() * J;

	//Eigen::VectorXd v = Eigen::VectorXd(); // Mean curvature vector
	//int m = G.rows(); // = G.cols()
	//for (int i = 0; i < m; i++)
	//	for (int j = 0; j < m; j++)
	//		v += G(j, i) * rrtConstraints->getConstraintHessian(vertPos1, i, j); // Mean curvature vector v

	//v = 1 / m * (Eigen::MatrixXd::Identity(ProjectionMatrix.rows(), ProjectionMatrix.cols()) - ProjectionMatrix) * v;
	//v = v / v.norm();

	//Eigen::MatrixXd Qn(m, m);
	//for (int i = 0; i < m; i++)
	//	for (int j = 0; j < m; j++)
	//		Qn(i, j) = rrtConstraints->getConstraintHessian(vertPos1, i, j).transpose() * v;

	//Eigen::EigenSolver<Eigen::MatrixXd> Temp(G.inverse() * Qn);
	//Eigen::VectorXd buffer(Temp.eigenvectors().rows());

	//tangentBasis.clear();
	//// Renew current TS basis and save local curvature
	//Eigen::VectorXd * tempCurvature = new Eigen::VectorXd;
	//for (int i = 0; i < Temp.eigenvectors().cols(); i++) {
	//	for (int j = 0; j < Temp.eigenvectors().rows(); j++)
	//		buffer(j) = real(Temp.eigenvectors()(i, j));
	//	tangentBasis[i] = new Eigen::VectorXd(buffer);
	//	(*tempCurvature)(i) = real(Temp.eigenvalues()(i));
	//}
	//localCurvatures.push_back(tempCurvature);

	Eigen::FullPivLU<Eigen::MatrixXd> lu(rrtConstraints->getConstraintJacobian(vertPos1));
	Eigen::MatrixXd null_basis = lu.kernel();
}*/

// Project point onto constraint manifold minimizing norm error
bool TBrrtManager::projectionNewtonRaphson(Eigen::VectorXd& jointval, double threshold, int maxIter)
{
	// consider joint limit during projection
	return rrtConstraints->project2ConstraintManifold(jointval, maxIter);

	//Eigen::MatrixXd Jacobian;
	//Eigen::VectorXd error;
	//int i = 0;
	//
	//while (i < maxIter)
	//{
	//	i++;
	//	error = rrtConstraints->getConstraintVector(jointval);
	//	if (error.norm() < threshold)
	//		return 1;
	//	Jacobian = rrtConstraints->getConstraintJacobian(jointval);
	//	jointval -= Jacobian.transpose()*(Jacobian * Jacobian.transpose()).inverse() * error; // pseudo-inverse of J * e
	//}
	//return 0;
}

/*Eigen::VectorXd * TBrrtManager::TBrandomSample(const Eigen::VectorXd qroot, double range)
{
	Eigen::VectorXd tmp = Eigen::VectorXd();
	for (unsigned int i = 0; i < tangentBasis.size(); i++)
		tmp += randomDouble(-range, range) * (*tangentBasis[i]);
	Eigen::VectorXd * qrand = new Eigen::VectorXd(tmp);
	return qrand;
}*/

void TBrrtManager::LazyProjection(list<Eigen::VectorXd>& path)
{
	list<Eigen::VectorXd>::iterator iter = path.begin();
	list<Eigen::VectorXd>::iterator iter_end = path.end(); iter_end--;
	advance(iter, 1);
	for (iter; iter != iter_end; iter++)
	{
		projectionNewtonRaphson((*iter));
	}
}

void TBrrtManager::LazyProjection(vector<Eigen::VectorXd>& path) {
	for (unsigned int i = 0; i < path.size(); i++)
		projectionNewtonRaphson(path[i]);
}

unsigned int TBrrtManager::selectTangentSpace()
{
	return 0;
}

void TBrrtManager::setThreshold(double threshold) 
{
	_error_threshold = threshold;
}

void TBrrtManager::setMaxSmoothingRange(int maxSmoothingRange)
{
	_max_smoothing_range = maxSmoothingRange;
}

void TBrrtManager::setConstraint(rrtConstraint* constraint)
{
	rrtConstraints = constraint;
}

void TBrrtManager::clearConstraints()
{
	rrtConstraints = NULL;
}

bool TBrrtManager::innerloop()
{
	const double eps = 1e-16;
	/* -------------------------------------------------------------  TREE 1 -------------------------------------------------------------*/
	rrtVertex* new_vertex = NULL;

	// 1. random node generation
	Eigen::VectorXd random_vertex_pos = generateRandomVertex();

	// 2. finding nearest vertex
	rrtVertex* nearest_vertex = nearestVertex(random_vertex_pos, TREE1);
	Eigen::VectorXd nearest_vertex_pos = nearest_vertex->posState;

#ifdef RRTExtCon
	// ExtCon in TB-RRT

	// 3. extend step size to random node (vector field is considered here)
	Eigen::VectorXd temp_vertex_pos = extendStepSize(nearest_vertex_pos, random_vertex_pos, eps, TREE1);

	// 4. (optional) constraint projection
	//if (rrtConstraints != NULL)
	//	rrtConstraints->project2ConstraintManifold(temp_vertex_pos);

	// 5. collision checking 
	new_vertex = generateNewVertex(nearest_vertex, temp_vertex_pos, step_size*0.1);

	if (new_vertex != NULL)
		pTargetTree1->insert(new_vertex);
	else
		new_vertex = nearest_vertex;
#endif

#ifndef RRTExtCon
	// ConCon in TB-RRT
	rrtVertex* old_vertex = nearest_vertex;

	while (1)
	{
		Eigen::VectorXd temp_vertex_pos = extendStepSize(old_vertex->posState, random_vertex_pos, eps, TREE1);
		rrtVertex* qr = generateNewVertex(old_vertex, temp_vertex_pos, step_size*0.1);
		if (qr == NULL)
		{
			break;
		}
		if ((random_vertex_pos - qr->posState).norm() > (random_vertex_pos - old_vertex->posState).norm())
			break;

		new_vertex = qr;
		pTargetTree1->insert(new_vertex);
		old_vertex = new_vertex;
	}
	if (new_vertex == NULL)
		new_vertex = nearest_vertex;
#endif


	/* -------------------------------------------------------------  TREE 2 -------------------------------------------------------------*/

	rrtVertex* new_tree2_vertex = NULL;
	rrtVertex* nearest_tree2_vertex = nearestVertex(new_vertex->posState, TREE2);

#ifdef RRTExtCon
	// ExtCon in TB-RRT

	//Eigen::VectorXd dir_2_tree1 = (new_vertex->posState - nearest_tree2_vertex->posState);
	//double distance_2_tree1 = dir_2_tree1.norm();

	Eigen::VectorXd temp_tree2_vertex_pos = extendStepSize(nearest_tree2_vertex->posState, new_vertex->posState, step_size, TREE2);
	// (optional) constraint projection
	//if (rrtConstraints != NULL)
	//	rrtConstraints->project2ConstraintManifold(temp_tree2_vertex_pos);

	// collision checking
	new_tree2_vertex = generateNewVertex(nearest_tree2_vertex, temp_tree2_vertex_pos, step_size*0.1);
	if (new_tree2_vertex != NULL)
		pTargetTree2->insert(new_tree2_vertex);
#endif // RRTExtCon

#ifndef RRTExtCon
	// ConCon in TB-RRT
	rrtVertex* old_tree2_vertex = nearest_tree2_vertex;

	while (1)
	{
		Eigen::VectorXd temp_tree2_vertex_pos = extendStepSize(old_tree2_vertex->posState, new_vertex->posState, eps, TREE2);
		rrtVertex* qr = generateNewVertex(old_tree2_vertex, temp_tree2_vertex_pos, step_size*0.1);
		if (qr == NULL)
		{
			break;
		}
		if ((new_vertex->posState - qr->posState).norm() > (new_vertex->posState - old_tree2_vertex->posState).norm())
			break;

		new_tree2_vertex = qr;
		pTargetTree2->insert(new_tree2_vertex);
		old_tree2_vertex = new_tree2_vertex;
	}
#endif


	/* -------------------------------------------------------------  CHECK CONNECTIVITY -------------------------------------------------------------*/
	bool printDist = true;
	if (new_tree2_vertex == NULL)
		return false;

	else
	{
		double dist = getDistance(new_tree2_vertex->posState, new_vertex->posState);
		if (printDist == true)
			cout << "distance b/w trees: " << dist << endl;
		if (dist < step_size && !collisionChecking(new_vertex->posState, new_tree2_vertex->posState, step_size*0.1))
		{
			rrtVertex* new_tree1_last_vertex = new TBrrtVertex;
			new_tree1_last_vertex->posState = new_tree2_vertex->posState;
			new_tree1_last_vertex->parentVertex = new_vertex;
			((TBrrtVertex*)new_tree1_last_vertex)->_tangentSpace = ((TBrrtVertex*)new_tree2_vertex)->_tangentSpace;
			pTargetTree1->insert(new_tree1_last_vertex);

			connectedVertex1 = new_tree1_last_vertex;
			connectedVertex2 = new_tree2_vertex;
			return true;
		}
		else
			return false;
	}
}

// %%%%%%%%%%%%%% rrtConstraint member functions %%%%%%%%%%%%%%

rrtConstraint::rrtConstraint()
{
}

rrtConstraint::~rrtConstraint()
{
}

// %%%%%%%%%%%%%% Tangent Space member functions %%%%%%%%%%%%%%

tangentSpace::tangentSpace()
{
	nodeNumber = 0;
}
tangentSpace::~tangentSpace()
{
}

tangentSpace::tangentSpace(Eigen::VectorXd vertex, rrtConstraint* constraints)
{
	Eigen::FullPivLU<Eigen::MatrixXd> lu(constraints->getConstraintJacobian(vertex));
	tangentBasis = lu.kernel();

	rootNode = vertex;

	// localCurvature =

	Eigen::MatrixXd dfdx = constraints->getConstraintJacobian(vertex);
	int n = dfdx.cols();
	int k = dfdx.rows();
	//Eigen::MatrixXd dfdu = dfdx.block(0, 0, k, n - k);
	//Eigen::MatrixXd dfdv = dfdx.block(0, n - k, k, k);
	//Eigen::MatrixXd J = Eigen::MatrixXd::Zero(n, n - k);
	//for (int i = 0; i < n - k; i++)
	//	J(i, i) = 1.0;
	//
	//printf("dfdu:\n");
	//cout << dfdu << endl << endl;
	//
	//printf("dfdv:\n");
	//cout << dfdv << endl << endl;
	//
	//printf("dfdv^-1:\n");
	//cout << dfdv.inverse() << endl << endl;
	//J.block(n - k, 0, k, n - k) = -dfdv.inverse() * dfdu;
	//printf("J:\n");
	//cout << J << endl << endl;

	//Eigen::MatrixXd G = J.transpose() * J;
	//printf("G^-1:\n");
	//cout << G.inverse() << endl << endl;
	//ProjectionMatrix = J * G.inverse() * J.transpose();
	
	///////////// svd /////////////////////
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(dfdx, Eigen::ComputeFullU | Eigen::ComputeFullV);

	ProjectionMatrix = svd.matrixV().block(0, k, n, n - k);
	ProjectionMatrix = ProjectionMatrix * ProjectionMatrix.transpose();

	nodeNumber = 1;
}

// Project point onto reference point's tangent space
void tangentSpace::projectOntoTangentSpace(Eigen::VectorXd& jointval)
{
	jointval = rootNode + ProjectionMatrix * (jointval - rootNode);
}

// %%%%%%%%%%%%%% TBrrtVertex member functions %%%%%%%%%%%%%%

TBrrtVertex::TBrrtVertex()
{
	_tangentSpace = NULL;
}
TBrrtVertex::~TBrrtVertex()
{
}