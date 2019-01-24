#include "TBrrtManager.h"
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

TBrrtManager::TBrrtManager() : _error_threshold(0.1)
{	
	rrtConstraints = NULL;
}


TBrrtManager::~TBrrtManager()
{
}

#define RRTExtCon
//#define RRTConCon

// Currently not using
Eigen::VectorXd TBrrtManager::extendStepSize(const Eigen::VectorXd& vertPos1, const Eigen::VectorXd& vertPos2, double criterion, TARGET_TREE tree /* = TARGET_TREE::TREE1*/)
{
	Eigen::VectorXd dir_2_random = (vertPos2 - vertPos1);
	Eigen::VectorXd temp_vertex_pos;
	Eigen::VectorXd temp_vertex_pos_old;

	#ifdef RRTExtCon
		// extend one step
		if (dir_2_random.norm() < criterion)
			temp_vertex_pos = vertPos2;
		else
		{
			dir_2_random.normalize();
			temp_vertex_pos = vertPos1 + step_size * dir_2_random;
		}
	
		if (rrtConstraints != NULL)
		{
			if (collisionChecking(vertPos1, temp_vertex_pos, step_size*0.1))
				return Eigen::VectorXd();
			if (rrtConstraints->getConstraintVector(temp_vertex_pos).norm() > _error_threshold)
			{
				if (projectionNewtonRaphson(temp_vertex_pos))
				{
					// Create new Tangent Space and Add to TS list
					tangentSpace * TS = new tangentSpace(temp_vertex_pos, rrtConstraints);
					TangentSpaces.push_back(TS);
				}
				else
					return Eigen::VectorXd();
			}
		}
		// Add vertex to tree
		rrtVertex* new_vertex = NULL;
		rrtVertex* nearest_vertex = nearestVertex(temp_vertex_pos, tree);
		new_vertex = generateNewVertex(nearest_vertex, temp_vertex_pos, step_size*0.1);
		pTargetTree1->insert(new_vertex);
		return temp_vertex_pos;
	#endif // RRTExtCon
	
	#ifdef RRTConCon
		Eigen::VectorXd qr;
		temp_vertex_pos_old = vertPos1; // qold
		while (temp_vertex_pos != vertPos2)
		{
			// extend one step
			if (dir_2_random.norm() < criterion)
				qr = vertPos2;
			else
			{
				dir_2_random.normalize();
				qr = vertPos1 + step_size * dir_2_random;
			}

			if (rrtConstraints != NULL)
			{
				if (collisionChecking(vertPos1, qr, step_size*0.1))
					return temp_vertex_pos;

				if (rrtConstraints->getConstraintVector(qr).norm() > _error_threshold)
				{
					if (projectionNewtonRaphson(qr))
					{
						// Add vertex to tree
						rrtVertex* new_vertex = NULL;
						rrtVertex* nearest_vertex = nearestVertex(qr, tree);
						new_vertex = generateNewVertex(nearest_vertex, qr, step_size*0.1);
						pTargetTree1->insert(new_vertex);
						// Create new Tangent Space and Add to TS list
						tangentSpace * TS = new tangentSpace(qr, rrtConstraints);
						TangentSpaces.push_back(TS);
						return qr;
					}
					else
						return temp_vertex_pos;
				}
				temp_vertex_pos = qr;
				// Add vertex to tree
				rrtVertex* new_vertex = NULL;
				rrtVertex* nearest_vertex = nearestVertex(temp_vertex_pos, tree);
				new_vertex = generateNewVertex(nearest_vertex, temp_vertex_pos, step_size*0.1);
				pTargetTree1->insert(new_vertex);
				temp_vertex_pos_old = temp_vertex_pos;
			}
			else
				return qr;
		} // end of while
	#endif // RRTConCon	
}

Eigen::VectorXd TBrrtManager::extendStepSizeSimple(const TBrrtVertex* nearVertex, const Eigen::VectorXd& vertPos2, double criterion, TARGET_TREE tree /* = TARGET_TREE::TREE1*/)
{
	Eigen::VectorXd vertPos1 = nearVertex->posState;
	Eigen::VectorXd dir_2_random = (vertPos2 - vertPos1);
	Eigen::VectorXd temp_vertex_pos;
	Eigen::VectorXd temp_vertex_pos_old;
	bool deviationFlag = 0;	// flag for excessive deviation from constraint manifold

#ifdef RRTExtCon
	// extend one step
	if (dir_2_random.norm() < criterion)
		temp_vertex_pos = vertPos2;
	else
	{
		dir_2_random.normalize();
		temp_vertex_pos = vertPos1 + step_size * dir_2_random;
	}
	nearVertex->_tangentSpace->projectOntoTangentSpace(temp_vertex_pos);

	if (rrtConstraints != NULL)
	{
		if (collisionChecking(vertPos1, temp_vertex_pos, step_size*0.1))
			return Eigen::VectorXd();
		if (rrtConstraints->getConstraintVector(temp_vertex_pos).norm() > _error_threshold)
		{
			if (projectionNewtonRaphson(temp_vertex_pos))
			{
				deviationFlag = 1;
				// Create new Tangent Space and Add to TS list
				tangentSpace * TS = new tangentSpace(temp_vertex_pos, rrtConstraints);
				TangentSpaces.push_back(TS);

				return temp_vertex_pos;
			}
			else
				return Eigen::VectorXd();
		}
	}
	// Add new TBrrtVertex to tree1
	TBrrtVertex* new_vertex = NULL;
	new_vertex = generateNewVertex(nearVertex, temp_vertex_pos, step_size*0.1);
	if (deviationFlag == 0)
		new_vertex->_tangentSpace = nearVertex->_tangentSpace;
	else
		new_vertex->_tangentSpace = TangentSpaces.back();
	pTargetTree1->insert(new_vertex);
	return temp_vertex_pos;
#endif // RRTExtCon

#ifdef RRTConCon
	Eigen::VectorXd qr;
	temp_vertex_pos_old = vertPos1; // qold
	tangentSpace* old_TS_ptr = nearVertex->_tangentSpace;
	while (1)
	{
		// extend one step
		if (dir_2_random.norm() < criterion)
			qr = vertPos2;
		else
		{
			dir_2_random.normalize();
			qr = vertPos1 + step_size * dir_2_random;
		}
		old_TS_ptr->projectOntoTangentSpace(qr);

		if (rrtConstraints != NULL)
		{
			if (collisionChecking(vertPos1, qr, step_size*0.1))
				return temp_vertex_pos;

			if (rrtConstraints->getConstraintVector(qr).norm() > _error_threshold)
			{
				if (projectionNewtonRaphson(qr))
				{
					deviationFlag = 1;
					// Create new Tangent Space and Add to TS list
					tangentSpace * TS = new tangentSpace(qr, rrtConstraints);
					TangentSpaces.push_back(TS);
				}
				else
					return temp_vertex_pos;
			}
			if ((vertPos2 - qr).norm() > (vertPos2 - temp_vertex_pos_old).norm())
				return temp_vertex_pos;

			temp_vertex_pos = qr;
			// Add new TBrrtVertex to tree1
			TBrrtVertex* new_vertex = NULL;
			new_vertex = (TBrrtVertex*)generateNewVertex((rrtVertex*)nearVertex, temp_vertex_pos, step_size*0.1);
			if (deviationFlag == 0)
				new_vertex->_tangentSpace = nearVertex->_tangentSpace;
			else
				new_vertex->_tangentSpace = TangentSpaces.back();
			pTargetTree1->insert(new_vertex);
			
			// Saving new vertex as old vertex for next iteration
			temp_vertex_pos_old = temp_vertex_pos;
			old_TS_ptr = new_vertex->_tangentSpace;
		}
		else
			return qr;
	} // end of while
#endif // RRTConCon	
}

vector<rrtVertex*> TBrrtManager::getConstrainedPathConnectingTwoVertices(rrtVertex* vertex1, rrtVertex* vertex2, double eps, int maxIter /*= 10000*/)
{
	vector<rrtVertex*> extendedVertex;
	rrtVertex* firstExtenedVertex = new rrtVertex;
	firstExtenedVertex->posState = vertex1->posState;
	extendedVertex.push_back(firstExtenedVertex);


	// find temporary vertex set which connect vertex1 and vertex2
	int iter = 0;
	while (iter < maxIter)
	{
		Eigen::VectorXd diff = vertex2->posState - extendedVertex[extendedVertex.size() - 1]->posState;
		double diffnorm = diff.norm();

		// if two random vertices (vertex1 and vertex2) are close enough, break
		if (diffnorm < eps && !collisionChecking(vertex2->posState, extendedVertex[extendedVertex.size() - 1]->posState, step_size*0.1))
			break;

		// if newly extended vertex is farther to vertex2 than previously extended vertex, break
		if (extendedVertex.size() > 2 && diffnorm > (vertex2->posState - extendedVertex[extendedVertex.size() - 2]->posState).norm())
		{
			extendedVertex.resize(extendedVertex.size() - 1);
			break;
		}

		// to get a temp vertex, from vertex1 go a step size to direction to vertex2 and project to constraint manifold
		Eigen::VectorXd temp_vertex = extendedVertex[extendedVertex.size() - 1]->posState + min(step_size, diffnorm)*diff;
		rrtConstraints->project2ConstraintManifold(temp_vertex);

		// if there is no collision, add temp vertex to the set of temporary vertices
		if (!collisionChecking(temp_vertex, extendedVertex[extendedVertex.size() - 1]->posState, step_size*0.1))
		{
			rrtVertex* tempVertex = new rrtVertex;
			tempVertex->posState = temp_vertex;
			//tempVertex->parentVertex = extendedVertex[extendedVertex.size() - 1];
			//tempVertex->distance2parent = (extendedVertex[extendedVertex.size() - 1]->posState - temp_vertex).norm();
			//tempVertex->cost_bw_parent = getCost(extendedVertex[extendedVertex.size() - 1], tempVertex);

			extendedVertex.push_back(tempVertex);
		}
		else
			break;
		iter++;
	}
	return extendedVertex;
}

vector<rrtVertex*> TBrrtManager::getCandidateVertices(vector<rrtVertex*> vertices)
{
	double eps = 0.5*step_size;
	vector<rrtVertex*> extendedVertex = getConstrainedPathConnectingTwoVertices(vertices[0], vertices[1], eps);
	double dist_cur = (vertices[1]->posState - extendedVertex[extendedVertex.size() - 1]->posState).norm();
	if (dist_cur < eps)
	{
		extendedVertex.push_back(vertices[1]);
		return extendedVertex;
	}
	else
	{
		// delete extended vertices
		for (unsigned int i = 0; i < extendedVertex.size(); i++)
			delete extendedVertex[i];
	}
	return vector<rrtVertex*>();
}

bool TBrrtManager::replaceVertices(list<rrtVertex*>& path, vector<rrtVertex*>& tempVertices, vector<rrtVertex*>& removedVertex)
{
	list<rrtVertex*> tmpPath;
	rrtVertex* vertex1 = removedVertex[removedVertex.size() - 1]->parentVertex;
	rrtVertex* vertex2 = tempVertices[tempVertices.size() - 1];
	if (tempVertices.size() == 2)
	{
		vertex2->parentVertex = vertex1;
		vertex2->distance2parent = getDistance(vertex1->posState, vertex2->posState);
		vertex2->cost_bw_parent = getCost(vertex1, vertex2);
	}
	else
	{
		tempVertices[1]->parentVertex = vertex1;
		tempVertices[1]->distance2parent = getDistance(vertex1->posState, tempVertices[1]->posState);
		tempVertices[1]->cost_bw_parent = getCost(vertex1, tempVertices[1]);
		for (unsigned int i = tempVertices.size() - 1; i > 1; i--)
		{
			tempVertices[i]->parentVertex = tempVertices[i - 1];
			tempVertices[i]->distance2parent = getDistance(tempVertices[i - 1]->posState, tempVertices[i]->posState);
			tempVertices[i]->cost_bw_parent = getCost(tempVertices[i - 1], tempVertices[i]);
		}
	}


	for (unsigned int i = 0; i < removedVertex.size() - 1; i++)
		path.remove(removedVertex[i]);

	// add new vertices to the path
	int n = path.size();
	tmpPath.resize(0);
	list<rrtVertex*>::reverse_iterator tmpriter = tmpPath.rbegin();
	list<rrtVertex*>::reverse_iterator riter = path.rbegin();
	tmpPath.push_back(*riter);
	int added_num = tempVertices.size() - 1;
	for (int i = 1; i < n + added_num - 1; i++, tmpriter++)
		tmpPath.push_front((*tmpriter)->parentVertex);
	path = tmpPath;
	delete tempVertices[0];

	return true;
}

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
bool TBrrtManager::projectionNewtonRaphson(Eigen::VectorXd& jointval, double threshold = 0.1, int maxIter = 100)
{
	Eigen::MatrixXd Jacobian;
	Eigen::VectorXd error;
	int i = 0;
	
	while (i < maxIter)
	{
		i++;
		error = rrtConstraints->getConstraintVector(jointval);
		if (error.norm() < threshold)
			return 1;
		Jacobian = rrtConstraints->getConstraintJacobian(jointval);
		jointval -= Jacobian.transpose()*(Jacobian * Jacobian.transpose()).inverse() * error; // pseudo-inverse of J * e
	}
	return 0;
}

/*Eigen::VectorXd * TBrrtManager::TBrandomSample(const Eigen::VectorXd qroot, double range)
{
	Eigen::VectorXd tmp = Eigen::VectorXd();
	for (unsigned int i = 0; i < tangentBasis.size(); i++)
		tmp += randomDouble(-range, range) * (*tangentBasis[i]);
	Eigen::VectorXd * qrand = new Eigen::VectorXd(tmp);
	return qrand;
}*/

void TBrrtManager::LazyProjection(vector<rrtVertex *>& path)
{
	for (int i = 0; i < path.size(); i++) 
	{
		Eigen::VectorXd buffer;
		projectionNewtonRaphson(buffer);
		path[i]->posState = buffer;
		path[i]->distance2parent = (path[i]->parentVertex->posState - path[i]->posState).norm();
	}
}

unsigned int TBrrtManager::selectTangentSpace()
{

}

void TBrrtManager::setThreshold(double threshold) 
{
	_error_threshold = threshold;
}

void TBrrtManager::setConstraint(rrtConstraint* constraint)
{
	rrtConstraints = constraint;
}

void TBrrtManager::clearConstraints()
{
	rrtConstraints = NULL;
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

	Eigen::MatrixXd J = constraints->getConstraintJacobian(vertex);
	Eigen::MatrixXd G = J.transpose() * J;
	ProjectionMatrix = J * G.inverse() * J.transpose();
	
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