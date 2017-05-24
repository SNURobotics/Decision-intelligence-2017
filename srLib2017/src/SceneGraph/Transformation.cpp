#include "Transformation.h"
#include "gl.h"

#include <iostream>
using std::cout;
using std::endl;

Transformation::Transformation(SE3 trans, double scale)
:transformMatrix(trans), scaleFactor(scale), pTargetSE3(NULL)
{

}

Transformation::~Transformation(void)
{
}

void Transformation::runTransformation()
{
	targetCheck();

	GLdouble matrixArray[16];
	(transformMatrix * localFrame).ToArray(matrixArray);
	//transformMatrix.ToArray(matrixArray);
	glMultMatrixd(matrixArray);
	glScaled(scaleFactor,scaleFactor,scaleFactor);

}


void Transformation::setTransformation(SE3 trans)
{
	transformMatrix = trans;
}

void Transformation::setScaleFactor(double scale)
{
	scaleFactor = scale;
}

void Transformation::multTransformation( SE3 trans )
{
	transformMatrix *= trans;
}

void Transformation::setLocalFrame( SE3 localFrame )
{
	this->localFrame = localFrame;
}

void Transformation::setTargetSE3Pointer( SE3* pSE3 )
{
	pTargetSE3 = pSE3;
}

void Transformation::targetCheck()
{
	if(pTargetSE3)
		transformMatrix = *pTargetSE3;
}



void Transform( Transformation* pTrans, SE3 transMatrix, double scale )
{
	pTrans->setTransformation(transMatrix);
	pTrans->setScaleFactor(scale);
}

void Transform( Node* pNode, SE3 transMatrix, double scale )
{
	Transformation* pTrans = dynamic_cast<Transformation*>(pNode);
	if(pTrans != NULL)
		Transform(pTrans,transMatrix,scale);
	else
	{
		cout << "Transfrom func. error : Node* cannot be casting to Transformation* " << endl;
	}
}