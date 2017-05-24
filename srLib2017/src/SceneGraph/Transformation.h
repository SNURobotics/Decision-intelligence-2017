#pragma once

#include "Node.h"
#include "../liegroup/LieGroup.h"


class Transformation;

void Transform(Transformation* pTrans, SE3 transMatrix, double scale = 1);
void Transform(Node* pNode, SE3 transMatrix, double scale =1);

class Transformation
{
public:
	Transformation(SE3 trans = SE3(), double scale = 1);
	virtual ~Transformation(void);
	
	void	runTransformation();

	void	setTransformation(SE3 trans);
	void	multTransformation(SE3 trans);
	void	setScaleFactor(double scale);
	void	setLocalFrame(SE3 localFrame);
	void	setTargetSE3Pointer(SE3* pSE3);


protected:
	void	targetCheck();

protected:
	SE3		transformMatrix;
	SE3		localFrame;
	double	scaleFactor;


	SE3*	pTargetSE3;

};

