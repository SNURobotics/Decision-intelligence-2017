#include "environment.h"


//Object::Object()
//{
//}
//
//Object::~Object()
//{
//}


void Object::setBaseLinkPosition(Vec3 position)
{
	this->GetBaseLink()->SetFrame(SE3(position));
}

void Object::setBaseLinkPosition(Eigen::VectorXd position)
{
	Vec3 tmpVec(position(0), position(1), position(2));
	this->GetBaseLink()->SetFrame(SE3(tmpVec));
}

void Object::setBaseLinkFrame(SE3 T)
{
	this->GetBaseLink()->SetFrame(T);
}

SE3 Object::getBaseLinkFrame()
{
	return this->GetBaseLink()->GetFrame();
}
