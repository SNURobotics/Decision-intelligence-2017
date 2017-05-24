#include "srGamasot/srRobot.h"

using namespace gamasot;

srRobot::srRobot()
{

}

srRobot::~srRobot()
{
	delete &mLink;
	delete &mCollision;
	delete &mJoint;
}

srLink* srRobot::getLink(string linkname)
{
	srLink* link;
	map < string, srLink* >::iterator item;

	item = mLink.find(linkname);
	if (item == mLink.end())
	{
		link = new srLink();
		mLink.insert(make_pair(linkname, link));
	}
	else
	{
		link = item->second;
	}

	return link;
}

srCollision* srRobot::getCollision(string collisionname)
{
	srCollision* collision;
	map < string, srCollision* >::iterator item;

	item = mCollision.find(collisionname);
	if (item == mCollision.end())
	{
		collision = new srCollision();
		mCollision.insert(make_pair(collisionname, collision));
	}
	else
	{
		collision = item->second;
	}

	return collision;
}

srJoint* srRobot::getJoint(string jointname, srJoint::JOINTTYPE jointType)
{
	srJoint* joint;
	map < string, srJoint* >::iterator item;

	item = mJoint.find(jointname);
	if (item == mJoint.end())
	{
		switch (jointType)
		{
		case srJoint::JOINTTYPE::REVOLUTE:
			joint = new srRevoluteJoint();
			break;
		case srJoint::JOINTTYPE::BALL:
			joint = new srBallJoint();
			break;
		case srJoint::JOINTTYPE::PRISMATIC:
			joint = new srPrismaticJoint();
			break;
		case srJoint::JOINTTYPE::UNIVERSAL:
			joint = new srUniversalJoint();
			break;
		case srJoint::JOINTTYPE::WELD:
			joint = new srWeldJoint();
			break;
		default:
			return NULL;
		}
		mJoint.insert(make_pair(jointname, joint));
	}
	else
	{
		joint = item->second;
	}

	return joint;
}

void gamasot::srRobot::setStateJointLimit(vector<double> ulim, vector<double> llim)
{
	int nStateJoint = 0;
	map<string, srJoint*>::iterator it;
	for (it = mJoint.begin(); it != mJoint.end(); it++)
	{
		if (it->second->GetType() == srJoint::REVOLUTE || it->second->GetType() == srJoint::PRISMATIC)
			nStateJoint += 1;
	}
	if (ulim.size() != llim.size() || ulim.size() != nStateJoint)
		printf("check number of limits!!!\n");
	nStateJoint = 0;
	for (it = mJoint.begin(); it != mJoint.end(); it++)
	{
		if (it->second->GetType() == srJoint::REVOLUTE || it->second->GetType() == srJoint::PRISMATIC)
		{
			((srStateJoint*)it->second)->m_PosLimit[0] = llim[nStateJoint];
			((srStateJoint*)it->second)->m_PosLimit[1] = ulim[nStateJoint];
			nStateJoint += 1;
		}
	}
}
