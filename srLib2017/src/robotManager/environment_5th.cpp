#include "makeSpecialCol.h"
#include "common\utils.h"
#include "environment_5th.h"

MovingContact::MovingContact(double collision_offset /*= 0.01*/)
{
	m_collision_offset = collision_offset;
	AssembleModel();
}

MovingContact::~MovingContact()
{
}

void MovingContact::AssembleModel()
{
	m_numLink = 5;
	m_numCollision = 5;
	m_numWeldJoint = 4;
	for (int i = 0; i < m_numLink; i++)
	{
		srLink* temp = new srLink;
		m_ObjLink.push_back(*temp);
	}
	
	for (int i = 0; i < m_numCollision; i++)
	{
		srCollision* temp = new srCollision;
		m_ObjCollision.push_back(*temp);
	}
	for (int i = 0; i < m_numWeldJoint; i++)
	{
		srWeldJoint* temp = new srWeldJoint;
		m_ObjWeldJoint.push_back(*temp);
	}

	// base
	Vec3 dim0(0.273, 0.450, 0.006);
	Vec3 dim1(0.015, 0.450, 0.114);
	Vec3 dim2(0.273, 0.020, 0.114);

	vector<Vec3> dims(m_numLink);
	dims[0] = dim0;
	dims[1] = dim1;
	dims[2] = dim1;
	dims[3] = dim2;
	dims[4] = dim2;

	for (int i = 0; i < m_numLink; i++)
	{
		m_ObjLink[i].GetGeomInfo().SetColor(0.3, 0.3, 0.3);
		m_ObjLink[i].GetGeomInfo().SetDimension(dims[i]);
		m_ObjLink[i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	}
	m_ObjLink[0].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	m_ObjLink[0].GetGeomInfo().SetFileName("../../../workspace/robot/objects_3ds/Moving_Contact_simplified.3ds");
	// xy plane
	m_ObjWeldJoint[0].SetParentLink(&m_ObjLink[0]);
	m_ObjWeldJoint[0].SetParentLinkFrame();
	m_ObjWeldJoint[0].SetChildLink(&m_ObjLink[1]);
	m_ObjWeldJoint[0].SetChildLinkFrame(SE3(Vec3(-0.5*(0.273 - 0.015), 0.0, -0.5*(0.114 + 0.006))));

	m_ObjWeldJoint[1].SetParentLink(&m_ObjLink[0]);
	m_ObjWeldJoint[1].SetParentLinkFrame();
	m_ObjWeldJoint[1].SetChildLink(&m_ObjLink[2]);
	m_ObjWeldJoint[1].SetChildLinkFrame(SE3(Vec3(0.5*(0.273 - 0.015), 0.0, -0.5*(0.114 + 0.006))));
	// yz plane
	m_ObjWeldJoint[2].SetParentLink(&m_ObjLink[0]);
	m_ObjWeldJoint[2].SetParentLinkFrame();
	m_ObjWeldJoint[2].SetChildLink(&m_ObjLink[3]);
	m_ObjWeldJoint[2].SetChildLinkFrame(SE3(Vec3(0.0, -0.5*(0.450 - 0.020), -0.5*(0.114 + 0.006))));

	m_ObjWeldJoint[3].SetParentLink(&m_ObjLink[0]);
	m_ObjWeldJoint[3].SetParentLinkFrame();
	m_ObjWeldJoint[3].SetChildLink(&m_ObjLink[4]);
	m_ObjWeldJoint[3].SetChildLinkFrame(SE3(Vec3(0.0, 0.5*(0.450 - 0.020), -0.5*(0.114 + 0.006))));

	///// set collision
	Vec3 colli_offset(m_collision_offset);
	for (int i = 0; i < m_numLink; i++)
	{
		m_ObjCollision[i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i].GetGeomInfo().SetDimension(dims[i] + colli_offset);
		m_ObjCollision[i].SetLocalFrame(SE3());
		m_ObjLink[i].AddCollision(&m_ObjCollision[i]);
	}
	
	// dummy link
	//srLink* temp = new srLink;
	//m_ObjLink.push_back(*temp);
	//m_ObjLink[m_numLink].GetGeomInfo().SetDimension(0.0);
	//srWeldJoint* temp1 = new srWeldJoint;
	//m_ObjWeldJoint.push_back(*temp1);
	//m_ObjWeldJoint[m_numWeldJoint].SetParentLink(&m_ObjLink[m_numLink]);
	//m_ObjWeldJoint[m_numWeldJoint].SetChildLink(&m_ObjLink[0]);
	//m_ObjWeldJoint[m_numWeldJoint].SetParentLinkFrame(SE3());
	//m_ObjWeldJoint[m_numWeldJoint].SetChildLinkFrame(SE3());

	this->SetSelfCollision(false);
	//this->SetBaseLink(&m_ObjLink[m_numLink]);
	this->SetBaseLink(&m_ObjLink[0]);
	this->SetBaseLinkType(srSystem::KINEMATIC);
}
