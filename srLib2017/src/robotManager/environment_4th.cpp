#include "makeSpecialCol.h"
#include "common\utils.h"
#include "environment_4th.h"

Bin::Bin(double collision_offset /*= 0.01*/)
{
	m_collision_offset = collision_offset;
	AssembleModel();
}

Bin::~Bin()
{
}

void Bin::AssembleModel()
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
		m_ObjLink[i].GetGeomInfo().SetColor(0.3f, 0.3f, 0.3f);
		m_ObjLink[i].GetGeomInfo().SetDimension(dims[i]);
		m_ObjLink[i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	}

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

Table4th::Table4th(double collision_offset /*= 0.01*/)
{
	m_collision_offset = collision_offset;
	AssembleModel();
}

Table4th::~Table4th()
{
}

void Table4th::AssembleModel()
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
	Vec3 dim0(0.8, 0.8, 0.3);
	Vec3 dim1(0.05, 0.05, 0.6);

	vector<Vec3> dims(m_numLink);
	dims[0] = dim0;
	dims[1] = dim1;
	dims[2] = dim1;
	dims[3] = dim1;
	dims[4] = dim1;

	for (int i = 0; i < m_numLink; i++)
	{
		m_ObjLink[i].GetGeomInfo().SetColor(0.3f, 0.3f, 0.3f);
		m_ObjLink[i].GetGeomInfo().SetDimension(dims[i]);
		m_ObjLink[i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	}

	// xy plane
	m_ObjWeldJoint[0].SetParentLink(&m_ObjLink[0]);
	m_ObjWeldJoint[0].SetParentLinkFrame();
	m_ObjWeldJoint[0].SetChildLink(&m_ObjLink[1]);
	m_ObjWeldJoint[0].SetChildLinkFrame(SE3(Vec3(-0.5*(dim0[0] - 0.05), -0.5*(dim0[1] - 0.05), 0.5*(dim1[2] + 0.05))));

	m_ObjWeldJoint[1].SetParentLink(&m_ObjLink[0]);
	m_ObjWeldJoint[1].SetParentLinkFrame();
	m_ObjWeldJoint[1].SetChildLink(&m_ObjLink[2]);
	m_ObjWeldJoint[1].SetChildLinkFrame(SE3(Vec3(-0.5*(dim0[0] - 0.05), 0.5*(dim0[1] - 0.05), 0.5*(dim1[2] + 0.05))));
	// yz plane
	m_ObjWeldJoint[2].SetParentLink(&m_ObjLink[0]);
	m_ObjWeldJoint[2].SetParentLinkFrame();
	m_ObjWeldJoint[2].SetChildLink(&m_ObjLink[3]);
	m_ObjWeldJoint[2].SetChildLinkFrame(SE3(Vec3(0.5*(dim0[0] - 0.05), -0.5*(dim0[1] - 0.05), 0.5*(dim1[2] + 0.05))));

	m_ObjWeldJoint[3].SetParentLink(&m_ObjLink[0]);
	m_ObjWeldJoint[3].SetParentLinkFrame();
	m_ObjWeldJoint[3].SetChildLink(&m_ObjLink[4]);
	m_ObjWeldJoint[3].SetChildLinkFrame(SE3(Vec3(0.5*(dim0[0] - 0.05), 0.5*(dim0[1] - 0.05), 0.5*(dim1[2] + 0.05))));

	///// set collision
	Vec3 colli_offset(m_collision_offset);
	for (int i = 0; i < m_numLink; i++)
	{
		m_ObjCollision[i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i].GetGeomInfo().SetDimension(dims[i] + colli_offset);
		m_ObjCollision[i].SetLocalFrame(SE3());
		m_ObjLink[i].AddCollision(&m_ObjCollision[i]);
	}
	
	this->SetSelfCollision(false);
	//this->SetBaseLink(&m_ObjLink[m_numLink]);
	this->SetBaseLink(&m_ObjLink[0]);
	this->SetBaseLinkType(srSystem::KINEMATIC);
}

workingObject::workingObject()
{
	AssembleModel();
}

workingObject::~workingObject()
{
}

void workingObject::AssembleModel()
{
	m_numLink = 2;
	m_numCollision = 10;
	m_numWeldJoint = 1;

	// dimension
	Vec3 dim(0.1, 0.07, 0.025); // change to exact value later

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

	m_ObjLink[0].GetGeomInfo().SetColor(0.3f, 0.4f, 0.3f); 
	//m_ObjLink[0].GetGeomInfo().SetDimension(dim);
	//m_ObjLink[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	//m_ObjLink[0].SetFrame(SE3(Vec3(0.0, 0.0, 0.5*0.05)));

	m_ObjLink[0].SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	m_ObjLink[0].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	m_ObjLink[0].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, SR_PI_HALF), Vec3(-0.01675, 0.0, -0.004)));
	m_ObjLink[0].GetGeomInfo().SetFileName("../../../workspace/robot/mh12_3ds/cover.3ds");
	//m_ObjLink[0].GetGeomInfo().SetFileName("../../../workspace/robot/busbar_3ds/busbar.3ds");
	
	//m_ObjWeldJoint[0].SetParentLink(&m_ObjLink[0]);
	//m_ObjWeldJoint[0].SetChildLink(&m_ObjLink[1]);
	//m_ObjWeldJoint[0].SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	//m_ObjWeldJoint[0].SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));

	// dummy link
	m_ObjLink[1].GetGeomInfo().SetDimension(Vec3(0.0, 0.0, 0.0));

	m_ObjWeldJoint[0].SetParentLink(&m_ObjLink[1]);
	//m_ObjWeldJoint[0].SetParentLinkFrame(SE3(Vec3(-0.01, 0.0, dim[2] - 0.0004))); // change to exact value later
	m_ObjWeldJoint[0].SetParentLinkFrame(); // change to exact value later
	m_ObjWeldJoint[0].SetChildLink(&m_ObjLink[0]);
	m_ObjWeldJoint[0].SetChildLinkFrame();

	///// set collision
	//m_ObjCollision[i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	//m_ObjCollision[i].GetGeomInfo().SetDimension(dim);
	//m_ObjCollision[i].SetLocalFrame(SE3());
	//m_ObjLink[i].AddCollision(&m_ObjCollision[i]);

	m_numCollision = 0;

	m_ObjLink[0].AddCollision(&m_ObjCollision[m_numCollision]);
	m_ObjCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.0645, 0.063, 0.004));
	m_ObjCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-0.01575, 0.0, 0.0016)));

	m_ObjLink[0].AddCollision(&m_ObjCollision[m_numCollision]);
	m_ObjCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.0165, 0.092, 0.007));
	m_ObjCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.02525, 0.0, 0.0021)));

	m_ObjLink[0].AddCollision(&m_ObjCollision[m_numCollision]);
	m_ObjCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.027, 0.014, 0.009));
	m_ObjCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-0.0535, 0.039, 0.0031)));

	m_ObjLink[0].AddCollision(&m_ObjCollision[m_numCollision]);
	m_ObjCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.027, 0.014, 0.009));
	m_ObjCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-0.0535, -0.039, 0.0031)));

	m_ObjLink[0].AddCollision(&m_ObjCollision[m_numCollision]);
	m_ObjCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.056, 0.0085, 0.005));
	m_ObjCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-0.0115, 0.03625, 0.0011)));

	m_ObjLink[0].AddCollision(&m_ObjCollision[m_numCollision]);
	m_ObjCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.056, 0.0085, 0.005));
	m_ObjCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-0.0115, -0.03625, 0.0011)));

	m_ObjLink[0].AddCollision(&m_ObjCollision[m_numCollision]);
	m_ObjCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.056, 0.00963769416, 0.0035));
	m_ObjCollision[m_numCollision++].SetLocalFrame(EulerZYX(Vec3(DEG2RAD(-22.38013505), 0.0, 0.0), Vec3(-0.0115, 0.02565255445, 0.00585)));

	m_ObjLink[0].AddCollision(&m_ObjCollision[m_numCollision]);
	m_ObjCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.058, 0.00963769416, 0.0035));
	m_ObjCollision[m_numCollision++].SetLocalFrame(EulerZYX(Vec3(DEG2RAD(22.38013505), 0.0, 0.0), Vec3(-0.0115, -0.02565255445, 0.00585)));

	m_ObjLink[0].AddCollision(&m_ObjCollision[m_numCollision]);
	m_ObjCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.028, 0.00855461015, 0.0035));
	m_ObjCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0025, 0.03622269493, 0.00585)));

	m_ObjLink[0].AddCollision(&m_ObjCollision[m_numCollision]);
	m_ObjCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.028, 0.00855461015, 0.0035));
	m_ObjCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0025, -0.03622269493, 0.00585)));


	
	this->SetSelfCollision(false);
	this->SetBaseLink(&m_ObjLink[1]);
	this->SetBaseLinkType(srSystem::KINEMATIC);
}

Barrier1::Barrier1(double collision_offset /*= 0.01*/)
{
	m_collision_offset = collision_offset;
	AssembleModel();
}
Barrier1::~Barrier1()
{

}
void Barrier1::AssembleModel()
{
	m_numLink = 3;
	m_numCollision = 2;
	m_numWeldJoint = 2;

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


	m_ObjLink[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjLink[1].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjLink[2].GetGeomInfo().SetShape(srGeometryInfo::BOX);

	double thickness = 0.08;
	m_ObjLink[0].GetGeomInfo().SetDimension(Vec3(0.0, 0.0, 0.0));
	m_ObjLink[1].GetGeomInfo().SetDimension(Vec3(thickness, thickness, 0.98));
	m_ObjLink[2].GetGeomInfo().SetDimension(Vec3(thickness, 1.1, thickness));

	m_ObjCollision[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[1].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[0].GetGeomInfo().SetDimension(Vec3(thickness + m_collision_offset, thickness + m_collision_offset, 0.98 + m_collision_offset));
	m_ObjCollision[1].GetGeomInfo().SetDimension(Vec3(thickness + m_collision_offset, 1.1 + m_collision_offset, thickness + m_collision_offset));
	m_ObjCollision[0].GetGeomInfo().SetLocalFrame(SE3());
	m_ObjCollision[1].GetGeomInfo().SetLocalFrame(SE3());

	m_ObjLink[1].AddCollision(&m_ObjCollision[0]);
	m_ObjLink[2].AddCollision(&m_ObjCollision[1]);

	m_ObjWeldJoint[0].SetParentLink(&m_ObjLink[0]);
	m_ObjWeldJoint[0].SetChildLink(&m_ObjLink[1]);
	m_ObjWeldJoint[0].SetParentLinkFrame(SE3(Vec3(0.74, -0.66, 0.49)));
	m_ObjWeldJoint[0].SetChildLinkFrame(SE3());

	m_ObjWeldJoint[1].SetParentLink(&m_ObjLink[0]);
	m_ObjWeldJoint[1].SetChildLink(&m_ObjLink[2]);
	m_ObjWeldJoint[1].SetParentLinkFrame(SE3(Vec3(0.74, -0.15, 0.94)));
	m_ObjWeldJoint[1].SetChildLinkFrame(SE3());

	this->SetSelfCollision(false);
	this->SetBaseLink(&m_ObjLink[0]);
	this->SetBaseLinkType(srSystem::KINEMATIC);
}

Barrier2::Barrier2(double collision_offset)
{
	m_collision_offset = collision_offset;
	AssembleModel();
}

Barrier2::~Barrier2()
{
}

void Barrier2::AssembleModel()
{
	m_numLink = 2;
	m_numCollision = 1;
	m_numWeldJoint = 1;
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

	m_ObjLink[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjLink[1].GetGeomInfo().SetShape(srGeometryInfo::BOX);

	m_ObjLink[0].GetGeomInfo().SetDimension(Vec3(0.0, 0.0, 0.0));
	Vec3 boxDim(0.5, 0.4, 0.98);
	m_ObjLink[1].GetGeomInfo().SetDimension(boxDim);

	m_ObjCollision[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[0].GetGeomInfo().SetDimension(Vec3(boxDim[0] + m_collision_offset, boxDim[1] + m_collision_offset, boxDim[2] + m_collision_offset));
	m_ObjCollision[0].GetGeomInfo().SetLocalFrame(SE3());
	
	m_ObjLink[1].AddCollision(&m_ObjCollision[0]);
	
	m_ObjWeldJoint[0].SetParentLink(&m_ObjLink[0]);
	m_ObjWeldJoint[0].SetChildLink(&m_ObjLink[1]);
	m_ObjWeldJoint[0].SetParentLinkFrame(SE3(Vec3(-0.3 - 0.5*boxDim[0], 0.45 + 0.5*boxDim[1], 0.5*boxDim[2])));
	m_ObjWeldJoint[0].SetChildLinkFrame(SE3());

	this->SetSelfCollision(false);
	this->SetBaseLink(&m_ObjLink[0]);
	this->SetBaseLinkType(srSystem::KINEMATIC);
}


TableRetarget::TableRetarget()
{
	AssembleModel();
}

TableRetarget::~TableRetarget()
{
}

void TableRetarget::AssembleModel()
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
	Vec3 dim0(1.0, 1.5, 0.05);
	Vec3 dim1(0.05, 0.05, 0.73-0.05);

	vector<Vec3> dims(m_numLink);
	dims[0] = dim0;
	dims[1] = dim1;
	dims[2] = dim1;
	dims[3] = dim1;
	dims[4] = dim1;

	for (int i = 0; i < m_numLink; i++)
	{
		m_ObjLink[i].GetGeomInfo().SetColor(0.3f, 0.3f, 0.3f);
		m_ObjLink[i].GetGeomInfo().SetDimension(dims[i]);
		m_ObjLink[i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	}

	// xy plane
	m_ObjWeldJoint[0].SetParentLink(&m_ObjLink[0]);
	m_ObjWeldJoint[0].SetParentLinkFrame();
	m_ObjWeldJoint[0].SetChildLink(&m_ObjLink[1]);
	m_ObjWeldJoint[0].SetChildLinkFrame(SE3(Vec3(-0.5*(1.0- 0.05), -0.5*(1.5 - 0.05), 0.5*(0.73 - 0.05 + 0.05))));

	m_ObjWeldJoint[1].SetParentLink(&m_ObjLink[0]);
	m_ObjWeldJoint[1].SetParentLinkFrame();
	m_ObjWeldJoint[1].SetChildLink(&m_ObjLink[2]);
	m_ObjWeldJoint[1].SetChildLinkFrame(SE3(Vec3(-0.5*(1.0 - 0.05), 0.5*(1.5- 0.05), 0.5*(0.73 - 0.05 + 0.05))));
	// yz plane
	m_ObjWeldJoint[2].SetParentLink(&m_ObjLink[0]);
	m_ObjWeldJoint[2].SetParentLinkFrame();
	m_ObjWeldJoint[2].SetChildLink(&m_ObjLink[3]);
	m_ObjWeldJoint[2].SetChildLinkFrame(SE3(Vec3(0.5*(1.0 - 0.05), -0.5*(1.5- 0.05), 0.5*(0.73 - 0.05 + 0.05))));

	m_ObjWeldJoint[3].SetParentLink(&m_ObjLink[0]);
	m_ObjWeldJoint[3].SetParentLinkFrame();
	m_ObjWeldJoint[3].SetChildLink(&m_ObjLink[4]);
	m_ObjWeldJoint[3].SetChildLinkFrame(SE3(Vec3(0.5*(1.0 - 0.05), 0.5*(1.5 - 0.05), 0.5*(0.73 - 0.05 + 0.05))));

	///// set collision
	for (int i = 0; i < m_numLink; i++)
	{
		m_ObjCollision[i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i].GetGeomInfo().SetDimension(dims[i]);
		m_ObjCollision[i].SetLocalFrame(SE3());
		m_ObjLink[i].AddCollision(&m_ObjCollision[i]);
	}

	this->SetSelfCollision(false);
	//this->SetBaseLink(&m_ObjLink[m_numLink]);
	this->SetBaseLink(&m_ObjLink[0]);
	this->SetBaseLinkType(srSystem::KINEMATIC);
}


BlueMaleConnector::BlueMaleConnector()
{
	AssembleModel();
}

BlueMaleConnector::~BlueMaleConnector()
{
}

void BlueMaleConnector::AssembleModel()
{
	// TO DO: add collision
	m_numLink = 2;
	m_numWeldJoint = 1;
	m_numCollision = 1;

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

	m_ObjLink[0].GetGeomInfo().SetColor(0.3f, 0.4f, 0.3f);

	m_ObjLink[0].SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	m_ObjLink[0].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	//m_ObjLink[0].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, SR_PI_HALF), Vec3(-0.01675, 0.0, -0.004)));
	//m_ObjLink[0].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(-0.10007, 0.09307, 0.0888)));
	m_ObjLink[0].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, SR_PI_HALF), Vec3(-0.10007, 0.09307, 0.0888))); // SetLocalFrame --> Orientation: srLib에서 solidworks로 가는 SO3, Position: srLib frame의 orientation 기준으로 원점 이동하고 싶은 만큼 입력
	m_ObjLink[0].GetGeomInfo().SetFileName("../../../workspace/robot/retarget_object/blue_male_connector.3ds");
	

	// dummy link
	m_ObjLink[1].GetGeomInfo().SetDimension(Vec3(0.0, 0.0, 0.0));

	m_ObjWeldJoint[0].SetParentLink(&m_ObjLink[1]);
	//m_ObjWeldJoint[0].SetParentLinkFrame(SE3(Vec3(-0.01, 0.0, dim[2] - 0.0004))); // change to exact value later
	m_ObjWeldJoint[0].SetParentLinkFrame(); // change to exact value later
	m_ObjWeldJoint[0].SetChildLink(&m_ObjLink[0]);
	m_ObjWeldJoint[0].SetChildLinkFrame();

	m_ObjLink[0].AddCollision(&m_ObjCollision[0]);
	m_ObjCollision[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[0].GetGeomInfo().SetDimension(Vec3(0.050, 0.050, 0.180));
	m_ObjCollision[0].SetLocalFrame(SE3(Vec3(0.0, 0.0, 0.180/2.0)));

	this->SetSelfCollision(false);
	this->SetBaseLink(&m_ObjLink[1]);
	this->SetBaseLinkType(srSystem::KINEMATIC);

}


RedMaleConnector::RedMaleConnector()
{
	AssembleModel();
}

RedMaleConnector::~RedMaleConnector()
{
}

void RedMaleConnector::AssembleModel()
{
	m_numLink = 2;
	m_numWeldJoint = 1;
	m_numCollision = 1;

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

	m_ObjLink[0].GetGeomInfo().SetColor(0.3f, 0.4f, 0.3f);

	m_ObjLink[0].SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	m_ObjLink[0].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	//m_ObjLink[0].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, SR_PI_HALF), Vec3(-0.01675, 0.0, -0.004)));
	//m_ObjLink[0].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(-0.10007, 0.09307, 0.0888)));
	m_ObjLink[0].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, SR_PI_HALF), Vec3(-0.10007, 0.09307, 0.07995))); // SetLocalFrame --> Orientation: srLib에서 solidworks로 가는 SO3, Position: srLib frame의 orientation 기준으로 원점 이동하고 싶은 만큼 입력
	m_ObjLink[0].GetGeomInfo().SetFileName("../../../workspace/robot/retarget_object/red_male_connector.3ds");


	// dummy link
	m_ObjLink[1].GetGeomInfo().SetDimension(Vec3(0.0, 0.0, 0.0));

	m_ObjWeldJoint[0].SetParentLink(&m_ObjLink[1]);
	//m_ObjWeldJoint[0].SetParentLinkFrame(SE3(Vec3(-0.01, 0.0, dim[2] - 0.0004))); // change to exact value later
	m_ObjWeldJoint[0].SetParentLinkFrame(); // change to exact value later
	m_ObjWeldJoint[0].SetChildLink(&m_ObjLink[0]);
	m_ObjWeldJoint[0].SetChildLinkFrame();



	m_ObjLink[0].AddCollision(&m_ObjCollision[0]);
	m_ObjCollision[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[0].GetGeomInfo().SetDimension(Vec3(0.061, 0.061, 0.180));
	m_ObjCollision[0].SetLocalFrame(SE3(Vec3(0.0, 0.0, 0.180 / 2.0)));

	this->SetSelfCollision(false);
	this->SetBaseLink(&m_ObjLink[1]);
	this->SetBaseLinkType(srSystem::KINEMATIC);

}


BlueFemaleConnector::BlueFemaleConnector()
{
	AssembleModel();
}

BlueFemaleConnector::~BlueFemaleConnector()
{
}

void BlueFemaleConnector::AssembleModel()
{
	m_numLink = 2;
	m_numWeldJoint = 1;
	m_numCollision = 2;

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

	m_ObjLink[0].GetGeomInfo().SetColor(0.3f, 0.4f, 0.3f);

	m_ObjLink[0].SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	m_ObjLink[0].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	//m_ObjLink[0].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, SR_PI_HALF), Vec3(-0.01675, 0.0, -0.004)));
	//m_ObjLink[0].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(-0.10007, 0.09307, 0.0888)));
	m_ObjLink[0].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, SR_PI_HALF), Vec3(0.0,0.0, 0.018))); // SetLocalFrame --> Orientation: srLib에서 solidworks로 가는 SO3, Position: srLib frame의 orientation 기준으로 원점 이동하고 싶은 만큼 입력
	m_ObjLink[0].GetGeomInfo().SetFileName("../../../workspace/robot/retarget_object/blue_female_connector.3ds");


	// dummy link
	m_ObjLink[1].GetGeomInfo().SetDimension(Vec3(0.0, 0.0, 0.0));

	m_ObjWeldJoint[0].SetParentLink(&m_ObjLink[1]);
	//m_ObjWeldJoint[0].SetParentLinkFrame(SE3(Vec3(-0.01, 0.0, dim[2] - 0.0004))); // change to exact value later
	m_ObjWeldJoint[0].SetParentLinkFrame(); // change to exact value later
	m_ObjWeldJoint[0].SetChildLink(&m_ObjLink[0]);
	m_ObjWeldJoint[0].SetChildLinkFrame();



	m_ObjLink[0].AddCollision(&m_ObjCollision[0]);
	m_ObjCollision[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[0].GetGeomInfo().SetDimension(Vec3(0.075, 0.075, 0.008));
	m_ObjCollision[0].SetLocalFrame(SE3(Vec3(0.0, 0.0, 0.022 )));

	m_ObjLink[0].AddCollision(&m_ObjCollision[1]);
	m_ObjCollision[1].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[1].GetGeomInfo().SetDimension(Vec3(0.055, 0.055, 0.060));
	m_ObjCollision[1].SetLocalFrame(SE3(Vec3(0.0, 0.0, 0.060 / 2.0)));

	this->SetSelfCollision(false);
	this->SetBaseLink(&m_ObjLink[1]);
	this->SetBaseLinkType(srSystem::KINEMATIC);

}


RedFemaleConnector::RedFemaleConnector()
{
	AssembleModel();
}

RedFemaleConnector::~RedFemaleConnector()
{
}

void RedFemaleConnector::AssembleModel()
{
	m_numLink = 2;
	m_numWeldJoint = 1;
	m_numCollision = 2;

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

	m_ObjLink[0].GetGeomInfo().SetColor(0.3f, 0.4f, 0.3f);

	m_ObjLink[0].SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	m_ObjLink[0].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	//m_ObjLink[0].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, SR_PI_HALF), Vec3(-0.01675, 0.0, -0.004)));
	//m_ObjLink[0].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(-0.10007, 0.09307, 0.0888)));
	m_ObjLink[0].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0,0.0,SR_PI_HALF), Vec3(0.0,0.0, 0.0))); // SetLocalFrame --> Orientation: SolidworkssrLib에서 solidworks로 가는 SO3, Position: srLib frame의 orientation 기준으로 원점 이동하고 싶은 만큼 입력
	m_ObjLink[0].GetGeomInfo().SetFileName("../../../workspace/robot/retarget_object/red_female_connector.3ds");


	// dummy link
	m_ObjLink[1].GetGeomInfo().SetDimension(Vec3(0.0, 0.0, 0.0));

	m_ObjWeldJoint[0].SetParentLink(&m_ObjLink[1]);
	//m_ObjWeldJoint[0].SetParentLinkFrame(SE3(Vec3(-0.01, 0.0, dim[2] - 0.0004))); // change to exact value later
	m_ObjWeldJoint[0].SetParentLinkFrame(); // change to exact value later
	m_ObjWeldJoint[0].SetChildLink(&m_ObjLink[0]);
	m_ObjWeldJoint[0].SetChildLinkFrame();



	m_ObjLink[0].AddCollision(&m_ObjCollision[0]);
	m_ObjCollision[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[0].GetGeomInfo().SetDimension(Vec3(0.092, 0.092, 0.008));
	m_ObjCollision[0].SetLocalFrame(SE3(Vec3(0.0, 0.0, 0.004 )));

	m_ObjLink[0].AddCollision(&m_ObjCollision[1]);
	m_ObjCollision[1].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[1].GetGeomInfo().SetDimension(Vec3(0.065, 0.070, 0.084));
	//m_ObjCollision[1].SetLocalFrame(SE3(Vec3(0.0, 0.0, 0.060 / 2.0)));
	m_ObjCollision[1].SetLocalFrame(EulerZYX(Vec3(0.0,0.0,SR_PI/9), Vec3(0.0,-0.001,0.001)));

	this->SetSelfCollision(false);
	this->SetBaseLink(&m_ObjLink[1]);
	this->SetBaseLinkType(srSystem::KINEMATIC);

}
