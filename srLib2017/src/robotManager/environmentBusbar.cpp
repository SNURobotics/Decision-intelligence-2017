#include "environmentBusbar.h"
#include "makeSpecialCol.h"



TableBusBar_HYU::TableBusBar_HYU(SE3 tableSurfaceCenter,Vec3 tableDim)
{
	m_tableSurfaceCenter = tableSurfaceCenter;
	m_tableDim = tableDim;
	AssembleModel();
	
}

TableBusBar_HYU::~TableBusBar_HYU()
{
}


void TableBusBar_HYU::AssembleModel()
{
	m_numLink = 6;
	m_numCollision = 5;
	m_numWeldJoint = 5;

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

	Vec3 tableCenter = m_tableSurfaceCenter.GetPosition();
	double tableHeight = tableCenter[2];

	//Vec3 tableDim(0.6, 1.2, 0.1);
	//Vec3 tableCenter(1.0, 0.0, tableHeight - tableDim[2] / 2);//0.8);
	Vec3 legDim(0.2, 0.1, tableHeight - m_tableDim[2]);//tableCenter[2]-tableDim[2]/2);

	// dummy link for table
	m_ObjLink[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjLink[0].GetGeomInfo().SetDimension(Vec3(0.0,0.0,0.0));
	m_ObjLink[0].GetGeomInfo().SetColor(0.15f, 0.15f, 0.15f, 1.0f);

	// link for table surface
	m_ObjLink[1].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjLink[1].GetGeomInfo().SetDimension(m_tableDim);
	m_ObjLink[1].GetGeomInfo().SetColor(0.15f, 0.15f, 0.15f, 1.0f);
	for (int i = 2; i < m_numLink; i++)
	{
		m_ObjLink[i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjLink[i].GetGeomInfo().SetDimension(legDim);
		m_ObjLink[i].GetGeomInfo().SetColor(0.15f, 0.15f, 0.15f, 1.0f);
	}

	m_ObjWeldJoint[0].SetActType(srJoint::PASSIVE);
	m_ObjWeldJoint[0].SetParentLink(&m_ObjLink[0]);
	m_ObjWeldJoint[0].SetChildLink(&m_ObjLink[1]);
	m_ObjWeldJoint[0].SetParentLinkFrame(SE3());
	m_ObjWeldJoint[0].SetChildLinkFrame(SE3(Vec3(0.0, 0.0, 0.5*m_tableDim[2])));

	for (int i = 1; i < m_numWeldJoint; i++)
	{
		m_ObjWeldJoint[i].SetActType(srJoint::PASSIVE);
		m_ObjWeldJoint[i].SetParentLink(&m_ObjLink[0]);
		m_ObjWeldJoint[i].SetChildLink(&m_ObjLink[i + 1]);
		m_ObjWeldJoint[i].SetParentLinkFrame(SE3());
	}
	Vec3 tmpVec((m_tableDim[0] - legDim[0]) / 2, (m_tableDim[1] - legDim[1])/2, (m_tableDim[2] + legDim[2]) / 2);
	m_ObjWeldJoint[1].SetChildLinkFrame(SE3(tmpVec));
	m_ObjWeldJoint[2].SetChildLinkFrame(SE3(Vec3(-tmpVec[0], tmpVec[1], tmpVec[2])));
	m_ObjWeldJoint[3].SetChildLinkFrame(SE3(Vec3(tmpVec[0], -tmpVec[1], tmpVec[2])));
	m_ObjWeldJoint[4].SetChildLinkFrame(SE3(Vec3(-tmpVec[0], -tmpVec[1], tmpVec[2])));


	m_ObjCollision[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[0].GetGeomInfo().SetDimension(m_tableDim);
	m_ObjCollision[0].SetLocalFrame(SE3());
	m_ObjLink[1].AddCollision(&m_ObjCollision[0]);
	for (int i = 1; i < m_numCollision; i++)
	{
		m_ObjCollision[i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i].GetGeomInfo().SetDimension(legDim);
		m_ObjCollision[i].SetLocalFrame(SE3());
		m_ObjLink[i + 1].AddCollision(&m_ObjCollision[i]);
	}

	m_ObjLink[0].SetFrame(m_tableSurfaceCenter);

	this->SetBaseLink(&m_ObjLink[0]);
	this->SetBaseLinkType(srSystem::KINEMATIC);
	this->SetSelfCollision(false);
}


Base_HYU::Base_HYU()
{
	AssembleModel();
}

Base_HYU::~Base_HYU()
{
}

void Base_HYU::AssembleModel()
{
	m_numLink = 2;
	m_numCollision = 9; // to be modifed
	
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

	m_ObjLink[0].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	m_ObjLink[0].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF),Vec3(0.0, 0.0, 0.0)));
	m_ObjLink[0].GetGeomInfo().SetFileName("../../../workspace/robot/busbar_3ds/base.3ds");


	m_ObjLink[0].GetGeomInfo().SetColor(0.35f, 0.35f, 0.35f, 1.0f);

	// Å« ±¸¸Û
	vector<pair<Vec3, SE3>> boxGeomInfo = makeRectangleHole(SE3(Vec3(0.0, 0.0, 0.015)), Vec3(0.391, 0.3, 0.03), Vec3(0.0, -0.03288, 0.0), Vec3(0.3659, 0.2228, 0.03));
	for (unsigned int i = 0; i < boxGeomInfo.size(); i++)
	{
		m_ObjCollision[i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i].GetGeomInfo().SetDimension(boxGeomInfo[i].first);
		m_ObjCollision[i].SetLocalFrame(boxGeomInfo[i].second);
		m_ObjLink[0].AddCollision(&m_ObjCollision[i]);
	}

	// 2Ãþ
	m_ObjCollision[4].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[4].GetGeomInfo().SetDimension(Vec3(0.391, 0.174, 0.02));
	m_ObjCollision[4].SetLocalFrame(SE3(Vec3(0.0, 0.5*(0.3 - 0.174), 0.01)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[4]);

	// 1Ãþ
	m_ObjCollision[5].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[5].GetGeomInfo().SetDimension(Vec3(0.391, 0.3, 0.01));
	m_ObjCollision[5].SetLocalFrame(SE3(Vec3(0.0, 0.0, 0.005)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[5]);

	// Ä­¸·ÀÌ
	m_ObjCollision[6].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[6].GetGeomInfo().SetDimension(Vec3(0.01455, 0.3, 0.03));
	m_ObjCollision[6].SetLocalFrame(SE3(Vec3(0.0, 0.0, 0.015)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[6]);

	m_ObjCollision[7].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[7].GetGeomInfo().SetDimension(Vec3(0.0147, 0.3, 0.03));
	m_ObjCollision[7].SetLocalFrame(SE3(Vec3(-0.10245+0.0147*0.5, 0.0, 0.015)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[7]);

	m_ObjCollision[8].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[8].GetGeomInfo().SetDimension(Vec3(0.0147, 0.3, 0.03));
	m_ObjCollision[8].SetLocalFrame(SE3(Vec3(+0.10245 - 0.0147*0.5, 0.0, 0.015)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[8]);

	m_numWeldJoint = 1;
	m_ObjWeldJoint.resize(m_numWeldJoint);
	m_ObjLink[1].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjLink[1].GetGeomInfo().SetDimension(Vec3(0.0, 0.0, 0.0));
	m_ObjWeldJoint[0].SetParentLink(&m_ObjLink[1]);
	m_ObjWeldJoint[0].SetParentLinkFrame(SE3());
	m_ObjWeldJoint[0].SetChildLink(&m_ObjLink[0]);
	m_ObjWeldJoint[0].SetChildLinkFrame(SE3());

	this->SetBaseLink(&m_ObjLink[1]);
	this->SetBaseLinkType(srSystem::KINEMATIC);
	this->SetSelfCollision(true);
}

Jig_HYU::Jig_HYU()
{
	AssembleModel();
}

Jig_HYU::~Jig_HYU()
{
}

void Jig_HYU::AssembleModel()
{


	m_numLink = 1;
	m_numCollision = 8; // to be modifed

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
	m_ObjLink[0].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	m_ObjLink[0].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0, 0.0)));
	m_ObjLink[0].GetGeomInfo().SetFileName("../../../workspace/robot/busbar_3ds/jig.3ds");

	m_ObjLink[0].GetGeomInfo().SetColor(0.2f, 0.2f, 0.2f, 1.0f);


	vector<pair<Vec3, SE3>> boxGeomInfo = makeRectangleHole(SE3(Vec3(0.00005, -0.0515, 0.005)), Vec3(0.0795, 0.1188, 0.01), Vec3(0.00001, -0.0124, 0.0), Vec3(0.047, 0.052, 0.01));
	for (unsigned int i = 0; i < boxGeomInfo.size(); i++)
	{
		m_ObjCollision[i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i].GetGeomInfo().SetDimension(boxGeomInfo[i].first);
		m_ObjCollision[i].SetLocalFrame(boxGeomInfo[i].second);
		m_ObjLink[0].AddCollision(&m_ObjCollision[i]);
	}

	boxGeomInfo = makeRectangleHole(SE3(Vec3(0.0, 0.0515, 0.015)), Vec3(0.0795, 0.1195, 0.01), Vec3(0.0, -0.00275, 0.0), Vec3(0.047, 0.052, 0.01));
	for (unsigned int i = 0; i < boxGeomInfo.size(); i++)
	{
		m_ObjCollision[i + 4].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i + 4].GetGeomInfo().SetDimension(boxGeomInfo[i].first);
		m_ObjCollision[i + 4].SetLocalFrame(boxGeomInfo[i].second);
		m_ObjLink[0].AddCollision(&m_ObjCollision[i + 4]);
	}


	this->SetBaseLink(&m_ObjLink[0]);
	this->SetBaseLinkType(srSystem::KINEMATIC);
	this->SetSelfCollision(false);
}

BusBar_HYU::BusBar_HYU()
{
	AssembleModel();
}

BusBar_HYU::~BusBar_HYU()
{
}

void BusBar_HYU::AssembleModel()
{


	m_numLink = 1;
	m_numCollision = 2; // to be modifed


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

	m_ObjLink[0].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	m_ObjLink[0].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0, 0.0)));
	m_ObjLink[0].GetGeomInfo().SetFileName("../../../workspace/robot/busbar_3ds/busbar.3ds");

	m_ObjCollision[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[0].GetGeomInfo().SetDimension(Vec3(0.045, 0.05, 0.01)); //Vec3(0.045, 0.05, 0.01)
	m_ObjCollision[0].SetLocalFrame(SE3(Vec3(0.0, 0.0, 0.005)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[0]);
	
	m_ObjCollision[1].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[1].GetGeomInfo().SetDimension(Vec3(0.01, 0.05, 0.038)); //Vec3(0.01, 0.05, 0.038)
	m_ObjCollision[1].SetLocalFrame(SE3(Vec3(0.0, 0.0, 0.01 + 0.019)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[1]);

	
	m_ObjLink[0].GetGeomInfo().SetColor(0.3f, 0.3f, 0.3f, 1.0f);


	double m = 0.29718;
	Vec3 r = Vec3(0.0, 0.0, 15.95e-3);
	Inertia busbarInertia(84440.59e-9, 121063.91e-9, 90554.97e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, m);
	m_ObjLink[0].SetInertia(busbarInertia.Transform(SE3(-r)));

	this->SetBaseLink(&m_ObjLink[0]);
	this->SetBaseLinkType(srSystem::DYNAMIC);
	this->SetSelfCollision(false);
}

srLink * BusBar_HYU::busbarLink()
{
	srLink* temp = new srLink;
	srCollision* tempColli = new srCollision;
	srCollision* tempColli2 = new srCollision;
	temp->GetGeomInfo().SetShape(srGeometryInfo::TDS);
	temp->GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0, 0.0)));
	temp->GetGeomInfo().SetFileName("../../../workspace/robot/busbar_3ds/busbar.3ds");

	tempColli->GetGeomInfo().SetShape(srGeometryInfo::BOX);
	tempColli->GetGeomInfo().SetDimension(Vec3(0.045, 0.05, 0.01)); //Vec3(0.045, 0.05, 0.01)
	tempColli->SetLocalFrame(SE3(Vec3(0.0, 0.0, 0.005)));
	temp->AddCollision(tempColli);

	tempColli2->GetGeomInfo().SetShape(srGeometryInfo::BOX);
	tempColli2->GetGeomInfo().SetDimension(Vec3(0.01, 0.05, 0.038)); //Vec3(0.01, 0.05, 0.038)
	tempColli2->SetLocalFrame(SE3(Vec3(0.0, 0.0, 0.01 + 0.019)));
	temp->AddCollision(tempColli2);


	temp->GetGeomInfo().SetColor(0.3f, 0.3f, 0.3f, 1.0f);
	return temp;
}


