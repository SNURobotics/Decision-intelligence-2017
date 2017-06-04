
#include "makeSpecialCol.h"
#include "environment_QBtech.h"



TableBusbar::TableBusbar(SE3 tableSurfaceCenter, Vec3 tableDim)
{
	m_tableSurfaceCenter = tableSurfaceCenter;
	m_tableDim = tableDim;
	AssembleModel();

}

TableBusbar::~TableBusbar()
{
}


void TableBusbar::AssembleModel()
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
	m_ObjLink[0].GetGeomInfo().SetDimension(Vec3(0.0, 0.0, 0.0));
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
	Vec3 tmpVec((m_tableDim[0] - legDim[0]) / 2, (m_tableDim[1] - legDim[1]) / 2, (m_tableDim[2] + legDim[2]) / 2);
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


Base::Base()
{
	AssembleModel();
}

Base::~Base()
{
}

void Base::AssembleModel()
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
	m_ObjLink[0].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0, 0.0)));
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
	m_ObjCollision[7].SetLocalFrame(SE3(Vec3(-0.10245 + 0.0147*0.5, 0.0, 0.015)));
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

Jig::Jig()
{
	AssembleModel();
}

Jig::~Jig()
{
}

void Jig::AssembleModel()
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


	vector<pair<Vec3, SE3>> boxGeomInfo = makeRectangleHole(SE3(Vec3(0.0, -0.0515, 0.005)), Vec3(0.0795, 0.1188, 0.01), Vec3(0.0, -0.0124, 0.0), Vec3(0.047, 0.052, 0.01));
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

BusBar::BusBar()
{
	AssembleModel();
}

BusBar::~BusBar()
{
}

void BusBar::AssembleModel()
{


	m_numLink = 1;
	m_numCollision = 9; // 2 holes and one collision


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
	double m = 0.29718;
	Vec3 r = Vec3(0.0, 0.0, 15.95e-3);
	Inertia busbarInertia(84440.59e-9, 121063.91e-9, 90554.97e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, m);
	m_ObjLink[0].SetInertia(busbarInertia.Transform(SE3(-r)));
	// Busbar hole

	vector<pair<Vec3, SE3>> boxGeomInfo = makeRectangleHole(SE3(Vec3(-0.01125, 0.0, 0.005)), Vec3(0.0225, 0.05, 0.01), Vec3(-(0.01125 - 0.0075), 0.0, 0.0), Vec3(0.0085, 0.0085, 0.01));


	for (unsigned int i = 0; i < boxGeomInfo.size(); i++)
	{
		m_ObjCollision[i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i].GetGeomInfo().SetDimension(boxGeomInfo[i].first);
		m_ObjCollision[i].SetLocalFrame(boxGeomInfo[i].second);
		m_ObjLink[0].AddCollision(&m_ObjCollision[i]);
	}



	boxGeomInfo = makeRectangleHole(SE3(Vec3(0.01125, 0.0, 0.005)), Vec3(0.0225, 0.05, 0.01), Vec3(0.01125 - 0.0075, 0.0, 0.0), Vec3(0.0085, 0.0085, 0.01));

	for (unsigned int i = 0; i < boxGeomInfo.size(); i++)
	{
		m_ObjCollision[i+4].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i+4].GetGeomInfo().SetDimension(boxGeomInfo[i].first);
		m_ObjCollision[i+4].SetLocalFrame(boxGeomInfo[i].second);
		m_ObjLink[0].AddCollision(&m_ObjCollision[i+4]);
	}



	m_ObjCollision[8].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[8].GetGeomInfo().SetDimension(Vec3(0.01, 0.05, 0.038)); //Vec3(0.01, 0.05, 0.038)
	m_ObjCollision[8].SetLocalFrame(SE3(Vec3(0.0, 0.0, 0.01 + 0.019)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[8]);


	m_ObjLink[0].GetGeomInfo().SetColor(0.25f, 0.25f, 0.25f, 1.0f);




	this->SetBaseLink(&m_ObjLink[0]);
	this->SetBaseLinkType(srSystem::DYNAMIC);
	this->SetSelfCollision(false);
}

UpperFrame::UpperFrame()
{
	AssembleModel();
}

UpperFrame::~UpperFrame()
{
}

void UpperFrame::AssembleModel()
{
	m_numLink = 1;
	m_numCollision = 12;

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
	m_ObjLink[0].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0 ), Vec3(0.0, 0.0, 0.0)));
	m_ObjLink[0].GetGeomInfo().SetFileName("../../../workspace/robot/QBtech_env_modeling/Upper_frame.3ds");

	// Big base
	m_ObjCollision[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[0].GetGeomInfo().SetDimension(Vec3(0.391, 0.3, 0.2));
	m_ObjCollision[0].SetLocalFrame(SE3(Vec3(0.0, 0.0, -0.1)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[0]);

	// small slots
	// from the right corner...
	// slot 1
	m_ObjCollision[1].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[1].GetGeomInfo().SetDimension(Vec3(0.004, 0.083, 0.01));
	m_ObjCollision[1].SetLocalFrame(SE3(Vec3(-0.1805+0.002, 0.133-.083*0.5, 0.005)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[1]);

	// slot 2
	m_ObjCollision[2].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[2].GetGeomInfo().SetDimension(Vec3(0.003, 0.083, 0.01));
	m_ObjCollision[2].SetLocalFrame(SE3(Vec3(-0.108 + 0.0015, 0.133- 0.083*0.5, 0.005)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[2]);

	// slot 3
	m_ObjCollision[3].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[3].GetGeomInfo().SetDimension(Vec3(0.003, 0.083, 0.01));
	m_ObjCollision[3].SetLocalFrame(SE3(Vec3(-0.083 + 0.0015, 0.133 - 0.083*0.5, 0.005)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[3]);

	// slot 4
	m_ObjCollision[4].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[4].GetGeomInfo().SetDimension(Vec3(0.003, 0.083, 0.01));
	m_ObjCollision[4].SetLocalFrame(SE3(Vec3(-0.015 + 0.0015, 0.133 - 0.083*0.5, 0.005)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[4]);

	// slot 5
	m_ObjCollision[5].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[5].GetGeomInfo().SetDimension(Vec3(0.003, 0.083, 0.01));
	m_ObjCollision[5].SetLocalFrame(SE3(Vec3(0.015 - 0.0015, 0.133 - 0.083*0.5, 0.005)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[5]);


	// slot 6
	m_ObjCollision[6].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[6].GetGeomInfo().SetDimension(Vec3(0.003, 0.083, 0.01));
	m_ObjCollision[6].SetLocalFrame(SE3(Vec3(0.083 - 0.0015, 0.133 - 0.083*0.5, 0.005)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[6]);

	// slot 7
	m_ObjCollision[7].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[7].GetGeomInfo().SetDimension(Vec3(0.003, 0.083, 0.01));
	m_ObjCollision[7].SetLocalFrame(SE3(Vec3(0.108 - 0.0015, 0.133 - 0.083*0.5, 0.005)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[7]);

	// slot 8
	m_ObjCollision[8].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[8].GetGeomInfo().SetDimension(Vec3(0.004, 0.083, 0.01));
	m_ObjCollision[8].SetLocalFrame(SE3(Vec3(0.1805 - 0.002, 0.133 - 0.083*0.5, 0.005)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[8]);

	// middle size slots
	// slot 9
	m_ObjCollision[9].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[9].GetGeomInfo().SetDimension(Vec3(0.005, 0.151, 0.01));
	m_ObjCollision[9].SetLocalFrame(SE3(Vec3(-0.0965 + 0.0025, -0.101+0.151*0.5, 0.005)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[9]);

	// slot 10
	m_ObjCollision[10].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[10].GetGeomInfo().SetDimension(Vec3(0.005, 0.151, 0.01));
	m_ObjCollision[10].SetLocalFrame(SE3(Vec3(0.0, -0.101 + 0.151*0.5, 0.005)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[10]);

	// slot 9
	m_ObjCollision[11].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[11].GetGeomInfo().SetDimension(Vec3(0.005, 0.151, 0.01));
	m_ObjCollision[11].SetLocalFrame(SE3(Vec3(0.0965 - 0.0025, -0.101 + 0.151*0.5, 0.005)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[11]);

	m_ObjLink[0].GetGeomInfo().SetColor(0.2f, 0.2f, 0.2f, 1.0f);


	this->SetBaseLink(&m_ObjLink[0]);
	this->SetBaseLinkType(srSystem::KINEMATIC);
	this->SetSelfCollision(false);

}

Jig_QB::Jig_QB()
{
	AssembleModel();
}

Jig_QB::~Jig_QB()
{
}

void Jig_QB::AssembleModel()
{
	m_numLink = 1;
	m_numCollision =5+4*8+24;
	int collisionCount = 0;
	int cnt_tmp = 0;
	vector<pair<Vec3, SE3>> holeGeomInfo;

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
	m_ObjLink[0].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	m_ObjLink[0].GetGeomInfo().SetFileName("../../../workspace/robot/QBtech_env_modeling/JIG.3ds");


	m_ObjLink[0].GetGeomInfo().SetColor(0.3f, 0.3f, 0.3f, 1.0f);

	// Collision from left corner
	// base part 1
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.240, 0.038, 0.02));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(0.0,-0.205+0.038*0.5, 0.01)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;
	// hole infos

	Vec3 rectangleDim(0.12, 0.049, 0.02);
	Vec3 holeDim(0.052, 0.047, 0.02);
	Vec3 upperHoleCenterfromRC(0.03, 0.0, 0.0);
	Vec3 lowerHoleCenterfromRC(-0.006, 0.0, 0.0);
	SE3 rectangleCenter;
	
	
	
	// base part 2
	rectangleCenter = SE3(Vec3(-0.06, -0.205 + 0.038 + 0.049*0.5, 0.01));
	holeGeomInfo = makeRectangleHole(rectangleCenter, rectangleDim, upperHoleCenterfromRC, holeDim);
	cnt_tmp = 0;
	for (unsigned int i = 0; i < holeGeomInfo.size(); i++)
	{
		m_ObjCollision[i+ collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i+ collisionCount].GetGeomInfo().SetDimension(holeGeomInfo[i].first);
		m_ObjCollision[i+ collisionCount].SetLocalFrame(holeGeomInfo[i].second);
		m_ObjLink[0].AddCollision(&m_ObjCollision[i+ collisionCount]);
		cnt_tmp++;
	}
	collisionCount += cnt_tmp;

	// base part 3
	rectangleCenter = SE3(Vec3(0.06, -0.205 + 0.038 + 0.049*0.5, 0.01));
	holeGeomInfo = makeRectangleHole(rectangleCenter, rectangleDim, lowerHoleCenterfromRC, holeDim);
	cnt_tmp = 0;
	for (unsigned int i = 0; i < holeGeomInfo.size(); i++)
	{
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetDimension(holeGeomInfo[i].first);
		m_ObjCollision[i + collisionCount].SetLocalFrame(holeGeomInfo[i].second);
		m_ObjLink[0].AddCollision(&m_ObjCollision[i + collisionCount]);
		cnt_tmp++;
	}
	collisionCount += cnt_tmp;

	// base part 4
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.240, 0.046, 0.02));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(0.0, -0.205 + 0.038+0.049+0.046*0.5, 0.01)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;

	// base part 5
	rectangleCenter = SE3(Vec3(-0.06, -0.205 + 0.038 + 0.049+0.046+0.049*0.5, 0.01));
	holeGeomInfo = makeRectangleHole(rectangleCenter, rectangleDim, upperHoleCenterfromRC, holeDim);
	cnt_tmp = 0;
	for (unsigned int i = 0; i < holeGeomInfo.size(); i++)
	{
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetDimension(holeGeomInfo[i].first);
		m_ObjCollision[i + collisionCount].SetLocalFrame(holeGeomInfo[i].second);
		m_ObjLink[0].AddCollision(&m_ObjCollision[i + collisionCount]);
		cnt_tmp++;
	}
	collisionCount += cnt_tmp;

	// base part 6
	rectangleCenter = SE3(Vec3(0.06, -0.205 + 0.038 + 0.049 + 0.046 + 0.049*0.5, 0.01));
	holeGeomInfo = makeRectangleHole(rectangleCenter, rectangleDim, lowerHoleCenterfromRC, holeDim);
	cnt_tmp = 0;
	for (unsigned int i = 0; i < holeGeomInfo.size(); i++)
	{
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetDimension(holeGeomInfo[i].first);
		m_ObjCollision[i + collisionCount].SetLocalFrame(holeGeomInfo[i].second);
		m_ObjLink[0].AddCollision(&m_ObjCollision[i + collisionCount]);
		cnt_tmp++;
	}
	collisionCount += cnt_tmp;

	// base part 7
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.240, 0.046, 0.02));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(0.0, 0.0, 0.01)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;

	// base part 8
	rectangleCenter = SE3(Vec3(-0.06, 0.205 - 0.038 - 0.049 - 0.046 - 0.049*0.5, 0.01));
	holeGeomInfo = makeRectangleHole(rectangleCenter, rectangleDim, upperHoleCenterfromRC, holeDim);
	cnt_tmp = 0;
	for (unsigned int i = 0; i < holeGeomInfo.size(); i++)
	{
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetDimension(holeGeomInfo[i].first);
		m_ObjCollision[i + collisionCount].SetLocalFrame(holeGeomInfo[i].second);
		m_ObjLink[0].AddCollision(&m_ObjCollision[i + collisionCount]);
		cnt_tmp++;
	}
	collisionCount += cnt_tmp;

	// base part 9
	rectangleCenter = SE3(Vec3(0.06, 0.205 - 0.038 - 0.049 - 0.046 - 0.049*0.5, 0.01));
	holeGeomInfo = makeRectangleHole(rectangleCenter, rectangleDim, lowerHoleCenterfromRC, holeDim);
	cnt_tmp = 0;
	for (unsigned int i = 0; i < holeGeomInfo.size(); i++)
	{
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetDimension(holeGeomInfo[i].first);
		m_ObjCollision[i + collisionCount].SetLocalFrame(holeGeomInfo[i].second);
		m_ObjLink[0].AddCollision(&m_ObjCollision[i + collisionCount]);
		cnt_tmp++;
	}
	collisionCount += cnt_tmp;

	// base part 10
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.240, 0.046, 0.02));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(0.0, 0.205 - 0.038 - 0.049 - 0.046*0.5, 0.01)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;
	

	// base part 11
	rectangleCenter = SE3(Vec3(-0.06, -(-0.205 + 0.038 + 0.049*0.5), 0.01));
	holeGeomInfo = makeRectangleHole(rectangleCenter, rectangleDim, upperHoleCenterfromRC, holeDim);
	cnt_tmp = 0;
	for (unsigned int i = 0; i < holeGeomInfo.size(); i++)
	{
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetDimension(holeGeomInfo[i].first);
		m_ObjCollision[i + collisionCount].SetLocalFrame(holeGeomInfo[i].second);
		m_ObjLink[0].AddCollision(&m_ObjCollision[i + collisionCount]);
		cnt_tmp++;
	}
	collisionCount += cnt_tmp;

	// base part 12
	rectangleCenter = SE3(Vec3(0.06, -(-0.205 + 0.038 + 0.049*0.5), 0.01));
	holeGeomInfo = makeRectangleHole(rectangleCenter, rectangleDim, lowerHoleCenterfromRC, holeDim);
	cnt_tmp = 0;
	for (unsigned int i = 0; i < holeGeomInfo.size(); i++)
	{
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetDimension(holeGeomInfo[i].first);
		m_ObjCollision[i + collisionCount].SetLocalFrame(holeGeomInfo[i].second);
		m_ObjLink[0].AddCollision(&m_ObjCollision[i + collisionCount]);
		cnt_tmp++;
	}
	collisionCount += cnt_tmp;
	
	// base part 13
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.240, 0.038, 0.02));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(0.0, -(-0.205 + 0.038*0.5), 0.01)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;

	// handle 
	// left
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.08, 0.023, 0.04));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(0.0, -0.205 + 0.023*0.5, 0.04)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;

	// right
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.08, 0.023, 0.04));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(0.0, -(-0.205 + 0.023*0.5), 0.04)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;


	this->SetBaseLink(&m_ObjLink[0]);
	this->SetBaseLinkType(srSystem::KINEMATIC);
	this->SetSelfCollision(false);
}

Insert::Insert()
{
	AssembleModel();
}

Insert::~Insert()
{
}

void Insert::AssembleModel()
{
	m_numLink = 1;
	m_numCollision =2+4;
	int collisionCount = 0;

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
	m_ObjLink[0].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	m_ObjLink[0].GetGeomInfo().SetFileName("../../../workspace/robot/QBtech_env_modeling/Insert.3ds");


	m_ObjLink[0].GetGeomInfo().SetColor(0.2f, 0.2f, 0.2f, 1.0f);

	// Collision 1
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.01725, 0.0805 - 0.012*2, 0.0245));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(0.0503-0.01725*0.5, 0.0, 0.0245*0.5)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;

	// Collision 2
	SE3 rectangleCenter = SE3(Vec3(-0.019575, 0.0, 0.0245*0.5));
	Vec3 rectangleDim(0.1225 - 0.01725, 0.0805, 0.0245);
	Vec3 holeDim(0.046, 0.046, 0.0245);
	Vec3 holeCenterfromRC(0.006+ 0.019575, 0.0, 0.0);

	vector<pair<Vec3, SE3>> holeGeomInfo = makeRectangleHole(rectangleCenter, rectangleDim, holeCenterfromRC, holeDim);
	int cnt_tmp = 0;

	for (unsigned int i = 0; i < holeGeomInfo.size(); i++)
	{
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetDimension(holeGeomInfo[i].first);
		m_ObjCollision[i + collisionCount].SetLocalFrame(holeGeomInfo[i].second);
		m_ObjLink[0].AddCollision(&m_ObjCollision[i + collisionCount]);
		cnt_tmp++;
	}
	collisionCount += cnt_tmp;

	// Collision 3
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.03, 0.0805, 0.0115));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(-0.0722+0.015, 0.0,  0.0245+0.0115*0.5)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;


	this->SetBaseLink(&m_ObjLink[0]);
	this->SetBaseLinkType(srSystem::KINEMATIC);
	this->SetSelfCollision(false);

}

LowerFrame::LowerFrame()
{
	AssembleModel();
}

LowerFrame::~LowerFrame()
{
}

void LowerFrame::AssembleModel()
{


	m_numLink = 1;
	m_numCollision = 100;
	int collisionCount = 0;
	int cnt_tmp = 0;
	vector<pair<Vec3, SE3>> holeGeomInfo;

	SE3 rectangleCenter;
	Vec3 rectangleDim;
	Vec3 holeCenterfromRC;
	Vec3 holeDim;
	

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
	m_ObjLink[0].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(SR_PI, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	m_ObjLink[0].GetGeomInfo().SetFileName("../../../workspace/robot/QBtech_env_modeling/Lower_frame.3ds");


	m_ObjLink[0].GetGeomInfo().SetColor(0.3f, 0.3f, 0.3f, 1.0f);


	// Collision from left corner
	// Holes first
	// Left back
	rectangleCenter = SE3(Vec3(-0.150 + 0.0245*0.5, -0.205 + 0.0235*0.5, 0.025));
	rectangleDim = Vec3(0.0245, 0.0235, 0.05);
	holeCenterfromRC = Vec3(0.01363 - 0.01225, 0.0146 - 0.01175, 0.0);
	holeDim = Vec3(0.014, 0.014, 0.05);

	holeGeomInfo = makeRectangleHole(rectangleCenter, rectangleDim, holeCenterfromRC, holeDim);
	cnt_tmp = 0;
	for (unsigned int i = 0; i < holeGeomInfo.size(); i++)
	{
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetDimension(holeGeomInfo[i].first);
		m_ObjCollision[i + collisionCount].SetLocalFrame(holeGeomInfo[i].second);
		m_ObjLink[0].AddCollision(&m_ObjCollision[i + collisionCount]);
		cnt_tmp++;
	}
	collisionCount += cnt_tmp;

	// Left front
	rectangleCenter = SE3(Vec3(0.150 - 0.02325*0.5, -0.1955 + 0.025*0.5, 0.025));
	rectangleDim = Vec3(0.02325, 0.025, 0.05);
	holeCenterfromRC = Vec3( 0.02325*0.5-0.0155, 0.0135- 0.025*0.5,0.0 );
	holeDim = Vec3(0.014, 0.014, 0.05);

	holeGeomInfo = makeRectangleHole(rectangleCenter, rectangleDim, holeCenterfromRC, holeDim);
	cnt_tmp = 0;
	for (unsigned int i = 0; i < holeGeomInfo.size(); i++)
	{
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetDimension(holeGeomInfo[i].first);
		m_ObjCollision[i + collisionCount].SetLocalFrame(holeGeomInfo[i].second);
		m_ObjLink[0].AddCollision(&m_ObjCollision[i + collisionCount]);
		cnt_tmp++;
	}
	collisionCount += cnt_tmp;


	// Second Left front
	rectangleCenter = SE3(Vec3(0.150 - 0.02375*0.5, -0.1135 + 0.039*0.5, 0.025));
	rectangleDim = Vec3(0.02375, 0.039, 0.05);
	holeCenterfromRC = Vec3(0.00825-0.02375*0.5, 0.0, 0.0);
	holeDim = Vec3(0.014, 0.014, 0.05);

	holeGeomInfo = makeRectangleHole(rectangleCenter, rectangleDim, holeCenterfromRC, holeDim);
	cnt_tmp = 0;
	for (unsigned int i = 0; i < holeGeomInfo.size(); i++)
	{
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetDimension(holeGeomInfo[i].first);
		m_ObjCollision[i + collisionCount].SetLocalFrame(holeGeomInfo[i].second);
		m_ObjLink[0].AddCollision(&m_ObjCollision[i + collisionCount]);
		cnt_tmp++;
	}
	collisionCount += cnt_tmp;


	// Middle front
	rectangleCenter = SE3(Vec3(0.150 - 0.02375*0.5, 0.0, 0.025));
	rectangleDim = Vec3(0.02375, 0.035, 0.05);
	holeCenterfromRC = Vec3(0.00825 - 0.02375*0.5, 0.0, 0.0);
	holeDim = Vec3(0.014, 0.014, 0.05);

	holeGeomInfo = makeRectangleHole(rectangleCenter, rectangleDim, holeCenterfromRC, holeDim);
	cnt_tmp = 0;
	for (unsigned int i = 0; i < holeGeomInfo.size(); i++)
	{
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetDimension(holeGeomInfo[i].first);
		m_ObjCollision[i + collisionCount].SetLocalFrame(holeGeomInfo[i].second);
		m_ObjLink[0].AddCollision(&m_ObjCollision[i + collisionCount]);
		cnt_tmp++;
	}
	collisionCount += cnt_tmp;

	// Second right front
	rectangleCenter = SE3(Vec3(0.150 - 0.02375*0.5, 0.1135 - 0.039*0.5, 0.025));
	rectangleDim = Vec3(0.02375, 0.039, 0.05);
	holeCenterfromRC = Vec3(0.00825 - 0.02375*0.5, 0.0, 0.0);
	holeDim = Vec3(0.014, 0.014, 0.05);

	holeGeomInfo = makeRectangleHole(rectangleCenter, rectangleDim, holeCenterfromRC, holeDim);
	cnt_tmp = 0;
	for (unsigned int i = 0; i < holeGeomInfo.size(); i++)
	{
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetDimension(holeGeomInfo[i].first);
		m_ObjCollision[i + collisionCount].SetLocalFrame(holeGeomInfo[i].second);
		m_ObjLink[0].AddCollision(&m_ObjCollision[i + collisionCount]);
		cnt_tmp++;
	}
	collisionCount += cnt_tmp;

	// Right front
	rectangleCenter = SE3(Vec3(0.150 - 0.02325*0.5, 0.1955 - 0.025*0.5, 0.025));
	rectangleDim = Vec3(0.02325, 0.025, 0.05);
	holeCenterfromRC = Vec3(0.02325*0.5 - 0.0155, -(0.0135 - 0.025*0.5), 0.0);
	holeDim = Vec3(0.014, 0.014, 0.05);

	holeGeomInfo = makeRectangleHole(rectangleCenter, rectangleDim, holeCenterfromRC, holeDim);
	cnt_tmp = 0;
	for (unsigned int i = 0; i < holeGeomInfo.size(); i++)
	{
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetDimension(holeGeomInfo[i].first);
		m_ObjCollision[i + collisionCount].SetLocalFrame(holeGeomInfo[i].second);
		m_ObjLink[0].AddCollision(&m_ObjCollision[i + collisionCount]);
		cnt_tmp++;
	}
	collisionCount += cnt_tmp;

	// Right back
	rectangleCenter = SE3(Vec3(-0.150 + 0.0245*0.5, 0.205 - 0.0235*0.5, 0.025));
	rectangleDim = Vec3(0.0245, 0.0235, 0.05);
	holeCenterfromRC = Vec3(0.01363 - 0.01225, -(0.0146 - 0.01175), 0.0);
	holeDim = Vec3(0.014, 0.014, 0.05);

	holeGeomInfo = makeRectangleHole(rectangleCenter, rectangleDim, holeCenterfromRC, holeDim);
	cnt_tmp = 0;
	for (unsigned int i = 0; i < holeGeomInfo.size(); i++)
	{
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetDimension(holeGeomInfo[i].first);
		m_ObjCollision[i + collisionCount].SetLocalFrame(holeGeomInfo[i].second);
		m_ObjLink[0].AddCollision(&m_ObjCollision[i + collisionCount]);
		cnt_tmp++;
	}
	collisionCount += cnt_tmp;

	// Slots & Fences
	// Left fence
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.25225, 0.014, 0.05));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(0.150 - 0.02325-0.25225*0.5, -0.1955+0.014*0.5, 0.025)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;


	// left slot
	rectangleCenter = SE3(Vec3(0.150 -0.02375-0.171*0.5, -0.101+ 0.014*0.5, 0.01));
	rectangleDim = Vec3(0.171, 0.014, 0.02);
	holeCenterfromRC = Vec3(0.0, 0.0, 0.0);
	holeDim = Vec3(0.1685, 0.0055, 0.02);

	holeGeomInfo = makeRectangleHole(rectangleCenter, rectangleDim, holeCenterfromRC, holeDim);
	cnt_tmp = 0;
	for (unsigned int i = 0; i < holeGeomInfo.size(); i++)
	{
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetDimension(holeGeomInfo[i].first);
		m_ObjCollision[i + collisionCount].SetLocalFrame(holeGeomInfo[i].second);
		m_ObjLink[0].AddCollision(&m_ObjCollision[i + collisionCount]);
		cnt_tmp++;
	}
	collisionCount += cnt_tmp;

	//  left slot's short fence
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.09525, 0.02, 0.02));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(-0.150 +0.01+0.09525*0.5, -0.104+0.02*0.5, 0.02*0.5)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;

	//  left slot's long fence
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.300, 0.014, 0.03));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(0.0, -0.104 + 0.02*0.5, 0.02+ 0.03*0.5)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;



	// middle slot
	rectangleCenter = SE3(Vec3(0.150 - 0.02375 - 0.171*0.5, 0.0, 0.01));
	rectangleDim = Vec3(0.171, 0.013, 0.02);
	holeCenterfromRC = Vec3(0.0, 0.0, 0.0);
	holeDim = Vec3(0.1685, 0.0055, 0.02);

	holeGeomInfo = makeRectangleHole(rectangleCenter, rectangleDim, holeCenterfromRC, holeDim);
	cnt_tmp = 0;
	for (unsigned int i = 0; i < holeGeomInfo.size(); i++)
	{
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetDimension(holeGeomInfo[i].first);
		m_ObjCollision[i + collisionCount].SetLocalFrame(holeGeomInfo[i].second);
		m_ObjLink[0].AddCollision(&m_ObjCollision[i + collisionCount]);
		cnt_tmp++;
	}
	collisionCount += cnt_tmp;

	//  middle slot's short fence
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.09525, 0.0224, 0.02));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(-0.150 + 0.01 + 0.09525*0.5, 0.0, 0.02*0.5)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;

	//  middle slot's long fence
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.300, 0.010, 0.03));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(0.0,0.0, 0.02 + 0.03*0.5)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;

	// right slot
	rectangleCenter = SE3(Vec3(0.150 - 0.02375 - 0.171*0.5, 0.101 - 0.014*0.5, 0.01));
	rectangleDim = Vec3(0.171, 0.014, 0.02);
	holeCenterfromRC = Vec3(0.0, 0.0, 0.0);
	holeDim = Vec3(0.1685, 0.0055, 0.02);

	holeGeomInfo = makeRectangleHole(rectangleCenter, rectangleDim, holeCenterfromRC, holeDim);
	cnt_tmp = 0;
	for (unsigned int i = 0; i < holeGeomInfo.size(); i++)
	{
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i + collisionCount].GetGeomInfo().SetDimension(holeGeomInfo[i].first);
		m_ObjCollision[i + collisionCount].SetLocalFrame(holeGeomInfo[i].second);
		m_ObjLink[0].AddCollision(&m_ObjCollision[i + collisionCount]);
		cnt_tmp++;
	}
	collisionCount += cnt_tmp;

	//  right slot's short fence
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.09525, 0.02, 0.02));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(-0.150 + 0.01 + 0.09525*0.5, 0.104 - 0.02*0.5, 0.02*0.5)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;

	//  right slot's long fence
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.300, 0.014, 0.03));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(0.0, 0.104 - 0.02*0.5, 0.02 + 0.03*0.5)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;




	// Right fence
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.25225, 0.014, 0.05));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(0.150 - 0.02325 - 0.25225*0.5, 0.1955 - 0.014*0.5, 0.025)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;

	// Front fence
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.007, 0.391, 0.05));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(0.150 - 0.007*0.5, 0.0, 0.025)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;

	// Back fence
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.01, 0.363, 0.05));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(-0.150+0.01*0.5, 0.0, 0.025)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;



	// First floor
	// left most base
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.12779, 0.0805, 0.0105));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(0.150 -0.12779*0.5, -0.1815+0.0805*0.5, 0.0105*0.5)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;

	// left middle base
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.12779, 0.0805, 0.0105));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(0.150 - 0.12779*0.5, -0.087 + 0.0805*0.5, 0.0105*0.5)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;

	// right middle base
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.12779, 0.0805, 0.0105));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(0.150 - 0.12779*0.5, 0.087 - 0.0805*0.5, 0.0105*0.5)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;

	// right most base
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.12779, 0.0805, 0.0105));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(0.150 - 0.12779*0.5, 0.1815 - 0.0805*0.5, 0.0105*0.5)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;
	
	// Second floor
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.1, 0.363, 0.020));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(-0.150 +0.0715+0.1*0.5 ,0.0, 0.015+0.02*0.5)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;

	// Third floor
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.0715, 0.363, 0.0105));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(-0.150 + 0.0715*0.5, 0.0, 0.0395+ 0.0105*0.5)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;


	// Four Cylinder
	// left most
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.039, 0.039, 0.035));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(0.0987, -0.142, 0.035*0.5)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;

	// left middle
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.039, 0.039, 0.035));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(0.0987, -0.0462, 0.035*0.5)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;

	// right middle
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.039, 0.039, 0.035));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(0.0987, 0.046, 0.035*0.5)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;

	// right most
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.039, 0.039, 0.035));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(0.0987, +0.142, 0.035*0.5)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;


	this->SetBaseLink(&m_ObjLink[0]);
	this->SetBaseLinkType(srSystem::KINEMATIC);
	this->SetSelfCollision(false);

}

JigAssem_QB::JigAssem_QB()
{
	AssembleModel();
}

JigAssem_QB::~JigAssem_QB()
{
}

void JigAssem_QB::AssembleModel()
{
	m_visionOffset = SE3(Vec3(-0.02725, 0.0, 0.0));		//-(0.15 - 0.0955)*0.5

	holeCenter.resize(8);
	for (unsigned int i = 0; i < holeCenter.size(); i++)
		holeCenter[i] = SE3();
	holeCenter[0] = SE3(Vec3(0.0021 - 0.025, 0.1628 - 0.0225, 0.0));
	holeCenter[1] = SE3(Vec3(0.0736 + 0.025, 0.16375 - 0.0225, 0.0));
	holeCenter[2] = SE3(Vec3(0.002 - 0.025, 0.0701 - 0.0225, 0.0));
	holeCenter[3] = SE3(Vec3(0.0737 + 0.025, 0.06845 - 0.0225, 0.0));
	holeCenter[4] = SE3(Vec3(0.002 - 0.025, -0.0701 + 0.0225, 0.0));
	holeCenter[5] = SE3(Vec3(0.0735 + 0.025, -0.06925 + 0.0225, 0.0));
	holeCenter[6] = SE3(Vec3(0.002 - 0.025, -0.1629 + 0.0225, 0.0));
	holeCenter[7] = SE3(Vec3(0.0733 + 0.025, -0.16375 + 0.0225, 0.0));

	m_numLink = 1;
	m_numCollision = 2 + 4 * 8 * 2;
	int collisionCount = 0;
	int cnt_tmp = 0;
	vector<pair<Vec3, SE3>> holeGeomInfo;

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
	m_ObjLink[0].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	m_ObjLink[0].GetGeomInfo().SetFileName("../../../workspace/robot/QBtech_env_modeling/jigassem.3ds");


	m_ObjLink[0].GetGeomInfo().SetColor(0.3f, 0.3f, 0.3f, 1.0f);

	// assem 1
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.3, 0.410, 0.184 - 0.025));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(0.0, 0.0, -0.5*(0.184 - 0.025) - 0.025)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;
	// assem 2
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.0545, 0.410, 0.015));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(- 0.15 + 0.5*0.0545, 0.0, -0.0075 - 0.01)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;
	// Collision from top right
	
	// hole infos

	Vec3 rectangleDim1(0.2455*0.5, 0.409*0.25, 0.025);
	//Vec3 rectangleDim2(0.2455*0.5, 0.409*0.25, 0.015);
	Vec3 holeDim1(0.052, 0.047, 0.025);
	//Vec3 holeDim2(0.1, 0.08, 0.015);
	SE3 rectangleCenter;

	vector<double> rcx(2);
	rcx[0] = -0.0955 + 0.5*0.2455 - 0.25*0.2455;
	rcx[1] = -0.0955 + 0.5*0.2455 + 0.25*0.2455;
	vector<double> rcy(4);
	rcy[0] = 0.375*0.409;
	rcy[1] = 0.125*0.409;
	rcy[2] = -0.125*0.409;
	rcy[3] = -0.375*0.409;

	for (unsigned int j = 0; j < holeCenter.size(); j++)
	{
		
		cnt_tmp = 0;
		rectangleCenter = Vec3(rcx[j % 2], rcy[j / 2], -0.0125);
		holeGeomInfo = makeRectangleHole(rectangleCenter, rectangleDim1, Vec3(holeCenter[j].GetPosition()[0] - rectangleCenter.GetPosition()[0], holeCenter[j].GetPosition()[1] - rectangleCenter.GetPosition()[1], 0.0), holeDim1);
		for (unsigned int i = 0; i < holeGeomInfo.size(); i++)
		{
			m_ObjCollision[i + collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
			m_ObjCollision[i + collisionCount].GetGeomInfo().SetDimension(holeGeomInfo[i].first);
			m_ObjCollision[i + collisionCount].SetLocalFrame(holeGeomInfo[i].second);
			m_ObjLink[0].AddCollision(&m_ObjCollision[i + collisionCount]);
			cnt_tmp++;
		}
		collisionCount += cnt_tmp;
	}
	


	this->SetBaseLink(&m_ObjLink[0]);
	this->SetBaseLinkType(srSystem::KINEMATIC);
	this->SetSelfCollision(false);
}

JigAssem_QB_bar::JigAssem_QB_bar(bool add)
{
	m_add = add;
	AssembleModel();
}

JigAssem_QB_bar::~JigAssem_QB_bar()
{
}

void JigAssem_QB_bar::AssembleModel()
{
	holeCenter.resize(8);
	for (unsigned int i = 0; i < holeCenter.size(); i++)
		holeCenter[i] = SE3();
	holeCenter[0] = SE3(Vec3(0.0021 - 0.025, 0.1628 - 0.0225, 0.0));
	holeCenter[1] = SE3(Vec3(0.0736 + 0.025, 0.16375 - 0.0225, 0.0));
	holeCenter[2] = SE3(Vec3(0.002 - 0.025, 0.0701 - 0.0225, 0.0));
	holeCenter[3] = SE3(Vec3(0.0737 + 0.025, 0.06845 - 0.0225, 0.0));
	holeCenter[4] = SE3(Vec3(0.002 - 0.025, -0.0701 + 0.0225, 0.0));
	holeCenter[5] = SE3(Vec3(0.0735 + 0.025, -0.06925 + 0.0225, 0.0));
	holeCenter[6] = SE3(Vec3(0.002 - 0.025, -0.1629 + 0.0225, 0.0));
	holeCenter[7] = SE3(Vec3(0.0733 + 0.025, -0.16375 + 0.0225, 0.0));

	m_numLink = 1;
	m_numCollision = 2 + 4 * 8 * 2;
	int collisionCount = 0;
	int cnt_tmp = 0;
	vector<pair<Vec3, SE3>> holeGeomInfo;

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
	m_ObjLink[0].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	m_ObjLink[0].GetGeomInfo().SetFileName("../../../workspace/robot/QBtech_env_modeling/jigassem.3ds");


	m_ObjLink[0].GetGeomInfo().SetColor(0.3f, 0.3f, 0.3f, 1.0f);

	// assem 1
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.3, 0.410, 0.184 - 0.025));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(0.0, 0.0, -0.5*(0.184 - 0.025) - 0.025)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;
	// assem 2
	m_ObjCollision[collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[collisionCount].GetGeomInfo().SetDimension(Vec3(0.0545, 0.410, 0.015));
	m_ObjCollision[collisionCount].SetLocalFrame(SE3(Vec3(-0.15 + 0.5*0.0545, 0.0, -0.0075 - 0.01)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[collisionCount]);
	collisionCount++;
	// Collision from top right

	// hole infos

	Vec3 rectangleDim1(0.2455*0.5, 0.409*0.25, 0.025);
	//Vec3 rectangleDim2(0.2455*0.5, 0.409*0.25, 0.015);
	Vec3 holeDim1(0.052, 0.047, 0.025);
	//Vec3 holeDim2(0.1, 0.08, 0.015);
	SE3 rectangleCenter;

	vector<double> rcx(2);
	rcx[0] = -0.0955 + 0.5*0.2455 - 0.25*0.2455;
	rcx[1] = -0.0955 + 0.5*0.2455 + 0.25*0.2455;
	vector<double> rcy(4);
	rcy[0] = 0.375*0.409;
	rcy[1] = 0.125*0.409;
	rcy[2] = -0.125*0.409;
	rcy[3] = -0.375*0.409;

	for (unsigned int j = 0; j < holeCenter.size(); j++)
	{

		cnt_tmp = 0;
		rectangleCenter = Vec3(rcx[j % 2], rcy[j / 2], -0.0125);
		holeGeomInfo = makeRectangleHole(rectangleCenter, rectangleDim1, Vec3(holeCenter[j].GetPosition()[0] - rectangleCenter.GetPosition()[0], holeCenter[j].GetPosition()[1] - rectangleCenter.GetPosition()[1], 0.0), holeDim1);
		for (unsigned int i = 0; i < holeGeomInfo.size(); i++)
		{
			m_ObjCollision[i + collisionCount].GetGeomInfo().SetShape(srGeometryInfo::BOX);
			m_ObjCollision[i + collisionCount].GetGeomInfo().SetDimension(holeGeomInfo[i].first);
			m_ObjCollision[i + collisionCount].SetLocalFrame(holeGeomInfo[i].second);
			m_ObjLink[0].AddCollision(&m_ObjCollision[i + collisionCount]);
			cnt_tmp++;
		}
		collisionCount += cnt_tmp;
	}



	if (m_add)
	{
		// add virtual collision to separate workspace
		m_numCollision += 1;
		srCollision* temp = new srCollision;
		m_ObjCollision.push_back(*temp);
		unsigned int colnum = m_ObjCollision.size() - 1;
		m_ObjCollision[colnum].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		double height = 0.8;
		m_ObjCollision[colnum].GetGeomInfo().SetDimension(Vec3(0.8, 0.0010, height));
		m_ObjCollision[colnum].SetLocalFrame(SE3(Vec3(0.0, 0.0, 0.5*height)));
		m_ObjLink[0].AddCollision(&m_ObjCollision[colnum]);
	}
	

	this->SetBaseLink(&m_ObjLink[0]);
	this->SetBaseLinkType(srSystem::KINEMATIC);
	this->SetSelfCollision(false);
}
