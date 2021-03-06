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
	m_numLink = 2;
	m_numCollision = 8;
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

	// base
	Vec3 dim0(0.145, 0.048, 0.105);
	Vec3 dim1(0.059, 0.039, 0.091);
	Vec3 dim2(0.045, 0.029, 0.004);
	Vec3 dim3(0.039, 0.059, 0.091);
	Vec3 dim4(0.073, 0.060, 0.095);
	Vec3 dim5(0.045, 0.095, 0.004);

	for (int i = 0; i < m_numLink; i++)
	{
		m_ObjLink[i].GetGeomInfo().SetColor(0.3, 0.3, 0.3);
		m_ObjLink[i].GetGeomInfo().SetDimension(0.0);
		m_ObjLink[i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	}
	m_ObjLink[0].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	m_ObjLink[0].GetGeomInfo().SetFileName("../../../workspace/robot/objects_3ds/Moving_Contact_simplified.3ds");
	// xy plane
	m_ObjWeldJoint[0].SetParentLink(&m_ObjLink[0]);
	m_ObjWeldJoint[0].SetParentLinkFrame();
	m_ObjWeldJoint[0].SetChildLink(&m_ObjLink[1]);
	m_ObjWeldJoint[0].SetChildLinkFrame(SE3(Vec3(0.0, 0.0, 0.0)));

	///// set collision 
	Vec3 colli_offset(m_collision_offset);

	/*m_ObjCollision[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[0].GetGeomInfo().SetDimension(dims[0] + colli_offset);
	m_ObjCollision[0].SetLocalFrame(SE3(Vec3(-0.047, 0.0, 0.0)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[0]);*/

	// main body
	m_ObjCollision[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[0].GetGeomInfo().SetDimension(dim0 + colli_offset);
	m_ObjCollision[0].SetLocalFrame(SE3(Vec3(-0.047, 0.024, 0.0)));
	m_ObjLink[1].AddCollision(&m_ObjCollision[0]);

	// round column
	m_ObjCollision[1].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[1].GetGeomInfo().SetDimension(dim1 + colli_offset);
	m_ObjCollision[1].SetLocalFrame(EulerXYZ(Vec3(SR_PI / 2, 0, 0), Vec3(0.0, 0.091 / 2, 0.0)));
	m_ObjLink[1].AddCollision(&m_ObjCollision[1]);

	m_ObjCollision[3].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[3].GetGeomInfo().SetDimension(dim3 + colli_offset);
	m_ObjCollision[3].SetLocalFrame(EulerXYZ(Vec3(SR_PI / 2, 0, 0), Vec3(0.0, 0.091 / 2, 0.0)));
	m_ObjLink[1].AddCollision(&m_ObjCollision[3]);

	// small braket
	m_ObjCollision[2].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[2].GetGeomInfo().SetDimension(dim2 + colli_offset);
	m_ObjCollision[2].SetLocalFrame(EulerXYZ(Vec3(SR_PI / 2, 0, 0), Vec3(0.045/2, 0.018, 0.0)));
	m_ObjLink[1].AddCollision(&m_ObjCollision[2]);

	// body2
	m_ObjCollision[4].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[4].GetGeomInfo().SetDimension(dim4 + colli_offset);
	m_ObjCollision[4].SetLocalFrame(EulerXYZ(Vec3(0, 0, 0.283), Vec3(-0.070, -0.025, 0.0)));
	m_ObjLink[1].AddCollision(&m_ObjCollision[4]);

	// body3
	m_ObjCollision[5].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[5].GetGeomInfo().SetDimension(dim5 + colli_offset);
	m_ObjCollision[5].SetLocalFrame(EulerXYZ(Vec3(SR_PI / 2, 0.283, 0), Vec3(-0.018, -0.006, 0.0)));
	m_ObjLink[1].AddCollision(&m_ObjCollision[5]);
	

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

FixedContact::FixedContact(double collision_offset)
{
	m_collision_offset = collision_offset;
	AssembleModel();
}

FixedContact::~FixedContact()
{
}

void FixedContact::AssembleModel()
{
	m_numLink = 2;
	m_numCollision = 18;
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


	for (int i = 0; i < m_numLink; i++)
	{
		m_ObjLink[i].GetGeomInfo().SetColor(0.3, 0.3, 0.3);
		m_ObjLink[i].GetGeomInfo().SetDimension(0.0);
		m_ObjLink[i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	}
	m_ObjLink[0].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	m_ObjLink[0].GetGeomInfo().SetFileName("../../../workspace/robot/objects_3ds/Fixed_Contact_simplified1.3ds");
	m_ObjLink[0].GetGeomInfo().SetLocalFrame(SE3(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(-0.5*0.0587, -0.5*0.0587, 0.0))));
	// xy plane
	m_ObjWeldJoint[0].SetParentLink(&m_ObjLink[0]);
	m_ObjWeldJoint[0].SetParentLinkFrame();
	m_ObjWeldJoint[0].SetChildLink(&m_ObjLink[1]);
	m_ObjWeldJoint[0].SetChildLinkFrame(SE3(Vec3(0.0, 0.0, 0.0)));

	//////////////// Add Collisions ///////////////

	// base
	Vec3 dim0(0.0587, 0.0587, 0.018);	// base part
	Vec3 dim1(0.045, 0.061, 0.0055);	// half-box with hole
	Vec3 holedim1(0.00635, 0.00635, 0.0055);
	Vec3 dim2(0.0857, 0.014, 0.008);	// top part 1
	Vec3 dim3(0.0656, 0.0315, 0.008);	// top part 2
	Vec3 dim4(0.024, 0.035, 0.0033);	// top part with hole
	Vec3 holedim2(0.0066, 0.0066, 0.0033);
	Vec3 dim6(0.046, 0.026, 0.0033);	// top part without hole
	Vec3 dim7(0.061, 0.01, 0.0033);	// top askew part 
	double askew_angle = atan(0.02 / 0.057);
	double s_th = sin(askew_angle);
	double c_th = cos(askew_angle);

	///// set collision
	Vec3 colli_offset(m_collision_offset);
	Vec3 colli_offset2(m_collision_offset, 0.0, m_collision_offset);
	Vec3 colli_offset3(0.0, m_collision_offset, m_collision_offset);

	// base part
	m_ObjCollision[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[0].GetGeomInfo().SetDimension(dim0 + colli_offset);
	m_ObjCollision[0].SetLocalFrame(SE3(Vec3(0.0, 0.0, dim0[2] * 0.5)));
	m_ObjLink[1].AddCollision(&m_ObjCollision[0]);

	// top part 1
	m_ObjCollision[1].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[1].GetGeomInfo().SetDimension(dim2 + colli_offset2);
	m_ObjCollision[1].SetLocalFrame(SE3(Vec3(0.0, -0.5*dim0[1] - 0.00115 + 0.0155 + 0.5*dim2[1], dim0[2] + dim1[2] + dim3[2] * 0.5)));
	m_ObjLink[1].AddCollision(&m_ObjCollision[1]);

	// top part 2
	m_ObjCollision[2].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[2].GetGeomInfo().SetDimension(dim3 + colli_offset3);
	m_ObjCollision[2].SetLocalFrame(SE3(Vec3(0.0, -0.5*dim0[1] - 0.00115 + 0.0155 + dim2[1] + 0.5*dim3[1], dim0[2] + dim1[2] + dim3[2] * 0.5)));
	m_ObjLink[1].AddCollision(&m_ObjCollision[2]);

	// top part without hole
	m_ObjCollision[3].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[3].GetGeomInfo().SetDimension(dim6 + colli_offset);
	m_ObjCollision[3].SetLocalFrame(SE3(Vec3(0.0, 0.5*dim0[1] + 0.00115 + 0.5*dim6[1], dim0[2] + dim1[2] + dim3[2] - dim6[2]*0.5)));
	m_ObjLink[1].AddCollision(&m_ObjCollision[3]);

	// top askew part 1
	m_ObjCollision[4].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[4].GetGeomInfo().SetDimension(dim7);
	m_ObjCollision[4].SetLocalFrame(EulerZYX(Vec3(SR_PI_HALF + askew_angle, 0.0, 0.0), Vec3(0.012 + 0.5*(dim7[0]*s_th - dim7[1]*c_th), 0.5*dim0[1] + 0.00115 + 0.057 - 0.5*(dim7[0]*c_th + dim7[1]*s_th), dim0[2] + dim1[2] + dim3[2] - dim6[2] * 0.5)));
	m_ObjLink[1].AddCollision(&m_ObjCollision[4]);

	// top askew part 2
	m_ObjCollision[5].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[5].GetGeomInfo().SetDimension(dim7);
	m_ObjCollision[5].SetLocalFrame(EulerZYX(Vec3(SR_PI_HALF - askew_angle, 0.0, 0.0), Vec3(- 0.012 - 0.5*(dim7[0] * s_th - dim7[1] * c_th), 0.5*dim0[1] + 0.00115 + 0.057 - 0.5*(dim7[0] * c_th + dim7[1] * s_th), dim0[2] + dim1[2] + dim3[2] - dim6[2] * 0.5)));
	m_ObjLink[1].AddCollision(&m_ObjCollision[5]);

	// half-box with hole 1
	Vec3 hole1_1centerfromrc(0.5*dim1[0] - 0.0057, -0.5*dim1[1] + 0.036, 0.0);
	vector<pair<Vec3, SE3>> rects1_1 = makeRectangleHole(SE3(Vec3(0.5*dim1[0], 0.0, dim0[2] + dim1[2]*0.5)), dim1 + colli_offset, hole1_1centerfromrc, holedim1);
	for (unsigned int i = 0; i < rects1_1.size(); i++)
	{
		m_ObjCollision[6 + i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[6 + i].GetGeomInfo().SetDimension(rects1_1[i].first);
		m_ObjCollision[6 + i].SetLocalFrame(rects1_1[i].second);
		m_ObjLink[1].AddCollision(&m_ObjCollision[6 + i]);
	}

	// half-box with hole 2
	Vec3 hole1_2centerfromrc( - 0.5*dim1[0] + 0.0057, -0.5*dim1[1] + 0.036, 0.0);
	vector<pair<Vec3, SE3>> rects1_2 = makeRectangleHole(SE3(Vec3( - 0.5*dim1[0], 0.0, dim0[2] + dim1[2] * 0.5)), dim1 + colli_offset, hole1_2centerfromrc, holedim1);
	for (unsigned int i = 0; i < rects1_2.size(); i++)
	{
		m_ObjCollision[10 + i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[10 + i].GetGeomInfo().SetDimension(rects1_2[i].first);
		m_ObjCollision[10 + i].SetLocalFrame(rects1_2[i].second);
		m_ObjLink[1].AddCollision(&m_ObjCollision[10 + i]);
	}

	// top part with hole
	Vec3 hole2centerfromrc(0.0, 0.5*dim4[1] - 0.008, 0.0);
	vector<pair<Vec3, SE3>> rects2 = makeRectangleHole(SE3(Vec3(0.0, 0.5*dim1[1] + 0.057 - 0.5*dim4[1], dim0[2] + dim1[2] + dim3[2] - 0.5*dim4[2])), dim4 + colli_offset, hole2centerfromrc, holedim2);
	for (unsigned int i = 0; i < rects2.size(); i++)
	{
		m_ObjCollision[14 + i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[14 + i].GetGeomInfo().SetDimension(rects2[i].first);
		m_ObjCollision[14 + i].SetLocalFrame(rects2[i].second);
		m_ObjLink[1].AddCollision(&m_ObjCollision[14 + i]);
	}
	//m_ObjCollision[1].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	//m_ObjCollision[1].GetGeomInfo().SetDimension(dims[0] + colli_offset);
	//m_ObjCollision[1].SetLocalFrame(SE3(Vec3(-0.047, 0.0, 0.0)));
	//m_ObjLink[1].AddCollision(&m_ObjCollision[1]);

	//m_ObjCollision[2].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	//m_ObjCollision[2].GetGeomInfo().SetDimension(dims[1] + colli_offset);
	//m_ObjCollision[2].SetLocalFrame(EulerXYZ(Vec3(SR_PI / 2, 0, 0), Vec3(0.0, 0.091 / 2, 0.0)));
	//m_ObjLink[2].AddCollision(&m_ObjCollision[2]);

	//m_ObjCollision[3].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	//m_ObjCollision[3].GetGeomInfo().SetDimension(dims[2] + colli_offset);
	//m_ObjCollision[3].SetLocalFrame(EulerXYZ(Vec3(SR_PI / 2, 0, 0), Vec3(0.045 / 2, 0.018, 0.0)));
	//m_ObjLink[3].AddCollision(&m_ObjCollision[3]);


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

BoxForTape::BoxForTape()
{
	AssembleModel();
}

BoxForTape::~BoxForTape()
{
}

void BoxForTape::AssembleModel()
{
	m_numLink = 1;
	m_numWeldJoint = 0;
	m_numCollision = 3;

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

	m_ObjLink[0].GetGeomInfo().SetColor(0.3, 0.4, 0.3);

	m_ObjLink[0].SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	m_ObjLink[0].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	//m_ObjLink[0].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, SR_PI_HALF), Vec3(-0.01675, 0.0, -0.004)));
	//m_ObjLink[0].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(-0.10007, 0.09307, 0.0888)));
	m_ObjLink[0].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0))); // SetLocalFrame --> Orientation: SolidworkssrLib에서 solidworks로 가는 SO3, Position: srLib frame의 orientation 기준으로 원점 이동하고 싶은 만큼 입력
	m_ObjLink[0].GetGeomInfo().SetFileName("../../../workspace/robot/env5_3ds/BoxForTape.3ds");

	m_ObjLink[0].AddCollision(&m_ObjCollision[0]);
	m_ObjCollision[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[0].GetGeomInfo().SetDimension(Vec3(0.06, 0.03, 0.02));
	m_ObjCollision[0].SetLocalFrame(SE3(Vec3(0.0, 0.0, 0.0)));

	m_ObjLink[0].AddCollision(&m_ObjCollision[1]);
	m_ObjCollision[1].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[1].GetGeomInfo().SetDimension(Vec3(0.002, 0.002, 0.03));
	//m_ObjCollision[1].SetLocalFrame(SE3(Vec3(0.0, 0.0, 0.060 / 2.0)));
	m_ObjCollision[1].SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.02, 0.0, 0.025)));

	m_ObjLink[0].AddCollision(&m_ObjCollision[2]);
	m_ObjCollision[2].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[2].GetGeomInfo().SetDimension(Vec3(0.002, 0.002, 0.03));
	//m_ObjCollision[1].SetLocalFrame(SE3(Vec3(0.0, 0.0, 0.060 / 2.0)));
	m_ObjCollision[2].SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(-0.02, 0.0, 0.025)));


	this->SetSelfCollision(false);
	this->SetBaseLink(&m_ObjLink[0]);
	this->SetBaseLinkType(srSystem::KINEMATIC);

}

Tape::Tape()
{
	AssembleModel();
}

Tape::~Tape()
{
}

void Tape::AssembleModel()
{
	m_numLink = 3;
	m_numWeldJoint = 2;
	m_numCollision = 5;

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

	m_ObjLink[0].GetGeomInfo().SetColor(0.5, 0.1, 0.2);
	m_ObjLink[0].SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	m_ObjLink[0].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	//m_ObjLink[0].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, SR_PI_HALF), Vec3(-0.01675, 0.0, -0.004)));
	//m_ObjLink[0].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(-0.10007, 0.09307, 0.0888)));
	m_ObjLink[0].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0))); // SetLocalFrame --> Orientation: SolidworkssrLib에서 solidworks로 가는 SO3, Position: srLib frame의 orientation 기준으로 원점 이동하고 싶은 만큼 입력
	m_ObjLink[0].GetGeomInfo().SetFileName("../../../workspace/robot/env5_3ds/Taping1.3ds");

	// dummy link
	m_ObjLink[1].GetGeomInfo().SetColor(0.3, 0.4, 0.3);
	m_ObjLink[1].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	m_ObjLink[1].GetGeomInfo().SetFileName("../../../workspace/robot/env5_3ds/Taping2.3ds");

	m_ObjLink[2].GetGeomInfo().SetColor(0.3, 0.4, 0.3);
	m_ObjLink[2].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	m_ObjLink[2].GetGeomInfo().SetFileName("../../../workspace/robot/env5_3ds/Taping3.3ds");

	m_ObjWeldJoint[0].SetParentLink(&m_ObjLink[0]);
	//m_ObjWeldJoint[0].SetParentLinkFrame(SE3(Vec3(-0.01, 0.0, dim[2] - 0.0004))); // change to exact value later
	m_ObjWeldJoint[0].SetParentLinkFrame(); // change to exact value later
	m_ObjWeldJoint[0].SetChildLink(&m_ObjLink[1]);
	m_ObjWeldJoint[0].SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(-0.01, 0.0, -0.0075)));

	m_ObjWeldJoint[1].SetParentLink(&m_ObjLink[1]);
	//m_ObjWeldJoint[0].SetParentLinkFrame(SE3(Vec3(-0.01, 0.0, dim[2] - 0.0004))); // change to exact value later
	m_ObjWeldJoint[1].SetParentLinkFrame(); // change to exact value later
	m_ObjWeldJoint[1].SetChildLink(&m_ObjLink[2]);
	m_ObjWeldJoint[1].SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.6, 0.0), Vec3(0.0, 0.0, 0.0)));

	m_ObjLink[0].AddCollision(&m_ObjCollision[0]);
	m_ObjCollision[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[0].GetGeomInfo().SetDimension(Vec3(0.08, 0.015, 0.005));
	m_ObjCollision[0].SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.02, 0.0, -0.0025)));

	m_ObjLink[0].AddCollision(&m_ObjCollision[1]);
	m_ObjCollision[1].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[1].GetGeomInfo().SetDimension(Vec3(0.06, 0.015, 0.06));
	//m_ObjCollision[1].SetLocalFrame(SE3(Vec3(0.0, 0.0, 0.060 / 2.0)));
	m_ObjCollision[1].SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(-0.05, 0.0, -0.03)));

	m_ObjLink[1].AddCollision(&m_ObjCollision[2]);
	m_ObjCollision[2].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[2].GetGeomInfo().SetDimension(Vec3(0.065, 0.015, 0.0075));
	//m_ObjCollision[1].SetLocalFrame(SE3(Vec3(0.0, 0.0, 0.060 / 2.0)));
	m_ObjCollision[2].SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0225, 0.0, -0.00375)));

	m_ObjLink[1].AddCollision(&m_ObjCollision[3]);
	m_ObjCollision[3].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[3].GetGeomInfo().SetDimension(Vec3(0.02, 0.015, 0.01));
	//m_ObjCollision[1].SetLocalFrame(SE3(Vec3(0.0, 0.0, 0.060 / 2.0)));
	m_ObjCollision[3].SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.005)));

	m_ObjLink[2].AddCollision(&m_ObjCollision[3]);
	m_ObjCollision[3].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[3].GetGeomInfo().SetDimension(Vec3(0.052, 0.011, 0.002));
	//m_ObjCollision[1].SetLocalFrame(SE3(Vec3(0.0, 0.0, 0.060 / 2.0)));
	m_ObjCollision[3].SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.026, 0.0, 0.001)));

	m_ObjLink[2].AddCollision(&m_ObjCollision[4]);
	m_ObjCollision[4].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[4].GetGeomInfo().SetDimension(Vec3(0.010, 0.011, 0.010));
	//m_ObjCollision[1].SetLocalFrame(SE3(Vec3(0.0, 0.0, 0.060 / 2.0)));
	m_ObjCollision[4].SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.047, 0.0, -0.005)));

	this->SetSelfCollision(false);
	this->SetBaseLink(&m_ObjLink[0]);
	this->SetBaseLinkType(srSystem::KINEMATIC);

}