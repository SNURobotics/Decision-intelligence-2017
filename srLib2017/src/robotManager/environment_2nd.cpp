#include "environment_2nd.h"


FeaturePoint::FeaturePoint()
{
	AssembleModel();
}

FeaturePoint::~FeaturePoint()
{
}

void FeaturePoint::AssembleModel()
{
	m_numLink = 1;
	m_numCollision = 0;
	m_numWeldJoint = 0;

	m_ObjLink.resize(m_numLink);
	m_ObjCollision.resize(m_numCollision);
	m_ObjWeldJoint.resize(m_numWeldJoint);


	double sphereRadius = 0.1;

	m_ObjLink[0].GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
	m_ObjLink[0].GetGeomInfo().SetDimension(sphereRadius);
	m_ObjLink[0].GetGeomInfo().SetColor(0.0f, 0.0f, 0.0f, 1.0f);

	//m_ObjCollision[0].GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
	//m_ObjCollision[0].GetGeomInfo().SetDimension(sphereRadius);
	//m_ObjCollision[0].SetLocalFrame(SE3());
	//m_ObjLink[0].AddCollision(&m_ObjCollision[0]);


	m_ObjLink[0].SetFrame(SE3(Vec3(1.0, 0.0, 0.0)));

	this->SetBaseLink(&m_ObjLink[0]);
	this->SetBaseLinkType(srSystem::KINEMATIC);
	this->SetSelfCollision(false);
}



Table::Table()
{
	m_tableHeight = 1.0;
	AssembleModel();
}

Table::~Table()
{
}

void Table::setLocalFramefromGlobal(Eigen::VectorXd posOri)
{
	Vec3 tmpVec(posOri(0), posOri(1), posOri(2));
	m_Center = SE3(Exp(posOri(3), posOri(4), posOri(5)), tmpVec);
}

void Table::AssembleModel()
{
	m_numLink = 5;
	m_numCollision = 5;
	m_numWeldJoint = 4;

	m_ObjLink.resize(m_numLink);
	m_ObjCollision.resize(m_numCollision);
	m_ObjWeldJoint.resize(m_numWeldJoint);


	m_tableDim[0] = 0.6;
	m_tableDim[1] = 1.2;
	m_tableDim[2] = 0.1;

	m_tableCenter[0] = 1.0;
	m_tableCenter[1] = 0.0;
	m_tableCenter[2] = m_tableHeight - m_tableDim[2] / 2;

	//Vec3 tableDim(0.6, 1.2, 0.1);
	//Vec3 tableCenter(1.0, 0.0, m_tableHeight - tableDim[2] / 2);//0.8);
	Vec3 legDim(0.2, 0.1, m_tableHeight - m_tableDim[2]);//tableCenter[2]-tableDim[2]/2);

	m_ObjLink[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjLink[0].GetGeomInfo().SetDimension(m_tableDim);
	m_ObjLink[0].GetGeomInfo().SetColor(0.3f, 0.3f, 0.0f, 1.0f);
	for (int i = 1; i < m_numLink; i++)
	{
		m_ObjLink[i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjLink[i].GetGeomInfo().SetDimension(legDim);
		m_ObjLink[i].GetGeomInfo().SetColor(0.3f, 0.3f, 0.0f, 1.0f);
	}

	for (int i = 0; i < m_numWeldJoint; i++)
	{
		m_ObjWeldJoint[i].SetActType(srJoint::PASSIVE);
		m_ObjWeldJoint[i].SetParentLink(&m_ObjLink[0]);
		m_ObjWeldJoint[i].SetChildLink(&m_ObjLink[i + 1]);
		m_ObjWeldJoint[i].SetParentLinkFrame(SE3());
	}
	Vec3 tmpVec((m_tableDim[0] - legDim[0]) / 2, (m_tableDim[1] - legDim[1])/2, (m_tableDim[2] + legDim[2]) / 2);
	m_ObjWeldJoint[0].SetChildLinkFrame(SE3(tmpVec));
	m_ObjWeldJoint[1].SetChildLinkFrame(SE3(Vec3(-tmpVec[0], tmpVec[1], tmpVec[2])));
	m_ObjWeldJoint[2].SetChildLinkFrame(SE3(Vec3(tmpVec[0], -tmpVec[1], tmpVec[2])));
	m_ObjWeldJoint[3].SetChildLinkFrame(SE3(Vec3(-tmpVec[0], -tmpVec[1], tmpVec[2])));


	m_ObjCollision[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[0].GetGeomInfo().SetDimension(m_tableDim);
	m_ObjCollision[0].SetLocalFrame(SE3());
	m_ObjLink[0].AddCollision(&m_ObjCollision[0]);
	for (int i = 1; i < m_numLink; i++)
	{
		m_ObjCollision[i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i].GetGeomInfo().SetDimension(legDim);
		m_ObjCollision[i].SetLocalFrame(SE3());
		m_ObjLink[i].AddCollision(&m_ObjCollision[i]);
	}

	m_ObjLink[0].SetFrame(SE3(m_tableCenter));

	this->SetBaseLink(&m_ObjLink[0]);
	this->SetBaseLinkType(srSystem::KINEMATIC);
	this->SetSelfCollision(false);
}



void Table::setTableHeight(double tableHeight)
{
	m_tableHeight = tableHeight;
	AssembleModel();
}

double Table::getTableHeight()
{
	return m_tableHeight;
}
Vec3 Table::getTableCenter()
{
	return m_tableCenter;
}


void TaskObject::setLocalFramefromGlobal(Eigen::VectorXd posOri)
{
	m_Center = SE3(Exp(posOri(3), posOri(4), posOri(5)), Vec3(posOri(0), posOri(1), posOri(2)));
	
	this->GetBaseLink()->SetFrame(m_Center*m_data2ref);

}

void TaskObject::setTable(Table2 * table)
{
	m_table = table;
}

void TaskObject::setLocalFramefromTable(SE3 frame)
{
	if (m_table != NULL)
	{
		m_Center = m_table->m_Center*frame;
		this->GetBaseLink()->SetFrame(m_Center * m_data2ref);
	}
	else
		m_Center = frame;
}


Supporter1::Supporter1()
{
	m_SupporterBaseLinkDim = Vec3(0.3, 0.06, 0.06);

	// TODO
	m_robot2obj = SE3();
	m_data2ref = SE3();
	AssembleModel();
}
Supporter1::~Supporter1()
{
}

void Supporter1::AssembleModel()
{
	m_numLink = 5;
	m_numCollision = 5;
	m_numWeldJoint = 4;

	m_ObjLink.resize(m_numLink);
	m_ObjCollision.resize(m_numCollision);
	m_ObjWeldJoint.resize(m_numWeldJoint);


	for (int i = 0; i < m_numLink; i++)
	{
		m_ObjLink[i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjLink[i].GetGeomInfo().SetColor(0.1f, 0.1f, 0.1f, 1.0f);
	}

	Vec3 bracketDim(0.03, 0.03, 0.04);
	m_ObjLink[0].GetGeomInfo().SetDimension(m_SupporterBaseLinkDim);
	m_ObjLink[1].GetGeomInfo().SetDimension(m_SupporterBaseLinkDim);
	m_ObjLink[2].GetGeomInfo().SetDimension(m_SupporterBaseLinkDim);
	m_ObjLink[3].GetGeomInfo().SetDimension(bracketDim);
	m_ObjLink[4].GetGeomInfo().SetDimension(bracketDim);
	
	for (int i = 0; i < m_numWeldJoint; i++)
	{
		m_ObjWeldJoint[i].SetActType(srJoint::PASSIVE);
		m_ObjWeldJoint[i].SetParentLink(&m_ObjLink[0]);
		m_ObjWeldJoint[i].SetChildLink(&m_ObjLink[i + 1]);
		m_ObjWeldJoint[i].SetParentLinkFrame(SE3());
	}
	Vec3 tmpVecProfile(0, 0, -m_SupporterBaseLinkDim[2]);
	Vec3 tmpVecBracket(-(m_SupporterBaseLinkDim[0]-bracketDim[0])/2, -(m_SupporterBaseLinkDim[1] - bracketDim[1]) / 2, -(m_SupporterBaseLinkDim[2] + bracketDim[2]) / 2);

	m_ObjWeldJoint[0].SetChildLinkFrame(SE3(tmpVecProfile));
	m_ObjWeldJoint[1].SetChildLinkFrame(SE3(tmpVecProfile));
	m_ObjWeldJoint[2].SetChildLinkFrame(SE3(tmpVecBracket));
	m_ObjWeldJoint[3].SetChildLinkFrame(SE3(Vec3(-tmpVecBracket[0], tmpVecBracket[1], tmpVecBracket[2])));

	double epsilon = 0.001;

	m_ObjCollision[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[0].GetGeomInfo().SetDimension(m_SupporterBaseLinkDim[0], m_SupporterBaseLinkDim[1], m_SupporterBaseLinkDim[2]-epsilon);
	m_ObjCollision[0].SetLocalFrame(SE3(Vec3(0.0, 0.0, epsilon/2)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[0]);

	for (int i = 1; i < 3; i++)
	{
		m_ObjCollision[i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i].GetGeomInfo().SetDimension(m_SupporterBaseLinkDim);
		m_ObjCollision[i].SetLocalFrame(SE3());
		m_ObjLink[i].AddCollision(&m_ObjCollision[i]);
	}

	for (int i = 3; i < m_numLink; i++)
	{
		m_ObjCollision[i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i].GetGeomInfo().SetDimension(bracketDim);
		m_ObjCollision[i].SetLocalFrame(SE3());
		m_ObjLink[i].AddCollision(&m_ObjCollision[i]);
	}


	m_ObjLink[0].SetFrame(m_Center);

	this->SetBaseLink(&m_ObjLink[0]);
	this->SetBaseLinkType(srSystem::KINEMATIC);
	this->SetSelfCollision(false);
}

Supporter2::Supporter2()
{
	m_Supporter2Dim = Vec3(0.2, 0.06, 0.06);

	// TODO
	m_robot2obj = SE3();
	m_data2ref = SE3();
	AssembleModel();
}

Supporter2::~Supporter2()
{
}

void Supporter2::AssembleModel()
{
	m_numLink = 1;
	m_numCollision = 1;
	m_numWeldJoint = 0;

	m_ObjLink.resize(m_numLink);
	m_ObjCollision.resize(m_numCollision);


	for (int i = 0; i < m_numLink; i++)
	{
		m_ObjLink[i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjLink[i].GetGeomInfo().SetColor(0.1f, 0.1f, 0.1f, 1.0f);
	}

	m_ObjLink[0].GetGeomInfo().SetDimension(m_Supporter2Dim);



	double epsilon = 0.001;

	m_ObjCollision[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[0].GetGeomInfo().SetDimension(m_Supporter2Dim[0], m_Supporter2Dim[1], m_Supporter2Dim[2] - epsilon);
	m_ObjCollision[0].SetLocalFrame(SE3(Vec3(0.0, 0.0, epsilon / 2)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[0]);


	m_ObjLink[0].SetFrame(/*m_Supporter2Center*/m_Center);

	this->SetBaseLink(&m_ObjLink[0]);
	this->SetBaseLinkType(srSystem::KINEMATIC);
	this->SetSelfCollision(false);
}


Supporter3::Supporter3()
{
	m_Supporter3Dim = Vec3(0.06, 0.06, 0.38);

	// TODO
	m_robot2obj = SE3();
	m_data2ref = SE3();
	AssembleModel();
}

Supporter3::~Supporter3()
{
}

void Supporter3::AssembleModel()
{
	m_numLink = 1;
	m_numCollision = 1;
	m_numWeldJoint = 0;

	m_ObjLink.resize(m_numLink);
	m_ObjCollision.resize(m_numCollision);


	for (int i = 0; i < m_numLink; i++)
	{
		m_ObjLink[i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjLink[i].GetGeomInfo().SetColor(0.1f, 0.1f, 0.1f, 1.0f);
	}

	m_ObjLink[0].GetGeomInfo().SetDimension(m_Supporter3Dim);



	double epsilon = 0.001;

	m_ObjCollision[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[0].GetGeomInfo().SetDimension(m_Supporter3Dim[0], m_Supporter3Dim[1], m_Supporter3Dim[2] - epsilon);
	m_ObjCollision[0].SetLocalFrame(SE3(Vec3(0.0, 0.0, epsilon / 2)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[0]);


	m_ObjLink[0].SetFrame(/*m_Supporter3Center*/m_Center);

	this->SetBaseLink(&m_ObjLink[0]);
	this->SetBaseLinkType(srSystem::KINEMATIC);
	this->SetSelfCollision(false);
}





Bracket1::Bracket1()
{
	m_Bracket1Dim = Vec3(0.03, 0.03, 0.04);
	
	// TODO
	m_robot2obj = SE3();
	m_data2ref = SE3();
	AssembleModel();
}

Bracket1::~Bracket1()
{
}

void Bracket1::AssembleModel()
{
	m_numLink = 1;
	m_numCollision = 1;
	m_numWeldJoint = 0;

	m_ObjLink.resize(m_numLink);
	m_ObjCollision.resize(m_numCollision);


	for (int i = 0; i < m_numLink; i++)
	{
		m_ObjLink[i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjLink[i].GetGeomInfo().SetColor(0.1f, 0.1f, 0.1f, 1.0f);
	}

	m_ObjLink[0].GetGeomInfo().SetDimension(m_Bracket1Dim);

	

	double epsilon = 0.001;

	m_ObjCollision[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[0].GetGeomInfo().SetDimension(m_Bracket1Dim[0], m_Bracket1Dim[1], m_Bracket1Dim[2] - epsilon);
	m_ObjCollision[0].SetLocalFrame(SE3(Vec3(0.0, 0.0, epsilon / 2)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[0]);


	m_ObjLink[0].SetFrame(/*m_Bracket1Center*/m_Center);

	this->SetBaseLink(&m_ObjLink[0]);
	this->SetBaseLinkType(srSystem::KINEMATIC);
	this->SetSelfCollision(false);
}

Bracket2::Bracket2()
{
	m_Bracket2Dim = Vec3(0.05, 0.06, 0.06);

	// TODO
	m_robot2obj = SE3();
	m_data2ref = SE3();
	AssembleModel();
}

Bracket2::~Bracket2()
{
}

void Bracket2::AssembleModel()
{
	m_numLink = 1;
	m_numCollision = 1;
	m_numWeldJoint = 0;

	m_ObjLink.resize(m_numLink);
	m_ObjCollision.resize(m_numCollision);


	for (int i = 0; i < m_numLink; i++)
	{
		m_ObjLink[i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjLink[i].GetGeomInfo().SetColor(0.1f, 0.1f, 0.1f, 1.0f);
	}

	m_ObjLink[0].GetGeomInfo().SetDimension(m_Bracket2Dim);



	double epsilon = 0.001;

	m_ObjCollision[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[0].GetGeomInfo().SetDimension(m_Bracket2Dim[0], m_Bracket2Dim[1], m_Bracket2Dim[2] - epsilon);
	m_ObjCollision[0].SetLocalFrame(SE3(Vec3(0.0, 0.0, epsilon / 2)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[0]);


	m_ObjLink[0].SetFrame(/*m_Bracket2Center*/m_Center);

	this->SetBaseLink(&m_ObjLink[0]);
	this->SetBaseLinkType(srSystem::KINEMATIC);
	this->SetSelfCollision(false);
}


AcrylicPlate::AcrylicPlate()
{
	m_AcrylicPlateDim = Vec3(0.23, 0.24, 0.002);
	//m_robot2objReach = EulerZYX(Vec3(SR_PI, 0.0, 0.0), Vec3(0.0, 0.12, 0.0));
	//m_robot2obj = EulerZYX(Vec3(SR_PI, 0.0, 0.0), Vec3(0.0, 0.11, 0.0));
	m_robot2objReach = EulerZYX(Vec3(-SR_PI_HALF, 0.0, -SR_PI_HALF), Vec3(0.0, 0.0, 0.12));
	m_robot2obj = EulerZYX(Vec3(-SR_PI_HALF, 0.0, -SR_PI_HALF), Vec3(0.0, 0.0, 0.11));
	m_data2ref = SE3();
	
#ifdef USE_MOCAP_DATA
	//m_robot2obj = EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, -0.13));
	//m_data2ref = EulerZYX(Vec3(0.0, SR_PI_HALF, 0.0), Vec3(-0.008, -0.001, -0.032));

	/* HYU1_1 */
	//m_data2ref[0] = -0.99994500944833242;
	//m_data2ref[1] = -0.010425790066020861;
	//m_data2ref[2] = -0.0011318042560626938;
	//m_data2ref[3] = -0.0011384514333515105;
	//m_data2ref[4] = 0.00063165788062463624;
	//m_data2ref[5] = 0.99999915246796878;
	//m_data2ref[6] = -0.010425066316752180;
	//m_data2ref[7] = 0.99994545046708483;
	//m_data2ref[8] = -0.00064349240105343554;
	//m_data2ref[9] = -0.032306900509522285;
	//m_data2ref[10] = -0.0030569902978911101;
	//m_data2ref[11] = -0.0011236350770703061;

	/* HYU1_2 */
	//m_data2ref[0] = -0.999723995747292;
	//m_data2ref[1] = 0.0163519826586890;
	//m_data2ref[2] = -0.008340945703477;
	//m_data2ref[3] = -0.008922108200853;
	//m_data2ref[4] = -0.039812681023233;
	//m_data2ref[5] = 0.999465288556814;
	//m_data2ref[6] = 0.016454532455190;
	//m_data2ref[7] = 0.998988023393354;
	//m_data2ref[8] = 0.039817326428549;
	//m_data2ref[9] = -0.030979507292778;
	//m_data2ref[10] = -0.003980663663816209;
	//m_data2ref[11] = 0.0003697650663085109;

	/* HYU1_3 */
	m_data2ref[0] = -0.999963086279170;
	m_data2ref[1] = -0.00195845995956697;
	m_data2ref[2] = -0.004142929927054;
	m_data2ref[3] = -0.004096422640295;
	m_data2ref[4] = -0.011010902752553;
	m_data2ref[5] = 0.999861348463224;
	m_data2ref[6] = -0.001782562578510;
	m_data2ref[7] = 0.999885631489225;
	m_data2ref[8] = 0.010902082303992;
	m_data2ref[9] = -0.032023002950197;
	m_data2ref[10] = -0.00009599968308220221;
	m_data2ref[11] = -0.000006932576838798984;

	/* HYU1_4 */
	//m_data2ref[0] = -0.99992356554447348;
	//m_data2ref[1] = -0.012265800559348686;
	//m_data2ref[2] = -0.0015534495374358408;
	//m_data2ref[3] = -0.0015581430684995295;
	//m_data2ref[4] = 0.00037310480399943557;
	//m_data2ref[5] = 0.99999871649066763;
	//m_data2ref[6] = -0.012265205216594015;
	//m_data2ref[7] = 0.99992470262987454;
	//m_data2ref[8] = -0.00039218815805841523;
	//m_data2ref[9] = -0.032268580306946126;
	//m_data2ref[10] = -0.0080088008118033432;
	//m_data2ref[11] = -0.0010632923074240241;

	/* KITECH2_2 */
	//m_data2ref[0] = -0.99994144909832317;
	//m_data2ref[1] = -0.010626446891285318;
	//m_data2ref[2] = -0.0020437714189252896;
	//m_data2ref[3] = -0.0020399515776506599;
	//m_data2ref[4] = -0.00037029213239874988;
	//m_data2ref[5] = 0.99999785073833924;
	//m_data2ref[6] = -0.010627180844747184;
	//m_data2ref[7] = 0.99994346915723387;
	//m_data2ref[8] = 0.00034859301436001786;
	//m_data2ref[9] = -0.033708180482574179;
	//m_data2ref[10] = -0.0034813521746966114;
	//m_data2ref[11] = -0.00016354996161492053;

#endif // USE_MOCAP_DATA

	
	AssembleModel();
}

AcrylicPlate::~AcrylicPlate()
{
}

void AcrylicPlate::AssembleModel()
{

	m_numLink = 2;
	m_numCollision = 1;
	m_numWeldJoint = 1;

	m_ObjLink.resize(m_numLink);
	m_ObjCollision.resize(m_numCollision);
	m_ObjWeldJoint.resize(m_numWeldJoint);

	for (int i = 0; i < m_numLink; i++)
	{
		m_ObjLink[i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjLink[i].GetGeomInfo().SetColor(0.3f, 0.3f, 0.3f, 1.0f);
	}
	Vec3 baseLinkDim(0.0, 0.0, 0.0);
	m_ObjLink[0].GetGeomInfo().SetDimension(baseLinkDim);

	m_ObjLink[1].GetGeomInfo().SetDimension(m_AcrylicPlateDim);

	for (int i = 0; i < m_numWeldJoint; i++)
	{
		m_ObjWeldJoint[i].SetActType(srJoint::PASSIVE);
		m_ObjWeldJoint[i].SetParentLink(&m_ObjLink[0]);
		m_ObjWeldJoint[i].SetChildLink(&m_ObjLink[i + 1]);
		m_ObjWeldJoint[i].SetParentLinkFrame(SE3());
	}

	double epsilon = 0.000;

	m_ObjCollision[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[0].GetGeomInfo().SetDimension(m_AcrylicPlateDim[0], m_AcrylicPlateDim[1], m_AcrylicPlateDim[2] - epsilon);
	m_ObjCollision[0].SetLocalFrame(SE3(Vec3(0.0, 0.0, epsilon / 2)));
	m_ObjLink[1].AddCollision(&m_ObjCollision[0]);


	m_ObjLink[0].SetFrame(m_Center);

	this->SetBaseLink(&m_ObjLink[0]);
	this->SetBaseLinkType(srSystem::KINEMATIC);
	this->SetSelfCollision(false);
}


PCBPlate::PCBPlate()
{
	m_PCBPlateDim = Vec3(0.13, 0.06, 0.002);
	//m_robot2objReach = EulerZYX(Vec3(SR_PI, 0.0, 0.0), Vec3(0.025, 0.03, 0.0));
	//m_robot2obj = EulerZYX(Vec3(SR_PI, 0.0, 0.0), Vec3(0.025, 0.02, 0.0));
	m_robot2objReach = EulerZYX(Vec3(-SR_PI_HALF, 0.0, -SR_PI_HALF), Vec3(0.0, 0.025, 0.03));
	m_robot2obj = EulerZYX(Vec3(-SR_PI_HALF, 0.0, -SR_PI_HALF), Vec3(0.0, 0.025, 0.02));
	m_data2ref = SE3();
#ifdef USE_MOCAP_DATA
	//m_robot2obj = EulerZYX(Vec3(0.0, 0.0, SR_PI), Vec3(0.0, 0.0, -0.02));
	//m_data2ref = EulerZYX(Vec3(0.0, -SR_PI_HALF, 0.0), Vec3(0.011, -0.032, -0.009));

	/* HYU1_1 */
	//m_data2ref[0] = 0.99955612832535701;
	//m_data2ref[1] = 0.013225670963443544;
	//m_data2ref[2] = 0.026695092335289657;
	//m_data2ref[3] = 0.026552730773266409;
	//m_data2ref[4] = 0.010825199340882228;
	//m_data2ref[5] = -0.99958879923082000;
	//m_data2ref[6] = -0.013509212253323195;
	//m_data2ref[7] = 0.99985393767629738;
	//m_data2ref[8] = 0.010469216661134208;
	//m_data2ref[9] = -0.010192534089548993;
	//m_data2ref[10] = -0.0058268652358129825;
	//m_data2ref[11] = 0.029177170706115509;

	/* HYU1_2 */
	//m_data2ref[0] = 0.998585343068410;
	//m_data2ref[1] = 0.0531603214342672;
	//m_data2ref[2] = 0.027776041458810;
	//m_data2ref[3] = 0.028193219488196;
	//m_data2ref[4] = -0.002311856577953;
	//m_data2ref[5] = -0.999765206817783;
	//m_data2ref[6] = -0.053006665541569;
	//m_data2ref[7] = 0.998800104006818;
	//m_data2ref[8] = -0.003419253740360;
	//m_data2ref[9] = -0.009225291412630;
	//m_data2ref[10] = -0.0004096976978560521;
	//m_data2ref[11] = 0.028927836459490;

	/* HYU1_3 */
	m_data2ref[0] = 0.998280396760548;
	m_data2ref[1] = 0.038237755943844;
	m_data2ref[2] = 0.021003911114303;
	m_data2ref[3] = 0.019535244034961;
	m_data2ref[4] = 0.034751434843081;
	m_data2ref[5] = -0.999266975283115;
	m_data2ref[6] = -0.039346464273763;
	m_data2ref[7] = 0.998321975246924;
	m_data2ref[8] = 0.033748031026290;
	m_data2ref[9] = -0.009166208202220;
	m_data2ref[10] = -0.001385889910071;
	m_data2ref[11] = 0.029605374527901;

	/* HYU1_4 */
	//m_data2ref[0] = 0.999432368919602;
	//m_data2ref[1] = 0.029811544182052;
	//m_data2ref[2] = 0.010090303039932;
	//m_data2ref[3] = 0.00957883463957736;
	//m_data2ref[4] = 0.029308171203992;
	//m_data2ref[5] = -1.00005384141200;
	//m_data2ref[6] = -0.029789561173372;
	//m_data2ref[7] = 0.999553499089501;
	//m_data2ref[8] = 0.0291078567780300;
	//m_data2ref[9] = -0.009524761100771;
	//m_data2ref[10] = -0.0008589071960929154;
	//m_data2ref[11] = 0.0290915108487229;

	/* KITECH2_2 */
	//m_data2ref[0] = 0.99991315346463250;
	//m_data2ref[1] = -0.0035354446147146438;
	//m_data2ref[2] = 0.012695911144536330;
	//m_data2ref[3] = 0.012676903519404308;
	//m_data2ref[4] = -0.0053477935139336803;
	//m_data2ref[5] = -0.99990534413097920;
	//m_data2ref[6] = 0.0036030050754043870;
	//m_data2ref[7] = 0.99997945065681659;
	//m_data2ref[8] = -0.0053025105863163521;
	//m_data2ref[9] = -0.011122340347525394;
	//m_data2ref[10] = -0.0080149332698183362;
	//m_data2ref[11] = 0.033324477128293679;

#endif // USE_MOCAP_DATA

	
	AssembleModel();
}

PCBPlate::~PCBPlate()
{
}

void PCBPlate::AssembleModel()
{

	m_numLink = 2;
	m_numCollision = 1;
	m_numWeldJoint = 1;

	m_ObjLink.resize(m_numLink);
	m_ObjCollision.resize(m_numCollision);
	m_ObjWeldJoint.resize(m_numWeldJoint);

	for (int i = 0; i < m_numLink; i++)
	{
		m_ObjLink[i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjLink[i].GetGeomInfo().SetColor(0.1f, 0.1f, 0.1f, 1.0f);
	}

	Vec3 baseLinkDim(0.0, 0.0, 0.0);
	m_ObjLink[0].GetGeomInfo().SetDimension(baseLinkDim);
	m_ObjLink[1].GetGeomInfo().SetDimension(m_PCBPlateDim);

	for (int i = 0; i < m_numWeldJoint; i++)
	{
		m_ObjWeldJoint[i].SetActType(srJoint::PASSIVE);
		m_ObjWeldJoint[i].SetParentLink(&m_ObjLink[0]);
		m_ObjWeldJoint[i].SetChildLink(&m_ObjLink[i + 1]);
		m_ObjWeldJoint[i].SetParentLinkFrame(SE3());
	}

	double epsilon = 0.000;

	m_ObjCollision[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[0].GetGeomInfo().SetDimension(m_PCBPlateDim[0], m_PCBPlateDim[1], m_PCBPlateDim[2] - epsilon);
	m_ObjCollision[0].SetLocalFrame(SE3(Vec3(0.0, 0.0, epsilon / 2)));
	m_ObjLink[1].AddCollision(&m_ObjCollision[0]);


	m_ObjLink[0].SetFrame(m_Center);

	this->SetBaseLink(&m_ObjLink[0]);
	this->SetBaseLinkType(srSystem::KINEMATIC);
	this->SetSelfCollision(false);
}


ScrewPlate::ScrewPlate()
{
	m_ScrewPlateDim = Vec3(0.17, 0.098, 0.02);

	// TODO
	m_robot2obj = SE3();
	m_data2ref = SE3();

#ifdef USE_MOCAP_DATA
	m_data2ref[0] = -0.00075183200167183717;
	m_data2ref[1] = -0.0018403482630915957;
	m_data2ref[2] = -0.99999802393150361;
	m_data2ref[3] = -0.99999197724840139;
	m_data2ref[4] = -0.0039330977658864064;
	m_data2ref[5] = 0.00075906573951850520;
	m_data2ref[6] = -0.0039344869391311765;
	m_data2ref[7] = 0.99999057188567275;
	m_data2ref[8] = -0.0018373764696684169;
	m_data2ref[9] = 0;
	m_data2ref[10] = 0;
	m_data2ref[11] = 0;
#endif // USE_MOCAP_DATA

	
	AssembleModel();
}

ScrewPlate::~ScrewPlate()
{
}

void ScrewPlate::AssembleModel()
{

	m_numLink = 2;
	m_numCollision = 1;
	m_numWeldJoint = 1;

	m_ObjLink.resize(m_numLink);
	m_ObjCollision.resize(m_numCollision);
	m_ObjWeldJoint.resize(m_numWeldJoint);

	for (int i = 0; i < m_numLink; i++)
	{
		m_ObjLink[i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjLink[i].GetGeomInfo().SetColor(0.1f, 0.1f, 0.1f, 1.0f);
	}

	Vec3 baseLinkDim(0.0, 0.0, 0.0);
	m_ObjLink[0].GetGeomInfo().SetDimension(baseLinkDim);

	m_ObjLink[1].GetGeomInfo().SetDimension(m_ScrewPlateDim);

	for (int i = 0; i < m_numWeldJoint; i++)
	{
		m_ObjWeldJoint[i].SetActType(srJoint::PASSIVE);
		m_ObjWeldJoint[i].SetParentLink(&m_ObjLink[0]);
		m_ObjWeldJoint[i].SetChildLink(&m_ObjLink[i + 1]);
		m_ObjWeldJoint[i].SetParentLinkFrame(SE3());
	}

	double epsilon = 0.001;

	m_ObjCollision[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[0].GetGeomInfo().SetDimension(m_ScrewPlateDim[0], m_ScrewPlateDim[1], m_ScrewPlateDim[2] - epsilon);
	m_ObjCollision[0].SetLocalFrame(SE3(Vec3(0.0, 0.0, epsilon / 2)));
	m_ObjLink[1].AddCollision(&m_ObjCollision[0]);


	m_ObjLink[0].SetFrame(m_Center);

	this->SetBaseLink(&m_ObjLink[0]);
	this->SetBaseLinkType(srSystem::KINEMATIC);
	this->SetSelfCollision(false);
}

Drill::Drill()
{
	m_DrillDim = Vec3(0.400, 0.02, 0.02);
	m_DrillHandleDim = Vec3(0.25, 0.05, 0.05);
	// TODO
	m_robot2obj = EulerXYZ(Vec3(SR_PI_HALF, -SR_PI_HALF, 0.0), Vec3(0.2, -0.00, 0.0500));
	m_data2ref = SE3();
#ifdef USE_MOCAP_DATA
	m_data2ref[0] = 0.096963773594410885;
	m_data2ref[1] = 0.071593109271857844;
	m_data2ref[2] = -0.99270965207109763;
	m_data2ref[3] = -0.86122497505797102;
	m_data2ref[4] = -0.49391648283160583;
	m_data2ref[5] = -0.11974159813386687;
	m_data2ref[6] = -0.49888833314352854;
	m_data2ref[7] = 0.86655694255602378;
	m_data2ref[8] = 0.013765767731146815;
	m_data2ref[9] = 0.015266067929789628;
	m_data2ref[10] = 0.0046741726408392643;
	m_data2ref[11] = 0.11271630183132847;
#endif // USE_MOCAP_DATA

	
	AssembleModel();
}

Drill::~Drill()
{
}

void Drill::AssembleModel()
{

	m_numLink = 3;
	m_numCollision = 1;
	m_numWeldJoint = 2;

	m_ObjLink.resize(m_numLink);
	m_ObjCollision.resize(m_numCollision);
	m_ObjWeldJoint.resize(m_numWeldJoint);

	for (int i = 0; i < m_numLink; i++)
	{
		m_ObjLink[i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjLink[i].GetGeomInfo().SetColor(0.1f, 0.1f, 0.1f, 1.0f);
	}

	m_ObjLink[0].GetGeomInfo().SetDimension(Vec3(0.01, 0.01, 0.01));
	m_ObjLink[1].GetGeomInfo().SetDimension(m_DrillDim);
	m_ObjLink[2].GetGeomInfo().SetDimension(m_DrillHandleDim);

	m_ObjWeldJoint[0].SetParentLink(&m_ObjLink[0]);
	m_ObjWeldJoint[0].SetParentLinkFrame(SE3());
	m_ObjWeldJoint[0].SetChildLink(&m_ObjLink[1]);
	m_ObjWeldJoint[0].SetChildLinkFrame(SE3(Vec3( m_DrillDim[0] * 1.0 / 3.0, 0.0, 0.0)));
	double epsilon = 0.001;

	m_ObjWeldJoint[1].SetParentLink(&m_ObjLink[0]);
	m_ObjWeldJoint[1].SetParentLinkFrame(SE3());
	m_ObjWeldJoint[1].SetChildLink(&m_ObjLink[2]);
	m_ObjWeldJoint[1].SetChildLinkFrame(SE3(Vec3(0.5*m_DrillHandleDim[0] - m_DrillDim[0] / 6.0, 0.0, 0.0)));

	m_ObjCollision[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[0].GetGeomInfo().SetDimension(m_DrillDim[0], m_DrillDim[1], m_DrillDim[2] - epsilon);
	m_ObjCollision[0].SetLocalFrame(SE3(Vec3(0.0, 0.0, epsilon / 2)));
	m_ObjLink[1].AddCollision(&m_ObjCollision[0]);


	m_ObjLink[0].SetFrame(m_Center);

	this->SetBaseLink(&m_ObjLink[0]);
	this->SetBaseLinkType(srSystem::KINEMATIC);
	this->SetSelfCollision(false);
}

AlProfile::AlProfile()
{
	m_AlProfileDim = Vec3(0.31, 0.06, 0.18);
	m_bracketDim = Vec3(0.03, 0.03, 0.04);
	// TODO
	m_robot2obj = SE3();
	m_data2ref = SE3();
#ifdef USE_MOCAP_DATA
	m_data2ref[0] = 0.012018530533228045;
	m_data2ref[1] = -0.0010825658211479627;
	m_data2ref[2] = -0.99992718883679965;
	m_data2ref[3] = -0.99992507182498924;
	m_data2ref[4] = 0.0023121622618454962;
	m_data2ref[5] = -0.012021008338106542;
	m_data2ref[6] = 0.0023250074433843748;
	m_data2ref[7] = 0.99999674097314828;
	m_data2ref[8] = -0.0010546959137250345;
	m_data2ref[9] = 0;
	m_data2ref[10] = 0;
	m_data2ref[11] = 0;
#endif // USE_MOCAP_DATA

	
	AssembleModel();
}

AlProfile::~AlProfile()
{
}

void AlProfile::AssembleModel()
{
	m_numLink = 7;
	m_numCollision = 6;
	m_numWeldJoint = 6;

	m_ObjLink.resize(m_numLink);
	m_ObjCollision.resize(m_numCollision);
	m_ObjWeldJoint.resize(m_numWeldJoint);


	for (int i = 0; i < m_numLink; i++)
	{
		m_ObjLink[i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjLink[i].GetGeomInfo().SetColor(0.1f, 0.1f, 0.1f, 1.0f);
	}

	int numProfile = 2;
	int numBracket = 4;

	
	Vec3 baseLinkDim(0.0, 0.0, 0.0); 
	m_ObjLink[0].GetGeomInfo().SetDimension(baseLinkDim);
	for (int i = 1; i < numProfile+1; i++)
		m_ObjLink[i].GetGeomInfo().SetDimension(m_AlProfileDim);
	for (int i = numProfile+1; i < m_numLink;i++)
		m_ObjLink[i].GetGeomInfo().SetDimension(m_bracketDim);

	for (int i = 0; i < m_numWeldJoint; i++)
	{
		m_ObjWeldJoint[i].SetActType(srJoint::PASSIVE);
		m_ObjWeldJoint[i].SetParentLink(&m_ObjLink[0]);
		m_ObjWeldJoint[i].SetChildLink(&m_ObjLink[i + 1]);
		m_ObjWeldJoint[i].SetParentLinkFrame(SE3());
	}
	
	Vec3 tmpBase2Profile(0, -(0.205 + m_AlProfileDim[1] )/2, m_AlProfileDim[2]/2); //left al profile
	Vec3 tmpBase2Bracket(-(m_AlProfileDim[0] - m_bracketDim[0]) / 2, -(0.205 + m_bracketDim[1]) / 2, -m_bracketDim[2]/2); // left front al profile

	m_ObjWeldJoint[0].SetChildLinkFrame(SE3(tmpBase2Profile));
	m_ObjWeldJoint[1].SetChildLinkFrame(SE3(Vec3(tmpBase2Profile[0], -tmpBase2Profile[1], tmpBase2Profile[2])));
	m_ObjWeldJoint[2].SetChildLinkFrame(SE3(tmpBase2Bracket));
	m_ObjWeldJoint[3].SetChildLinkFrame(SE3(Vec3(-tmpBase2Bracket[0], tmpBase2Bracket[1], tmpBase2Bracket[2])));
	m_ObjWeldJoint[4].SetChildLinkFrame(SE3(Vec3(tmpBase2Bracket[0], -tmpBase2Bracket[1], tmpBase2Bracket[2])));
	m_ObjWeldJoint[5].SetChildLinkFrame(SE3(Vec3(-tmpBase2Bracket[0], -tmpBase2Bracket[1], tmpBase2Bracket[2])));


	for (int i = 1; i < 1+ numProfile; i++)
	{
		m_ObjCollision[i-1].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i-1].GetGeomInfo().SetDimension(m_AlProfileDim);
		m_ObjCollision[i-1].SetLocalFrame(SE3());
		m_ObjLink[i].AddCollision(&m_ObjCollision[i-1]);
	}
	for (int i = 1 + numProfile; i < m_numLink; i++)
	{
		m_ObjCollision[i - 1].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		m_ObjCollision[i - 1].GetGeomInfo().SetDimension(m_bracketDim);
		m_ObjCollision[i - 1].SetLocalFrame(SE3());
		m_ObjLink[i].AddCollision(&m_ObjCollision[i - 1]);
	}

	m_ObjLink[0].SetFrame(m_Center);

	this->SetBaseLink(&m_ObjLink[0]);
	this->SetBaseLinkType(srSystem::KINEMATIC);
	this->SetSelfCollision(false);
}

Table2::Table2()
{
	//m_tableHeight = 0.0; // temp
	m_tableHeight = -0.09; // real setting
	AssembleModel();
}

Table2::~Table2()
{
}

void Table2::setLocalFramefromGlobal(Eigen::VectorXd posOri)
{
	Vec3 tmpVec(posOri(0), posOri(1), posOri(2));
	m_Center = SE3(Exp(posOri(3), posOri(4), posOri(5)), tmpVec + Vec3(0.0, 0.0, m_tableDim[2] * 0.5));
	this->GetBaseLink()->SetFrame(SE3(Exp(posOri(3), posOri(4), posOri(5)), tmpVec));
}

void Table2::AssembleModel()
{
	m_numLink = 2;
	m_numCollision = 1;
	m_numWeldJoint = 1;

	m_ObjLink.resize(m_numLink);
	m_ObjCollision.resize(m_numCollision);
	m_ObjWeldJoint.resize(m_numWeldJoint);


	m_tableDim[0] = 0.6;
	m_tableDim[1] = 1.2;
	m_tableDim[2] = 0.1;

	
	m_tableCenter[0] = 0.22 + 0.175 + 0.5*m_tableDim[0];	// real setting
	//m_tableCenter[0] = 0.5;	// temp
	m_tableCenter[1] = 0.0;
	m_tableCenter[2] = m_tableHeight - m_tableDim[2] / 2;

	//Vec3 tableDim(0.6, 1.2, 0.1);
	//Vec3 tableCenter(1.0, 0.0, m_tableHeight - tableDim[2] / 2);//0.8);
	//Vec3 legDim(0.2, 0.1, m_tableHeight - m_tableDim[2]);//tableCenter[2]-tableDim[2]/2);

	Vec3 baseLinkDim(0.0, 0.0, 0.0);
	m_ObjLink[0].GetGeomInfo().SetDimension(baseLinkDim);

	m_ObjLink[1].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjLink[1].GetGeomInfo().SetDimension(m_tableDim);
	m_ObjLink[1].GetGeomInfo().SetColor(0.3f, 0.3f, 0.0f, 1.0f);
	
	for (int i = 0; i < m_numWeldJoint; i++)
	{
		m_ObjWeldJoint[i].SetActType(srJoint::PASSIVE);
		m_ObjWeldJoint[i].SetParentLink(&m_ObjLink[0]);
		m_ObjWeldJoint[i].SetChildLink(&m_ObjLink[i + 1]);
		m_ObjWeldJoint[i].SetParentLinkFrame(SE3());
	}
	
	m_ObjCollision[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[0].GetGeomInfo().SetDimension(m_tableDim);
	m_ObjCollision[0].SetLocalFrame(SE3());
	m_ObjLink[1].AddCollision(&m_ObjCollision[0]);
	

	m_ObjLink[0].SetFrame(SE3(m_tableCenter));
	m_Center = SE3(m_tableCenter + Vec3(0, 0, m_tableDim[2] * 0.5));
	this->SetBaseLink(&m_ObjLink[0]);
	this->SetBaseLinkType(srSystem::KINEMATIC);
	this->SetSelfCollision(false);
}



void Table2::setTableHeight(double tableHeight)
{
	m_tableHeight = tableHeight;
	AssembleModel();
}

double Table2::getTableHeight()
{
	return m_tableHeight;
}
Vec3 Table2::getTableCenter()
{
	return m_tableCenter;
}