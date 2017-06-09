#include "makeSpecialCol.h"
#include "common\utils.h"
#include "environment_workcell.h"

WorkCell::WorkCell(int mode)
{
	m_mode = mode;
	AssembleModel();
}

WorkCell::~WorkCell()
{
	for (unsigned int i = 0; i < rJoint.size(); i++)
		SR_SAFE_DELETE(rJoint[i]);
	for (unsigned int i = 0; i < pJoint.size(); i++)
		SR_SAFE_DELETE(pJoint[i]);
}

void WorkCell::AssembleModel()
{
	m_numLink = 100;
	m_numCollision = 100;
	m_numWeldJoint = 100;
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

	SE3 TsrLib2cad = EulerZYX(Vec3(SR_PI_HALF, 0.0, -SR_PI_HALF), Vec3(0.670, -4.70334, 1.52486));

	// cell frame
	m_ObjLink[0].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	m_ObjLink[0].GetGeomInfo().SetLocalFrame(TsrLib2cad);
	m_ObjLink[0].GetGeomInfo().SetFileName("../../../workspace/robot/workcell_3ds/cellframe.3ds");
	m_ObjLink[0].GetGeomInfo().SetColor(0.35f, 0.35f, 0.35f, 1.0f);

	m_ObjLink[8].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	m_ObjLink[8].GetGeomInfo().SetLocalFrame(TsrLib2cad);
	m_ObjLink[8].GetGeomInfo().SetFileName("../../../workspace/robot/workcell_3ds/robot_base1.3ds");
	m_ObjLink[8].GetGeomInfo().SetColor(0.35f, 0.35f, 0.35f, 1.0f);

	m_ObjLink[9].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	m_ObjLink[9].GetGeomInfo().SetLocalFrame(TsrLib2cad);
	m_ObjLink[9].GetGeomInfo().SetFileName("../../../workspace/robot/workcell_3ds/robot_base2.3ds");
	m_ObjLink[9].GetGeomInfo().SetColor(0.35f, 0.35f, 0.35f, 1.0f);

	m_ObjWeldJoint[4].SetParentLink(&m_ObjLink[0]);
	m_ObjWeldJoint[4].SetChildLink(&m_ObjLink[8]);
	m_ObjWeldJoint[4].SetChildLinkFrame(SE3(Vec3(0.0, 0.12, 0.0)));

	m_ObjWeldJoint[5].SetParentLink(&m_ObjLink[0]);
	m_ObjWeldJoint[5].SetChildLink(&m_ObjLink[9]);
	m_ObjWeldJoint[5].SetChildLinkFrame(SE3(Vec3(0.0, -0.12, 0.0)));

	m_ObjCollision[0].GetGeomInfo().SetDimension(Vec3(1.5, 4.5, 0.91));
	m_ObjCollision[0].SetLocalFrame(SE3(Vec3(0.0, -0.25, 0.455)));
	m_ObjCollision[11].GetGeomInfo().SetDimension(Vec3(0.559, 1.857, 0.04));
	m_ObjCollision[11].SetLocalFrame(SE3(Vec3(0.44, 1.0005, 2.061)));
	m_ObjCollision[12].GetGeomInfo().SetDimension(Vec3(0.559, 1.857, 0.04));
	m_ObjCollision[12].SetLocalFrame(SE3(Vec3(-0.399, 1.0005, 2.061)));
	m_ObjCollision[13].GetGeomInfo().SetDimension(Vec3(1.398, 0.04, 0.04));
	m_ObjCollision[13].SetLocalFrame(SE3(Vec3(0.0205, 0.092, 2.061)));
	m_ObjCollision[14].GetGeomInfo().SetDimension(Vec3(1.398, 0.04, 0.04));
	m_ObjCollision[14].SetLocalFrame(SE3(Vec3(0.0205, 1.909, 2.061)));

	m_ObjCollision[15].GetGeomInfo().SetDimension(Vec3(0.04, 0.04, 1.131));
	m_ObjCollision[15].SetLocalFrame(SE3(Vec3(0.6995, 0.092, 1.4755)));
	m_ObjCollision[16].GetGeomInfo().SetDimension(Vec3(0.04, 0.04, 1.131));
	m_ObjCollision[16].SetLocalFrame(SE3(Vec3(-0.6585, 0.092, 1.4755)));
	m_ObjCollision[17].GetGeomInfo().SetDimension(Vec3(0.04, 0.04, 1.131));
	m_ObjCollision[17].SetLocalFrame(SE3(Vec3(0.6995, 1.909, 1.4755)));
	m_ObjCollision[18].GetGeomInfo().SetDimension(Vec3(0.04, 0.04, 1.131));
	m_ObjCollision[18].SetLocalFrame(SE3(Vec3(-0.6585, 1.909, 1.4755)));
	m_ObjCollision[19].GetGeomInfo().SetDimension(Vec3(0.04, 0.04, 1.131));
	m_ObjCollision[19].SetLocalFrame(SE3(Vec3(0.0205, 1.909, 1.4755)));
	m_ObjCollision[20].GetGeomInfo().SetDimension(Vec3(0.04, 0.04, 1.131));
	m_ObjCollision[20].SetLocalFrame(SE3(Vec3(-0.6585, 0.9805, 1.4755)));

	m_ObjLink[0].AddCollision(&m_ObjCollision[0]);
	m_ObjLink[0].AddCollision(&m_ObjCollision[11]);
	m_ObjLink[0].AddCollision(&m_ObjCollision[12]);
	m_ObjLink[0].AddCollision(&m_ObjCollision[13]);
	m_ObjLink[0].AddCollision(&m_ObjCollision[14]);
	m_ObjLink[0].AddCollision(&m_ObjCollision[15]);
	m_ObjLink[0].AddCollision(&m_ObjCollision[16]);
	m_ObjLink[0].AddCollision(&m_ObjCollision[17]);
	m_ObjLink[0].AddCollision(&m_ObjCollision[18]);
	m_ObjLink[0].AddCollision(&m_ObjCollision[19]);
	m_ObjLink[0].AddCollision(&m_ObjCollision[20]);

	m_ObjCollision[21].GetGeomInfo().SetDimension(Vec3(0.26, 0.26, 0.108));
	m_ObjCollision[21].SetLocalFrame(SE3(Vec3(0.0205, 0.4005, 2.027)));
	m_ObjCollision[22].GetGeomInfo().SetDimension(Vec3(0.26, 0.26, 0.108));
	m_ObjCollision[22].SetLocalFrame(SE3(Vec3(0.0205, 1.6005, 2.027)));

	m_ObjLink[8].AddCollision(&m_ObjCollision[21]);
	m_ObjLink[9].AddCollision(&m_ObjCollision[22]);

	//// Collision
	////// left first bar
	m_ObjCollision[61].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[61].GetGeomInfo().SetDimension(Vec3(0.04, 0.08, 1.230));
	m_ObjCollision[61].SetLocalFrame(SE3(Vec3(0.749 - 0.04*0.5, 0.0, 0.910 + 1.230*0.5)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[61]);
	////// left second bar
	m_ObjCollision[62].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[62].GetGeomInfo().SetDimension(Vec3(0.04, 0.08, 1.230));
	m_ObjCollision[62].SetLocalFrame(SE3(Vec3(0.749 - 0.04 - 0.2895 - 0.04*0.5, 0.0, 0.910 + 1.230*0.5)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[62]);
	////// left third bar
	m_ObjCollision[63].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[63].GetGeomInfo().SetDimension(Vec3(0.04, 0.08, 1.230));
	m_ObjCollision[63].SetLocalFrame(SE3(Vec3(0.749 - 0.04 - 0.2895 - 0.04 - 0.760 - 0.04*0.5, 0.0, 0.910 + 1.230*0.5)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[63]);
	////// left fourth bar
	m_ObjCollision[64].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[64].GetGeomInfo().SetDimension(Vec3(0.04, 0.08, 1.230));
	m_ObjCollision[64].SetLocalFrame(SE3(Vec3(0.749 - 0.04 - 0.2895 - 0.04 - 0.760 - 0.04 - 0.2895 - 0.04*0.5, 0.0, 0.910 + 1.230*0.5)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[64]);
	////// left bottom bar
	m_ObjCollision[65].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[65].GetGeomInfo().SetDimension(Vec3(1.499, 0.08, 0.04));
	m_ObjCollision[65].SetLocalFrame(SE3(Vec3(0.0, 0.0, 0.910 + 0.04*0.5)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[65]);
	////// left top bar
	m_ObjCollision[66].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[66].GetGeomInfo().SetDimension(Vec3(1.499, 0.08, 0.04));
	m_ObjCollision[66].SetLocalFrame(SE3(Vec3(0.0, 0.0, 0.910 + 1.230 - 0.04*0.5)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[66]);
	////// right first bar
	m_ObjCollision[67].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[67].GetGeomInfo().SetDimension(Vec3(0.04, 0.04, 1.230));
	m_ObjCollision[67].SetLocalFrame(SE3(Vec3(0.749 - 0.04*0.5, 1.959 + 0.04*0.5, 0.910 + 1.230*0.5)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[67]);
	////// right second bar
	m_ObjCollision[68].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[68].GetGeomInfo().SetDimension(Vec3(0.04, 0.04, 1.230));
	m_ObjCollision[68].SetLocalFrame(SE3(Vec3(0.749 - 0.04 - 0.6895 - 0.04*0.5, 1.959 + 0.04*0.5, 0.910 + 1.230*0.5)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[68]);
	////// right third bar
	m_ObjCollision[69].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[69].GetGeomInfo().SetDimension(Vec3(0.04, 0.04, 1.230));
	m_ObjCollision[69].SetLocalFrame(SE3(Vec3(0.749 - 0.04 - 0.6895 - 0.04 - 0.6895 - 0.04*0.5, 1.959 + 0.04*0.5, 0.910 + 1.230*0.5)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[69]);
	////// right bottom bar
	m_ObjCollision[70].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[70].GetGeomInfo().SetDimension(Vec3(1.499, 0.04, 0.04));
	m_ObjCollision[70].SetLocalFrame(SE3(Vec3(0.0, 1.959 + 0.04*0.5, 0.910 + 0.04*0.5)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[70]);
	////// right top bar
	m_ObjCollision[71].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[71].GetGeomInfo().SetDimension(Vec3(1.499, 0.04, 0.04));
	m_ObjCollision[71].SetLocalFrame(SE3(Vec3(0.0, 1.959 + 0.04*0.5, 0.910 + 1.230 - 0.04*0.5)));
	m_ObjLink[0].AddCollision(&m_ObjCollision[71]);

	// conveyer belt
	m_ObjLink[1].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	m_ObjLink[1].GetGeomInfo().SetLocalFrame(TsrLib2cad);
	m_ObjLink[1].GetGeomInfo().SetFileName("../../../workspace/robot/workcell_3ds/Conveyor1.3ds");
	m_ObjLink[1].GetGeomInfo().SetColor(0.35f, 0.35f, 0.35f, 1.0f);
	m_ObjWeldJoint[0].SetParentLink(&m_ObjLink[0]);
	m_ObjWeldJoint[0].SetChildLink(&m_ObjLink[1]);

	m_ObjCollision[1].GetGeomInfo().SetDimension(Vec3(0.51, 2.068, 0.1511));
	m_ObjLink[1].AddCollision(&m_ObjCollision[1]);
	double trans_x = 0.01;
	m_ObjCollision[1].SetLocalFrame(SE3(Vec3(-0.005 + trans_x, -0.29972, 1.03555)));
	m_ObjCollision[72].GetGeomInfo().SetDimension(Vec3(0.01, 0.02, 0.05));
	m_ObjCollision[72].SetLocalFrame(SE3(Vec3(0.5*(0.44 - 0.01) - 0.005 + trans_x, 0.5*2.068 - 0.175 - 0.29972, 0.5*(0.1511 + 0.05)+ 1.03555)));
	m_ObjCollision[73].GetGeomInfo().SetDimension(Vec3(0.01, 0.02, 0.05));
	m_ObjCollision[73].SetLocalFrame(SE3(Vec3(-0.5*(0.44 - 0.01) - 0.005, 0.5*2.068 - 0.175 - 0.29972, 0.5*(0.1511 + 0.05)+ 1.03555)));
	m_ObjLink[1].AddCollision(&m_ObjCollision[72]);
	m_ObjLink[1].AddCollision(&m_ObjCollision[73]);
	// tool frame
	m_ObjLink[2].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	m_ObjLink[2].GetGeomInfo().SetLocalFrame(TsrLib2cad);
	m_ObjLink[2].GetGeomInfo().SetFileName("../../../workspace/robot/workcell_3ds/toolframe.3ds");
	m_ObjLink[2].GetGeomInfo().SetColor(0.35f, 0.35f, 0.35f, 1.0f);
	m_ObjWeldJoint[1].SetParentLink(&m_ObjLink[0]);
	m_ObjWeldJoint[1].SetChildLink(&m_ObjLink[2]);
	//// Collision
	//// UPPER PLATE
	////// left first bar
	m_ObjCollision[50].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[50].GetGeomInfo().SetDimension(Vec3(0.1225, 0.045, 0.01));
	m_ObjCollision[50].SetLocalFrame(SE3(Vec3(-0.3995 - 0.1225*0.5, 0.780 + 0.045*0.5, 1.120 - 0.005)));
	m_ObjLink[2].AddCollision(&m_ObjCollision[50]);
	////// Second bar
	m_ObjCollision[51].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[51].GetGeomInfo().SetDimension(Vec3(0.1225, 0.050, 0.01));
	m_ObjCollision[51].SetLocalFrame(SE3(Vec3(-0.3995 - 0.1225*0.5, 0.780 + 0.045 + 0.05 + 0.05*0.5, 1.120 - 0.005)));
	m_ObjLink[2].AddCollision(&m_ObjCollision[51]);
	////// Third bar
	m_ObjCollision[52].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[52].GetGeomInfo().SetDimension(Vec3(0.1225, 0.050, 0.01));
	m_ObjCollision[52].SetLocalFrame(SE3(Vec3(-0.3995 - 0.1225*0.5, 0.780 + 0.045 + 0.05 + 0.05 + 0.05 + 0.05*0.5, 1.120 - 0.005)));
	m_ObjLink[2].AddCollision(&m_ObjCollision[52]);
	////// Fourth bar
	m_ObjCollision[53].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[53].GetGeomInfo().SetDimension(Vec3(0.180, 0.050, 0.01));
	m_ObjCollision[53].SetLocalFrame(SE3(Vec3(-0.3995 - 0.180*0.5, 0.780 + 0.045 + 0.05 + 0.05 + 0.05 + 0.05 + 0.05 + 0.05*0.5, 1.120 - 0.005)));
	m_ObjLink[2].AddCollision(&m_ObjCollision[53]);
	////// Fifth bar
	m_ObjCollision[54].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[54].GetGeomInfo().SetDimension(Vec3(0.180, 0.045, 0.01));
	m_ObjCollision[54].SetLocalFrame(SE3(Vec3(-0.3995 - 0.180*0.5, 0.780 + 0.045 + 0.05 + 0.05 + 0.05 + 0.05 + 0.05 + 0.05 + 0.05 + 0.045*0.5, 1.120 - 0.005)));
	m_ObjLink[2].AddCollision(&m_ObjCollision[54]);
	////// left big plate
	m_ObjCollision[55].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[55].GetGeomInfo().SetDimension(Vec3(0.200 - 0.1225, 0.245, 0.01));
	m_ObjCollision[55].SetLocalFrame(SE3(Vec3(-0.3995 - 0.200 + (0.200 - 0.1225)*0.5, 0.780 + 0.245*0.5, 1.120 - 0.005)));
	m_ObjLink[2].AddCollision(&m_ObjCollision[55]);
	////// RIGHT big plate
	m_ObjCollision[56].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[56].GetGeomInfo().SetDimension(Vec3(0.200 - 0.180, 0.195, 0.01));
	m_ObjCollision[56].SetLocalFrame(SE3(Vec3(-0.3995 - 0.200 + 0.02*0.5, 0.780 + 0.245 + 0.195*0.5, 1.120 - 0.005)));
	m_ObjLink[2].AddCollision(&m_ObjCollision[56]);
	//// Legs
	////// Left front
	m_ObjCollision[57].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[57].GetGeomInfo().SetDimension(Vec3(0.060, 0.030, 0.200));
	m_ObjCollision[57].SetLocalFrame(SE3(Vec3(-0.3985 - 0.06*0.5, 0.781 + 0.03*0.5, 0.910 + 0.200*0.5)));
	m_ObjLink[2].AddCollision(&m_ObjCollision[57]);
	////// Left back
	m_ObjCollision[58].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[58].GetGeomInfo().SetDimension(Vec3(0.060, 0.030, 0.200));
	m_ObjCollision[58].SetLocalFrame(SE3(Vec3(-0.3985 - 0.06 - 0.08 - 0.060*0.5, 0.781 + 0.03*0.5, 0.910 + 0.200*0.5)));
	m_ObjLink[2].AddCollision(&m_ObjCollision[58]);
	////// Right front
	m_ObjCollision[59].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[59].GetGeomInfo().SetDimension(Vec3(0.060, 0.030, 0.200));
	m_ObjCollision[59].SetLocalFrame(SE3(Vec3(-0.3985 - 0.06*0.5, 0.781 + 0.380 + 0.03*0.5 + 0.03, 0.910 + 0.200*0.5)));
	m_ObjLink[2].AddCollision(&m_ObjCollision[59]);
	////// Right back
	m_ObjCollision[60].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_ObjCollision[60].GetGeomInfo().SetDimension(Vec3(0.060, 0.030, 0.200));
	m_ObjCollision[60].SetLocalFrame(SE3(Vec3(-0.3985 - 0.06 - 0.08 - 0.060*0.5, 0.781 + 0.380 + 0.03*0.5 + 0.03, 0.910 + 0.200*0.5)));
	m_ObjLink[2].AddCollision(&m_ObjCollision[60]);
	// button
	m_ObjLink[3].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	m_ObjLink[3].GetGeomInfo().SetLocalFrame(TsrLib2cad);
	m_ObjLink[3].GetGeomInfo().SetFileName("../../../workspace/robot/workcell_3ds/button.3ds");
	m_ObjLink[3].GetGeomInfo().SetColor(0.35f, 0.35f, 0.35f, 1.0f);
	m_ObjWeldJoint[2].SetParentLink(&m_ObjLink[0]);
	m_ObjWeldJoint[2].SetChildLink(&m_ObjLink[3]);

	m_ObjCollision[2].GetGeomInfo().SetDimension(Vec3(0.15, 0.25, 0.205));
	m_ObjCollision[2].SetLocalFrame(SE3(Vec3(-0.3485, 0.615, 1.0125)));
	m_ObjCollision[7].GetGeomInfo().SetDimension(Vec3(0.03, 0.03, 0.02));
	m_ObjCollision[7].SetLocalFrame(SE3(Vec3(-0.3485, 0.54, 1.21)));
	m_ObjCollision[8].GetGeomInfo().SetDimension(Vec3(0.03, 0.03, 0.02));
	m_ObjCollision[8].SetLocalFrame(SE3(Vec3(-0.3485, 0.615, 1.21)));
	m_ObjCollision[9].GetGeomInfo().SetDimension(Vec3(0.03, 0.03, 0.02));
	m_ObjCollision[9].SetLocalFrame(SE3(Vec3(-0.3485, 0.69, 1.21)));
	m_ObjCollision[10].GetGeomInfo().SetDimension(Vec3(0.086, 0.25, 0.085));
	m_ObjCollision[10].SetLocalFrame(SE3(Vec3(-0.3485, 0.615, 1.1575)));

	m_ObjLink[3].AddCollision(&m_ObjCollision[2]);
	m_ObjLink[3].AddCollision(&m_ObjCollision[7]);
	m_ObjLink[3].AddCollision(&m_ObjCollision[8]);
	m_ObjLink[3].AddCollision(&m_ObjCollision[9]);
	m_ObjLink[3].AddCollision(&m_ObjCollision[10]);

	// XYZ stage
	pJoint.resize(0);
	rJoint.resize(0);

	if (m_mode == 1)
	{
		srPrismaticJoint* temp = new srPrismaticJoint;
		pJoint.push_back(temp);
		temp = new srPrismaticJoint;
		pJoint.push_back(temp);

		srRevoluteJoint* rtemp = new srRevoluteJoint;
		rJoint.push_back(rtemp);

		m_ObjLink[4].GetGeomInfo().SetShape(srGeometryInfo::TDS);
		m_ObjLink[4].GetGeomInfo().SetLocalFrame(TsrLib2cad);
		m_ObjLink[4].GetGeomInfo().SetFileName("../../../workspace/robot/workcell_3ds/XYZ_stage_1.3ds");
		m_ObjLink[4].GetGeomInfo().SetColor(0.35f, 0.35f, 0.35f, 1.0f);
		m_ObjWeldJoint[3].SetParentLink(&m_ObjLink[0]);
		m_ObjWeldJoint[3].SetChildLink(&m_ObjLink[4]);

		m_ObjCollision[3].GetGeomInfo().SetDimension(Vec3(0.4286, 0.116, 0.066));
		m_ObjCollision[3].SetLocalFrame(SE3(Vec3(0.0643, 1.112, 0.943)));
		m_ObjLink[4].AddCollision(&m_ObjCollision[3]);


		m_ObjLink[5].GetGeomInfo().SetShape(srGeometryInfo::TDS);
		m_ObjLink[5].GetGeomInfo().SetLocalFrame(TsrLib2cad);
		m_ObjLink[5].GetGeomInfo().SetFileName("../../../workspace/robot/workcell_3ds/XYZ_stage_2.3ds");
		m_ObjLink[5].GetGeomInfo().SetColor(0.25f, 0.25f, 0.25f, 1.0f);

		m_ObjCollision[4].GetGeomInfo().SetDimension(Vec3(0.116, 0.4286, 0.076));
		m_ObjCollision[4].SetLocalFrame(SE3(Vec3(0.017, 1.0557, 1.014)));
		m_ObjLink[5].AddCollision(&m_ObjCollision[4]);

		pJoint[0]->SetParentLink(&m_ObjLink[4]);
		pJoint[0]->SetParentLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF, 0.0), Vec3(0.0, 0.0, 0.0)));
		pJoint[0]->SetChildLink(&m_ObjLink[5]);
		pJoint[0]->SetChildLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF, 0.0), Vec3(0.0, 0.0, 0.0)));
		pJoint[0]->SetActType(srJoint::ACTTYPE::HYBRID);

		m_ObjLink[6].GetGeomInfo().SetShape(srGeometryInfo::TDS);
		m_ObjLink[6].GetGeomInfo().SetLocalFrame(TsrLib2cad);
		m_ObjLink[6].GetGeomInfo().SetFileName("../../../workspace/robot/workcell_3ds/XYZ_stage_3.3ds");
		m_ObjLink[6].GetGeomInfo().SetColor(0.25f, 0.25f, 0.25f, 1.0f);

		m_ObjCollision[6].GetGeomInfo().SetDimension(Vec3(0.135, 0.166, 0.116));
		m_ObjCollision[6].SetLocalFrame(SE3(Vec3(0.025, 1.095, 1.109)));
		m_ObjLink[6].AddCollision(&m_ObjCollision[6]);

		pJoint[1]->SetParentLink(&m_ObjLink[5]);
		pJoint[1]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0, 0.0)));
		pJoint[1]->SetChildLink(&m_ObjLink[6]);
		pJoint[1]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0, 0.0)));
		pJoint[1]->SetActType(srJoint::ACTTYPE::HYBRID);

		m_ObjLink[7].GetGeomInfo().SetShape(srGeometryInfo::TDS);
		m_ObjLink[7].GetGeomInfo().SetLocalFrame(TsrLib2cad);
		m_ObjLink[7].GetGeomInfo().SetFileName("../../../workspace/robot/workcell_3ds/XYZ_stage_4.3ds");
		m_ObjLink[7].GetGeomInfo().SetColor(0.25f, 0.25f, 0.25f, 1.0f);

		m_ObjCollision[5].GetGeomInfo().SetDimension(Vec3(0.5, 0.5, 0.008));
		m_ObjCollision[5].SetLocalFrame(SE3(Vec3(0.025, 1.095, 1.172)));
		m_ObjLink[7].AddCollision(&m_ObjCollision[5]);




		rJoint[0]->SetParentLink(&m_ObjLink[6]);
		rJoint[0]->SetParentLinkFrame(SE3(Vec3(0.025, 1.095, 0.0)));
		rJoint[0]->SetChildLink(&m_ObjLink[7]);
		rJoint[0]->SetChildLinkFrame(SE3(Vec3(0.025, 1.095, 0.0)));
		rJoint[0]->SetActType(srJoint::ACTTYPE::HYBRID);
	}
	else if (m_mode == 2)
	{
		m_ObjWeldJoint[3].SetParentLink(&m_ObjLink[0]);
		m_ObjWeldJoint[3].SetChildLink(&m_ObjLink[7]);


		m_ObjLink[7].GetGeomInfo().SetShape(srGeometryInfo::TDS);
		double mount_height = 0.105;
		m_ObjLink[7].GetGeomInfo().SetLocalFrame(SE3(Vec3(0.0, 0.0, 0.91 + mount_height + 0.008 - 1.176)) * TsrLib2cad);
		m_ObjLink[7].GetGeomInfo().SetFileName("../../../workspace/robot/workcell_3ds/XYZ_stage_4.3ds");
		m_ObjLink[7].GetGeomInfo().SetColor(0.25f, 0.25f, 0.25f, 1.0f);

		m_ObjCollision[5].GetGeomInfo().SetDimension(Vec3(0.5, 0.5, 0.008));
		m_ObjCollision[5].SetLocalFrame(SE3(Vec3(0.025, 1.095, 0.91 + mount_height + 0.5*0.008)));
		m_ObjLink[7].AddCollision(&m_ObjCollision[5]);

		m_ObjCollision[6].GetGeomInfo().SetDimension(Vec3(0.135, 0.166, mount_height));
		m_ObjCollision[6].SetLocalFrame(SE3(Vec3(0.025, 1.095, 0.91 + 0.5*mount_height)));
		m_ObjLink[7].AddCollision(&m_ObjCollision[6]);
	}

	this->SetBaseLink(&m_ObjLink[0]);
	this->SetBaseLinkType(srSystem::KINEMATIC);
	this->SetSelfCollision(true);
}

void WorkCell::setStageVal(const Eigen::VectorXd& stageVal)
{
	for (unsigned int i = 0; i < pJoint.size(); i++)
		((srStateJoint*)pJoint[i])->m_State.m_rValue[i] = stageVal[i];
	for (unsigned int i = 0; i < rJoint.size(); i++)
		((srStateJoint*)rJoint[i])->m_State.m_rValue[i] = stageVal[pJoint.size() + i];
}

srLink * WorkCell::getStagePlate() const
{
	return (srLink*)&(m_ObjLink[7]);
}
