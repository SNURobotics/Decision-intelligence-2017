#include "MH12Robot.h"
#include "common\utils.h"
#include "makeSpecialCol.h"

MH12Robot::MH12Robot(bool elbowUp, double gripperRot)
{
	for (int i = 0; i < NUM_OF_RJOINT_MH12; i++)
		gJoint[i] = new srRevoluteJoint;
	
	//for (int i = 0; i < NUM_OF_GRIPERJOINT_MH12; i++)
	//	gGripJoint[i] = new srPrismaticJoint;

	for (int i = 0; i < NUM_OF_WJOINT_UR3; i++)
		gWeldJoint[i] = new srWeldJoint;

	AssembleModel(gripperRot);
	AssembleCollision();
	SetJointLimit(elbowUp);
	SetInitialConfiguration();
	SetInertia();
	homePos = Eigen::VectorXd::Zero(6);
	homePos[1] = -SR_PI_HALF; homePos[3] = -SR_PI_HALF;
	qInvKinInit = Eigen::VectorXd::Zero(6);
	qInvKinInit[0] = DEG2RAD(-2.43); 	qInvKinInit[1] = DEG2RAD(-68.79);	qInvKinInit[2] = DEG2RAD(93.97);
	qInvKinInit[3] = DEG2RAD(-115.47);	qInvKinInit[4] = DEG2RAD(-94.96);	qInvKinInit[5] = DEG2RAD(0.0);
	if (elbowUp)
	{
		//qInvKinInit[0] = -0.224778; qInvKinInit[1] = -1.91949; qInvKinInit[2] = -0.384219; qInvKinInit[3] = 1.5708; qInvKinInit[4] = -0.73291; qInvKinInit[5] = 1.79557;
	}
	else
	{
		//qInvKinInit[0] = -0.074913; qInvKinInit[1] = -0.612778; qInvKinInit[2] = -2.488023; qInvKinInit[3] = 1.570796; qInvKinInit[4] = -1.530005; qInvKinInit[5] = 1.645710;
	}
	TsrLinkbase2robotbase = EulerZYX(Vec3(SR_PI, 0.0, 0.0), Vec3(0.0, 0.0, 0.69195 - 0.69511));
	//TsrLinkbase2robotbase = EulerZYX(Vec3(SR_PI, 0.0, 0.0), Vec3(0.0, 0.0, 0.69195 - 0.69511 + 0.00195));
	this->SetSelfCollision(true);
}

MH12Robot::~MH12Robot()
{
	for (int i = 0; i<NUM_OF_RJOINT_MH12; i++)
		SR_SAFE_DELETE(gJoint[i]);
	//for (int i = 0; i<NUM_OF_GRIPERJOINT_MH12; i++)
	//	SR_SAFE_DELETE(gGripJoint[i]);

}

void MH12Robot::SetActType(srJoint::ACTTYPE actType /*= srJoint::HYBRID*/)
{
	for (int i = 0; i < NUM_OF_RJOINT_MH12; i++)
		gJoint[i]->SetActType(actType);
}

void MH12Robot::SetGripperActType(srJoint::ACTTYPE actType /*= srJoint::HYBRID*/)
{
	//for (int i = 0; i < NUM_OF_GRIPERJOINT_MH12; i++)
	//	gGripJoint[i]->SetActType(actType);
}

void MH12Robot::SetGripperActType(srJoint::ACTTYPE actType, vector<int> gpJointIdx)
{
	//for (unsigned int i = 0; i < gpJointIdx.size(); i++)
	//	gGripJoint[gpJointIdx[i]]->SetActType(actType);
}

void MH12Robot::SetJointLimit(bool elbowUp)
{
	// unit: deg
	// from robot spec
	
	// R joint
	if (elbowUp)
	{
		UpperJointLimit[MH12_Index::JOINT_1] = 170;
		UpperJointLimit[MH12_Index::JOINT_2] = 155;
		UpperJointLimit[MH12_Index::JOINT_3] = 240;
		UpperJointLimit[MH12_Index::JOINT_4] = 180;
		UpperJointLimit[MH12_Index::JOINT_5] = 135;
		UpperJointLimit[MH12_Index::JOINT_6] = 360;

		LowerJointLimit[MH12_Index::JOINT_1] = -170;
		LowerJointLimit[MH12_Index::JOINT_2] = -90;
		LowerJointLimit[MH12_Index::JOINT_3] = -175;
		LowerJointLimit[MH12_Index::JOINT_4] = -180;
		LowerJointLimit[MH12_Index::JOINT_5] = -135;
		LowerJointLimit[MH12_Index::JOINT_6] = -360;
	}
	else
	{
		UpperJointLimit[MH12_Index::JOINT_1] = 170;
		UpperJointLimit[MH12_Index::JOINT_2] = 155;
		UpperJointLimit[MH12_Index::JOINT_3] = 240;
		UpperJointLimit[MH12_Index::JOINT_4] = 180;
		UpperJointLimit[MH12_Index::JOINT_5] = 135;
		UpperJointLimit[MH12_Index::JOINT_6] = 360;

		LowerJointLimit[MH12_Index::JOINT_1] = -170;
		LowerJointLimit[MH12_Index::JOINT_2] = -90;
		LowerJointLimit[MH12_Index::JOINT_3] = -175;
		LowerJointLimit[MH12_Index::JOINT_4] = -180;
		LowerJointLimit[MH12_Index::JOINT_5] = -135;
		LowerJointLimit[MH12_Index::JOINT_6] = -360;
	}


	///////////////////////// add gripper joint limit??



	for (int i = 0; i < DEGREE_OF_FREEDOM_MH12_JOINT; i++)
		gJoint[i]->SetPositionLimit(LowerJointLimit[i], UpperJointLimit[i]);

}

void MH12Robot::SetVelocityLimit()
{
	// unit: deg/s
	VelocityLimit[MH12_Index::JOINT_1] = 220;
	VelocityLimit[MH12_Index::JOINT_2] = 200;
	VelocityLimit[MH12_Index::JOINT_3] = 220;
	VelocityLimit[MH12_Index::JOINT_4] = 410;
	VelocityLimit[MH12_Index::JOINT_5] = 410;
	VelocityLimit[MH12_Index::JOINT_6] = 610;

}

void MH12Robot::SetInitialConfiguration()
{
	for (int i = 0; i < DEGREE_OF_FREEDOM_MH12_JOINT; i++)
		gJoint[i]->m_State.m_rValue[0] = DEG2RAD(0);

	KIN_UpdateFrame_All_The_Entity();
}

void MH12Robot::AssembleModel(double gripperRot)
{
	// default color
	for (int i = 0; i < NUM_OF_LINK_MH12; i++)
		gLink[i].GetGeomInfo().SetColor(0.3f, 0.3f, 0.3f, 1.0f);

	SE3 Tcad2srlib = EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0, 0.0));

	gLink[MH12_Index::LINK_1].SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	gLink[MH12_Index::LINK_1].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[MH12_Index::LINK_1].GetGeomInfo().SetLocalFrame(Tcad2srlib);
	gLink[MH12_Index::LINK_1].GetGeomInfo().SetFileName("../../../workspace/robot/mh12_3ds/link1.3ds");
	


	gJoint[MH12_Index::JOINT_1]->SetActType(srJoint::HYBRID);
	gJoint[MH12_Index::JOINT_1]->SetParentLink(&gLink[MH12_Index::LINK_1]);
	gJoint[MH12_Index::JOINT_1]->SetChildLink(&gLink[MH12_Index::LINK_2]);
	gJoint[MH12_Index::JOINT_1]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0,0.0), Vec3(0.0, 0.0, 0.0)));
	gJoint[MH12_Index::JOINT_1]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0,0.0), Vec3(0.0, 0.0, 0.0)));
	gJoint[MH12_Index::JOINT_1]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gJoint[MH12_Index::JOINT_1]->MakePositionLimit(false);

	gLink[MH12_Index::LINK_2].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[MH12_Index::LINK_2].GetGeomInfo().SetLocalFrame(Tcad2srlib);
	gLink[MH12_Index::LINK_2].GetGeomInfo().SetFileName("../../../workspace/robot/mh12_3ds/link2.3ds");
	//gLink[MH12_Index::LINK_2].GetGeomInfo().SetColor(0.15f, 0.15f, 0.15f, 1.0f);

	gJoint[MH12_Index::JOINT_2]->SetActType(srJoint::HYBRID);
	gJoint[MH12_Index::JOINT_2]->SetParentLink(&gLink[MH12_Index::LINK_2]);
	gJoint[MH12_Index::JOINT_2]->SetChildLink(&gLink[MH12_Index::LINK_3]);
	gJoint[MH12_Index::JOINT_2]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(-0.155, 0.0, 0.450)));
	//gJoint[MH12_Index::JOINT_2]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.0, 0.0, 0.1519)));
	gJoint[MH12_Index::JOINT_2]->SetChildLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF, SR_PI_HALF), Vec3(-0.155, 0.0, 0.450)));
	gJoint[MH12_Index::JOINT_2]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gJoint[MH12_Index::JOINT_2]->MakePositionLimit(false);

	gLink[MH12_Index::LINK_3].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[MH12_Index::LINK_3].GetGeomInfo().SetLocalFrame(Tcad2srlib);
	gLink[MH12_Index::LINK_3].GetGeomInfo().SetFileName("../../../workspace/robot/mh12_3ds/link3.3ds");

	gJoint[MH12_Index::JOINT_3]->SetActType(srJoint::HYBRID);
	gJoint[MH12_Index::JOINT_3]->SetParentLink(&gLink[MH12_Index::LINK_3]);
	gJoint[MH12_Index::JOINT_3]->SetChildLink(&gLink[MH12_Index::LINK_4]);
	gJoint[MH12_Index::JOINT_3]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(-0.155, 0.0, 1.064)));
	gJoint[MH12_Index::JOINT_3]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(-0.155, 0.0, 1.064)));
	gJoint[MH12_Index::JOINT_3]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gJoint[MH12_Index::JOINT_3]->MakePositionLimit(false);


	gLink[MH12_Index::LINK_4].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[MH12_Index::LINK_4].GetGeomInfo().SetLocalFrame(Tcad2srlib);
	gLink[MH12_Index::LINK_4].GetGeomInfo().SetFileName("../../../workspace/robot/mh12_3ds/link4.3ds");
	//gLink[MH12_Index::LINK_4].GetGeomInfo().SetColor(0.15f, 0.15f, 0.15f, 1.0f);

	gJoint[MH12_Index::JOINT_4]->SetActType(srJoint::HYBRID);
	gJoint[MH12_Index::JOINT_4]->SetParentLink(&gLink[MH12_Index::LINK_4]);
	gJoint[MH12_Index::JOINT_4]->SetChildLink(&gLink[MH12_Index::LINK_5]);
	gJoint[MH12_Index::JOINT_4]->SetParentLinkFrame(EulerZYX(Vec3(0.0, -SR_PI_HALF, 0.0), Vec3(-0.255, 0.0, 1.264)));
	//gJoint[MH12_Index::JOINT_4]->SetChildLinkFrame(EulerZYX(Vec3(0.0,0.0, -SR_PI_HALF), Vec3(0.0, 0.0, 0.60855)));
	gJoint[MH12_Index::JOINT_4]->SetChildLinkFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, SR_PI_HALF), Vec3(-0.255, 0.0, 1.264)));
	gJoint[MH12_Index::JOINT_4]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gJoint[MH12_Index::JOINT_4]->MakePositionLimit(false);

	gLink[MH12_Index::LINK_5].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[MH12_Index::LINK_5].GetGeomInfo().SetLocalFrame(Tcad2srlib);
	gLink[MH12_Index::LINK_5].GetGeomInfo().SetFileName("../../../workspace/robot/mh12_3ds/link5.3ds");


	gJoint[MH12_Index::JOINT_5]->SetActType(srJoint::HYBRID);
	gJoint[MH12_Index::JOINT_5]->SetParentLink(&gLink[MH12_Index::LINK_5]);
	gJoint[MH12_Index::JOINT_5]->SetChildLink(&gLink[MH12_Index::LINK_6]);
	gJoint[MH12_Index::JOINT_5]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(-0.795, 0.0, 1.264)));
	gJoint[MH12_Index::JOINT_5]->SetChildLinkFrame(EulerZYX(Vec3(0.0, -SR_PI_HALF, SR_PI_HALF), Vec3(-0.795, 0.0, 1.264)));
	gJoint[MH12_Index::JOINT_5]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gJoint[MH12_Index::JOINT_5]->MakePositionLimit(false);

	gLink[MH12_Index::LINK_6].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[MH12_Index::LINK_6].GetGeomInfo().SetLocalFrame(Tcad2srlib);
	gLink[MH12_Index::LINK_6].GetGeomInfo().SetFileName("../../../workspace/robot/mh12_3ds/link6.3ds");
	//gLink[MH12_Index::LINK_6].GetGeomInfo().SetColor(0.15f, 0.15f, 0.15f, 1.0f);

	gJoint[MH12_Index::JOINT_6]->SetActType(srJoint::HYBRID);
	gJoint[MH12_Index::JOINT_6]->SetParentLink(&gLink[MH12_Index::LINK_6]);
	gJoint[MH12_Index::JOINT_6]->SetChildLink(&gLink[MH12_Index::ENDEFFECTOR]);
	gJoint[MH12_Index::JOINT_6]->SetParentLinkFrame(EulerZYX(Vec3(0.0, -SR_PI_HALF, 0.0), Vec3(-0.895, 0.0, 1.264)));
	gJoint[MH12_Index::JOINT_6]->SetChildLinkFrame(EulerZYX(Vec3(0.0, -SR_PI_HALF, 0.0), Vec3(-0.895, 0.0, 1.264)));
	gJoint[MH12_Index::JOINT_6]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gJoint[MH12_Index::JOINT_6]->MakePositionLimit(false);

	gLink[MH12_Index::ENDEFFECTOR].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[MH12_Index::ENDEFFECTOR].GetGeomInfo().SetLocalFrame(Tcad2srlib);
	gLink[MH12_Index::ENDEFFECTOR].GetGeomInfo().SetFileName("../../../workspace/robot/mh12_3ds/link7.3ds");

	
	//gripper
	gWeldJoint[MH12_Index::WELDJOINT_COUPLING]->SetParentLink(&gLink[MH12_Index::ENDEFFECTOR]);
	gWeldJoint[MH12_Index::WELDJOINT_COUPLING]->SetChildLink(&gLink[MH12_Index::COUPLING]);
	gWeldJoint[MH12_Index::WELDJOINT_COUPLING]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0, 0.0)));
	gWeldJoint[MH12_Index::WELDJOINT_COUPLING]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	gWeldJoint[MH12_Index::WELDJOINT_COUPLING]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);

	gLink[MH12_Index::COUPLING].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[MH12_Index::COUPLING].GetGeomInfo().SetLocalFrame(SE3());
	gLink[MH12_Index::COUPLING].GetGeomInfo().SetFileName("../../../workspace/robot/mh12_3ds/coupling.3ds");

	gWeldJoint[MH12_Index::WELDJOINT_GRIPJOINT_1]->SetParentLink(&gLink[MH12_Index::COUPLING]);
	gWeldJoint[MH12_Index::WELDJOINT_GRIPJOINT_1]->SetChildLink(&gLink[MH12_Index::GRIPPER_1]);
	gWeldJoint[MH12_Index::WELDJOINT_GRIPJOINT_1]->SetParentLinkFrame(SE3());
	gWeldJoint[MH12_Index::WELDJOINT_GRIPJOINT_1]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	gWeldJoint[MH12_Index::WELDJOINT_GRIPJOINT_1]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);

	gLink[MH12_Index::GRIPPER_1].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[MH12_Index::GRIPPER_1].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	gLink[MH12_Index::GRIPPER_1].GetGeomInfo().SetColor(0.2, 0.2, 0.2);
	gLink[MH12_Index::GRIPPER_1].GetGeomInfo().SetFileName("../../../workspace/robot/mh12_3ds/gripper1.3ds");


	gWeldJoint[MH12_Index::WELDJOINT_GRIPJOINT_2]->SetParentLink(&gLink[MH12_Index::COUPLING]);
	gWeldJoint[MH12_Index::WELDJOINT_GRIPJOINT_2]->SetChildLink(&gLink[MH12_Index::GRIPPER_2]);
	gWeldJoint[MH12_Index::WELDJOINT_GRIPJOINT_2]->SetParentLinkFrame(SE3());
	gWeldJoint[MH12_Index::WELDJOINT_GRIPJOINT_2]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	gWeldJoint[MH12_Index::WELDJOINT_GRIPJOINT_2]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);

	gLink[MH12_Index::GRIPPER_2].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[MH12_Index::GRIPPER_2].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	gLink[MH12_Index::GRIPPER_2].GetGeomInfo().SetColor(0.2, 0.2, 0.2);
	gLink[MH12_Index::GRIPPER_2].GetGeomInfo().SetFileName("../../../workspace/robot/mh12_3ds/gripper2.3ds");

	gWeldJoint[MH12_Index::WELDJOINT_GRIPJOINT_3]->SetParentLink(&gLink[MH12_Index::COUPLING]);
	gWeldJoint[MH12_Index::WELDJOINT_GRIPJOINT_3]->SetChildLink(&gLink[MH12_Index::GRIPPER_3]);
	gWeldJoint[MH12_Index::WELDJOINT_GRIPJOINT_3]->SetParentLinkFrame(SE3());
	gWeldJoint[MH12_Index::WELDJOINT_GRIPJOINT_3]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	gWeldJoint[MH12_Index::WELDJOINT_GRIPJOINT_3]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);

	gLink[MH12_Index::GRIPPER_3].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[MH12_Index::GRIPPER_3].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	gLink[MH12_Index::GRIPPER_3].GetGeomInfo().SetColor(0.2, 0.2, 0.2);
	gLink[MH12_Index::GRIPPER_3].GetGeomInfo().SetFileName("../../../workspace/robot/mh12_3ds/gripper3.3ds");


	gWeldJoint[MH12_Index::WELDJOINT_GRIP_MARKER]->SetActType(srJoint::PASSIVE);
	gWeldJoint[MH12_Index::WELDJOINT_GRIP_MARKER]->SetParentLink(&gLink[MH12_Index::ENDEFFECTOR]);
	gWeldJoint[MH12_Index::WELDJOINT_GRIP_MARKER]->SetChildLink(&gMarkerLink[MH12_Index::MLINK_GRIP]);
	gWeldJoint[MH12_Index::WELDJOINT_GRIP_MARKER]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI), Vec3(0.0, 0.0, 0.0)));
	gWeldJoint[MH12_Index::WELDJOINT_GRIP_MARKER]->SetChildLinkFrame(SE3(Vec3(1.036, 0.0, 1.264)));		// consider offset for gripper assembly
																															//gWeldJoint[UR3_Index::WELDJOINT_GRIP_MARKER]->SetChildLinkFrame(SE3(Vec3(0.0, 0.00195, -(0.1928 + 0.2003 + 0.001))));		// consider offset for gripper assembly
	
	// marker links
	gMarkerLink[MH12_Index::MLINK_GRIP].GetGeomInfo().SetDimension(Vec3(0.00, 0.00, 0.00));
	gMarkerLink[MH12_Index::MLINK_GRIP].GetGeomInfo().SetColor(0.1f, 0.1f, 0.1f);
	gMarkerLink[MH12_Index::MLINK_GRIP].SetInertia(Inertia(0.001));
	

	this->SetBaseLink(&gLink[MH12_Index::LINK_1]);
	this->SetBaseLinkType(srSystem::FIXED);
}

void MH12Robot::AssembleCollision()
{
	m_numCollision = 0;

	int numBox = 5;
	double thickness = 0.03;
	double space = 0.005;
	vector<pair<Vec3, SE3>> boxSet;	

	// Link1: 
	// Cylinder size: H = 86.05, D = 90
	// Center position = (0,0,86.05*0.5)
	
	//boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.08605*0.5)), 0.09*0.5, 0.08605, thickness, space, numBox);
	//for (int i = 0; i < numBox; i++)
	//{
	//	gLink[MH12_Index::LINK_1].AddCollision(&gCollision[m_numCollision]);
	//	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	//	gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
	//	gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	//}

	gLink[MH12_Index::LINK_1].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.54, 0.33, 0.17));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.11, 0.0, 0.085)));


	gLink[MH12_Index::LINK_2].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.46, 0.4, 0.11));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.005, 0.0, 0.23)));

	gLink[MH12_Index::LINK_2].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.2, 0.4, 0.07));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.05, 0.0, 0.325)));

	gLink[MH12_Index::LINK_2].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.18, 0.25, 0.28));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-0.16, -0.08, 0.43)));

	
	gLink[MH12_Index::LINK_3].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.175, 0.1, 0.76));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-0.155, 0.1, 0.76)));

	gLink[MH12_Index::LINK_3].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.024, 0.09, 0.55));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-0.053, 0.105, 0.71)));

	gLink[MH12_Index::LINK_3].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.155, 0.05, 0.5));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-0.12, 0.18, 0.77)));


	gLink[MH12_Index::LINK_4].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.33, 0.20, 0.36));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-0.25, -0.055, 1.17)));

	gLink[MH12_Index::LINK_4].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.175, 0.2, 0.17));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-0.315, 0.15, 1.255)));

	gLink[MH12_Index::LINK_4].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.05, 0.15, 0.085));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-0.2, 0.125, 1.265)));
	
	gLink[MH12_Index::LINK_4].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.05, 0.07, 0.03));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-0.26, 0.2, 1.152)));


	gLink[MH12_Index::LINK_5].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.18, 0.2, 0.18));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-0.51, 0.0, 1.276)));

	gLink[MH12_Index::LINK_5].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.255, 0.04, 0.155));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-0.73, 0.08, 1.28)));

	gLink[MH12_Index::LINK_5].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.255, 0.04, 0.155));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-0.73, -0.08, 1.28)));


	gLink[MH12_Index::LINK_6].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.154, 0.115, 0.14));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-0.814, 0.0, 1.267)));

	gLink[MH12_Index::ENDEFFECTOR].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.02, 0.08, 0.08));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-0.905, 0.0, 1.264)));

	gLink[MH12_Index::COUPLING].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.09, 0.08, 0.08));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-0.965, 1.264, 0.0)));



	gLink[MH12_Index::GRIPPER_1].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.02, 0.014, 0.014));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-1.026, 1.269, 0.0087)));

	gLink[MH12_Index::GRIPPER_2].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.02, 0.014, 0.014));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-1.026, 1.254, 0.0)));

	gLink[MH12_Index::GRIPPER_3].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.02, 0.014, 0.014));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-1.026, 1.269, -0.0087)));
	
}

void MH12Robot::SetInertia()
{
//	double m1 = 5.598009;
//	double m2 = 4.722018;
//	double m3 = 6.17608;
//	double m4 = 6.549344;
//	double m5 = 1.923968;
//	double m6 = 3.3099;
//	double m7 = 0.0913;
//	Vec3 r1 = Vec3(0.001027, -0.000056, 0.087474);
//	Vec3 r2 = Vec3(-0.000007, 0.0256, 0.255037);
//	Vec3 r3 = Vec3(-0.002282, -0.006782, 0.469538);
//	Vec3 r4 = Vec3(0.010135, 0.016094, 0.735077);
//	Vec3 r5 = Vec3(0.316302, -0.000768, 0.785255);
//	Vec3 r6 = Vec3(0.423187, 0.000899, 0.873598);
//	Vec3 r7 = Vec3(0.552202, 0.0, 0.995299);
//	
//	Inertia G1(0.031682, 0.031384, 0.017293, -0.000023, -0.000017, -0.000468, 0.0, 0.0, 0.0, m1);
//	Inertia G2(0.02108, 0.010309, 0.021876, 0, 0.000901, 0.000001, 0.0, 0.0, 0.0, m2);
//	Inertia G3(0.216937, 0.167656, 0.067914, -0.000322, -0.001387, -0.001728, 0.0, 0.0, 0.0, m3);
//	Inertia G4(0.033518, 0.046678, 0.042127, -0.001069, -0.004744, 0.003172, 0.0, 0.0, 0.0, m4);
//	Inertia G5(0.004299, 0.024684, 0.023256, -0.000053, -0.000005, -0.000499, 0.0, 0.0, 0.0, m5);
//	Inertia G6(0.022679, 0.028158, 0.00752, -0.000064, 0.000104, 0.001751, 0.0, 0.0, 0.0, m6);
//	Inertia G7(0.000101, 0.000101, 0.0001, 0, 0, 0, 0.0, 0.0, 0.0, m7);
//
//	gLink[Indy_Index::LINK_1].SetInertia(G1.Transform(SE3(-r1)));
//	gLink[Indy_Index::LINK_2].SetInertia(G2.Transform(SE3(-r2)));
//	gLink[Indy_Index::LINK_3].SetInertia(G3.Transform(SE3(-r3)));
//	gLink[Indy_Index::LINK_4].SetInertia(G4.Transform(SE3(-r4)));
//	gLink[Indy_Index::LINK_5].SetInertia(G5.Transform(SE3(-r5)));
//	gLink[Indy_Index::LINK_6].SetInertia(G6.Transform(SE3(-r6)));
//	gLink[Indy_Index::ENDEFFECTOR].SetInertia(G7.Transform(SE3(-r7)));
//		
//
//	// GIMATIC MPLM1630N GRIPPER MODEL
//	double m_g1 = 0.10629;
//	double m_g2 = 0.16205;
//	double m_g3 = 0.00515;
//	double m_g4 = 0.00515;
//	Vec3 r_g1 = Vec3(0.01e-3, -17.35e-3, 0.04e-3);
//	Vec3 r_g2 = Vec3(-0.01e-3, -61.66e-3, -0.06e-3);
//	Vec3 r_g3 = Vec3(-0.61e-3, -106.69e-3, 22.76e-3);
//	Vec3 r_g4 = Vec3(0.61e-3, -106.69e-3, -22.76e-3);
//	Inertia G_g1(42161.46e-9, 63838.67e-9, 42131.49e-9, 9.63e-9, 8.46e-9, -37.38e-9, 0.0, 0.0, 0.0, m_g1);
//	Inertia G_g2(94794.64e-9, 82474.62e-9, 77696.95e-9, -22.07e-9, -57.59e-9, 408.09e-9, 0.0, 0.0, 0.0, m_g2);
//	double ratio = 1.0;
//	Inertia G_g3(ratio * 530.34e-9, ratio * 177.96e-9, ratio * 551.94e-9, -ratio * 33.21e-9, ratio * 56.44e-9, -ratio * 3.22e-9, 0.0, 0.0, 0.0, ratio * m_g3);
//	Inertia G_g4(ratio * 530.34e-9, ratio * 177.96e-9, ratio * 551.94e-9, ratio * 33.21e-9, -ratio * 56.44e-9, -ratio * 3.22e-9, 0.0, 0.0, 0.0, ratio * m_g4);
//	gLink[Indy_Index::SENSOR].SetInertia(G_g1.Transform(SE3(-r_g1)));
//	gLink[Indy_Index::GRIPPER].SetInertia(G_g2.Transform(SE3(-r_g2)));
//	gLink[Indy_Index::GRIPPER_FINGER_L].SetInertia(G_g3.Transform(SE3(-r_g3)));
//	gLink[Indy_Index::GRIPPER_FINGER_U].SetInertia(G_g4.Transform(SE3(-r_g4)));
}
//
void MH12Robot::SetTorqueLimit()
{
	UpperTorqueLimit[MH12_Index::JOINT_1] = DEG2RAD(300);
	UpperTorqueLimit[MH12_Index::JOINT_2] = DEG2RAD(300);
	UpperTorqueLimit[MH12_Index::JOINT_3] = DEG2RAD(300);
	UpperTorqueLimit[MH12_Index::JOINT_4] = DEG2RAD(300);
	UpperTorqueLimit[MH12_Index::JOINT_5] = DEG2RAD(300);
	UpperTorqueLimit[MH12_Index::JOINT_6] = DEG2RAD(300);


	LowerTorqueLimit[MH12_Index::JOINT_1] = DEG2RAD(-300);
	LowerTorqueLimit[MH12_Index::JOINT_2] = DEG2RAD(-300);
	LowerTorqueLimit[MH12_Index::JOINT_3] = DEG2RAD(-300);
	LowerTorqueLimit[MH12_Index::JOINT_4] = DEG2RAD(-300);
	LowerTorqueLimit[MH12_Index::JOINT_5] = DEG2RAD(-300);
	LowerTorqueLimit[MH12_Index::JOINT_6] = DEG2RAD(-300);

	for (int i = 0; i < DEGREE_OF_FREEDOM_MH12_JOINT; i++)
		gJoint[i]->SetTorqueLimit(LowerTorqueLimit[i], UpperTorqueLimit[i]);
}

Eigen::VectorXd MH12Robot::getLowerJointLimit() const
{
	Eigen::VectorXd llim(DEGREE_OF_FREEDOM_MH12_JOINT);
	for (int i = 0; i < DEGREE_OF_FREEDOM_MH12_JOINT; i++)
		llim[i] =  DEG2RAD(LowerJointLimit[i]);
	return llim;
}

Eigen::VectorXd MH12Robot::getUpperJointLimit() const
{
	Eigen::VectorXd ulim(DEGREE_OF_FREEDOM_MH12_JOINT);
	for (int i = 0; i < DEGREE_OF_FREEDOM_MH12_JOINT; i++)
		ulim[i] = DEG2RAD(UpperJointLimit[i]);
	return ulim;
}
