#include "Franka.h"
#include "common\utils.h"
#include "makeSpecialCol.h"

Franka::Franka(bool elbowUp, double gripperRot)
{
	for (int i = 0; i < NUM_OF_RJOINT_Franka; i++)
		gJoint[i] = new srRevoluteJoint;
	
	for (int i = 0; i < NUM_OF_GRIPERJOINT_Franka; i++)
		gGripJoint[i] = new srRevoluteJoint;

	for (int i = 0; i < NUM_OF_WJOINT_Franka; i++)
		gWeldJoint[i] = new srWeldJoint;

	AssembleModel(gripperRot);
	AssembleCollision();
	SetJointLimit(elbowUp);
	SetInitialConfiguration();
	SetInertia();
	homePos = Eigen::VectorXd::Zero(NUM_OF_RJOINT_Franka);
	qInvKinInit = Eigen::VectorXd::Zero(NUM_OF_RJOINT_Franka);
	//qInvKinInit[0] = DEG2RAD(-2.43); 	qInvKinInit[1] = DEG2RAD(-68.79);	qInvKinInit[2] = DEG2RAD(93.97);
	//qInvKinInit[3] = DEG2RAD(-115.47);	qInvKinInit[4] = DEG2RAD(-94.96);	qInvKinInit[5] = DEG2RAD(0.0);
	if (elbowUp)
	{
		//qInvKinInit[0] = -0.224778; qInvKinInit[1] = -1.91949; qInvKinInit[2] = -0.384219; qInvKinInit[3] = 1.5708; qInvKinInit[4] = -0.73291; qInvKinInit[5] = 1.79557;
	}
	else
	{
		//qInvKinInit[0] = -0.074913; qInvKinInit[1] = -0.612778; qInvKinInit[2] = -2.488023; qInvKinInit[3] = 1.570796; qInvKinInit[4] = -1.530005; qInvKinInit[5] = 1.645710;
	}
	TsrLinkbase2robotbase = SE3();
	this->SetSelfCollision(true);
}

Franka::~Franka()
{
	for (int i = 0; i<NUM_OF_RJOINT_Franka; i++)
		SR_SAFE_DELETE(gJoint[i]);
	for (int i = 0; i<NUM_OF_GRIPERJOINT_Franka; i++)
		SR_SAFE_DELETE(gGripJoint[i]);
	for (int i = 0; i<NUM_OF_WJOINT_Franka; i++)
		SR_SAFE_DELETE(gWeldJoint[i]);

}

void Franka::SetActType(srJoint::ACTTYPE actType /*= srJoint::HYBRID*/)
{
	for (int i = 0; i < NUM_OF_RJOINT_Franka; i++)
		gJoint[i]->SetActType(actType);
}

void Franka::SetGripperActType(srJoint::ACTTYPE actType /*= srJoint::HYBRID*/)
{
	for (int i = 0; i < NUM_OF_GRIPERJOINT_Franka; i++)
		gGripJoint[i]->SetActType(actType);
}

void Franka::SetGripperActType(srJoint::ACTTYPE actType, vector<int> gpJointIdx)
{
	for (unsigned int i = 0; i < gpJointIdx.size(); i++)
		gGripJoint[gpJointIdx[i]]->SetActType(actType);
}

void Franka::SetJointLimit(bool elbowUp)
{
	// unit: deg
	// from robot spec
	
	// R joint
	if (elbowUp)
	{
		UpperJointLimit[Franka_Index::JOINT_1] = 166.003;
		UpperJointLimit[Franka_Index::JOINT_2] = 101.001;
		UpperJointLimit[Franka_Index::JOINT_3] = 166.003;
		UpperJointLimit[Franka_Index::JOINT_4] = -3.99925;
		UpperJointLimit[Franka_Index::JOINT_5] = 166.003;
		UpperJointLimit[Franka_Index::JOINT_6] = 215.0024;
		UpperJointLimit[Franka_Index::JOINT_7] = 166.003;

		LowerJointLimit[Franka_Index::JOINT_1] = -166.003;
		LowerJointLimit[Franka_Index::JOINT_2] = -101.001;
		LowerJointLimit[Franka_Index::JOINT_3] = -166.003;
		LowerJointLimit[Franka_Index::JOINT_4] = -176.0012;
		LowerJointLimit[Franka_Index::JOINT_5] = -166.003;
		LowerJointLimit[Franka_Index::JOINT_6] = -1.00268;
		LowerJointLimit[Franka_Index::JOINT_7] = -166.003;
	}
	else
	{
		UpperJointLimit[Franka_Index::JOINT_1] = 166.003;
		UpperJointLimit[Franka_Index::JOINT_2] = 101.001;
		UpperJointLimit[Franka_Index::JOINT_3] = 166.003;
		UpperJointLimit[Franka_Index::JOINT_4] = -3.99925;
		UpperJointLimit[Franka_Index::JOINT_5] = 166.003;
		UpperJointLimit[Franka_Index::JOINT_6] = 215.0024;
		UpperJointLimit[Franka_Index::JOINT_7] = 166.003;

		LowerJointLimit[Franka_Index::JOINT_1] = -166.003;
		LowerJointLimit[Franka_Index::JOINT_2] = -101.001;
		LowerJointLimit[Franka_Index::JOINT_3] = -166.003;
		LowerJointLimit[Franka_Index::JOINT_4] = -176.0012;
		LowerJointLimit[Franka_Index::JOINT_5] = -166.003;
		LowerJointLimit[Franka_Index::JOINT_6] = -1.00268;
		LowerJointLimit[Franka_Index::JOINT_7] = -166.003;
	}


	///////////////////////// add gripper joint limit??



	for (int i = 0; i < DEGREE_OF_FREEDOM_Franka_JOINT; i++)
		gJoint[i]->SetPositionLimit(LowerJointLimit[i], UpperJointLimit[i]);

}

void Franka::SetVelocityLimit()
{
	// unit: deg/s
	VelocityLimit[Franka_Index::JOINT_1] = 124.61832;
	VelocityLimit[Franka_Index::JOINT_2] = 124.61832;
	VelocityLimit[Franka_Index::JOINT_3] = 124.61832;
	VelocityLimit[Franka_Index::JOINT_4] = 124.61832;
	VelocityLimit[Franka_Index::JOINT_5] = 149.542;
	VelocityLimit[Franka_Index::JOINT_6] = 149.542;
	VelocityLimit[Franka_Index::JOINT_7] = 149.542;

}

void Franka::SetInitialConfiguration()
{
	for (int i = 0; i < DEGREE_OF_FREEDOM_Franka_JOINT; i++)
		gJoint[i]->m_State.m_rValue[0] = DEG2RAD(0);

	KIN_UpdateFrame_All_The_Entity();
}

void Franka::AssembleModel(double gripperRot)
{
	// default color
	for (int i = 0; i < NUM_OF_LINK_Franka; i++)
		gLink[i].GetGeomInfo().SetColor(0.3f, 0.3f, 0.3f, 1.0f);

	SE3 Tcad2srlib = EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0, 0.0));

	// WHEN CAD MODELS ARE AVAILABLE...
	//gLink[Franka_Index::LINK_1].SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	//gLink[Franka_Index::LINK_1].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	//gLink[Franka_Index::LINK_1].GetGeomInfo().SetLocalFrame(Tcad2srlib);
	//gLink[Franka_Index::LINK_1].GetGeomInfo().SetFileName("../../../workspace/robot/Franka_3ds/link1.3ds");
	
	// TEMPORARY MODELINGS
	gLink[Franka_Index::LINK_1].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gLink[Franka_Index::LINK_1].GetGeomInfo().SetDimension(Vec3(0.1, 0.1, 0.333));

	gJoint[Franka_Index::JOINT_1]->SetActType(srJoint::HYBRID);
	gJoint[Franka_Index::JOINT_1]->SetParentLink(&gLink[Franka_Index::LINK_1]);
	gJoint[Franka_Index::JOINT_1]->SetChildLink(&gLink[Franka_Index::LINK_2]);
	gJoint[Franka_Index::JOINT_1]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.1665)));
	gJoint[Franka_Index::JOINT_1]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	gJoint[Franka_Index::JOINT_1]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gJoint[Franka_Index::JOINT_1]->MakePositionLimit(false);

	gLink[Franka_Index::LINK_2].GetGeomInfo().SetShape(srGeometryInfo::BOX);   //dummy link
	gLink[Franka_Index::LINK_2].GetGeomInfo().SetDimension(Vec3(0.0, 0.0, 0.0));

	gJoint[Franka_Index::JOINT_2]->SetActType(srJoint::HYBRID);
	gJoint[Franka_Index::JOINT_2]->SetParentLink(&gLink[Franka_Index::LINK_2]);
	gJoint[Franka_Index::JOINT_2]->SetChildLink(&gLink[Franka_Index::LINK_3]);		
	gJoint[Franka_Index::JOINT_2]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0, 0.0)));
	gJoint[Franka_Index::JOINT_2]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0, -0.158)));
	gJoint[Franka_Index::JOINT_2]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gJoint[Franka_Index::JOINT_2]->MakePositionLimit(false);

	gLink[Franka_Index::LINK_3].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gLink[Franka_Index::LINK_3].GetGeomInfo().SetDimension(Vec3(0.05, 0.05, 0.316));

	gJoint[Franka_Index::JOINT_3]->SetActType(srJoint::HYBRID);
	gJoint[Franka_Index::JOINT_3]->SetParentLink(&gLink[Franka_Index::LINK_3]);
	gJoint[Franka_Index::JOINT_3]->SetChildLink(&gLink[Franka_Index::LINK_4]);		
	gJoint[Franka_Index::JOINT_3]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.158)));
	gJoint[Franka_Index::JOINT_3]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(-0.0825, 0.0, 0.0)));
	gJoint[Franka_Index::JOINT_3]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gJoint[Franka_Index::JOINT_3]->MakePositionLimit(false);

	gLink[Franka_Index::LINK_4].GetGeomInfo().SetShape(srGeometryInfo::BOX);    //dummy link
	gLink[Franka_Index::LINK_4].GetGeomInfo().SetDimension(Vec3(0.0, 0.0, 0.0));

	gJoint[Franka_Index::JOINT_4]->SetActType(srJoint::HYBRID);
	gJoint[Franka_Index::JOINT_4]->SetParentLink(&gLink[Franka_Index::LINK_4]);
	gJoint[Franka_Index::JOINT_4]->SetChildLink(&gLink[Franka_Index::LINK_5]);
	gJoint[Franka_Index::JOINT_4]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0, 0.0)));
	gJoint[Franka_Index::JOINT_4]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0825, 0.0, -0.192)));
	gJoint[Franka_Index::JOINT_4]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gJoint[Franka_Index::JOINT_4]->MakePositionLimit(false);

	gLink[Franka_Index::LINK_5].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gLink[Franka_Index::LINK_5].GetGeomInfo().SetDimension(Vec3(0.1, 0.1, 0.384));

	gJoint[Franka_Index::JOINT_5]->SetActType(srJoint::HYBRID);
	gJoint[Franka_Index::JOINT_5]->SetParentLink(&gLink[Franka_Index::LINK_5]);
	gJoint[Franka_Index::JOINT_5]->SetChildLink(&gLink[Franka_Index::LINK_6]);
	gJoint[Franka_Index::JOINT_5]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.192)));
	gJoint[Franka_Index::JOINT_5]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	gJoint[Franka_Index::JOINT_5]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gJoint[Franka_Index::JOINT_5]->MakePositionLimit(false);

	gLink[Franka_Index::LINK_6].GetGeomInfo().SetShape(srGeometryInfo::BOX);    //dummy link
	gLink[Franka_Index::LINK_6].GetGeomInfo().SetDimension(Vec3(0.0, 0.0, 0.0));

	gJoint[Franka_Index::JOINT_6]->SetActType(srJoint::HYBRID);
	gJoint[Franka_Index::JOINT_6]->SetParentLink(&gLink[Franka_Index::LINK_6]);
	gJoint[Franka_Index::JOINT_6]->SetChildLink(&gLink[Franka_Index::LINK_7]);
	gJoint[Franka_Index::JOINT_6]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0, 0.0)));
	gJoint[Franka_Index::JOINT_6]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(-0.044, 0.0, 0.0)));
	gJoint[Franka_Index::JOINT_6]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gJoint[Franka_Index::JOINT_6]->MakePositionLimit(false);

	gLink[Franka_Index::LINK_7].GetGeomInfo().SetShape(srGeometryInfo::BOX);    
	gLink[Franka_Index::LINK_7].GetGeomInfo().SetDimension(Vec3(0.088, 0.05, 0.05));

	gJoint[Franka_Index::JOINT_7]->SetActType(srJoint::HYBRID);
	gJoint[Franka_Index::JOINT_7]->SetParentLink(&gLink[Franka_Index::LINK_7]);
	gJoint[Franka_Index::JOINT_7]->SetChildLink(&gLink[Franka_Index::ENDEFFECTOR]);
	gJoint[Franka_Index::JOINT_7]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 2*SR_PI_HALF), Vec3(0.044, 0.0, 0.0)));
	gJoint[Franka_Index::JOINT_7]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, -0.0535)));
	gJoint[Franka_Index::JOINT_7]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gJoint[Franka_Index::JOINT_7]->MakePositionLimit(false);

	// ADD FOR OTHER LINKS AND JOINTS
	
	gLink[Franka_Index::ENDEFFECTOR].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gLink[Franka_Index::ENDEFFECTOR].GetGeomInfo().SetDimension(Vec3(0.05, 0.07, 0.107));
	

	gWeldJoint[Franka_Index::WELDJOINT_GRIP_MARKER]->SetActType(srJoint::PASSIVE);
	gWeldJoint[Franka_Index::WELDJOINT_GRIP_MARKER]->SetParentLink(&gLink[Franka_Index::ENDEFFECTOR]);
	gWeldJoint[Franka_Index::WELDJOINT_GRIP_MARKER]->SetChildLink(&gMarkerLink[Franka_Index::MLINK_GRIP]);
	gWeldJoint[Franka_Index::WELDJOINT_GRIP_MARKER]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	gWeldJoint[Franka_Index::WELDJOINT_GRIP_MARKER]->SetChildLinkFrame(EulerZYX(Vec3(0.0, SR_PI, 0.0), Vec3(0.088, 0.0, 0.333 + 0.316 + 0.384 - 0.107 - 0.12)));		// consider offset for gripper assembly
	
	// marker links (act as the reference frames for the outputs from forward kinematics etc.)
	gMarkerLink[Franka_Index::MLINK_GRIP].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gMarkerLink[Franka_Index::MLINK_GRIP].GetGeomInfo().SetDimension(Vec3(0.01, 0.01, 0.01));
	gMarkerLink[Franka_Index::MLINK_GRIP].GetGeomInfo().SetColor(0.1f, 0.1f, 0.1f);
	gMarkerLink[Franka_Index::MLINK_GRIP].SetInertia(Inertia(0.001));
	
	this->SetBaseLink(&gLink[Franka_Index::LINK_1]);
	this->SetBaseLinkType(srSystem::FIXED);
}

void Franka::AssembleCollision()
{
	m_numCollision = 0;

	gLink[Franka_Index::LINK_2].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.1, 0.1, 0.333));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, 0.0, 0.0)));
	
	// ADD FOR OTHER LINKS
}

void Franka::SetInertia()
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
void Franka::SetTorqueLimit()
{
	UpperTorqueLimit[Franka_Index::JOINT_1] = DEG2RAD(300);
	UpperTorqueLimit[Franka_Index::JOINT_2] = DEG2RAD(300);
	UpperTorqueLimit[Franka_Index::JOINT_3] = DEG2RAD(300);
	UpperTorqueLimit[Franka_Index::JOINT_4] = DEG2RAD(300);
	UpperTorqueLimit[Franka_Index::JOINT_5] = DEG2RAD(300);
	UpperTorqueLimit[Franka_Index::JOINT_6] = DEG2RAD(300);
	UpperTorqueLimit[Franka_Index::JOINT_7] = DEG2RAD(300);


	LowerTorqueLimit[Franka_Index::JOINT_1] = DEG2RAD(-300);
	LowerTorqueLimit[Franka_Index::JOINT_2] = DEG2RAD(-300);
	LowerTorqueLimit[Franka_Index::JOINT_3] = DEG2RAD(-300);
	LowerTorqueLimit[Franka_Index::JOINT_4] = DEG2RAD(-300);
	LowerTorqueLimit[Franka_Index::JOINT_5] = DEG2RAD(-300);
	LowerTorqueLimit[Franka_Index::JOINT_6] = DEG2RAD(-300);
	LowerTorqueLimit[Franka_Index::JOINT_7] = DEG2RAD(-300);

	for (int i = 0; i < DEGREE_OF_FREEDOM_Franka_JOINT; i++)
		gJoint[i]->SetTorqueLimit(LowerTorqueLimit[i], UpperTorqueLimit[i]);
}

Eigen::VectorXd Franka::getLowerJointLimit() const
{
	Eigen::VectorXd llim(DEGREE_OF_FREEDOM_Franka_JOINT);
	for (int i = 0; i < DEGREE_OF_FREEDOM_Franka_JOINT; i++)
		llim[i] =  DEG2RAD(LowerJointLimit[i]);
	return llim;
}

Eigen::VectorXd Franka::getUpperJointLimit() const
{
	Eigen::VectorXd ulim(DEGREE_OF_FREEDOM_Franka_JOINT);
	for (int i = 0; i < DEGREE_OF_FREEDOM_Franka_JOINT; i++)
		ulim[i] = DEG2RAD(UpperJointLimit[i]);
	return ulim;
}
