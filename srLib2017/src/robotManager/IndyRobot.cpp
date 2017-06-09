#include "IndyRobot.h"
#include "common\utils.h"
#include "makeSpecialCol.h"

IndyRobot::IndyRobot(bool elbowUp, double gripperRot)
{
	for (int i = 0; i < NUM_OF_RJOINT_INDY; i++)
		gJoint[i] = new srRevoluteJoint;

	for (int i = 0; i < NUM_OF_PJOINT_INDY; i++)
		gPjoint[i] = new srPrismaticJoint;

	for (int i = 0; i < NUM_OF_WJOINT_INDY; i++)
		gWeldJoint[i] = new srWeldJoint;

	AssembleModel(gripperRot);
	AssembleCollision();
	SetJointLimit(elbowUp);
	SetInitialConfiguration();
	SetInertia();
	homePos = Eigen::VectorXd::Zero(6);
	qInvKinInit = Eigen::VectorXd::Zero(6);
	if (elbowUp)
	{
		homePos[1] = -SR_PI_HALF; homePos[3] = SR_PI_HALF; homePos[4] = -0.5 * SR_PI;
		qInvKinInit[0] = -0.224778; qInvKinInit[1] = -1.91949; qInvKinInit[2] = -0.384219; qInvKinInit[3] = 1.5708; qInvKinInit[4] = -0.73291; qInvKinInit[5] = 1.79557;
	}
	else
	{
		homePos[1] = DEG2RAD(30); homePos[2] = DEG2RAD(-220); homePos[3] = DEG2RAD(90); homePos[4] = DEG2RAD(-100); 
		qInvKinInit[0] = -0.074913; qInvKinInit[1] = -0.612778; qInvKinInit[2] = -2.488023; qInvKinInit[3] = 1.570796; qInvKinInit[4] = -1.530005; qInvKinInit[5] = 1.645710;
	}
	this->SetSelfCollision(true);
}

IndyRobot::~IndyRobot()
{
	for (int i = 0; i<NUM_OF_RJOINT_INDY; i++)
		SR_SAFE_DELETE(gJoint[i]);
	for (int i = 0; i<NUM_OF_PJOINT_INDY; i++)
		SR_SAFE_DELETE(gPjoint[i]);
	for (int i = 0; i<NUM_OF_WJOINT_INDY; i++)
		SR_SAFE_DELETE(gWeldJoint[i]);

}

void IndyRobot::SetActType(srJoint::ACTTYPE actType /*= srJoint::HYBRID*/)
{
	for (int i = 0; i < NUM_OF_RJOINT_INDY; i++)
		gJoint[i]->SetActType(actType);
}

void IndyRobot::SetGripperActType(srJoint::ACTTYPE actType /*= srJoint::HYBRID*/)
{
	for (int i = 0; i < NUM_OF_PJOINT_INDY; i++)
		gPjoint[i]->SetActType(actType);
}

void IndyRobot::SetGripperActType(srJoint::ACTTYPE actType, vector<int> gpJointIdx)
{
	for (unsigned int i = 0; i < gpJointIdx.size(); i++)
		gPjoint[gpJointIdx[i]]->SetActType(actType);
}

void IndyRobot::SetJointLimit(bool elbowUp)
{
	// unit: deg
	// from robot spec
	
	// R joint
	if (elbowUp)
	{
		UpperJointLimit[Indy_Index::JOINT_1] = 130;
		UpperJointLimit[Indy_Index::JOINT_2] = 135;
		UpperJointLimit[Indy_Index::JOINT_3] = 80;
		UpperJointLimit[Indy_Index::JOINT_4] = 180;
		UpperJointLimit[Indy_Index::JOINT_5] = 180;
		UpperJointLimit[Indy_Index::JOINT_6] = 200;

		LowerJointLimit[Indy_Index::JOINT_1] = -130;
		LowerJointLimit[Indy_Index::JOINT_2] = -135;
		LowerJointLimit[Indy_Index::JOINT_3] = -100;
		LowerJointLimit[Indy_Index::JOINT_4] = -90;
		LowerJointLimit[Indy_Index::JOINT_5] = -180;
		LowerJointLimit[Indy_Index::JOINT_6] = -200;
	}
	else
	{
		UpperJointLimit[Indy_Index::JOINT_1] = 130;
		UpperJointLimit[Indy_Index::JOINT_2] = 135;
		UpperJointLimit[Indy_Index::JOINT_3] = -90;			// 80, to maintain elbow down shape (reducing range will beneficial for fast planning)
		UpperJointLimit[Indy_Index::JOINT_4] = 180;
		UpperJointLimit[Indy_Index::JOINT_5] = 180;
		UpperJointLimit[Indy_Index::JOINT_6] = 200;

		LowerJointLimit[Indy_Index::JOINT_1] = -130;
		LowerJointLimit[Indy_Index::JOINT_2] = -135;
		LowerJointLimit[Indy_Index::JOINT_3] = -235;		// -100
		LowerJointLimit[Indy_Index::JOINT_4] = -90;
		LowerJointLimit[Indy_Index::JOINT_5] = -180;
		LowerJointLimit[Indy_Index::JOINT_6] = -200;
	}

	// P joint
	UpperPJointLimit[Indy_Index::GRIPJOINT_L] = 0.0294*0.5;
	UpperPJointLimit[Indy_Index::GRIPJOINT_U] = 0.0;
	
	LowerPJointLimit[Indy_Index::GRIPJOINT_L] = 0.0;
	LowerPJointLimit[Indy_Index::GRIPJOINT_U] = -0.0294*0.5;


	// temp
	//UpperJointLimit[Indy_Index::JOINT_1] = 360;
	//UpperJointLimit[Indy_Index::JOINT_2] = 360;
	//UpperJointLimit[Indy_Index::JOINT_3] = 360;
	//UpperJointLimit[Indy_Index::JOINT_4] = 360;
	//UpperJointLimit[Indy_Index::JOINT_5] = 360;
	//UpperJointLimit[Indy_Index::JOINT_6] = 360;


	//LowerJointLimit[Indy_Index::JOINT_1] = -360;
	//LowerJointLimit[Indy_Index::JOINT_2] = -360;
	//LowerJointLimit[Indy_Index::JOINT_3] = -360;
	//LowerJointLimit[Indy_Index::JOINT_4] = -360;
	//LowerJointLimit[Indy_Index::JOINT_5] = -360;
	//LowerJointLimit[Indy_Index::JOINT_6] = -360;

	for (int i = 0; i < DEGREE_OF_FREEDOM_INDY_JOINT; i++)
		gJoint[i]->SetPositionLimit(LowerJointLimit[i], UpperJointLimit[i]);
	for (int i = 0; i < NUM_OF_PJOINT_INDY; i++)
		gPjoint[i]->SetPositionLimit(LowerPJointLimit[i], UpperPJointLimit[i]);
}

void IndyRobot::SetVelocityLimit()
{
	// unit: deg/s
	VelocityLimit[Indy_Index::JOINT_1] = 100;
	VelocityLimit[Indy_Index::JOINT_2] = 100;
	VelocityLimit[Indy_Index::JOINT_3] = 100;
	VelocityLimit[Indy_Index::JOINT_4] = 150;
	VelocityLimit[Indy_Index::JOINT_5] = 150;
	VelocityLimit[Indy_Index::JOINT_6] = 200;

}

void IndyRobot::SetInitialConfiguration()
{
	for (int i = 0; i < DEGREE_OF_FREEDOM_INDY_JOINT; i++)
		gJoint[i]->m_State.m_rValue[0] = DEG2RAD(0);

	KIN_UpdateFrame_All_The_Entity();
}

void IndyRobot::AssembleModel(double gripperRot)
{
	// default color
	for (int i = 0; i < NUM_OF_LINK_INDY; i++)
		gLink[i].GetGeomInfo().SetColor(0.3f, 0.3f, 0.3f, 1.0f);



	gLink[Indy_Index::LINK_1].SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));

	gLink[Indy_Index::LINK_1].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[Indy_Index::LINK_1].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.1574)));
	gLink[Indy_Index::LINK_1].GetGeomInfo().SetFileName("../../../workspace/robot/Indy_3ds/LINK_1.3ds");
	


	gJoint[Indy_Index::JOINT_1]->SetActType(srJoint::HYBRID);
	gJoint[Indy_Index::JOINT_1]->SetParentLink(&gLink[Indy_Index::LINK_1]);
	gJoint[Indy_Index::JOINT_1]->SetChildLink(&gLink[Indy_Index::LINK_2]);
	gJoint[Indy_Index::JOINT_1]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	gJoint[Indy_Index::JOINT_1]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	gJoint[Indy_Index::JOINT_1]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gJoint[Indy_Index::JOINT_1]->MakePositionLimit(false);

	gLink[Indy_Index::LINK_2].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[Indy_Index::LINK_2].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.1574)));
	gLink[Indy_Index::LINK_2].GetGeomInfo().SetFileName("../../../workspace/robot/Indy_3ds/LINK_2.3ds");
	gLink[Indy_Index::LINK_2].GetGeomInfo().SetColor(0.15f, 0.15f, 0.15f, 1.0f);


	gJoint[Indy_Index::JOINT_2]->SetActType(srJoint::HYBRID);
	gJoint[Indy_Index::JOINT_2]->SetParentLink(&gLink[Indy_Index::LINK_2]);
	gJoint[Indy_Index::JOINT_2]->SetChildLink(&gLink[Indy_Index::LINK_3]);
	gJoint[Indy_Index::JOINT_2]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.0, 0.0, 0.2634)));
	gJoint[Indy_Index::JOINT_2]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.0, 0.0, 0.2634)));
	gJoint[Indy_Index::JOINT_2]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gJoint[Indy_Index::JOINT_2]->MakePositionLimit(false);

	gLink[Indy_Index::LINK_3].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[Indy_Index::LINK_3].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.1574)));
	gLink[Indy_Index::LINK_3].GetGeomInfo().SetFileName("../../../workspace/robot/Indy_3ds/LINK_3.3ds");

	gJoint[Indy_Index::JOINT_3]->SetActType(srJoint::HYBRID);
	gJoint[Indy_Index::JOINT_3]->SetParentLink(&gLink[Indy_Index::LINK_3]);
	gJoint[Indy_Index::JOINT_3]->SetChildLink(&gLink[Indy_Index::LINK_4]);
	gJoint[Indy_Index::JOINT_3]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.0, 0.0, 0.6884)));
	gJoint[Indy_Index::JOINT_3]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.0, 0.0, 0.6884)));
	gJoint[Indy_Index::JOINT_3]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gJoint[Indy_Index::JOINT_3]->MakePositionLimit(false);

	gLink[Indy_Index::LINK_4].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[Indy_Index::LINK_4].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.1574)));
	gLink[Indy_Index::LINK_4].GetGeomInfo().SetFileName("../../../workspace/robot/Indy_3ds/LINK_4.3ds");
	gLink[Indy_Index::LINK_4].GetGeomInfo().SetColor(0.15f, 0.15f, 0.15f, 1.0f);

	gJoint[Indy_Index::JOINT_4]->SetActType(srJoint::HYBRID);
	gJoint[Indy_Index::JOINT_4]->SetParentLink(&gLink[Indy_Index::LINK_4]);
	gJoint[Indy_Index::JOINT_4]->SetChildLink(&gLink[Indy_Index::LINK_5]);
	gJoint[Indy_Index::JOINT_4]->SetParentLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF, 0.0), Vec3(0.0, 0.0, 0.631+ 0.1574)));
	gJoint[Indy_Index::JOINT_4]->SetChildLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF,0.0), Vec3(0.0, 0.0, 0.631 + 0.1574)));
	gJoint[Indy_Index::JOINT_4]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gJoint[Indy_Index::JOINT_4]->MakePositionLimit(false);

	gLink[Indy_Index::LINK_5].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	//gLink[Indy_Index::LINK_5].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.1574)));
	gLink[Indy_Index::LINK_5].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.0, -0.631 , 0.7884)));
	gLink[Indy_Index::LINK_5].GetGeomInfo().SetFileName("../../../workspace/robot/Indy_3ds/LINK_5.3ds");

	gJoint[Indy_Index::JOINT_5]->SetActType(srJoint::HYBRID);
	gJoint[Indy_Index::JOINT_5]->SetParentLink(&gLink[Indy_Index::LINK_5]);
	gJoint[Indy_Index::JOINT_5]->SetChildLink(&gLink[Indy_Index::LINK_6]);
	gJoint[Indy_Index::JOINT_5]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.4155, 0.0, 0.7884)));
	gJoint[Indy_Index::JOINT_5]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.4155, 0.0, 0.7884)));
	gJoint[Indy_Index::JOINT_5]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gJoint[Indy_Index::JOINT_5]->MakePositionLimit(false);

	gLink[Indy_Index::LINK_6].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	//gLink[Indy_Index::LINK_6].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.1574)));
	gLink[Indy_Index::LINK_6].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(SR_PI_HALF,0.0,  0.0), Vec3(0.4155, -0.4155,0.0))*EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.0, -0.631, 0.7884)));
	gLink[Indy_Index::LINK_6].GetGeomInfo().SetFileName("../../../workspace/robot/Indy_3ds/LINK_6.3ds");
	gLink[Indy_Index::LINK_6].GetGeomInfo().SetColor(0.15f, 0.15f, 0.15f, 1.0f);

	gJoint[Indy_Index::JOINT_6]->SetActType(srJoint::HYBRID);
	gJoint[Indy_Index::JOINT_6]->SetParentLink(&gLink[Indy_Index::LINK_6]);
	gJoint[Indy_Index::JOINT_6]->SetChildLink(&gLink[Indy_Index::ENDEFFECTOR]);
	gJoint[Indy_Index::JOINT_6]->SetParentLinkFrame(EulerZYX(Vec3(0.0,SR_PI_HALF, 0.0), Vec3(0.4155, 0.0, 0.631 + 0.1574+0.1529)));
	gJoint[Indy_Index::JOINT_6]->SetChildLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF, 0.0), Vec3(0.4155, 0.0, 0.631 + 0.1574 + 0.1529)));
	gJoint[Indy_Index::JOINT_6]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gJoint[Indy_Index::JOINT_6]->MakePositionLimit(false);

	gLink[Indy_Index::ENDEFFECTOR].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[Indy_Index::ENDEFFECTOR].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.4155, -0.4155, 0.0))*EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.0, -0.631, 0.7884)));
	gLink[Indy_Index::ENDEFFECTOR].GetGeomInfo().SetFileName("../../../workspace/robot/Indy_3ds/ENDEFFECTOR.3ds");

	// GIMATIC MPLM1630N GRIPPER MODEL
	gWeldJoint[Indy_Index::WELDJOINT_SENSOR]->SetParentLink(&gLink[Indy_Index::ENDEFFECTOR]);
	gWeldJoint[Indy_Index::WELDJOINT_SENSOR]->SetChildLink(&gLink[Indy_Index::SENSOR]);
	gWeldJoint[Indy_Index::WELDJOINT_SENSOR]->SetParentLinkFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.4155 + 0.1089 - 0.01, 0.0, 0.7884 + 0.1529)));
	gWeldJoint[Indy_Index::WELDJOINT_SENSOR]->SetChildLinkFrame(SE3());
	gWeldJoint[Indy_Index::WELDJOINT_SENSOR]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);

	gLink[Indy_Index::SENSOR].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[Indy_Index::SENSOR].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	gLink[Indy_Index::SENSOR].GetGeomInfo().SetFileName("../../../workspace/robot/Indy_3ds/ftsensor.3ds");

	gWeldJoint[Indy_Index::WELDJOINT_GRIPPER]->SetParentLink(&gLink[Indy_Index::SENSOR]);
	gWeldJoint[Indy_Index::WELDJOINT_GRIPPER]->SetChildLink(&gLink[Indy_Index::GRIPPER]);
	gWeldJoint[Indy_Index::WELDJOINT_GRIPPER]->SetParentLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF, SR_PI_HALF), Vec3(0.0, -0.0355, 0.0)));
	//gWeldJoint[Indy_Index::WELDJOINT_GRIPPER]->SetChildLinkFrame(EulerZYX(Vec3(SR_PI, 0.0, -SR_PI_HALF), Vec3(0.0, -0.0355, 0.0)));
	gWeldJoint[Indy_Index::WELDJOINT_GRIPPER]->SetChildLinkFrame(EulerZYX(Vec3(0.0, gripperRot, SR_PI_HALF), Vec3(0.0, -0.0355, 0.0)) * EulerZYX(Vec3(SR_PI, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));		// consider offset for gripper assembly
	gWeldJoint[Indy_Index::WELDJOINT_GRIPPER]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);

	gLink[Indy_Index::GRIPPER].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[Indy_Index::GRIPPER].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	gLink[Indy_Index::GRIPPER].GetGeomInfo().SetFileName("../../../workspace/robot/Indy_3ds/gripper_body.3ds");


	//gPjoint[Indy_Index::GRIPJOINT_L]->SetActType(srJoint::HYBRID);
	gPjoint[Indy_Index::GRIPJOINT_L]->SetActType(srJoint::TORQUE);
	gPjoint[Indy_Index::GRIPJOINT_L]->SetParentLink(&gLink[Indy_Index::GRIPPER]);
	gPjoint[Indy_Index::GRIPJOINT_L]->SetChildLink(&gLink[Indy_Index::GRIPPER_FINGER_L]);
	gPjoint[Indy_Index::GRIPJOINT_L]->SetParentLinkFrame(SE3());
	gPjoint[Indy_Index::GRIPJOINT_L]->SetChildLinkFrame(SE3());
	gPjoint[Indy_Index::GRIPJOINT_L]->MakePositionLimit(false);

	gLink[Indy_Index::GRIPPER_FINGER_L].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[Indy_Index::GRIPPER_FINGER_L].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.015)));
	gLink[Indy_Index::GRIPPER_FINGER_L].GetGeomInfo().SetFileName("../../../workspace/robot/Indy_3ds/finger1.3ds");
	gLink[Indy_Index::GRIPPER_FINGER_L].GetGeomInfo().SetColor(0.15f, 0.15f, 0.15f, 1.0f);

	//gPjoint[Indy_Index::GRIPJOINT_U]->SetActType(srJoint::HYBRID);
	gPjoint[Indy_Index::GRIPJOINT_U]->SetActType(srJoint::TORQUE);
	gPjoint[Indy_Index::GRIPJOINT_U]->SetParentLink(&gLink[Indy_Index::GRIPPER]);
	gPjoint[Indy_Index::GRIPJOINT_U]->SetChildLink(&gLink[Indy_Index::GRIPPER_FINGER_U]);
	gPjoint[Indy_Index::GRIPJOINT_U]->SetParentLinkFrame(SE3());
	gPjoint[Indy_Index::GRIPJOINT_U]->SetChildLinkFrame(SE3());
	gPjoint[Indy_Index::GRIPJOINT_U]->MakePositionLimit(false);

	gLink[Indy_Index::GRIPPER_FINGER_U].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[Indy_Index::GRIPPER_FINGER_U].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, -0.015)));
	gLink[Indy_Index::GRIPPER_FINGER_U].GetGeomInfo().SetFileName("../../../workspace/robot/Indy_3ds/finger2.3ds");
	//gLink[Indy_Index::GRIPPER_FINGER_U].GetGeomInfo().SetColor(GRAYCOLORSDA20D);

	// dummy links to simulate torque controlled grasp
	gPjoint[Indy_Index::GRIPJOINT_L_DUMMY]->SetActType(srJoint::HYBRID);
	gPjoint[Indy_Index::GRIPJOINT_L_DUMMY]->SetParentLink(&gLink[Indy_Index::GRIPPER]);
	gPjoint[Indy_Index::GRIPJOINT_L_DUMMY]->SetChildLink(&gLink[Indy_Index::GRIPPER_FINGER_L_DUMMY]);
	gPjoint[Indy_Index::GRIPJOINT_L_DUMMY]->SetParentLinkFrame(SE3());
	gPjoint[Indy_Index::GRIPJOINT_L_DUMMY]->SetChildLinkFrame(SE3());
	gPjoint[Indy_Index::GRIPJOINT_L_DUMMY]->MakePositionLimit(false);

	gLink[Indy_Index::GRIPPER_FINGER_L_DUMMY].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gLink[Indy_Index::GRIPPER_FINGER_L_DUMMY].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	gLink[Indy_Index::GRIPPER_FINGER_L_DUMMY].GetGeomInfo().SetDimension(Vec3(0.00, 0.00, 0.00));
	gLink[Indy_Index::GRIPPER_FINGER_L_DUMMY].SetInertia(Inertia(0.001));
	//gLink[Indy_Index::GRIPPER_FINGER_L].GetGeomInfo().SetColor(GRAYCOLORSDA20D);

	gPjoint[Indy_Index::GRIPJOINT_U_DUMMY]->SetActType(srJoint::HYBRID);
	gPjoint[Indy_Index::GRIPJOINT_U_DUMMY]->SetParentLink(&gLink[Indy_Index::GRIPPER]);
	gPjoint[Indy_Index::GRIPJOINT_U_DUMMY]->SetChildLink(&gLink[Indy_Index::GRIPPER_FINGER_U_DUMMY]);
	gPjoint[Indy_Index::GRIPJOINT_U_DUMMY]->SetParentLinkFrame(SE3());
	gPjoint[Indy_Index::GRIPJOINT_U_DUMMY]->SetChildLinkFrame(SE3());
	gPjoint[Indy_Index::GRIPJOINT_U_DUMMY]->MakePositionLimit(false);

	gLink[Indy_Index::GRIPPER_FINGER_U_DUMMY].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gLink[Indy_Index::GRIPPER_FINGER_U_DUMMY].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	gLink[Indy_Index::GRIPPER_FINGER_U_DUMMY].GetGeomInfo().SetDimension(Vec3(0.00, 0.00, 0.00));
	gLink[Indy_Index::GRIPPER_FINGER_U_DUMMY].SetInertia(Inertia(0.001));
	//gLink[Indy_Index::GRIPPER_FINGER_U].GetGeomInfo().SetColor(GRAYCOLORSDA20D);

	// marker links
	gMarkerLink[Indy_Index::MLINK_GRIP].GetGeomInfo().SetDimension(Vec3(0.00, 0.00, 0.00));
	gMarkerLink[Indy_Index::MLINK_GRIP].GetGeomInfo().SetColor(0.1f, 0.1f, 0.1f);
	gMarkerLink[Indy_Index::MLINK_GRIP].SetInertia(Inertia(0.001));

	gWeldJoint[Indy_Index::WELDJOINT_GRIP_MARKER]->SetActType(srJoint::PASSIVE);
	gWeldJoint[Indy_Index::WELDJOINT_GRIP_MARKER]->SetParentLink(&gLink[Indy_Index::SENSOR]);
	gWeldJoint[Indy_Index::WELDJOINT_GRIP_MARKER]->SetChildLink(&gMarkerLink[Indy_Index::MLINK_GRIP]);
	gWeldJoint[Indy_Index::WELDJOINT_GRIP_MARKER]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	gWeldJoint[Indy_Index::WELDJOINT_GRIP_MARKER]->SetChildLinkFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, -SR_PI_HALF), Vec3(0.0, 0.0, -0.128)));		// consider offset for gripper assembly

	////// old model
	//gWeldJoint[Indy_Index::WELDJOINT_GRIPPER]->SetParentLink(&gLink[Indy_Index::ENDEFFECTOR]);
	//gWeldJoint[Indy_Index::WELDJOINT_GRIPPER]->SetChildLink(&gLink[Indy_Index::GRIPPER]);
	//gWeldJoint[Indy_Index::WELDJOINT_GRIPPER]->SetParentLinkFrame(SE3());
	//gWeldJoint[Indy_Index::WELDJOINT_GRIPPER]->SetChildLinkFrame(Inv(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0),Vec3(0.4155+0.1089, 0.0, 0.7884+0.1529))));
	//gWeldJoint[Indy_Index::WELDJOINT_GRIPPER]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);

	//gLink[Indy_Index::GRIPPER].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	//gLink[Indy_Index::GRIPPER].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, SR_PI, -SR_PI_HALF), Vec3(0.0, 0.1735, 0.0595)));
	//gLink[Indy_Index::GRIPPER].GetGeomInfo().SetFileName("../../../workspace/robot/Indy_3ds/LINK_LEFT_GRIPPER2_b.3ds");


	////gPjoint[Indy_Index::GRIPJOINT_L]->SetActType(srJoint::HYBRID);
	//gPjoint[Indy_Index::GRIPJOINT_L]->SetActType(srJoint::TORQUE);
	//gPjoint[Indy_Index::GRIPJOINT_L]->SetParentLink(&gLink[Indy_Index::GRIPPER]);
	//gPjoint[Indy_Index::GRIPJOINT_L]->SetChildLink(&gLink[Indy_Index::GRIPPER_FINGER_L]);
	//gPjoint[Indy_Index::GRIPJOINT_L]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.1805, 0.0)));
	//gPjoint[Indy_Index::GRIPJOINT_L]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, -0.0595, 0.0)));
	//gPjoint[Indy_Index::GRIPJOINT_L]->MakePositionLimit(false);

	//gLink[Indy_Index::GRIPPER_FINGER_L].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	//gLink[Indy_Index::GRIPPER_FINGER_L].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(SR_PI, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	//gLink[Indy_Index::GRIPPER_FINGER_L].GetGeomInfo().SetFileName("../../../workspace/robot/Indy_3ds/LINK_LEFT_GRIPPER2_finger_l.3ds");
	////gLink[Indy_Index::GRIPPER_FINGER_L].GetGeomInfo().SetColor(GRAYCOLORSDA20D);

	////gPjoint[Indy_Index::GRIPJOINT_U]->SetActType(srJoint::HYBRID);
	//gPjoint[Indy_Index::GRIPJOINT_U]->SetActType(srJoint::TORQUE);
	//gPjoint[Indy_Index::GRIPJOINT_U]->SetParentLink(&gLink[Indy_Index::GRIPPER]);
	//gPjoint[Indy_Index::GRIPJOINT_U]->SetChildLink(&gLink[Indy_Index::GRIPPER_FINGER_U]);
	//gPjoint[Indy_Index::GRIPJOINT_U]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.1805, 0.0)));
	//gPjoint[Indy_Index::GRIPJOINT_U]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0595, 0.0)));
	//gPjoint[Indy_Index::GRIPJOINT_U]->MakePositionLimit(false);

	//gLink[Indy_Index::GRIPPER_FINGER_U].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	//gLink[Indy_Index::GRIPPER_FINGER_U].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(SR_PI, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	//gLink[Indy_Index::GRIPPER_FINGER_U].GetGeomInfo().SetFileName("../../../workspace/robot/Indy_3ds/LINK_LEFT_GRIPPER2_finger_u.3ds");
	////gLink[Indy_Index::GRIPPER_FINGER_U].GetGeomInfo().SetColor(GRAYCOLORSDA20D);


	//// marker links
	//gMarkerLink[Indy_Index::MLINK_GRIP].GetGeomInfo().SetDimension(Vec3(0.00, 0.00, 0.00));
	//gMarkerLink[Indy_Index::MLINK_GRIP].GetGeomInfo().SetColor(0.1f, 0.1f, 0.1f);
	//gMarkerLink[Indy_Index::MLINK_GRIP].SetInertia(Inertia(0.0));

	//gWeldJoint[Indy_Index::WELDJOINT_GRIP_MARKER]->SetActType(srJoint::PASSIVE);
	//gWeldJoint[Indy_Index::WELDJOINT_GRIP_MARKER]->SetParentLink(&gLink[Indy_Index::GRIPPER]);
	//gWeldJoint[Indy_Index::WELDJOINT_GRIP_MARKER]->SetChildLink(&gMarkerLink[Indy_Index::MLINK_GRIP]);
	//gWeldJoint[Indy_Index::WELDJOINT_GRIP_MARKER]->SetParentLinkFrame(EulerZYX(Vec3(-SR_PI_HALF, SR_PI_HALF, 0.0), Vec3(0.0, 0.0, 0.0)));
	//gWeldJoint[Indy_Index::WELDJOINT_GRIP_MARKER]->SetChildLinkFrame(EulerZYX(Vec3(0.0, SR_PI, 0.0), Vec3(0.0485, 0.0, -0.2645)));

	///////////////////// old model end

	this->SetBaseLink(&gLink[Indy_Index::LINK_1]);
	this->SetBaseLinkType(srSystem::FIXED);
}

void IndyRobot::AssembleCollision()
{
	m_numCollision = 0;

	int numBox = 5;
	double thickness = 0.03;
	double space = 0.005;
	vector<pair<Vec3, SE3>> boxSet;	
	// Link1: 
	// Cylinder size: H = 182.5, D = 154
	// Center position = (0,0,182.5*0.5)
	
	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.1825*0.5)), 0.154*0.5, 0.1825, thickness, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[Indy_Index::LINK_1].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}

	// Link2: 
	// Cylinder size: H = 251 , D = 130
	// Center position = ((59.5-36.5)/2, 0, 263.4)

	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.0, (0.0595 - 0.0365)*0.5, 0.2634)), 0.130*0.5*0.8, 0.251, thickness*0.5, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[Indy_Index::LINK_2].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}

	// Link3:
	// BOX1 size: D_Z = 425-130 , D_Y = 36.5, D_X = 130
	// BOX1 Center position = (0.0, (-155-36.5)/2 , 425/2 + 263.4)

	// BOX2 size: D_Z = 425-130 , D_Y = 20 , D_X = 130
	// BOX2 Center position = (0.0, (155+20)/2 , 425/2 + 263.4)

	gLink[Indy_Index::LINK_3].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.130, 0.0365, 0.425-0.130));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, (-0.155-0.0365)*0.5, 0.425*0.5+0.2634)));

	gLink[Indy_Index::LINK_3].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.130, 0.020, 0.425 - 0.130));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, (0.155 + 0.020)*0.5, 0.425*0.5 + 0.2634)));

	//Link4:
	// Cylinder1 size: H = 251 , D = 130
	// Center1 position = ((59.5-36.5)/2, 0, 688.4)

	// Cylinder2 size: H = 446 , D = 98
	// Center2 position = (344.5 - 446/2 , 0, 788.4)

	numBox = 5;
	thickness = 0.03;

	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.0, (0.0595 - 0.0365)*0.5, 0.6884)), 0.130*0.5*0.8, 0.251, thickness*0.5, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[Indy_Index::LINK_4].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}

	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, SR_PI_HALF, 0.0), Vec3(0.3445-0.446*0.5, 0.0, 0.7884)), 0.098*0.5, 0.446, thickness*0.25, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[Indy_Index::LINK_4].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}

	// Link5:
	// Cylinder size: H = 156 , D = 115
	// Center position = (344.5+71 , 0, 788.4)
	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.3445+0.071, 0.0, 0.7884)), 0.115*0.5, 0.156, thickness*0.25, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[Indy_Index::LINK_5].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}

	// Link6:
	// Cylinder1 size: H = 74.9-79/2 , D = 73
	// Center1 position = (344.5+71 , 0, H/2 + 788.4+156/2)

	// Cylinder2 size: H = 210.4 - 10, D = 79
	// Center2 position = (344.5+71 + (108.9-101.5)/2 - 5, 0, H + 788.4+156/2+79/2)

	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.3445 + 0.071, 0.0, (0.0749 - 0.0790*0.5)*0.5+0.7884+0.156*0.5)), 0.073*0.5, (0.0749 - 0.0790*0.5), thickness*0.25, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[Indy_Index::LINK_6].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}

	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, SR_PI_HALF, 0.0), Vec3(0.3445 + 0.071+ (0.1089 - 0.1015) *0.5 - 0.005, 0.0, (0.0749 - 0.0790*0.5)+ 0.7884 + 0.156*0.5+0.079*0.5)), 0.079*0.5, 0.2104 - 0.01, thickness*0.25, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[Indy_Index::LINK_6].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}


	
	// Gripper
	double cylheight = 0.0475;
	numBox = 5;
	thickness = 0.03;
	space = 0.005;

	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, -0.02475, 0.0)), 0.037, cylheight, 0.01, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[Indy_Index::GRIPPER].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}

	double eps_colsize = 0.0001;

	gLink[Indy_Index::GRIPPER].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(0.044, 0.043 - eps_colsize, 0.062);
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, -0.0495 - 0.0215, 0.0)));

	gLink[Indy_Index::GRIPPER_FINGER_L].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(0.019, 0.005, 0.015);
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-0.003, -0.096, 0.0075 + 0.015)));

	gLink[Indy_Index::GRIPPER_FINGER_L].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(0.013, 0.01, 0.015);
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, -0.0985, 0.0075 + 0.015)));

	gLink[Indy_Index::GRIPPER_FINGER_L].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(0.013, 0.035, 0.0063);
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, -0.111, 0.00315 + 0.015)));

	gLink[Indy_Index::GRIPPER_FINGER_L].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(0.013, 0.0235, 0.005);
	gCollision[m_numCollision++].SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.38), Vec3(0.0, -(0.0935 + 0.021 - 0.0025*sin(0.38)), 0.0106 - 0.0025*cos(0.38) + 0.015)));


	gLink[Indy_Index::GRIPPER_FINGER_U].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(0.019, 0.005, 0.015);
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.003, -0.096, -0.0075 - 0.015)));

	gLink[Indy_Index::GRIPPER_FINGER_U].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(0.013, 0.01, 0.015);
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, -0.0985, -0.0075 - 0.015)));

	gLink[Indy_Index::GRIPPER_FINGER_U].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(0.013, 0.035, 0.0063);
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, -0.111, -0.00315 - 0.015)));

	gLink[Indy_Index::GRIPPER_FINGER_U].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(0.013, 0.0235, 0.005);
	gCollision[m_numCollision++].SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, -0.38), Vec3(0.0, -(0.0935 + 0.021 - 0.0025*sin(0.38)), - 0.0106 + 0.0025*cos(0.38) - 0.015)));


	double eps = 0.00001;
	gLink[Indy_Index::GRIPPER_FINGER_L_DUMMY].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(0.019, 0.005, 0.005);
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-0.003, -0.096, 0.0175 + 0.015 + eps)));

	gLink[Indy_Index::GRIPPER_FINGER_U_DUMMY].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(0.019, 0.005, 0.005);
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.003, -0.096, -0.0175 - 0.015 - eps)));


	// old model
	//double cylheight = 0.0805;
	//numBox = 5;
	//thickness = 0.03;
	//space = 0.005;

	//boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.04, 0.0)), 0.05, cylheight, 0.01, space, numBox);
	//for (int i = 0; i < numBox; i++)
	//{
	//	gLink[Indy_Index::GRIPPER].AddCollision(&gCollision[m_numCollision]);
	//	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	//	gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
	//	gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	//}

	//gLink[Indy_Index::GRIPPER].AddCollision(&gCollision[m_numCollision]);
	//gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	//gCollision[m_numCollision].GetGeomInfo().SetDimension(0.055, 0.085, 0.080);
	//gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, 0.13, 0.0)));

	//gLink[Indy_Index::GRIPPER].AddCollision(&gCollision[m_numCollision]);
	//gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	//gCollision[m_numCollision].GetGeomInfo().SetDimension(0.03, 0.03, 0.02);
	//gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.025, 0.04, 0.06)));

	//gLink[Indy_Index::GRIPPER].AddCollision(&gCollision[m_numCollision]);
	//gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	//gCollision[m_numCollision].GetGeomInfo().SetDimension(0.055, 0.020, 0.125);
	//gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, 0.18, 0.0)));

	//gLink[Indy_Index::GRIPPER_FINGER_L].AddCollision(&gCollision[m_numCollision]);
	//gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	//gCollision[m_numCollision].GetGeomInfo().SetDimension(0.025, 0.055, 0.030);
	//gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, -0.0157, 0.03)));

	//gLink[Indy_Index::GRIPPER_FINGER_L].AddCollision(&gCollision[m_numCollision]);
	//gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	//gCollision[m_numCollision].GetGeomInfo().SetDimension(0.025, 0.005, 0.038);
	//gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, 0.0063, 0.065)));

	//gLink[Indy_Index::GRIPPER_FINGER_U].AddCollision(&gCollision[m_numCollision]);
	//gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	//gCollision[m_numCollision].GetGeomInfo().SetDimension(0.025, 0.045, 0.05);
	//gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, 0.0207, 0.04)));

	//gLink[Indy_Index::GRIPPER_FINGER_U].AddCollision(&gCollision[m_numCollision]);
	//gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	//gCollision[m_numCollision].GetGeomInfo().SetDimension(0.025, 0.048, 0.015);
	//gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, 0.0692, 0.0555)));

	//gLink[Indy_Index::GRIPPER_FINGER_U].AddCollision(&gCollision[m_numCollision]);
	//gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	//gCollision[m_numCollision].GetGeomInfo().SetDimension(0.025, 0.01, 0.021);
	//gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, 0.0692+0.048*0.5-0.005, 0.0735)));

}

void IndyRobot::SetInertia()
{
	double m1 = 5.598009;
	double m2 = 4.722018;
	double m3 = 6.17608;
	double m4 = 6.549344;
	double m5 = 1.923968;
	double m6 = 3.3099;
	double m7 = 0.0913;
	Vec3 r1 = Vec3(0.001027, -0.000056, 0.087474);
	Vec3 r2 = Vec3(-0.000007, 0.0256, 0.255037);
	Vec3 r3 = Vec3(-0.002282, -0.006782, 0.469538);
	Vec3 r4 = Vec3(0.010135, 0.016094, 0.735077);
	Vec3 r5 = Vec3(0.316302, -0.000768, 0.785255);
	Vec3 r6 = Vec3(0.423187, 0.000899, 0.873598);
	Vec3 r7 = Vec3(0.552202, 0.0, 0.995299);
	
	Inertia G1(0.031682, 0.031384, 0.017293, -0.000023, -0.000017, -0.000468, 0.0, 0.0, 0.0, m1);
	Inertia G2(0.02108, 0.010309, 0.021876, 0, 0.000901, 0.000001, 0.0, 0.0, 0.0, m2);
	Inertia G3(0.216937, 0.167656, 0.067914, -0.000322, -0.001387, -0.001728, 0.0, 0.0, 0.0, m3);
	Inertia G4(0.033518, 0.046678, 0.042127, -0.001069, -0.004744, 0.003172, 0.0, 0.0, 0.0, m4);
	Inertia G5(0.004299, 0.024684, 0.023256, -0.000053, -0.000005, -0.000499, 0.0, 0.0, 0.0, m5);
	Inertia G6(0.022679, 0.028158, 0.00752, -0.000064, 0.000104, 0.001751, 0.0, 0.0, 0.0, m6);
	Inertia G7(0.000101, 0.000101, 0.0001, 0, 0, 0, 0.0, 0.0, 0.0, m7);

	gLink[Indy_Index::LINK_1].SetInertia(G1.Transform(SE3(-r1)));
	gLink[Indy_Index::LINK_2].SetInertia(G2.Transform(SE3(-r2)));
	gLink[Indy_Index::LINK_3].SetInertia(G3.Transform(SE3(-r3)));
	gLink[Indy_Index::LINK_4].SetInertia(G4.Transform(SE3(-r4)));
	gLink[Indy_Index::LINK_5].SetInertia(G5.Transform(SE3(-r5)));
	gLink[Indy_Index::LINK_6].SetInertia(G6.Transform(SE3(-r6)));
	gLink[Indy_Index::ENDEFFECTOR].SetInertia(G7.Transform(SE3(-r7)));
		

	// GIMATIC MPLM1630N GRIPPER MODEL
	double m_g1 = 0.10629;
	double m_g2 = 0.16205;
	double m_g3 = 0.00515;
	double m_g4 = 0.00515;
	Vec3 r_g1 = Vec3(0.01e-3, -17.35e-3, 0.04e-3);
	Vec3 r_g2 = Vec3(-0.01e-3, -61.66e-3, -0.06e-3);
	Vec3 r_g3 = Vec3(-0.61e-3, -106.69e-3, 22.76e-3);
	Vec3 r_g4 = Vec3(0.61e-3, -106.69e-3, -22.76e-3);
	Inertia G_g1(42161.46e-9, 63838.67e-9, 42131.49e-9, 9.63e-9, 8.46e-9, -37.38e-9, 0.0, 0.0, 0.0, m_g1);
	Inertia G_g2(94794.64e-9, 82474.62e-9, 77696.95e-9, -22.07e-9, -57.59e-9, 408.09e-9, 0.0, 0.0, 0.0, m_g2);
	double ratio = 1.0;
	Inertia G_g3(ratio * 530.34e-9, ratio * 177.96e-9, ratio * 551.94e-9, -ratio * 33.21e-9, ratio * 56.44e-9, -ratio * 3.22e-9, 0.0, 0.0, 0.0, ratio * m_g3);
	Inertia G_g4(ratio * 530.34e-9, ratio * 177.96e-9, ratio * 551.94e-9, ratio * 33.21e-9, -ratio * 56.44e-9, -ratio * 3.22e-9, 0.0, 0.0, 0.0, ratio * m_g4);
	gLink[Indy_Index::SENSOR].SetInertia(G_g1.Transform(SE3(-r_g1)));
	gLink[Indy_Index::GRIPPER].SetInertia(G_g2.Transform(SE3(-r_g2)));
	gLink[Indy_Index::GRIPPER_FINGER_L].SetInertia(G_g3.Transform(SE3(-r_g3)));
	gLink[Indy_Index::GRIPPER_FINGER_U].SetInertia(G_g4.Transform(SE3(-r_g4)));
}

void IndyRobot::SetTorqueLimit()
{
	UpperTorqueLimit[Indy_Index::JOINT_1] = DEG2RAD(300);
	UpperTorqueLimit[Indy_Index::JOINT_2] = DEG2RAD(300);
	UpperTorqueLimit[Indy_Index::JOINT_3] = DEG2RAD(300);
	UpperTorqueLimit[Indy_Index::JOINT_4] = DEG2RAD(300);
	UpperTorqueLimit[Indy_Index::JOINT_5] = DEG2RAD(300);
	UpperTorqueLimit[Indy_Index::JOINT_6] = DEG2RAD(300);


	LowerTorqueLimit[Indy_Index::JOINT_1] = DEG2RAD(-300);
	LowerTorqueLimit[Indy_Index::JOINT_2] = DEG2RAD(-300);
	LowerTorqueLimit[Indy_Index::JOINT_3] = DEG2RAD(-300);
	LowerTorqueLimit[Indy_Index::JOINT_4] = DEG2RAD(-300);
	LowerTorqueLimit[Indy_Index::JOINT_5] = DEG2RAD(-300);
	LowerTorqueLimit[Indy_Index::JOINT_6] = DEG2RAD(-300);

	for (int i = 0; i < DEGREE_OF_FREEDOM_INDY_JOINT; i++)
		gJoint[i]->SetTorqueLimit(LowerTorqueLimit[i], UpperTorqueLimit[i]);
}

Eigen::VectorXd IndyRobot::getLowerJointLimit() const
{
	Eigen::VectorXd llim(DEGREE_OF_FREEDOM_INDY_JOINT);
	for (int i = 0; i < DEGREE_OF_FREEDOM_INDY_JOINT; i++)
		llim[i] =  DEG2RAD(LowerJointLimit[i]);
	return llim;
}

Eigen::VectorXd IndyRobot::getUpperJointLimit() const
{
	Eigen::VectorXd ulim(DEGREE_OF_FREEDOM_INDY_JOINT);
	for (int i = 0; i < DEGREE_OF_FREEDOM_INDY_JOINT; i++)
		ulim[i] = DEG2RAD(UpperJointLimit[i]);
	return ulim;
}
