#include "UR3Robot.h"
#include "common\utils.h"
#include "makeSpecialCol.h"

UR3Robot::UR3Robot(bool elbowUp, double gripperRot)
{
	for (int i = 0; i < NUM_OF_RJOINT_UR3; i++)
		gJoint[i] = new srRevoluteJoint;
	
	for (int i = 0; i < NUM_OF_GRIPERJOINT_UR3; i++)
		gGripJoint[i] = new srRevoluteJoint;

	for (int i = 0; i < NUM_OF_WJOINT_UR3; i++)
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
		//homePos[1] = -SR_PI_HALF; homePos[3] = SR_PI_HALF; homePos[4] = -0.5 * SR_PI;
		homePos[1] = -SR_PI_HALF; homePos[2] = DEG2RAD(40.0); homePos[3] = SR_PI_HALF; homePos[4] = DEG2RAD(40);
		qInvKinInit[0] = -0.224778; qInvKinInit[1] = -1.91949; qInvKinInit[2] = -0.384219; qInvKinInit[3] = 1.5708; qInvKinInit[4] = -0.73291; qInvKinInit[5] = 1.79557;
	}
	else
	{
		homePos[1] = DEG2RAD(30); homePos[2] = DEG2RAD(-220); homePos[3] = DEG2RAD(90); homePos[4] = DEG2RAD(-100); 
		qInvKinInit[0] = -0.074913; qInvKinInit[1] = -0.612778; qInvKinInit[2] = -2.488023; qInvKinInit[3] = 1.570796; qInvKinInit[4] = -1.530005; qInvKinInit[5] = 1.645710;
	}
	this->SetSelfCollision(true);
}

UR3Robot::~UR3Robot()
{
	for (int i = 0; i<NUM_OF_RJOINT_UR3; i++)
		SR_SAFE_DELETE(gJoint[i]);
	for (int i = 0; i<NUM_OF_GRIPERJOINT_UR3; i++)
		SR_SAFE_DELETE(gGripJoint[i]);
	for (int i = 0; i<NUM_OF_WJOINT_UR3; i++)
		SR_SAFE_DELETE(gWeldJoint[i]);

}

void UR3Robot::SetActType(srJoint::ACTTYPE actType /*= srJoint::HYBRID*/)
{
	for (int i = 0; i < NUM_OF_RJOINT_UR3; i++)
		gJoint[i]->SetActType(actType);
}

void UR3Robot::SetGripperActType(srJoint::ACTTYPE actType /*= srJoint::HYBRID*/)
{
	for (int i = 0; i < NUM_OF_GRIPERJOINT_UR3; i++)
		gGripJoint[i]->SetActType(actType);
}

void UR3Robot::SetGripperActType(srJoint::ACTTYPE actType, vector<int> gpJointIdx)
{
	for (unsigned int i = 0; i < gpJointIdx.size(); i++)
		gGripJoint[gpJointIdx[i]]->SetActType(actType);
}

void UR3Robot::SetJointLimit(bool elbowUp)
{
	// unit: deg
	// from robot spec
	
	// R joint
	if (elbowUp)
	{
		UpperJointLimit[UR3_Index::JOINT_1] = 360;
		UpperJointLimit[UR3_Index::JOINT_2] = 360;
		UpperJointLimit[UR3_Index::JOINT_3] = 360;
		UpperJointLimit[UR3_Index::JOINT_4] = 360;
		UpperJointLimit[UR3_Index::JOINT_5] = 360;
		UpperJointLimit[UR3_Index::JOINT_6] = 360;

		LowerJointLimit[UR3_Index::JOINT_1] = -360;
		LowerJointLimit[UR3_Index::JOINT_2] = -360;
		LowerJointLimit[UR3_Index::JOINT_3] = -360;
		LowerJointLimit[UR3_Index::JOINT_4] = -360;
		LowerJointLimit[UR3_Index::JOINT_5] = -360;
		LowerJointLimit[UR3_Index::JOINT_6] = -360;
	}
	else
	{
		UpperJointLimit[UR3_Index::JOINT_1] = 360;
		UpperJointLimit[UR3_Index::JOINT_2] = 360;
		UpperJointLimit[UR3_Index::JOINT_3] = 360;
		UpperJointLimit[UR3_Index::JOINT_4] = 360;
		UpperJointLimit[UR3_Index::JOINT_5] = 360;
		UpperJointLimit[UR3_Index::JOINT_6] = 360;

		LowerJointLimit[UR3_Index::JOINT_1] = -360;
		LowerJointLimit[UR3_Index::JOINT_2] = -360;
		LowerJointLimit[UR3_Index::JOINT_3] = -360;
		LowerJointLimit[UR3_Index::JOINT_4] = -360;
		LowerJointLimit[UR3_Index::JOINT_5] = -360;
		LowerJointLimit[UR3_Index::JOINT_6] = -360;
	}


	///////////////////////// add gripper joint limit??



	for (int i = 0; i < DEGREE_OF_FREEDOM_UR3_JOINT; i++)
		gJoint[i]->SetPositionLimit(LowerJointLimit[i], UpperJointLimit[i]);

}

void UR3Robot::SetVelocityLimit()
{
	// unit: deg/s
	VelocityLimit[UR3_Index::JOINT_1] = 180;
	VelocityLimit[UR3_Index::JOINT_2] = 180;
	VelocityLimit[UR3_Index::JOINT_3] = 180;
	VelocityLimit[UR3_Index::JOINT_4] = 180;
	VelocityLimit[UR3_Index::JOINT_5] = 180;
	VelocityLimit[UR3_Index::JOINT_6] = 180;

}

void UR3Robot::SetInitialConfiguration()
{
	for (int i = 0; i < DEGREE_OF_FREEDOM_UR3_JOINT; i++)
		gJoint[i]->m_State.m_rValue[0] = DEG2RAD(0);

	KIN_UpdateFrame_All_The_Entity();
}

void UR3Robot::AssembleModel(double gripperRot)
{
	// default color
	for (int i = 0; i < NUM_OF_LINK_UR3; i++)
		gLink[i].GetGeomInfo().SetColor(0.3f, 0.3f, 0.3f, 1.0f);



	gLink[UR3_Index::LINK_1].SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	gLink[UR3_Index::LINK_1].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR3_Index::LINK_1].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0, 0.0)));
	gLink[UR3_Index::LINK_1].GetGeomInfo().SetFileName("../../../workspace/robot/ur3_3ds/link1.3ds");
	


	gJoint[UR3_Index::JOINT_1]->SetActType(srJoint::HYBRID);
	gJoint[UR3_Index::JOINT_1]->SetParentLink(&gLink[UR3_Index::LINK_1]);
	gJoint[UR3_Index::JOINT_1]->SetChildLink(&gLink[UR3_Index::LINK_2]);
	gJoint[UR3_Index::JOINT_1]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0,0.0), Vec3(0.0, 0.0, 0.0)));
	gJoint[UR3_Index::JOINT_1]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0,0.0), Vec3(0.0, 0.0, 0.0)));
	gJoint[UR3_Index::JOINT_1]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gJoint[UR3_Index::JOINT_1]->MakePositionLimit(false);

	gLink[UR3_Index::LINK_2].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR3_Index::LINK_2].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0, 0.0)));
	gLink[UR3_Index::LINK_2].GetGeomInfo().SetFileName("../../../workspace/robot/ur3_3ds/link2.3ds");
	//gLink[UR3_Index::LINK_2].GetGeomInfo().SetColor(0.15f, 0.15f, 0.15f, 1.0f);

	gJoint[UR3_Index::JOINT_2]->SetActType(srJoint::HYBRID);
	gJoint[UR3_Index::JOINT_2]->SetParentLink(&gLink[UR3_Index::LINK_2]);
	gJoint[UR3_Index::JOINT_2]->SetChildLink(&gLink[UR3_Index::LINK_3]);
	gJoint[UR3_Index::JOINT_2]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.0, 0.0, 0.1519)));
	gJoint[UR3_Index::JOINT_2]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.0, 0.0, 0.1519)));
	gJoint[UR3_Index::JOINT_2]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gJoint[UR3_Index::JOINT_2]->MakePositionLimit(false);

	gLink[UR3_Index::LINK_3].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR3_Index::LINK_3].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0, 0.0)));
	gLink[UR3_Index::LINK_3].GetGeomInfo().SetFileName("../../../workspace/robot/ur3_3ds/link3.3ds");

	gJoint[UR3_Index::JOINT_3]->SetActType(srJoint::HYBRID);
	gJoint[UR3_Index::JOINT_3]->SetParentLink(&gLink[UR3_Index::LINK_3]);
	gJoint[UR3_Index::JOINT_3]->SetChildLink(&gLink[UR3_Index::LINK_4]);
	gJoint[UR3_Index::JOINT_3]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.0, 0.0, 0.39555)));
	gJoint[UR3_Index::JOINT_3]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.0, 0.0, 0.39555)));
	gJoint[UR3_Index::JOINT_3]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gJoint[UR3_Index::JOINT_3]->MakePositionLimit(false);


	gLink[UR3_Index::LINK_4].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR3_Index::LINK_4].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0, 0.0)));
	gLink[UR3_Index::LINK_4].GetGeomInfo().SetFileName("../../../workspace/robot/ur3_3ds/link4.3ds");
	//gLink[UR3_Index::LINK_4].GetGeomInfo().SetColor(0.15f, 0.15f, 0.15f, 1.0f);

	gJoint[UR3_Index::JOINT_4]->SetActType(srJoint::HYBRID);
	gJoint[UR3_Index::JOINT_4]->SetParentLink(&gLink[UR3_Index::LINK_4]);
	gJoint[UR3_Index::JOINT_4]->SetChildLink(&gLink[UR3_Index::LINK_5]);
	gJoint[UR3_Index::JOINT_4]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.0, 0.0, 0.60855)));
	gJoint[UR3_Index::JOINT_4]->SetChildLinkFrame(EulerZYX(Vec3(0.0,0.0, -SR_PI_HALF), Vec3(0.0, 0.0, 0.60855)));
	gJoint[UR3_Index::JOINT_4]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gJoint[UR3_Index::JOINT_4]->MakePositionLimit(false);

	gLink[UR3_Index::LINK_5].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR3_Index::LINK_5].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0, 0.0)));
	gLink[UR3_Index::LINK_5].GetGeomInfo().SetFileName("../../../workspace/robot/ur3_3ds/link5.3ds");


	gJoint[UR3_Index::JOINT_5]->SetActType(srJoint::HYBRID);
	gJoint[UR3_Index::JOINT_5]->SetParentLink(&gLink[UR3_Index::LINK_5]);
	gJoint[UR3_Index::JOINT_5]->SetChildLink(&gLink[UR3_Index::LINK_6]);
	gJoint[UR3_Index::JOINT_5]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.1104,0.0)));
	gJoint[UR3_Index::JOINT_5]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.1104, 0.0)));
	gJoint[UR3_Index::JOINT_5]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gJoint[UR3_Index::JOINT_5]->MakePositionLimit(false);

	gLink[UR3_Index::LINK_6].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR3_Index::LINK_6].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0, 0.0)));
	gLink[UR3_Index::LINK_6].GetGeomInfo().SetFileName("../../../workspace/robot/ur3_3ds/link6.3ds");
	//gLink[UR3_Index::LINK_6].GetGeomInfo().SetColor(0.15f, 0.15f, 0.15f, 1.0f);

	gJoint[UR3_Index::JOINT_6]->SetActType(srJoint::HYBRID);
	gJoint[UR3_Index::JOINT_6]->SetParentLink(&gLink[UR3_Index::LINK_6]);
	gJoint[UR3_Index::JOINT_6]->SetChildLink(&gLink[UR3_Index::ENDEFFECTOR]);
	gJoint[UR3_Index::JOINT_6]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.0, 0.0, 0.69195)));
	gJoint[UR3_Index::JOINT_6]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.0, 0.0, 0.69195)));
	gJoint[UR3_Index::JOINT_6]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gJoint[UR3_Index::JOINT_6]->MakePositionLimit(false);

	gLink[UR3_Index::ENDEFFECTOR].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR3_Index::ENDEFFECTOR].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0, 0.0)));
	gLink[UR3_Index::ENDEFFECTOR].GetGeomInfo().SetFileName("../../../workspace/robot/ur3_3ds/link7.3ds");

	// sensor
	gWeldJoint[UR3_Index::WELDJOINT_SENSOR]->SetParentLink(&gLink[UR3_Index::ENDEFFECTOR]);
	gWeldJoint[UR3_Index::WELDJOINT_SENSOR]->SetChildLink(&gLink[UR3_Index::SENSOR]);
	gWeldJoint[UR3_Index::WELDJOINT_SENSOR]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.0, 0.0, 0.69195)));
	gWeldJoint[UR3_Index::WELDJOINT_SENSOR]->SetChildLinkFrame(SE3(Vec3(0.0, 0.0, -(0.1928 + 0.0375))));
	gWeldJoint[UR3_Index::WELDJOINT_SENSOR]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);

	gLink[UR3_Index::SENSOR].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR3_Index::SENSOR].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI), Vec3(0.0, 0.0,0.0)));
	gLink[UR3_Index::SENSOR].GetGeomInfo().SetFileName("../../../workspace/robot/ur3_3ds/ROBOTIQ_FTS-300-UR-KIT.3ds");

	gWeldJoint[UR3_Index::WELDJOINT_COUPLING]->SetParentLink(&gLink[UR3_Index::SENSOR]);
	gWeldJoint[UR3_Index::WELDJOINT_COUPLING]->SetChildLink(&gLink[UR3_Index::COUPLING]);
	gWeldJoint[UR3_Index::WELDJOINT_COUPLING]->SetParentLinkFrame(SE3());
	gWeldJoint[UR3_Index::WELDJOINT_COUPLING]->SetChildLinkFrame(SE3(Vec3(0.0, 0.0, - 0.004)));
	gWeldJoint[UR3_Index::WELDJOINT_COUPLING]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);


	gLink[UR3_Index::COUPLING].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR3_Index::COUPLING].GetGeomInfo().SetLocalFrame(SE3());
	gLink[UR3_Index::COUPLING].GetGeomInfo().SetFileName("../../../workspace/robot/ur3_3ds/coupling.3ds");

	gWeldJoint[UR3_Index::WELDJOINT_GRIPPER]->SetParentLink(&gLink[UR3_Index::COUPLING]);
	gWeldJoint[UR3_Index::WELDJOINT_GRIPPER]->SetChildLink(&gLink[UR3_Index::GRIPPER]);
	gWeldJoint[UR3_Index::WELDJOINT_GRIPPER]->SetParentLinkFrame(SE3());
	gWeldJoint[UR3_Index::WELDJOINT_GRIPPER]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.0, 0.0, 0.0))* SE3(Vec3(0.0, 0.0, -0.09-0.004)));
	gWeldJoint[UR3_Index::WELDJOINT_GRIPPER]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);

	gLink[UR3_Index::GRIPPER].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR3_Index::GRIPPER].GetGeomInfo().SetLocalFrame(SE3());
	gLink[UR3_Index::GRIPPER].GetGeomInfo().SetFileName("../../../workspace/robot/ur3_3ds/Robotiq_body.3ds");


	gGripJoint[UR3_Index::GRIPJOINT_1_M]->SetParentLink(&gLink[UR3_Index::GRIPPER]);
	gGripJoint[UR3_Index::GRIPJOINT_1_M]->SetParentLinkFrame(SE3(Vec3(-0.0127, -0.02858, 0.0)));
	gGripJoint[UR3_Index::GRIPJOINT_1_M]->SetChildLink(&gLink[UR3_Index::GRIPPER_LINK_1_M]);
	gGripJoint[UR3_Index::GRIPJOINT_1_M]->SetChildLinkFrame(SE3(Vec3(-0.0127, -0.02858, 0.0)));

	gLink[UR3_Index::GRIPPER_LINK_1_M].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR3_Index::GRIPPER_LINK_1_M].GetGeomInfo().SetLocalFrame(SE3());
	gLink[UR3_Index::GRIPPER_LINK_1_M].GetGeomInfo().SetFileName("../../../workspace/robot/ur3_3ds/Robotiq_link1_m.3ds");

	gGripJoint[UR3_Index::GRIPJOINT_1_P]->SetParentLink(&gLink[UR3_Index::GRIPPER]);
	gGripJoint[UR3_Index::GRIPJOINT_1_P]->SetParentLinkFrame(SE3(Vec3(0.0127, -0.02858, 0.0)));
	gGripJoint[UR3_Index::GRIPJOINT_1_P]->SetChildLink(&gLink[UR3_Index::GRIPPER_LINK_1_P]);
	gGripJoint[UR3_Index::GRIPJOINT_1_P]->SetChildLinkFrame(SE3(Vec3(0.0127, -0.02858, 0.0)));

	gLink[UR3_Index::GRIPPER_LINK_1_P].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR3_Index::GRIPPER_LINK_1_P].GetGeomInfo().SetLocalFrame(SE3());
	gLink[UR3_Index::GRIPPER_LINK_1_P].GetGeomInfo().SetFileName("../../../workspace/robot/ur3_3ds/Robotiq_link1_p.3ds");

	gGripJoint[UR3_Index::GRIPJOINT_2_M]->SetParentLink(&gLink[UR3_Index::GRIPPER]);
	gGripJoint[UR3_Index::GRIPJOINT_2_M]->SetParentLinkFrame(SE3(Vec3(-0.0306, -0.0351, 0.0)));
	gGripJoint[UR3_Index::GRIPJOINT_2_M]->SetChildLink(&gLink[UR3_Index::GRIPPER_LINK_2_M]);
	gGripJoint[UR3_Index::GRIPJOINT_2_M]->SetChildLinkFrame(SE3(Vec3(-0.0306, -0.0351, 0.0)));

	gLink[UR3_Index::GRIPPER_LINK_2_M].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR3_Index::GRIPPER_LINK_2_M].GetGeomInfo().SetLocalFrame(SE3());
	gLink[UR3_Index::GRIPPER_LINK_2_M].GetGeomInfo().SetFileName("../../../workspace/robot/ur3_3ds/Robotiq_link2a_m.3ds");

	gGripJoint[UR3_Index::GRIPJOINT_2_P]->SetParentLink(&gLink[UR3_Index::GRIPPER]);
	gGripJoint[UR3_Index::GRIPJOINT_2_P]->SetParentLinkFrame(SE3(Vec3(0.0306, -0.0351, 0.0)));
	gGripJoint[UR3_Index::GRIPJOINT_2_P]->SetChildLink(&gLink[UR3_Index::GRIPPER_LINK_2_P]);
	gGripJoint[UR3_Index::GRIPJOINT_2_P]->SetChildLinkFrame(SE3(Vec3(0.0306, -0.0351, 0.0)));

	gLink[UR3_Index::GRIPPER_LINK_2_P].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR3_Index::GRIPPER_LINK_2_P].GetGeomInfo().SetLocalFrame(SE3());
	gLink[UR3_Index::GRIPPER_LINK_2_P].GetGeomInfo().SetFileName("../../../workspace/robot/ur3_3ds/Robotiq_link2a_p.3ds");

	gGripJoint[UR3_Index::GRIPJOINT_F_M]->SetParentLink(&gLink[UR3_Index::GRIPPER_LINK_1_M]);
	gGripJoint[UR3_Index::GRIPJOINT_F_M]->SetParentLinkFrame(SE3(Vec3(-0.04985, 0.01485, 0.0)));
	gGripJoint[UR3_Index::GRIPJOINT_F_M]->SetChildLink(&gLink[UR3_Index::GRIPPER_FINGER_M]);
	gGripJoint[UR3_Index::GRIPJOINT_F_M]->SetChildLinkFrame(SE3(Vec3(-0.04985, 0.01485, 0.0)));

	gLink[UR3_Index::GRIPPER_FINGER_M].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR3_Index::GRIPPER_FINGER_M].GetGeomInfo().SetLocalFrame(SE3());
	gLink[UR3_Index::GRIPPER_FINGER_M].GetGeomInfo().SetFileName("../../../workspace/robot/ur3_3ds/Robotiq_tip_m.3ds");

	gGripJoint[UR3_Index::GRIPJOINT_F_P]->SetParentLink(&gLink[UR3_Index::GRIPPER_LINK_1_P]);
	gGripJoint[UR3_Index::GRIPJOINT_F_P]->SetParentLinkFrame(SE3(Vec3(0.04985, 0.01485, 0.0)));
	gGripJoint[UR3_Index::GRIPJOINT_F_P]->SetChildLink(&gLink[UR3_Index::GRIPPER_FINGER_P]);
	gGripJoint[UR3_Index::GRIPJOINT_F_P]->SetChildLinkFrame(SE3(Vec3(0.04985, 0.01485, 0.0)));


	gLink[UR3_Index::GRIPPER_FINGER_P].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR3_Index::GRIPPER_FINGER_P].GetGeomInfo().SetLocalFrame(SE3());
	gLink[UR3_Index::GRIPPER_FINGER_P].GetGeomInfo().SetFileName("../../../workspace/robot/ur3_3ds/Robotiq_tip_p.3ds");

	gWeldJoint[UR3_Index::WELDJOINT_GRIP_MARKER]->SetActType(srJoint::PASSIVE);
	gWeldJoint[UR3_Index::WELDJOINT_GRIP_MARKER]->SetParentLink(&gLink[UR3_Index::ENDEFFECTOR]);
	gWeldJoint[UR3_Index::WELDJOINT_GRIP_MARKER]->SetChildLink(&gMarkerLink[UR3_Index::MLINK_GRIP]);
	gWeldJoint[UR3_Index::WELDJOINT_GRIP_MARKER]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.0, 0.0, 0.69195)));
	gWeldJoint[UR3_Index::WELDJOINT_GRIP_MARKER]->SetChildLinkFrame(SE3( Vec3(0.0, 0.0, -(0.1928+0.2003))));		// consider offset for gripper assembly
																													// marker links
	gMarkerLink[UR3_Index::MLINK_GRIP].GetGeomInfo().SetDimension(Vec3(0.00, 0.00, 0.00));
	gMarkerLink[UR3_Index::MLINK_GRIP].GetGeomInfo().SetColor(0.1f, 0.1f, 0.1f);
	gMarkerLink[UR3_Index::MLINK_GRIP].SetInertia(Inertia(0.001));




	this->SetBaseLink(&gLink[UR3_Index::LINK_1]);
	this->SetBaseLinkType(srSystem::FIXED);
}

void UR3Robot::AssembleCollision()
{
	m_numCollision = 0;

	int numBox = 5;
	double thickness = 0.03;
	double space = 0.005;
	vector<pair<Vec3, SE3>> boxSet;	

	// Link1: 
	// Cylinder size: H = 86.05, D = 90
	// Center position = (0,0,86.05*0.5)
	
	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.08605*0.5)), 0.09*0.5, 0.08605, thickness, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[UR3_Index::LINK_1].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}



	gLink[UR3_Index::LINK_2].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.09, 0.09, 0.10644));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, 0.0,0.20284-0.10644*0.5)));
	
	gLink[UR3_Index::LINK_3].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.09,0.1108, 0.09));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, 0.17079-0.1108*0.5, 0.1519)));

	gLink[UR3_Index::LINK_3].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.075, 0.075, 0.1264));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, 0.11985,0.34155-0.1264*0.5)));

	gLink[UR3_Index::LINK_3].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.075, 0.10373, 0.075));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0,0.17705-0.10373*0.5,0.39555)));


	gLink[UR3_Index::LINK_4].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.075, 0.04275+0.063*0.5, 0.075));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0,0.069 - (0.04275 + 0.063*0.5)*0.5, 0.39555)));

	gLink[UR3_Index::LINK_4].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.063, 0.063, 0.1199));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, 0.026, 0.56775- 0.1199*0.5)));

	gLink[UR3_Index::LINK_4].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.063, 0.08472, 0.063));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, 0.06960- 0.08472*0.5, 0.60855)));

	gLink[UR3_Index::LINK_5].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.063, 0.063, 0.08472));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0,0.1117, 0.65115- 0.08472*0.5)));

	gLink[UR3_Index::LINK_6].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.063, 0.08472, 0.063));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, 0.153- 0.08472*0.5,0.69195)));

	gLink[UR3_Index::ENDEFFECTOR].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.063, 0.032, 0.063));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, 0.1928- 0.032*0.5, 0.69195)));

	gLink[UR3_Index::SENSOR].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.075, 0.075, 0.031));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0,0.0, -0.0065-0.031*0.5)));

	gLink[UR3_Index::COUPLING].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.075, 0.075, 0.005));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, 0.0, 0.0)));



	gLink[UR3_Index::GRIPPER_FINGER_P].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.03, 0.048, 0.02));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.03*0.5 + 0.04385, 0.01 + 0.05285 - 0.5*0.048, 0.0)));

	gLink[UR3_Index::GRIPPER_FINGER_M].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.03, 0.048, 0.02));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-(0.03*0.5 + 0.04385), 0.01 + 0.05285 - 0.5*0.048, 0.0)));

	gLink[UR3_Index::GRIPPER].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.15, 0.09 + 0.01485, 0.085));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, 0.5*(0.01485 - 0.09), 0.0)));
	
}

void UR3Robot::SetInertia()
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
void UR3Robot::SetTorqueLimit()
{
	UpperTorqueLimit[UR3_Index::JOINT_1] = DEG2RAD(300);
	UpperTorqueLimit[UR3_Index::JOINT_2] = DEG2RAD(300);
	UpperTorqueLimit[UR3_Index::JOINT_3] = DEG2RAD(300);
	UpperTorqueLimit[UR3_Index::JOINT_4] = DEG2RAD(300);
	UpperTorqueLimit[UR3_Index::JOINT_5] = DEG2RAD(300);
	UpperTorqueLimit[UR3_Index::JOINT_6] = DEG2RAD(300);


	LowerTorqueLimit[UR3_Index::JOINT_1] = DEG2RAD(-300);
	LowerTorqueLimit[UR3_Index::JOINT_2] = DEG2RAD(-300);
	LowerTorqueLimit[UR3_Index::JOINT_3] = DEG2RAD(-300);
	LowerTorqueLimit[UR3_Index::JOINT_4] = DEG2RAD(-300);
	LowerTorqueLimit[UR3_Index::JOINT_5] = DEG2RAD(-300);
	LowerTorqueLimit[UR3_Index::JOINT_6] = DEG2RAD(-300);

	for (int i = 0; i < DEGREE_OF_FREEDOM_UR3_JOINT; i++)
		gJoint[i]->SetTorqueLimit(LowerTorqueLimit[i], UpperTorqueLimit[i]);
}

Eigen::VectorXd UR3Robot::getLowerJointLimit() const
{
	Eigen::VectorXd llim(DEGREE_OF_FREEDOM_UR3_JOINT);
	for (int i = 0; i < DEGREE_OF_FREEDOM_UR3_JOINT; i++)
		llim[i] =  DEG2RAD(LowerJointLimit[i]);
	return llim;
}

Eigen::VectorXd UR3Robot::getUpperJointLimit() const
{
	Eigen::VectorXd ulim(DEGREE_OF_FREEDOM_UR3_JOINT);
	for (int i = 0; i < DEGREE_OF_FREEDOM_UR3_JOINT; i++)
		ulim[i] = DEG2RAD(UpperJointLimit[i]);
	return ulim;
}
