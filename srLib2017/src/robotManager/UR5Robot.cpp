#include "UR5Robot.h"
#include "common\utils.h"
#include "makeSpecialCol.h"

UR5Robot::UR5Robot(bool elbowUp, double gripperRot)
{
	for (int i = 0; i < NUM_OF_RJOINT_UR5; i++)
		gJoint[i] = new srRevoluteJoint;
	
	for (int i = 0; i < NUM_OF_GRIPERJOINT_UR5; i++)
		gGripJoint[i] = new srRevoluteJoint;

	for (int i = 0; i < NUM_OF_WJOINT_UR5; i++)
		gWeldJoint[i] = new srWeldJoint;

	AssembleModel(gripperRot);
	AssembleCollision();
	SetJointLimit(elbowUp);
	SetInitialConfiguration();
	SetInertia();
	homePos = Eigen::VectorXd::Zero(6);
	//homePos[1] = -SR_PI_HALF; 
	homePos[0] = SR_PI_HALF;
	homePos[2] = SR_PI_HALF;
	//homePos[3] = -SR_PI_HALF;
	qInvKinInit = Eigen::VectorXd::Zero(6);
	//qInvKinInit[0] = DEG2RAD(-2.43); 	qInvKinInit[1] = DEG2RAD(-68.79);	qInvKinInit[2] = DEG2RAD(93.97);
	//qInvKinInit[3] = DEG2RAD(-115.47);	qInvKinInit[4] = DEG2RAD(-94.96);	qInvKinInit[5] = DEG2RAD(0.0);

	qInvKinInit[0] = 0.797052; qInvKinInit[1] = 0.728445; qInvKinInit[2] = 0.495759; 
	qInvKinInit[3] = 0.346593; qInvKinInit[4] = -1.5708; qInvKinInit[5] = -2.34454;

	if (elbowUp)
	{
		//qInvKinInit[0] = -0.224778; qInvKinInit[1] = -1.91949; qInvKinInit[2] = -0.384219; qInvKinInit[3] = 1.5708; qInvKinInit[4] = -0.73291; qInvKinInit[5] = 1.79557;
	}
	else
	{
		//qInvKinInit[0] = -0.074913; qInvKinInit[1] = -0.612778; qInvKinInit[2] = -2.488023; qInvKinInit[3] = 1.570796; qInvKinInit[4] = -1.530005; qInvKinInit[5] = 1.645710;
	}
	TsrLinkbase2robotbase = SE3();
	//TsrLinkbase2robotbase = EulerZYX(Vec3(SR_PI, 0.0, 0.0), Vec3(0.0, 0.0, 0.69195 - 0.69511 + 0.00195));
	//this->SetSelfCollision(true);
	this->SetSelfCollision(false);
}

UR5Robot::~UR5Robot()
{
	for (int i = 0; i<NUM_OF_RJOINT_UR5; i++)
		SR_SAFE_DELETE(gJoint[i]);
	//for (int i = 0; i<NUM_OF_GRIPERJOINT_UR5; i++)
	//	SR_SAFE_DELETE(gGripJoint[i]);

}

void UR5Robot::SetActType(srJoint::ACTTYPE actType /*= srJoint::HYBRID*/)
{
	for (int i = 0; i < NUM_OF_RJOINT_UR5; i++)
		gJoint[i]->SetActType(actType);
}

void UR5Robot::SetGripperActType(srJoint::ACTTYPE actType /*= srJoint::HYBRID*/)
{
	//for (int i = 0; i < NUM_OF_GRIPERJOINT_UR5; i++)
	//	gGripJoint[i]->SetActType(actType);
}

void UR5Robot::SetGripperActType(srJoint::ACTTYPE actType, vector<int> gpJointIdx)
{
	//for (unsigned int i = 0; i < gpJointIdx.size(); i++)
	//	gGripJoint[gpJointIdx[i]]->SetActType(actType);
}

void UR5Robot::SetJointLimit(bool elbowUp)
{
	// unit: deg
	// from robot spec
	
	// R joint
	if (elbowUp)
	{
		UpperJointLimit[UR5_Index::JOINT_1] = 360;
		UpperJointLimit[UR5_Index::JOINT_2] = 360;
		UpperJointLimit[UR5_Index::JOINT_3] = 360;
		UpperJointLimit[UR5_Index::JOINT_4] = 360;
		UpperJointLimit[UR5_Index::JOINT_5] = 360;
		UpperJointLimit[UR5_Index::JOINT_6] = 360;

		LowerJointLimit[UR5_Index::JOINT_1] = -360;
		LowerJointLimit[UR5_Index::JOINT_2] = -360;
		LowerJointLimit[UR5_Index::JOINT_3] = -360;
		LowerJointLimit[UR5_Index::JOINT_4] = -360;
		LowerJointLimit[UR5_Index::JOINT_5] = -360;
		LowerJointLimit[UR5_Index::JOINT_6] = -360;
	}
	else
	{
		UpperJointLimit[UR5_Index::JOINT_1] = 360;
		UpperJointLimit[UR5_Index::JOINT_2] = 360;
		UpperJointLimit[UR5_Index::JOINT_3] = 360;
		UpperJointLimit[UR5_Index::JOINT_4] = 360;
		UpperJointLimit[UR5_Index::JOINT_5] = 360;
		UpperJointLimit[UR5_Index::JOINT_6] = 360;

		LowerJointLimit[UR5_Index::JOINT_1] = -360;
		LowerJointLimit[UR5_Index::JOINT_2] = -360;
		LowerJointLimit[UR5_Index::JOINT_3] = -360;
		LowerJointLimit[UR5_Index::JOINT_4] = -360;
		LowerJointLimit[UR5_Index::JOINT_5] = -360;
		LowerJointLimit[UR5_Index::JOINT_6] = -360;
	}


	///////////////////////// add gripper joint limit??



	for (int i = 0; i < DEGREE_OF_FREEDOM_UR5_JOINT; i++)
		gJoint[i]->SetPositionLimit(LowerJointLimit[i], UpperJointLimit[i]);

}

void UR5Robot::SetVelocityLimit()
{
	// unit: deg/s
	VelocityLimit[UR5_Index::JOINT_1] = 180;
	VelocityLimit[UR5_Index::JOINT_2] = 180;
	VelocityLimit[UR5_Index::JOINT_3] = 180;
	VelocityLimit[UR5_Index::JOINT_4] = 180;
	VelocityLimit[UR5_Index::JOINT_5] = 180;
	VelocityLimit[UR5_Index::JOINT_6] = 180;

}

void UR5Robot::SetInitialConfiguration()
{
	for (int i = 0; i < DEGREE_OF_FREEDOM_UR5_JOINT; i++)
		gJoint[i]->m_State.m_rValue[0] = DEG2RAD(0);

	KIN_UpdateFrame_All_The_Entity();
}

void UR5Robot::AssembleModel(double gripperRot)
{
	// default color
	for (int i = 0; i < NUM_OF_LINK_UR5; i++)
		gLink[i].GetGeomInfo().SetColor(0.3f, 0.3f, 0.3f, 1.0f);

	SE3 Tcad2srlib = EulerZYX(Vec3(0.0, SR_PI_HALF, SR_PI_HALF), Vec3(0.0, 0.0, 0.0));

	gLink[UR5_Index::LINK_1].SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	gLink[UR5_Index::LINK_1].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR5_Index::LINK_1].GetGeomInfo().SetLocalFrame(Tcad2srlib);
	gLink[UR5_Index::LINK_1].GetGeomInfo().SetFileName("../../../workspace/robot/ur5_3ds/link1.3ds");
	
	gJoint[UR5_Index::JOINT_1]->SetActType(srJoint::HYBRID);
	gJoint[UR5_Index::JOINT_1]->SetParentLink(&gLink[UR5_Index::LINK_1]);
	gJoint[UR5_Index::JOINT_1]->SetChildLink(&gLink[UR5_Index::LINK_2]);
	gJoint[UR5_Index::JOINT_1]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0,0.0), Vec3(0.0, 0.0, 0.0)));
	gJoint[UR5_Index::JOINT_1]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0,0.0), Vec3(0.0, 0.0, 0.0)));
	gJoint[UR5_Index::JOINT_1]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gJoint[UR5_Index::JOINT_1]->MakePositionLimit(false);

	gLink[UR5_Index::LINK_2].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR5_Index::LINK_2].GetGeomInfo().SetLocalFrame(Tcad2srlib);
	gLink[UR5_Index::LINK_2].GetGeomInfo().SetFileName("../../../workspace/robot/ur5_3ds/link2.3ds");
	//gLink[UR5_Index::LINK_2].GetGeomInfo().SetColor(0.15f, 0.15f, 0.15f, 1.0f);

	gJoint[UR5_Index::JOINT_2]->SetActType(srJoint::HYBRID);
	gJoint[UR5_Index::JOINT_2]->SetParentLink(&gLink[UR5_Index::LINK_2]);
	gJoint[UR5_Index::JOINT_2]->SetChildLink(&gLink[UR5_Index::LINK_3]);
	gJoint[UR5_Index::JOINT_2]->SetParentLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF, 0.0), Vec3(0.0, 0.0, 0.0862)));
	//gJoint[UR5_Index::JOINT_2]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.0, 0.0, 0.1519)));
	gJoint[UR5_Index::JOINT_2]->SetChildLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF, 0.0), Vec3(0.0, 0.0, 0.0862)));
	gJoint[UR5_Index::JOINT_2]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gJoint[UR5_Index::JOINT_2]->MakePositionLimit(false);

	gLink[UR5_Index::LINK_3].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR5_Index::LINK_3].GetGeomInfo().SetLocalFrame(Tcad2srlib);
	gLink[UR5_Index::LINK_3].GetGeomInfo().SetFileName("../../../workspace/robot/ur5_3ds/link3.3ds");

	gJoint[UR5_Index::JOINT_3]->SetActType(srJoint::HYBRID);
	gJoint[UR5_Index::JOINT_3]->SetParentLink(&gLink[UR5_Index::LINK_3]);
	gJoint[UR5_Index::JOINT_3]->SetChildLink(&gLink[UR5_Index::LINK_4]);
	gJoint[UR5_Index::JOINT_3]->SetParentLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF,0.0), Vec3(0.0, 0.0, (0.0862 + 0.425))));
	gJoint[UR5_Index::JOINT_3]->SetChildLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF, 0.0), Vec3(0.0, 0.0, (0.0862 + 0.425))));
	gJoint[UR5_Index::JOINT_3]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gJoint[UR5_Index::JOINT_3]->MakePositionLimit(false);


	gLink[UR5_Index::LINK_4].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR5_Index::LINK_4].GetGeomInfo().SetLocalFrame(Tcad2srlib);
	gLink[UR5_Index::LINK_4].GetGeomInfo().SetFileName("../../../workspace/robot/ur5_3ds/link4.3ds");
	//gLink[UR5_Index::LINK_4].GetGeomInfo().SetColor(0.15f, 0.15f, 0.15f, 1.0f);

	gJoint[UR5_Index::JOINT_4]->SetActType(srJoint::HYBRID);
	gJoint[UR5_Index::JOINT_4]->SetParentLink(&gLink[UR5_Index::LINK_4]);
	gJoint[UR5_Index::JOINT_4]->SetChildLink(&gLink[UR5_Index::LINK_5]);
	gJoint[UR5_Index::JOINT_4]->SetParentLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF, 0.0), Vec3(0.0, 0.0, (0.0862 + 0.425 + 0.392))));
	//gJoint[UR5_Index::JOINT_4]->SetChildLinkFrame(EulerZYX(Vec3(0.0,0.0, -SR_PI_HALF), Vec3(0.0, 0.0, 0.60855)));
	gJoint[UR5_Index::JOINT_4]->SetChildLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF, 0.0), Vec3(0.0, 0.0, (0.0862 + 0.425 + 0.392))));
	gJoint[UR5_Index::JOINT_4]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gJoint[UR5_Index::JOINT_4]->MakePositionLimit(false);

	gLink[UR5_Index::LINK_5].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR5_Index::LINK_5].GetGeomInfo().SetLocalFrame(Tcad2srlib);
	gLink[UR5_Index::LINK_5].GetGeomInfo().SetFileName("../../../workspace/robot/ur5_3ds/link5.3ds");


	gJoint[UR5_Index::JOINT_5]->SetActType(srJoint::HYBRID);
	gJoint[UR5_Index::JOINT_5]->SetParentLink(&gLink[UR5_Index::LINK_5]);
	gJoint[UR5_Index::JOINT_5]->SetChildLink(&gLink[UR5_Index::LINK_6]);
	gJoint[UR5_Index::JOINT_5]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3((0.134 - 0.119 + 0.09475), 0.0, (0.0862 + 0.425 + 0.392))));
	gJoint[UR5_Index::JOINT_5]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3((0.134 - 0.119 + 0.09475), 0.0, (0.0862 + 0.425 + 0.392))));
	gJoint[UR5_Index::JOINT_5]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gJoint[UR5_Index::JOINT_5]->MakePositionLimit(false);

	gLink[UR5_Index::LINK_6].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR5_Index::LINK_6].GetGeomInfo().SetLocalFrame(Tcad2srlib);
	gLink[UR5_Index::LINK_6].GetGeomInfo().SetFileName("../../../workspace/robot/ur5_3ds/link6.3ds");
	//gLink[UR5_Index::LINK_6].GetGeomInfo().SetColor(0.15f, 0.15f, 0.15f, 1.0f);

	gJoint[UR5_Index::JOINT_6]->SetActType(srJoint::HYBRID);
	gJoint[UR5_Index::JOINT_6]->SetParentLink(&gLink[UR5_Index::LINK_6]);
	gJoint[UR5_Index::JOINT_6]->SetChildLink(&gLink[UR5_Index::ENDEFFECTOR]);
	gJoint[UR5_Index::JOINT_6]->SetParentLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF, 0.0), Vec3((0.134 - 0.119 + 0.09475), 0.0, (0.0862 + 0.425 + 0.392 + 0.09475))));
	gJoint[UR5_Index::JOINT_6]->SetChildLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF, 0.0), Vec3((0.134 - 0.119 + 0.09475), 0.0, (0.0862 + 0.425 + 0.392 + 0.09475))));
	gJoint[UR5_Index::JOINT_6]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gJoint[UR5_Index::JOINT_6]->MakePositionLimit(false);

	gLink[UR5_Index::ENDEFFECTOR].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR5_Index::ENDEFFECTOR].GetGeomInfo().SetLocalFrame(Tcad2srlib);
	gLink[UR5_Index::ENDEFFECTOR].GetGeomInfo().SetFileName("../../../workspace/robot/ur5_3ds/link7.3ds");

#ifndef _3_Finger
	//gripper
	gWeldJoint[UR5_Index::WELDJOINT_COUPLING]->SetParentLink(&gLink[UR5_Index::ENDEFFECTOR]);
	gWeldJoint[UR5_Index::WELDJOINT_COUPLING]->SetChildLink(&gLink[UR5_Index::GRIPPER]);
	gWeldJoint[UR5_Index::WELDJOINT_COUPLING]->SetParentLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF, SR_PI_HALF), Vec3(0.0, 0.0, 0.0)));
	gWeldJoint[UR5_Index::WELDJOINT_COUPLING]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	gWeldJoint[UR5_Index::WELDJOINT_COUPLING]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);

	gLink[UR5_Index::GRIPPER].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR5_Index::GRIPPER].GetGeomInfo().SetLocalFrame(SE3());
	gLink[UR5_Index::GRIPPER].GetGeomInfo().SetFileName("../../../workspace/robot/ur5_3ds/gripper.3ds");


	gWeldJoint[UR5_Index::WELDJOINT_GRIP_MARKER]->SetActType(srJoint::PASSIVE);
	gWeldJoint[UR5_Index::WELDJOINT_GRIP_MARKER]->SetParentLink(&gLink[UR5_Index::ENDEFFECTOR]);
	gWeldJoint[UR5_Index::WELDJOINT_GRIP_MARKER]->SetChildLink(&gMarkerLink[UR5_Index::MLINK_GRIP]);
	gWeldJoint[UR5_Index::WELDJOINT_GRIP_MARKER]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	//gWeldJoint[UR5_Index::WELDJOINT_GRIP_MARKER]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(1.036, 0.0, 1.264)));
	gWeldJoint[UR5_Index::WELDJOINT_GRIP_MARKER]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(-(0.134 - 0.119 + 0.09475 + 0.0815 + 0.143), 0.0, -(0.0862 + 0.425 + 0.392 + 0.09475))));// consider offset for gripper assembly
																															//gWeldJoint[UR3_Index::WELDJOINT_GRIP_MARKER]->SetChildLinkFrame(SE3(Vec3(0.0, 0.00195, -(0.1928 + 0.2003 + 0.001))));		// consider offset for gripper assembly
#endif
#ifdef _3_Finger
	// 0.2, -0.005, 1.05 -SR_PI_HALF, SR_PI, 0.0
	gWeldJoint[UR5_Index::WELDJOINT_COUPLING]->SetParentLink(&gLink[UR5_Index::ENDEFFECTOR]);
	gWeldJoint[UR5_Index::WELDJOINT_COUPLING]->SetChildLink(&gLink[UR5_Index::GRIPPER_WRIST]);
	gWeldJoint[UR5_Index::WELDJOINT_COUPLING]->SetParentLinkFrame(SE3());
	gWeldJoint[UR5_Index::WELDJOINT_COUPLING]->SetChildLinkFrame(SE3(Vec3(-0.08, 0.0, 0.0)));
	gWeldJoint[UR5_Index::WELDJOINT_COUPLING]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);

	gLink[UR5_Index::GRIPPER_WRIST].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR5_Index::GRIPPER_WRIST].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.100, 0.055, 0.942)));
	gLink[UR5_Index::GRIPPER_WRIST].GetGeomInfo().SetFileName("../../../workspace/robot/ur5_3ds/wrist.3ds");

	gWeldJoint[UR5_Index::GRIPJOINT_W_1]->SetParentLink(&gLink[UR5_Index::GRIPPER_WRIST]);
	gWeldJoint[UR5_Index::GRIPJOINT_W_1]->SetChildLink(&gLink[UR5_Index::GRIPPER_PALM]);
	gWeldJoint[UR5_Index::GRIPJOINT_W_1]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	gWeldJoint[UR5_Index::GRIPJOINT_W_1]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(-0.04128, 0.0, 0.0)));
	gWeldJoint[UR5_Index::GRIPJOINT_W_1]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);

	gLink[UR5_Index::GRIPPER_PALM].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR5_Index::GRIPPER_PALM].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.073, 0.062, 0.932)));
	gLink[UR5_Index::GRIPPER_PALM].GetGeomInfo().SetFileName("../../../workspace/robot/ur5_3ds/palm.3ds");

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	gGripJoint[UR5_Index::GRIPJOINT_Pal_1]->SetActType(srJoint::HYBRID);
	gGripJoint[UR5_Index::GRIPJOINT_Pal_1]->SetParentLink(&gLink[UR5_Index::GRIPPER_PALM]);
	gGripJoint[UR5_Index::GRIPJOINT_Pal_1]->SetChildLink(&gLink[UR5_Index::GRIPPER_FINGER_BASE_1]);
	gGripJoint[UR5_Index::GRIPJOINT_Pal_1]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.134, 0.062, 0.9615)));
	gGripJoint[UR5_Index::GRIPJOINT_Pal_1]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.134, 0.062, 0.9615)));
	gGripJoint[UR5_Index::GRIPJOINT_Pal_1]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gGripJoint[UR5_Index::GRIPJOINT_Pal_1]->MakePositionLimit(false);

	gLink[UR5_Index::GRIPPER_FINGER_BASE_1].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	//gLink[UR5_Index::GRIPPER_FINGER_BASE_1].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gLink[UR5_Index::GRIPPER_FINGER_BASE_1].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.073, 0.062, 0.9250)));
	gLink[UR5_Index::GRIPPER_FINGER_BASE_1].GetGeomInfo().SetFileName("../../../workspace/robot/ur5_3ds/base_1.3ds");

	gGripJoint[UR5_Index::GRIPJOINT_B_1]->SetActType(srJoint::HYBRID);
	gGripJoint[UR5_Index::GRIPJOINT_B_1]->SetParentLink(&gLink[UR5_Index::GRIPPER_FINGER_BASE_1]);
	gGripJoint[UR5_Index::GRIPJOINT_B_1]->SetChildLink(&gLink[UR5_Index::GRIPPER_FINGER_PROX_1]);
	gGripJoint[UR5_Index::GRIPJOINT_B_1]->SetParentLinkFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.134, 0.062, 0.9615) + Vec3(0.022, -0.015, 0.0)));
	gGripJoint[UR5_Index::GRIPJOINT_B_1]->SetChildLinkFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.134, 0.062, 0.9615) + Vec3(0.022, -0.015, 0.0)));
	gGripJoint[UR5_Index::GRIPJOINT_B_1]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gGripJoint[UR5_Index::GRIPJOINT_B_1]->MakePositionLimit(false);

	gLink[UR5_Index::GRIPPER_FINGER_PROX_1].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	//gLink[UR5_Index::GRIPPER_FINGER_PROX_1].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gLink[UR5_Index::GRIPPER_FINGER_PROX_1].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.073, 0.123, 0.9250)));
	gLink[UR5_Index::GRIPPER_FINGER_PROX_1].GetGeomInfo().SetFileName("../../../workspace/robot/ur5_3ds/prox_1.3ds");

	gGripJoint[UR5_Index::GRIPJOINT_P_1]->SetActType(srJoint::HYBRID);
	gGripJoint[UR5_Index::GRIPJOINT_P_1]->SetParentLink(&gLink[UR5_Index::GRIPPER_FINGER_PROX_1]);
	gGripJoint[UR5_Index::GRIPJOINT_P_1]->SetChildLink(&gLink[UR5_Index::GRIPPER_FINGER_MED_1]);
	gGripJoint[UR5_Index::GRIPJOINT_P_1]->SetParentLinkFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.134, 0.062, 0.9615) + Vec3(0.022, -0.015, 0.0) + Vec3(0.0465, 0.033, 0.0)));
	gGripJoint[UR5_Index::GRIPJOINT_P_1]->SetChildLinkFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.134, 0.062, 0.9615) + Vec3(0.022, -0.015, 0.0) + Vec3(0.0465, 0.033, 0.0)));
	gGripJoint[UR5_Index::GRIPJOINT_P_1]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gGripJoint[UR5_Index::GRIPJOINT_P_1]->MakePositionLimit(false);

	gLink[UR5_Index::GRIPPER_FINGER_MED_1].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	//gLink[UR5_Index::GRIPPER_FINGER_MED_1].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gLink[UR5_Index::GRIPPER_FINGER_MED_1].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.073, 0.131, 0.9250)));
	gLink[UR5_Index::GRIPPER_FINGER_MED_1].GetGeomInfo().SetFileName("../../../workspace/robot/ur5_3ds/med_1.3ds");

	gGripJoint[UR5_Index::GRIPJOINT_M_1]->SetActType(srJoint::HYBRID);
	gGripJoint[UR5_Index::GRIPJOINT_M_1]->SetParentLink(&gLink[UR5_Index::GRIPPER_FINGER_MED_1]);
	gGripJoint[UR5_Index::GRIPJOINT_M_1]->SetChildLink(&gLink[UR5_Index::GRIPPER_FINGER_DIST_1]);
	gGripJoint[UR5_Index::GRIPJOINT_M_1]->SetParentLinkFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.134, 0.062, 0.9615) + Vec3(0.022, -0.015, 0.0) + Vec3(0.0465, 0.033, 0.0) + Vec3(0.0305, 0.0218, 0.0)));
	gGripJoint[UR5_Index::GRIPJOINT_M_1]->SetChildLinkFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.134, 0.062, 0.9615) + Vec3(0.022, -0.015, 0.0) + Vec3(0.0465, 0.033, 0.0) + Vec3(0.0305, 0.0218, 0.0)));
	gGripJoint[UR5_Index::GRIPJOINT_M_1]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gGripJoint[UR5_Index::GRIPJOINT_M_1]->MakePositionLimit(false);

	gLink[UR5_Index::GRIPPER_FINGER_DIST_1].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	//gLink[UR5_Index::GRIPPER_FINGER_DIST_1].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gLink[UR5_Index::GRIPPER_FINGER_DIST_1].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.073, 0.1205, 0.9250)));
	gLink[UR5_Index::GRIPPER_FINGER_DIST_1].GetGeomInfo().SetFileName("../../../workspace/robot/ur5_3ds/dist_1.3ds");

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	gGripJoint[UR5_Index::GRIPJOINT_Pal_2]->SetActType(srJoint::HYBRID);
	gGripJoint[UR5_Index::GRIPJOINT_Pal_2]->SetParentLink(&gLink[UR5_Index::GRIPPER_PALM]);
	gGripJoint[UR5_Index::GRIPJOINT_Pal_2]->SetChildLink(&gLink[UR5_Index::GRIPPER_FINGER_BASE_2]);
	gGripJoint[UR5_Index::GRIPJOINT_Pal_2]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.134, 0.062, 1.0345)));
	gGripJoint[UR5_Index::GRIPJOINT_Pal_2]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.134, 0.062, 1.0345)));
	gGripJoint[UR5_Index::GRIPJOINT_Pal_2]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gGripJoint[UR5_Index::GRIPJOINT_Pal_2]->MakePositionLimit(false);
	
	gLink[UR5_Index::GRIPPER_FINGER_BASE_2].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR5_Index::GRIPPER_FINGER_BASE_2].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.073, 0.062, 0.9976)));
	gLink[UR5_Index::GRIPPER_FINGER_BASE_2].GetGeomInfo().SetFileName("../../../workspace/robot/ur5_3ds/base_1.3ds");

	gGripJoint[UR5_Index::GRIPJOINT_B_2]->SetActType(srJoint::HYBRID);
	gGripJoint[UR5_Index::GRIPJOINT_B_2]->SetParentLink(&gLink[UR5_Index::GRIPPER_FINGER_BASE_2]);
	gGripJoint[UR5_Index::GRIPJOINT_B_2]->SetChildLink(&gLink[UR5_Index::GRIPPER_FINGER_PROX_2]);
	gGripJoint[UR5_Index::GRIPJOINT_B_2]->SetParentLinkFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.134, 0.062, 1.0345)+Vec3(0.022, -0.015, 0.0)));
	gGripJoint[UR5_Index::GRIPJOINT_B_2]->SetChildLinkFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.134, 0.062, 1.0345)+Vec3(0.022, -0.015, 0.0)));
	gGripJoint[UR5_Index::GRIPJOINT_B_2]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gGripJoint[UR5_Index::GRIPJOINT_B_2]->MakePositionLimit(false);

	gLink[UR5_Index::GRIPPER_FINGER_PROX_2].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR5_Index::GRIPPER_FINGER_PROX_2].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.073, 0.123, 0.9976)));
	gLink[UR5_Index::GRIPPER_FINGER_PROX_2].GetGeomInfo().SetFileName("../../../workspace/robot/ur5_3ds/prox_1.3ds");

	gGripJoint[UR5_Index::GRIPJOINT_P_2]->SetActType(srJoint::HYBRID);
	gGripJoint[UR5_Index::GRIPJOINT_P_2]->SetParentLink(&gLink[UR5_Index::GRIPPER_FINGER_PROX_2]);
	gGripJoint[UR5_Index::GRIPJOINT_P_2]->SetChildLink(&gLink[UR5_Index::GRIPPER_FINGER_MED_2]);
	gGripJoint[UR5_Index::GRIPJOINT_P_2]->SetParentLinkFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.134, 0.062, 1.0345) + Vec3(0.022, -0.015, 0.0) + Vec3(0.0465, 0.033, 0.0)));
	gGripJoint[UR5_Index::GRIPJOINT_P_2]->SetChildLinkFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.134, 0.062, 1.0345) + Vec3(0.022, -0.015, 0.0) + Vec3(0.0465, 0.033, 0.0)));
	gGripJoint[UR5_Index::GRIPJOINT_P_2]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gGripJoint[UR5_Index::GRIPJOINT_P_2]->MakePositionLimit(false);

	gLink[UR5_Index::GRIPPER_FINGER_MED_2].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR5_Index::GRIPPER_FINGER_MED_2].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.073, 0.131, 0.9976)));
	gLink[UR5_Index::GRIPPER_FINGER_MED_2].GetGeomInfo().SetFileName("../../../workspace/robot/ur5_3ds/med_1.3ds");

	gGripJoint[UR5_Index::GRIPJOINT_M_2]->SetActType(srJoint::HYBRID);
	gGripJoint[UR5_Index::GRIPJOINT_M_2]->SetParentLink(&gLink[UR5_Index::GRIPPER_FINGER_MED_2]);
	gGripJoint[UR5_Index::GRIPJOINT_M_2]->SetChildLink(&gLink[UR5_Index::GRIPPER_FINGER_DIST_2]);
	gGripJoint[UR5_Index::GRIPJOINT_M_2]->SetParentLinkFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.134, 0.062, 1.0345) + Vec3(0.022, -0.015, 0.0) + Vec3(0.0465, 0.033, 0.0) + Vec3(0.0305, 0.0218, 0.0)));
	gGripJoint[UR5_Index::GRIPJOINT_M_2]->SetChildLinkFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.134, 0.062, 1.0345) + Vec3(0.022, -0.015, 0.0) + Vec3(0.0465, 0.033, 0.0) + Vec3(0.0305, 0.0218, 0.0)));
	gGripJoint[UR5_Index::GRIPJOINT_M_2]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gGripJoint[UR5_Index::GRIPJOINT_M_2]->MakePositionLimit(false);

	gLink[UR5_Index::GRIPPER_FINGER_DIST_2].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR5_Index::GRIPPER_FINGER_DIST_2].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.073, 0.1205, 0.9976)));
	gLink[UR5_Index::GRIPPER_FINGER_DIST_2].GetGeomInfo().SetFileName("../../../workspace/robot/ur5_3ds/dist_1.3ds");

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	gGripJoint[UR5_Index::GRIPJOINT_Pal_3]->SetActType(srJoint::HYBRID);
	gGripJoint[UR5_Index::GRIPJOINT_Pal_3]->SetParentLink(&gLink[UR5_Index::GRIPPER_PALM]);
	gGripJoint[UR5_Index::GRIPJOINT_Pal_3]->SetChildLink(&gLink[UR5_Index::GRIPPER_FINGER_BASE_3]);
	gGripJoint[UR5_Index::GRIPJOINT_Pal_3]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.134, -0.062, (0.9615+1.0345)/2)));
	gGripJoint[UR5_Index::GRIPJOINT_Pal_3]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.134, -0.062, 1.0345)));
	gGripJoint[UR5_Index::GRIPJOINT_Pal_3]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gGripJoint[UR5_Index::GRIPJOINT_Pal_3]->MakePositionLimit(false);

	gLink[UR5_Index::GRIPPER_FINGER_BASE_3].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR5_Index::GRIPPER_FINGER_BASE_3].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.073, -0.0, 1.0176)));
	gLink[UR5_Index::GRIPPER_FINGER_BASE_3].GetGeomInfo().SetFileName("../../../workspace/robot/ur5_3ds/base_3.3ds");

	gGripJoint[UR5_Index::GRIPJOINT_B_3]->SetActType(srJoint::HYBRID);
	gGripJoint[UR5_Index::GRIPJOINT_B_3]->SetParentLink(&gLink[UR5_Index::GRIPPER_FINGER_BASE_3]);
	gGripJoint[UR5_Index::GRIPJOINT_B_3]->SetChildLink(&gLink[UR5_Index::GRIPPER_FINGER_PROX_3]);
	gGripJoint[UR5_Index::GRIPJOINT_B_3]->SetParentLinkFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.134, -0.062, 1.0345) + Vec3(0.022, 0.015, 0.0)));
	gGripJoint[UR5_Index::GRIPJOINT_B_3]->SetChildLinkFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.134, -0.062, 1.0345) + Vec3(0.022, 0.015, 0.0)));
	gGripJoint[UR5_Index::GRIPJOINT_B_3]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gGripJoint[UR5_Index::GRIPJOINT_B_3]->MakePositionLimit(false);

	gLink[UR5_Index::GRIPPER_FINGER_PROX_3].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR5_Index::GRIPPER_FINGER_PROX_3].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.073, -0.0, 1.0191)));
	gLink[UR5_Index::GRIPPER_FINGER_PROX_3].GetGeomInfo().SetFileName("../../../workspace/robot/ur5_3ds/prox_3.3ds");

	gGripJoint[UR5_Index::GRIPJOINT_P_3]->SetActType(srJoint::HYBRID);
	gGripJoint[UR5_Index::GRIPJOINT_P_3]->SetParentLink(&gLink[UR5_Index::GRIPPER_FINGER_PROX_3]);
	gGripJoint[UR5_Index::GRIPJOINT_P_3]->SetChildLink(&gLink[UR5_Index::GRIPPER_FINGER_MED_3]);
	gGripJoint[UR5_Index::GRIPJOINT_P_3]->SetParentLinkFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.134, -0.062, 1.0345) + Vec3(0.022, 0.015, 0.0) + Vec3(0.0465, -0.033, 0.0)));
	gGripJoint[UR5_Index::GRIPJOINT_P_3]->SetChildLinkFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.134, -0.062, 1.0345) + Vec3(0.022, 0.015, 0.0) + Vec3(0.0465, -0.033, 0.0)));
	gGripJoint[UR5_Index::GRIPJOINT_P_3]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gGripJoint[UR5_Index::GRIPJOINT_P_3]->MakePositionLimit(false);

	gLink[UR5_Index::GRIPPER_FINGER_MED_3].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR5_Index::GRIPPER_FINGER_MED_3].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.073, -0.0, 1.0191)));
	gLink[UR5_Index::GRIPPER_FINGER_MED_3].GetGeomInfo().SetFileName("../../../workspace/robot/ur5_3ds/med_3.3ds");

	gGripJoint[UR5_Index::GRIPJOINT_M_3]->SetActType(srJoint::HYBRID);
	gGripJoint[UR5_Index::GRIPJOINT_M_3]->SetParentLink(&gLink[UR5_Index::GRIPPER_FINGER_MED_3]);
	gGripJoint[UR5_Index::GRIPJOINT_M_3]->SetChildLink(&gLink[UR5_Index::GRIPPER_FINGER_DIST_3]);
	gGripJoint[UR5_Index::GRIPJOINT_M_3]->SetParentLinkFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.134, -0.062, 1.0345) + Vec3(0.022, 0.015, 0.0) + Vec3(0.0465, -0.033, 0.0) + Vec3(0.0305, -0.0218, 0.0)));
	gGripJoint[UR5_Index::GRIPJOINT_M_3]->SetChildLinkFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.134, -0.062, 1.0345) + Vec3(0.022, 0.015, 0.0) + Vec3(0.0465, -0.033, 0.0) + Vec3(0.0305, -0.0218, 0.0)));
	gGripJoint[UR5_Index::GRIPJOINT_M_3]->GetGeomInfo().SetDimension(0.0, 0.0, 0.0);
	gGripJoint[UR5_Index::GRIPJOINT_M_3]->MakePositionLimit(false);

	gLink[UR5_Index::GRIPPER_FINGER_DIST_3].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[UR5_Index::GRIPPER_FINGER_DIST_3].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.073, -0.0, 1.0191)));
	gLink[UR5_Index::GRIPPER_FINGER_DIST_3].GetGeomInfo().SetFileName("../../../workspace/robot/ur5_3ds/dist_3.3ds");



#endif
																															// marker links
	gMarkerLink[UR5_Index::MLINK_GRIP].GetGeomInfo().SetDimension(Vec3(0.00, 0.00, 0.00));
	gMarkerLink[UR5_Index::MLINK_GRIP].GetGeomInfo().SetColor(0.1f, 0.1f, 0.1f);
	gMarkerLink[UR5_Index::MLINK_GRIP].SetInertia(Inertia(0.001));
	

	gWeldJoint[UR5_Index::WELDJOINT_GRIP_MARKER]->SetActType(srJoint::PASSIVE);
	gWeldJoint[UR5_Index::WELDJOINT_GRIP_MARKER]->SetParentLink(&gLink[UR5_Index::ENDEFFECTOR]);
	gWeldJoint[UR5_Index::WELDJOINT_GRIP_MARKER]->SetChildLink(&gMarkerLink[UR5_Index::MLINK_GRIP]);
	gWeldJoint[UR5_Index::WELDJOINT_GRIP_MARKER]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	//gWeldJoint[UR5_Index::WELDJOINT_GRIP_MARKER]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(1.036, 0.0, 1.264)));
	gWeldJoint[UR5_Index::WELDJOINT_GRIP_MARKER]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(-(0.134 - 0.119 + 0.09475 + 0.0815 + 0.143), 0.0, -(0.0862 + 0.425 + 0.392 + 0.09475))));

	this->SetBaseLink(&gLink[UR5_Index::LINK_1]);
	this->SetBaseLinkType(srSystem::FIXED);
}

void UR5Robot::AssembleCollision()
{
	m_numCollision = 0;

	int numBox = 5;
	double thickness = 0.03;
	double space = 0.005;
	vector<pair<Vec3, SE3>> boxSet;	

	gLink[UR5_Index::LINK_1].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.16, 0.16, 0.02));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, 0.0, 0.002)));


	gLink[UR5_Index::LINK_2].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.15, 0.16, 0.15));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-0.005, 0.0, 0.09)));


	gLink[UR5_Index::LINK_3].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.14, 0.13, 0.56));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3((0.14 + 0.14 + 0.01) / 2.0, 0.0, 0.56 / 2.0 + 0.02)));

	
	gLink[UR5_Index::LINK_4].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.114, 0.12, 0.5));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3((0.14 - 0.114) / 2.0, 0.0, (0.0862 + 0.425) + 0.37 / 2.0)));


	gLink[UR5_Index::LINK_5].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.08, 0.085, 0.11));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3((0.14 + 0.08 + 0.01) / 2.0, 0.0, (0.0862 + 0.425 + 0.392) - 0.005)));


	gLink[UR5_Index::LINK_6].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.11, 0.085, 0.08));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3((0.14 + 0.06) / 2.0, 0.0, (0.0862 + 0.425 + 0.392 + 0.09475))));

	
	gLink[UR5_Index::ENDEFFECTOR].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.08, 0.02, 0.0));
	gCollision[m_numCollision++].SetLocalFrame(EulerZYX(Vec3(0.0, SR_PI_HALF, 0.0), Vec3(0.09 + (0.14 + 0.02) / 2.0, 0.0, (0.0862 + 0.425 + 0.392 + 0.09475))));

#ifndef _3_Finger
	gLink[UR5_Index::GRIPPER].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.085, 0.12, 0.0));
	//gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-1.019, 1.269, 0.0087)));
	gCollision[m_numCollision++].SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(-(0.0862 + 0.425 + 0.392 + 0.09475), 0.165 + (0.14 + 0.02) / 2.0, 0.0)));

	gLink[UR5_Index::GRIPPER].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.03, 0.1, 0.028));
	//gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-1.019, 1.269, 0.0087)));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-(0.0862 + 0.425 + 0.392 + 0.09475) + (0.085 + 0.026 + 0.01) / 2.0, 0.205 + (0.14 + 0.02) / 2.0, 0.0)));

	gLink[UR5_Index::GRIPPER].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.03, 0.1, 0.028));
	//gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-1.019, 1.269, 0.0087)));
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-(0.0862 + 0.425 + 0.392 + 0.09475) - (0.085 + 0.026 + 0.01) / 2.0, 0.205 + (0.14 + 0.02) / 2.0, 0.0)));
#else
	gLink[UR5_Index::GRIPPER_PALM].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.11, 0.14, 0.16));
	gCollision[m_numCollision++].SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.123, 0.0, 1.002)));

	gLink[UR5_Index::GRIPPER_FINGER_DIST_1].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.03, 0.02, 0.035));
	gCollision[m_numCollision++].SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.253, 0.1025, 0.9620)));

	gLink[UR5_Index::GRIPPER_FINGER_DIST_2].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.03, 0.02, 0.035));
	gCollision[m_numCollision++].SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.253, 0.1025, 0.9620 + 0.0726)));

	gLink[UR5_Index::GRIPPER_FINGER_DIST_3].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.03, 0.02, 0.035));
	gCollision[m_numCollision++].SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.253, -0.1005 , 0.9620 + 0.0726)));
#endif

	
}

void UR5Robot::SetInertia()
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
void UR5Robot::SetTorqueLimit()
{
	UpperTorqueLimit[UR5_Index::JOINT_1] = DEG2RAD(300);
	UpperTorqueLimit[UR5_Index::JOINT_2] = DEG2RAD(300);
	UpperTorqueLimit[UR5_Index::JOINT_3] = DEG2RAD(300);
	UpperTorqueLimit[UR5_Index::JOINT_4] = DEG2RAD(300);
	UpperTorqueLimit[UR5_Index::JOINT_5] = DEG2RAD(300);
	UpperTorqueLimit[UR5_Index::JOINT_6] = DEG2RAD(300);


	LowerTorqueLimit[UR5_Index::JOINT_1] = DEG2RAD(-300);
	LowerTorqueLimit[UR5_Index::JOINT_2] = DEG2RAD(-300);
	LowerTorqueLimit[UR5_Index::JOINT_3] = DEG2RAD(-300);
	LowerTorqueLimit[UR5_Index::JOINT_4] = DEG2RAD(-300);
	LowerTorqueLimit[UR5_Index::JOINT_5] = DEG2RAD(-300);
	LowerTorqueLimit[UR5_Index::JOINT_6] = DEG2RAD(-300);

	for (int i = 0; i < DEGREE_OF_FREEDOM_UR5_JOINT; i++)
		gJoint[i]->SetTorqueLimit(LowerTorqueLimit[i], UpperTorqueLimit[i]);
}

Eigen::VectorXd UR5Robot::getLowerJointLimit() const
{
	Eigen::VectorXd llim(DEGREE_OF_FREEDOM_UR5_JOINT);
	for (int i = 0; i < DEGREE_OF_FREEDOM_UR5_JOINT; i++)
		llim[i] =  DEG2RAD(LowerJointLimit[i]);
	return llim;
}

Eigen::VectorXd UR5Robot::getUpperJointLimit() const
{
	Eigen::VectorXd ulim(DEGREE_OF_FREEDOM_UR5_JOINT);
	for (int i = 0; i < DEGREE_OF_FREEDOM_UR5_JOINT; i++)
		ulim[i] = DEG2RAD(UpperJointLimit[i]);
	return ulim;
}
