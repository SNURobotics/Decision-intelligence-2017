#include "SDA20DRobot.h"
#include "makeSpecialCol.h"
#include "common/utils.h"

#define GRAYCOLORSDA20D		0.15f, 0.15f, 0.15f, 1.0f
#define SHOW_GRIPPER
SDA20D::SDA20D()
{

	for (int i = 0; i < NUM_OF_RJOINT_SDA20D; i++)
		gJoint[i] = new srRevoluteJoint;
	for (int i = 0; i < 4; i++)
		gWjoint[i] = new srWeldJoint;
	for (int i = 0; i < NUM_OF_PJOINT_SDA20D; i++)
		gPjoint[i] = new srPrismaticJoint;
	AssembleModel();
	AssembleCollision();
	SetInitialConfiguration();
	SetJointLimit();
	TsrLinkbase2robotbase = SE3();
	homePos = Eigen::VectorXd::Zero(NUM_OF_RJOINT_SDA20D);
	qInvKinInit = Eigen::VectorXd::Zero(NUM_OF_RJOINT_SDA20D);
	qInvKinInit[1] = 1.3*SR_PI_HALF;
	qInvKinInit[8] = 1.3*SR_PI_HALF;
	qInvKinInit[2] = 0.5*SR_PI_HALF;
	qInvKinInit[9] = 0.5*SR_PI_HALF;
	qInvKinInit[3] = -SR_PI_HALF;
	qInvKinInit[10] = -SR_PI_HALF;
	qInvKinInit[4] = -SR_PI_HALF;
	qInvKinInit[11] = -SR_PI_HALF;
	this->SetSelfCollision(true);
}
SDA20D::~SDA20D()
{
	for (int i = 0; i<NUM_OF_RJOINT_SDA20D; i++)
		SR_SAFE_DELETE(gJoint[i]);
	for (int i = 0; i < 2; i++)
		SR_SAFE_DELETE(gWjoint[i]);
	for (int i = 0; i < NUM_OF_LINK_SDA20D; i++)
		delete &gLink[i];

}

void SDA20D::AssembleModel()
{

	// default color
	for (int i = 0; i < NUM_OF_LINK_SDA20D; i++)
		gLink[i].GetGeomInfo().SetColor(0.3f, 0.3f, 0.3f, 1.0f);

	SE3 baseSE3 = EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.3));		//0.1, 0.3

	// simulator global frame to robot hw global frame
	TsrLinkbase2robotbase = baseSE3 * SE3(Vec3(0.15, 0.0, 0.45));

	// BODY
	gLink[SDA20D_Index::LINK_BASE].SetFrame(baseSE3);    

	gLink[SDA20D_Index::LINK_BASE].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[SDA20D_Index::LINK_BASE].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, -0.1)));
	gLink[SDA20D_Index::LINK_BASE].GetGeomInfo().SetFileName("../../../workspace/robot/sda20d_3ds/LINK_BASE.3ds");

	gJoint[SDA20D_Index::JOINT_WAIST]->SetActType(srJoint::HYBRID);
	gJoint[SDA20D_Index::JOINT_WAIST]->SetParentLink(&gLink[SDA20D_Index::LINK_BASE]);
	gJoint[SDA20D_Index::JOINT_WAIST]->SetChildLink(&gLink[SDA20D_Index::LINK_UPPERBASE]);
	gJoint[SDA20D_Index::JOINT_WAIST]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.1)));
	gJoint[SDA20D_Index::JOINT_WAIST]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, -0.245)));
	gJoint[SDA20D_Index::JOINT_WAIST]->MakePositionLimit(false);

	gLink[SDA20D_Index::LINK_UPPERBASE].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[SDA20D_Index::LINK_UPPERBASE].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, -0.2 - 0.245)));
	gLink[SDA20D_Index::LINK_UPPERBASE].GetGeomInfo().SetFileName("../../../workspace/robot/sda20d_3ds/LINK_UPPERBASE.3ds");

	// RIGHT
	gJoint[SDA20D_Index::JOINT_RIGHT_S]->SetActType(srJoint::HYBRID);
	gJoint[SDA20D_Index::JOINT_RIGHT_S]->SetParentLink(&gLink[SDA20D_Index::LINK_UPPERBASE]);
	gJoint[SDA20D_Index::JOINT_RIGHT_S]->SetChildLink(&gLink[SDA20D_Index::LINK_RIGHT_SHOULDER]);
	gJoint[SDA20D_Index::JOINT_RIGHT_S]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.15, -0.21, 0.55 - 0.2 - 0.245)));
	gJoint[SDA20D_Index::JOINT_RIGHT_S]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.0, (0.385 - 0.21) / 2, 0.0)));
	gJoint[SDA20D_Index::JOINT_RIGHT_S]->MakePositionLimit(false);

	gLink[SDA20D_Index::LINK_RIGHT_SHOULDER].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[SDA20D_Index::LINK_RIGHT_SHOULDER].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(-0.15, 0.385 - 0.093, -0.55)));
	gLink[SDA20D_Index::LINK_RIGHT_SHOULDER].GetGeomInfo().SetFileName("../../../workspace/robot/sda20d_3ds/LINK_RIGHT_SHOULDER1.3ds");
	gLink[SDA20D_Index::LINK_RIGHT_SHOULDER].GetGeomInfo().SetColor(GRAYCOLORSDA20D);

	gJoint[SDA20D_Index::JOINT_RIGHT_L]->SetActType(srJoint::HYBRID);
	gJoint[SDA20D_Index::JOINT_RIGHT_L]->SetParentLink(&gLink[SDA20D_Index::LINK_RIGHT_SHOULDER]);
	gJoint[SDA20D_Index::JOINT_RIGHT_L]->SetChildLink(&gLink[SDA20D_Index::LINK_RIGHT_UPPERARM]);
	gJoint[SDA20D_Index::JOINT_RIGHT_L]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, -(0.385 - 0.21) / 2, 0.0)));
	gJoint[SDA20D_Index::JOINT_RIGHT_L]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	gJoint[SDA20D_Index::JOINT_RIGHT_L]->MakePositionLimit(false);


	gLink[SDA20D_Index::LINK_RIGHT_UPPERARM].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[SDA20D_Index::LINK_RIGHT_UPPERARM].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.385, 0.15, -0.55)));
	gLink[SDA20D_Index::LINK_RIGHT_UPPERARM].GetGeomInfo().SetFileName("../../../workspace/robot/sda20d_3ds/LINK_RIGHT_UPPERARM1.3ds");

	gJoint[SDA20D_Index::JOINT_RIGHT_E]->SetActType(srJoint::HYBRID);
	gJoint[SDA20D_Index::JOINT_RIGHT_E]->SetParentLink(&gLink[SDA20D_Index::LINK_RIGHT_UPPERARM]);
	gJoint[SDA20D_Index::JOINT_RIGHT_E]->SetChildLink(&gLink[SDA20D_Index::LINK_RIGHT_LOWERARM]);
	gJoint[SDA20D_Index::JOINT_RIGHT_E]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.0, -0.49 / 2, 0.0)));
	gJoint[SDA20D_Index::JOINT_RIGHT_E]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.0, 0.0, 0.0)));
	gJoint[SDA20D_Index::JOINT_RIGHT_E]->MakePositionLimit(false);

	gLink[SDA20D_Index::LINK_RIGHT_LOWERARM].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[SDA20D_Index::LINK_RIGHT_LOWERARM].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.385, 0.49/2 + 0.15, -0.55)));
	gLink[SDA20D_Index::LINK_RIGHT_LOWERARM].GetGeomInfo().SetFileName("../../../workspace/robot/sda20d_3ds/LINK_RIGHT_LOWERARM.3ds");
	gLink[SDA20D_Index::LINK_RIGHT_LOWERARM].GetGeomInfo().SetColor(GRAYCOLORSDA20D);

	gJoint[SDA20D_Index::JOINT_RIGHT_U]->SetActType(srJoint::HYBRID);
	gJoint[SDA20D_Index::JOINT_RIGHT_U]->SetParentLink(&gLink[SDA20D_Index::LINK_RIGHT_LOWERARM]);
	gJoint[SDA20D_Index::JOINT_RIGHT_U]->SetChildLink(&gLink[SDA20D_Index::LINK_RIGHT_ELBOW]);
	gJoint[SDA20D_Index::JOINT_RIGHT_U]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI), Vec3(0.0, -0.49 / 2, 0.0)));
	gJoint[SDA20D_Index::JOINT_RIGHT_U]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI), Vec3(0.0, 0.0, 0.0)));
	gJoint[SDA20D_Index::JOINT_RIGHT_U]->MakePositionLimit(false);

	gLink[SDA20D_Index::LINK_RIGHT_ELBOW].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[SDA20D_Index::LINK_RIGHT_ELBOW].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.385, 0.49 + 0.15, -0.55)));
	gLink[SDA20D_Index::LINK_RIGHT_ELBOW].GetGeomInfo().SetFileName("../../../workspace/robot/sda20d_3ds/LINK_RIGHT_ELBOW.3ds");

	gJoint[SDA20D_Index::JOINT_RIGHT_R]->SetActType(srJoint::HYBRID);
	gJoint[SDA20D_Index::JOINT_RIGHT_R]->SetParentLink(&gLink[SDA20D_Index::LINK_RIGHT_ELBOW]);
	gJoint[SDA20D_Index::JOINT_RIGHT_R]->SetChildLink(&gLink[SDA20D_Index::LINK_RIGHT_WRIST1]);
	gJoint[SDA20D_Index::JOINT_RIGHT_R]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, -0.42 / 2, 0.0)));
	gJoint[SDA20D_Index::JOINT_RIGHT_R]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0, 0.0)));
	gJoint[SDA20D_Index::JOINT_RIGHT_R]->MakePositionLimit(false);

	gLink[SDA20D_Index::LINK_RIGHT_WRIST1].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[SDA20D_Index::LINK_RIGHT_WRIST1].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.385, 0.49 + 0.42/2 + 0.15, -0.55)));
	gLink[SDA20D_Index::LINK_RIGHT_WRIST1].GetGeomInfo().SetFileName("../../../workspace/robot/sda20d_3ds/LINK_RIGHT_WRIST1.3ds");

	gJoint[SDA20D_Index::JOINT_RIGHT_B]->SetActType(srJoint::HYBRID);
	gJoint[SDA20D_Index::JOINT_RIGHT_B]->SetParentLink(&gLink[SDA20D_Index::LINK_RIGHT_WRIST1]);
	gJoint[SDA20D_Index::JOINT_RIGHT_B]->SetChildLink(&gLink[SDA20D_Index::LINK_RIGHT_WRIST2]);
	gJoint[SDA20D_Index::JOINT_RIGHT_B]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI), Vec3(0.0, -0.42 / 2, 0.0)));
	gJoint[SDA20D_Index::JOINT_RIGHT_B]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI), Vec3(0.0, 0.0, 0.0)));
	gJoint[SDA20D_Index::JOINT_RIGHT_B]->MakePositionLimit(false);

	gLink[SDA20D_Index::LINK_RIGHT_WRIST2].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[SDA20D_Index::LINK_RIGHT_WRIST2].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.385, 0.49 + 0.42 + 0.15, -0.55)));
	gLink[SDA20D_Index::LINK_RIGHT_WRIST2].GetGeomInfo().SetFileName("../../../workspace/robot/sda20d_3ds/LINK_RIGHT_WRIST2.3ds");
	gLink[SDA20D_Index::LINK_RIGHT_WRIST2].GetGeomInfo().SetColor(GRAYCOLORSDA20D);

	gJoint[SDA20D_Index::JOINT_RIGHT_T]->SetActType(srJoint::HYBRID);
	gJoint[SDA20D_Index::JOINT_RIGHT_T]->SetParentLink(&gLink[SDA20D_Index::LINK_RIGHT_WRIST2]);
	gJoint[SDA20D_Index::JOINT_RIGHT_T]->SetChildLink(&gLink[SDA20D_Index::LINK_RIGHT_GRIPPER]);
	gJoint[SDA20D_Index::JOINT_RIGHT_T]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, -0.18, 0.0)));
	gJoint[SDA20D_Index::JOINT_RIGHT_T]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0, 0.0)));
	gJoint[SDA20D_Index::JOINT_RIGHT_T]->MakePositionLimit(false);


	gLink[SDA20D_Index::LINK_RIGHT_GRIPPER].GetGeomInfo().SetDimension(0.001);

#ifdef SHOW_GRIPPER
	gLink[SDA20D_Index::LINK_RIGHT_GRIPPER].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[SDA20D_Index::LINK_RIGHT_GRIPPER].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(-SR_PI_HALF, SR_PI_HALF, 0.0), Vec3(0.0595, -0.1735, 0.0)));
	gLink[SDA20D_Index::LINK_RIGHT_GRIPPER].GetGeomInfo().SetFileName("../../../workspace/robot/sda20d_3ds/LINK_RIGHT_GRIPPER2_b.3ds");


	gPjoint[SDA20D_Index::JOINT_RIGHT_GRIPPER_L]->SetActType(srJoint::HYBRID);
	gPjoint[SDA20D_Index::JOINT_RIGHT_GRIPPER_L]->SetParentLink(&gLink[SDA20D_Index::LINK_RIGHT_GRIPPER]);
	gPjoint[SDA20D_Index::JOINT_RIGHT_GRIPPER_L]->SetChildLink(&gLink[SDA20D_Index::LINK_RIGHT_GRIPPER_L]);
	gPjoint[SDA20D_Index::JOINT_RIGHT_GRIPPER_L]->SetParentLinkFrame(EulerZYX(Vec3(SR_PI, -SR_PI_HALF, 0.0), Vec3(0.0, -0.1805, 0.0)));
	gPjoint[SDA20D_Index::JOINT_RIGHT_GRIPPER_L]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0595, 0.0)));
	gPjoint[SDA20D_Index::JOINT_RIGHT_GRIPPER_L]->MakePositionLimit(false);

	gLink[SDA20D_Index::LINK_RIGHT_GRIPPER_L].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[SDA20D_Index::LINK_RIGHT_GRIPPER_L].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	gLink[SDA20D_Index::LINK_RIGHT_GRIPPER_L].GetGeomInfo().SetFileName("../../../workspace/robot/sda20d_3ds/LINK_RIGHT_GRIPPER2_finger_l.3ds");
	gLink[SDA20D_Index::LINK_RIGHT_GRIPPER_L].GetGeomInfo().SetColor(GRAYCOLORSDA20D);

	gPjoint[SDA20D_Index::JOINT_RIGHT_GRIPPER_U]->SetActType(srJoint::HYBRID);
	gPjoint[SDA20D_Index::JOINT_RIGHT_GRIPPER_U]->SetParentLink(&gLink[SDA20D_Index::LINK_RIGHT_GRIPPER]);
	gPjoint[SDA20D_Index::JOINT_RIGHT_GRIPPER_U]->SetChildLink(&gLink[SDA20D_Index::LINK_RIGHT_GRIPPER_U]);
	gPjoint[SDA20D_Index::JOINT_RIGHT_GRIPPER_U]->SetParentLinkFrame(EulerZYX(Vec3(SR_PI, -SR_PI_HALF, 0.0), Vec3(0.0, -0.1805, 0.0)));
	gPjoint[SDA20D_Index::JOINT_RIGHT_GRIPPER_U]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, -0.0595, 0.0)));
	gPjoint[SDA20D_Index::JOINT_RIGHT_GRIPPER_U]->MakePositionLimit(false);

	gLink[SDA20D_Index::LINK_RIGHT_GRIPPER_U].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[SDA20D_Index::LINK_RIGHT_GRIPPER_U].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	gLink[SDA20D_Index::LINK_RIGHT_GRIPPER_U].GetGeomInfo().SetFileName("../../../workspace/robot/sda20d_3ds/LINK_RIGHT_GRIPPER2_finger_u.3ds");
	gLink[SDA20D_Index::LINK_RIGHT_GRIPPER_U].GetGeomInfo().SetColor(GRAYCOLORSDA20D);
#endif // SHOW_GRIPPER
	
	// LEFT ARM
	gJoint[SDA20D_Index::JOINT_LEFT_S]->SetActType(srJoint::HYBRID);
	gJoint[SDA20D_Index::JOINT_LEFT_S]->SetParentLink(&gLink[SDA20D_Index::LINK_UPPERBASE]);
	gJoint[SDA20D_Index::JOINT_LEFT_S]->SetChildLink(&gLink[SDA20D_Index::LINK_LEFT_SHOULDER]);
	gJoint[SDA20D_Index::JOINT_LEFT_S]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.15, 0.21, 0.55 - 0.2 - 0.245)));
	gJoint[SDA20D_Index::JOINT_LEFT_S]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.0, -(0.385 - 0.21) / 2, 0.0)));
	gJoint[SDA20D_Index::JOINT_LEFT_S]->MakePositionLimit(false);


	gLink[SDA20D_Index::LINK_LEFT_SHOULDER].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[SDA20D_Index::LINK_LEFT_SHOULDER].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(-0.15, -0.385 + 0.093, -0.55)));
	gLink[SDA20D_Index::LINK_LEFT_SHOULDER].GetGeomInfo().SetFileName("../../../workspace/robot/sda20d_3ds/LINK_LEFT_SHOULDER.3ds");
	gLink[SDA20D_Index::LINK_LEFT_SHOULDER].GetGeomInfo().SetColor(GRAYCOLORSDA20D);


	gJoint[SDA20D_Index::JOINT_LEFT_L]->SetActType(srJoint::HYBRID);
	gJoint[SDA20D_Index::JOINT_LEFT_L]->SetParentLink(&gLink[SDA20D_Index::LINK_LEFT_SHOULDER]);
	gJoint[SDA20D_Index::JOINT_LEFT_L]->SetChildLink(&gLink[SDA20D_Index::LINK_LEFT_UPPERARM]);
	gJoint[SDA20D_Index::JOINT_LEFT_L]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI), Vec3(0.0, (0.385 - 0.21) / 2, 0.0)));
	gJoint[SDA20D_Index::JOINT_LEFT_L]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI), Vec3(0.0, 0.0, 0.0)));
	gJoint[SDA20D_Index::JOINT_LEFT_L]->MakePositionLimit(false);

	gLink[SDA20D_Index::LINK_LEFT_UPPERARM].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[SDA20D_Index::LINK_LEFT_UPPERARM].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.385, -0.15, -0.55)));
	gLink[SDA20D_Index::LINK_LEFT_UPPERARM].GetGeomInfo().SetFileName("../../../workspace/robot/sda20d_3ds/LINK_LEFT_UPPERARM.3ds");

	gJoint[SDA20D_Index::JOINT_LEFT_E]->SetActType(srJoint::HYBRID);
	gJoint[SDA20D_Index::JOINT_LEFT_E]->SetParentLink(&gLink[SDA20D_Index::LINK_LEFT_UPPERARM]);
	gJoint[SDA20D_Index::JOINT_LEFT_E]->SetChildLink(&gLink[SDA20D_Index::LINK_LEFT_LOWERARM]);
	gJoint[SDA20D_Index::JOINT_LEFT_E]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.0, 0.49 / 2 , 0.0)));
	gJoint[SDA20D_Index::JOINT_LEFT_E]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI_HALF), Vec3(0.0, 0.0, 0.0)));
	gJoint[SDA20D_Index::JOINT_LEFT_E]->MakePositionLimit(false);

	gLink[SDA20D_Index::LINK_LEFT_LOWERARM].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[SDA20D_Index::LINK_LEFT_LOWERARM].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.385, -(0.49 / 2 + 0.15), -0.55)));
	gLink[SDA20D_Index::LINK_LEFT_LOWERARM].GetGeomInfo().SetFileName("../../../workspace/robot/sda20d_3ds/LINK_LEFT_LOWERARM.3ds");
	gLink[SDA20D_Index::LINK_LEFT_LOWERARM].GetGeomInfo().SetColor(GRAYCOLORSDA20D);

	gJoint[SDA20D_Index::JOINT_LEFT_U]->SetActType(srJoint::HYBRID);
	gJoint[SDA20D_Index::JOINT_LEFT_U]->SetParentLink(&gLink[SDA20D_Index::LINK_LEFT_LOWERARM]);
	gJoint[SDA20D_Index::JOINT_LEFT_U]->SetChildLink(&gLink[SDA20D_Index::LINK_LEFT_ELBOW]);
	gJoint[SDA20D_Index::JOINT_LEFT_U]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.49 / 2, 0.0)));
	gJoint[SDA20D_Index::JOINT_LEFT_U]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	gJoint[SDA20D_Index::JOINT_LEFT_U]->MakePositionLimit(false);

	gLink[SDA20D_Index::LINK_LEFT_ELBOW].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[SDA20D_Index::LINK_LEFT_ELBOW].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.385, -(0.49 + 0.15), -0.55)));
	gLink[SDA20D_Index::LINK_LEFT_ELBOW].GetGeomInfo().SetFileName("../../../workspace/robot/sda20d_3ds/LINK_LEFT_ELBOW.3ds");

	gJoint[SDA20D_Index::JOINT_LEFT_R]->SetActType(srJoint::HYBRID);
	gJoint[SDA20D_Index::JOINT_LEFT_R]->SetParentLink(&gLink[SDA20D_Index::LINK_LEFT_ELBOW]);
	gJoint[SDA20D_Index::JOINT_LEFT_R]->SetChildLink(&gLink[SDA20D_Index::LINK_LEFT_WRIST1]);
	gJoint[SDA20D_Index::JOINT_LEFT_R]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.42 / 2, 0.0)));
	gJoint[SDA20D_Index::JOINT_LEFT_R]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0, 0.0)));
	gJoint[SDA20D_Index::JOINT_LEFT_R]->MakePositionLimit(false);

	gLink[SDA20D_Index::LINK_LEFT_WRIST1].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[SDA20D_Index::LINK_LEFT_WRIST1].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.385, -(0.49 + 0.42 / 2 + 0.15), -0.55)));
	gLink[SDA20D_Index::LINK_LEFT_WRIST1].GetGeomInfo().SetFileName("../../../workspace/robot/sda20d_3ds/LINK_LEFT_WRIST1.3ds");


	gJoint[SDA20D_Index::JOINT_LEFT_B]->SetActType(srJoint::HYBRID);
	gJoint[SDA20D_Index::JOINT_LEFT_B]->SetParentLink(&gLink[SDA20D_Index::LINK_LEFT_WRIST1]);
	gJoint[SDA20D_Index::JOINT_LEFT_B]->SetChildLink(&gLink[SDA20D_Index::LINK_LEFT_WRIST2]);
	gJoint[SDA20D_Index::JOINT_LEFT_B]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.42 / 2, 0.0)));
	gJoint[SDA20D_Index::JOINT_LEFT_B]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	gJoint[SDA20D_Index::JOINT_LEFT_B]->MakePositionLimit(false);

	gLink[SDA20D_Index::LINK_LEFT_WRIST2].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[SDA20D_Index::LINK_LEFT_WRIST2].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.385, -(0.49 + 0.42 + 0.15), -0.55)));
	gLink[SDA20D_Index::LINK_LEFT_WRIST2].GetGeomInfo().SetFileName("../../../workspace/robot/sda20d_3ds/LINK_LEFT_WRIST2.3ds");
	gLink[SDA20D_Index::LINK_LEFT_WRIST2].GetGeomInfo().SetColor(GRAYCOLORSDA20D);


	gJoint[SDA20D_Index::JOINT_LEFT_T]->SetActType(srJoint::HYBRID);
	gJoint[SDA20D_Index::JOINT_LEFT_T]->SetParentLink(&gLink[SDA20D_Index::LINK_LEFT_WRIST2]);
	gJoint[SDA20D_Index::JOINT_LEFT_T]->SetChildLink(&gLink[SDA20D_Index::LINK_LEFT_GRIPPER]);
	gJoint[SDA20D_Index::JOINT_LEFT_T]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.18, 0.0)));
	//gJoint[SDA20D_Index::JOINT_LEFT_T]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0, 0.0)));
	gJoint[SDA20D_Index::JOINT_LEFT_T]->SetChildLinkFrame(EulerZYX(Vec3(-SR_PI_HALF, SR_PI_HALF, 0.0), Vec3(0.0, 0.0, 0.0)));			// match to robot hardware
	gJoint[SDA20D_Index::JOINT_LEFT_T]->MakePositionLimit(false);

	gLink[SDA20D_Index::LINK_LEFT_GRIPPER].GetGeomInfo().SetDimension(0.001);
#ifdef SHOW_GRIPPER
	gLink[SDA20D_Index::LINK_LEFT_GRIPPER].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	//gLink[SDA20D_Index::LINK_LEFT_GRIPPER].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.385, -(0.49 + 0.42 + 0.18 + 0.15), -0.55))); // old gripper
	//gLink[SDA20D_Index::LINK_LEFT_GRIPPER].GetGeomInfo().SetFileName("../../../workspace/robot/sda20d_3ds/LINK_LEFT_GRIPPER.3ds"); // old gripper
	gLink[SDA20D_Index::LINK_LEFT_GRIPPER].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.0, SR_PI, -SR_PI_HALF), Vec3(0.0, 0.1735, 0.0595)));
	gLink[SDA20D_Index::LINK_LEFT_GRIPPER].GetGeomInfo().SetFileName("../../../workspace/robot/sda20d_3ds/LINK_LEFT_GRIPPER2_b.3ds");

	// ADD GRIPPER, GRIPPER_U, GRIPPER_L


	gPjoint[SDA20D_Index::JOINT_LEFT_GRIPPER_L]->SetActType(srJoint::HYBRID);
	gPjoint[SDA20D_Index::JOINT_LEFT_GRIPPER_L]->SetParentLink(&gLink[SDA20D_Index::LINK_LEFT_GRIPPER]);
	gPjoint[SDA20D_Index::JOINT_LEFT_GRIPPER_L]->SetChildLink(&gLink[SDA20D_Index::LINK_LEFT_GRIPPER_L]);
	gPjoint[SDA20D_Index::JOINT_LEFT_GRIPPER_L]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.1805, 0.0)));
	gPjoint[SDA20D_Index::JOINT_LEFT_GRIPPER_L]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, -0.0595, 0.0)));
	gPjoint[SDA20D_Index::JOINT_LEFT_GRIPPER_L]->MakePositionLimit(false);
	
	gLink[SDA20D_Index::LINK_LEFT_GRIPPER_L].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[SDA20D_Index::LINK_LEFT_GRIPPER_L].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(SR_PI, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	gLink[SDA20D_Index::LINK_LEFT_GRIPPER_L].GetGeomInfo().SetFileName("../../../workspace/robot/sda20d_3ds/LINK_LEFT_GRIPPER2_finger_l.3ds");
	gLink[SDA20D_Index::LINK_LEFT_GRIPPER_L].GetGeomInfo().SetColor(GRAYCOLORSDA20D);

	gPjoint[SDA20D_Index::JOINT_LEFT_GRIPPER_U]->SetActType(srJoint::HYBRID);
	gPjoint[SDA20D_Index::JOINT_LEFT_GRIPPER_U]->SetParentLink(&gLink[SDA20D_Index::LINK_LEFT_GRIPPER]);
	gPjoint[SDA20D_Index::JOINT_LEFT_GRIPPER_U]->SetChildLink(&gLink[SDA20D_Index::LINK_LEFT_GRIPPER_U]);
	gPjoint[SDA20D_Index::JOINT_LEFT_GRIPPER_U]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.1805, 0.0)));
	gPjoint[SDA20D_Index::JOINT_LEFT_GRIPPER_U]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0595, 0.0)));
	gPjoint[SDA20D_Index::JOINT_LEFT_GRIPPER_U]->MakePositionLimit(false);

	gLink[SDA20D_Index::LINK_LEFT_GRIPPER_U].GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gLink[SDA20D_Index::LINK_LEFT_GRIPPER_U].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(SR_PI, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	gLink[SDA20D_Index::LINK_LEFT_GRIPPER_U].GetGeomInfo().SetFileName("../../../workspace/robot/sda20d_3ds/LINK_LEFT_GRIPPER2_finger_u.3ds");
	gLink[SDA20D_Index::LINK_LEFT_GRIPPER_U].GetGeomInfo().SetColor(GRAYCOLORSDA20D);
#endif // SHOW_GRIPPER

	////////////////////// marker links
	gMarkerLink[SDA20D_Index::MLINK_RIGHT_T].GetGeomInfo().SetDimension(0.001);
	gMarkerLink[SDA20D_Index::MLINK_LEFT_T].GetGeomInfo().SetDimension(0.001);

	//double size_test = 0.3;
	gMarkerLink[SDA20D_Index::MLINK_RIGHT_GRIPPER].GetGeomInfo().SetDimension(0.0);
	gMarkerLink[SDA20D_Index::MLINK_RIGHT_GRIPPER].GetGeomInfo().SetColor(0.2, 0.2, 0.2);
	gMarkerLink[SDA20D_Index::MLINK_LEFT_GRIPPER].GetGeomInfo().SetDimension(0.0);
	gMarkerLink[SDA20D_Index::MLINK_LEFT_GRIPPER].GetGeomInfo().SetColor(0.2, 0.2, 0.2);
	// right arm
	gWjoint[SDA20D_Index::MLINK_RIGHT_T]->SetActType(srJoint::PASSIVE);
	gWjoint[SDA20D_Index::MLINK_RIGHT_T]->SetParentLink(gJoint[SDA20D_Index::JOINT_RIGHT_T]->m_ChildLink);
	gWjoint[SDA20D_Index::MLINK_RIGHT_T]->SetChildLink(&gMarkerLink[SDA20D_Index::MLINK_RIGHT_T]);
	gWjoint[SDA20D_Index::MLINK_RIGHT_T]->SetParentLinkFrame(gJoint[SDA20D_Index::JOINT_RIGHT_T]->GetChildLinkFrame());
	gWjoint[SDA20D_Index::MLINK_RIGHT_T]->SetChildLinkFrame(SE3());

	gWjoint[SDA20D_Index::MLINK_RIGHT_GRIPPER]->SetActType(srJoint::PASSIVE);
	gWjoint[SDA20D_Index::MLINK_RIGHT_GRIPPER]->SetParentLink(gJoint[SDA20D_Index::JOINT_RIGHT_T]->m_ChildLink);
	gWjoint[SDA20D_Index::MLINK_RIGHT_GRIPPER]->SetChildLink(&gMarkerLink[SDA20D_Index::MLINK_RIGHT_GRIPPER]);
	gWjoint[SDA20D_Index::MLINK_RIGHT_GRIPPER]->SetParentLinkFrame(gJoint[SDA20D_Index::JOINT_RIGHT_T]->GetChildLinkFrame());
	gWjoint[SDA20D_Index::MLINK_RIGHT_GRIPPER]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.2025, 0.033)));

	// left arm
	gWjoint[SDA20D_Index::MLINK_LEFT_T]->SetActType(srJoint::PASSIVE);
	gWjoint[SDA20D_Index::MLINK_LEFT_T]->SetParentLink(gJoint[SDA20D_Index::JOINT_LEFT_T]->m_ChildLink);
	gWjoint[SDA20D_Index::MLINK_LEFT_T]->SetChildLink(&gMarkerLink[SDA20D_Index::MLINK_LEFT_T]);
	gWjoint[SDA20D_Index::MLINK_LEFT_T]->SetParentLinkFrame(gJoint[SDA20D_Index::JOINT_LEFT_T]->GetChildLinkFrame());
	//gWjoint[SDA20D_Index::MLINK_LEFT_T]->SetChildLinkFrame(SE3());
	gWjoint[SDA20D_Index::MLINK_LEFT_T]->SetChildLinkFrame(EulerZYX(Vec3(0.0, SR_PI, 0.0), Vec3(0.0, 0.0, 0.0)));			// match to robot api

	gWjoint[SDA20D_Index::MLINK_LEFT_GRIPPER]->SetActType(srJoint::PASSIVE);
	gWjoint[SDA20D_Index::MLINK_LEFT_GRIPPER]->SetParentLink(gJoint[SDA20D_Index::JOINT_LEFT_T]->m_ChildLink);
	gWjoint[SDA20D_Index::MLINK_LEFT_GRIPPER]->SetChildLink(&gMarkerLink[SDA20D_Index::MLINK_LEFT_GRIPPER]);
	gWjoint[SDA20D_Index::MLINK_LEFT_GRIPPER]->SetParentLinkFrame(gJoint[SDA20D_Index::JOINT_LEFT_T]->GetChildLinkFrame());
	//gWjoint[SDA20D_Index::MLINK_LEFT_GRIPPER]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, -0.2025, 0.033)));
	//gWjoint[SDA20D_Index::MLINK_LEFT_GRIPPER]->SetChildLinkFrame(EulerZYX(Vec3(0.0, SR_PI, 0.0), Vec3(0.033, 0.0, -0.2025)));		// match to robot api (old gripper)
	gWjoint[SDA20D_Index::MLINK_LEFT_GRIPPER]->SetChildLinkFrame(EulerZYX(Vec3(0.0, SR_PI, 0.0), Vec3(0.0485, 0.0, -0.2645)));		// match to robot api

	this->SetBaseLink(&gLink[SDA20D_Index::LINK_BASE]);
	this->SetBaseLinkType(srSystem::FIXED);


}

void SDA20D::AssembleCollision()
{

	//////// attach collision to link
	m_numCollision = 0;
	//gLink[SDA20D_Index::LINK_BASE].AddCollision(&gCollision[m_numCollision]);
	//gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	//gCollision[m_numCollision].GetGeomInfo().SetDimension(0.6, 0.4, 0.18);
	//gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-0.08, 0.0, 0.0)));

	gLink[SDA20D_Index::LINK_BASE].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(0.5, 0.35, 0.18);
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-0.08, 0.0, 0.0)));


	//gLink[SDA20D_Index::LINK_UPPERBASE].AddCollision(&gCollision[m_numCollision]);
	//gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	//gCollision[m_numCollision].GetGeomInfo().SetDimension(0.5, 0.34, 0.21);
	//gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, 0.0, -(0.55 - 0.43 + 0.015))));
	//
	gLink[SDA20D_Index::LINK_UPPERBASE].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(0.5, 0.34, 0.14);
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, 0.0, -(0.55 - 0.43 + 0.015)+0.035)));

	//gLink[SDA20D_Index::LINK_UPPERBASE].AddCollision(&gCollision[m_numCollision]);
	//gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	//gCollision[m_numCollision].GetGeomInfo().SetDimension(0.25, 0.3, 0.25);
	//gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(-0.12, 0.0, 0.105)));

	gLink[SDA20D_Index::LINK_UPPERBASE].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(0.27, 0.13, 0.25);
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.16, 0.0, 0.105)));


	int numBox = 5;
	double thickness = 0.03;
	double space = 0.005;
	vector<pair<Vec3, SE3>> boxSet;

	// right arm
	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.15, 0.0)), 0.12, 0.14, thickness, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[SDA20D_Index::LINK_RIGHT_SHOULDER].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}
	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, -0.0875, 0.06)), 0.11, 0.1, thickness, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[SDA20D_Index::LINK_RIGHT_SHOULDER].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}
	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, -0.07)), 0.12, 0.1, thickness, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[SDA20D_Index::LINK_RIGHT_UPPERARM].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}
	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, -0.19, 0.0)), 0.09, 0.1, thickness, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[SDA20D_Index::LINK_RIGHT_UPPERARM].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}
	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, -0.05, 0.0)), 0.09, 0.09, thickness, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[SDA20D_Index::LINK_RIGHT_LOWERARM].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}
	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, -0.49 / 2, 0.06)), 0.1, 0.09, thickness, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[SDA20D_Index::LINK_RIGHT_LOWERARM].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}
	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, -0.05)), 0.095, 0.08, thickness, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[SDA20D_Index::LINK_RIGHT_ELBOW].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}
	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, -0.18, 0.0)), 0.06, 0.05, thickness, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[SDA20D_Index::LINK_RIGHT_ELBOW].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}
	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, -0.08, 0.0)), 0.065, 0.08, thickness, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[SDA20D_Index::LINK_RIGHT_WRIST1].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}
	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, -0.42 / 2, -0.04)), 0.055, 0.07, thickness, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[SDA20D_Index::LINK_RIGHT_WRIST1].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}
	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.05)), 0.055, 0.065, thickness, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[SDA20D_Index::LINK_RIGHT_WRIST2].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}
	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, -0.13, 0.0)), 0.055, 0.07, thickness, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[SDA20D_Index::LINK_RIGHT_WRIST2].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}

#ifdef SHOW_GRIPPER
	// right gripper
	double cylheight = 0.0905;
	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, -0.04, 0.0)), 0.05, -cylheight, 0.01, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[SDA20D_Index::LINK_RIGHT_GRIPPER].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}
	//gLink[SDA20D_Index::LINK_RIGHT_GRIPPER].AddCollision(&gCollision[m_numCollision]);
	//gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	//gCollision[m_numCollision].GetGeomInfo().SetDimension(Vec3(0.2,0.4, 0.6));
	//gCollision[m_numCollision++].SetLocalFrame(SE3());

	gLink[SDA20D_Index::LINK_RIGHT_GRIPPER].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(0.080, -0.085, 0.055);
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, -0.13, 0.0)));

	gLink[SDA20D_Index::LINK_RIGHT_GRIPPER].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(0.015, 0.03, 0.025);
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.058, -0.04, 0.030)));

	gLink[SDA20D_Index::LINK_RIGHT_GRIPPER].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(0.125, 0.020, 0.055);
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, -0.18, 0.0)));

	gLink[SDA20D_Index::LINK_RIGHT_GRIPPER_L].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(0.025, 0.055, 0.030);
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, 0.0157, 0.03)));

	gLink[SDA20D_Index::LINK_RIGHT_GRIPPER_L].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(0.025, 0.005, 0.038);
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, -0.0063, 0.065)));

	gLink[SDA20D_Index::LINK_RIGHT_GRIPPER_U].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(0.025, -0.045, 0.05);
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, -0.0207, 0.04)));

	gLink[SDA20D_Index::LINK_RIGHT_GRIPPER_U].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(0.025, 0.048, 0.036);
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, -0.0692, 0.066)));

#endif // SHOW_GRIPPER

	// left arm
	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, -0.15, 0.0)), 0.12, 0.14, thickness, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[SDA20D_Index::LINK_LEFT_SHOULDER].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}
	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0875, 0.06)), 0.11, 0.1, thickness, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[SDA20D_Index::LINK_LEFT_SHOULDER].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}
	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, -0.07)), 0.12, 0.1, thickness, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[SDA20D_Index::LINK_LEFT_UPPERARM].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}
	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.19, 0.0)), 0.09, 0.1, thickness, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[SDA20D_Index::LINK_LEFT_UPPERARM].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}
	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.05, 0.0)), 0.09, 0.09, thickness, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[SDA20D_Index::LINK_LEFT_LOWERARM].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}
	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.49 / 2, 0.06)), 0.1, 0.09, thickness, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[SDA20D_Index::LINK_LEFT_LOWERARM].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}
	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, -0.05)), 0.095, 0.08, thickness, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[SDA20D_Index::LINK_LEFT_ELBOW].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}
	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.18, 0.0)), 0.06, 0.05, thickness, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[SDA20D_Index::LINK_LEFT_ELBOW].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}
	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.08, 0.0)), 0.065, 0.08, thickness, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[SDA20D_Index::LINK_LEFT_WRIST1].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}
	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.42 / 2, -0.04)), 0.055, 0.07, thickness, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[SDA20D_Index::LINK_LEFT_WRIST1].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}
	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.05)), 0.055, 0.065, thickness, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[SDA20D_Index::LINK_LEFT_WRIST2].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}
	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.13, 0.0)), 0.055, 0.07, thickness, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[SDA20D_Index::LINK_LEFT_WRIST2].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}

#ifdef SHOW_GRIPPER
	// left gripper
	//double cylheight = 0.0905;
	boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.04, 0.0)), 0.05, cylheight, 0.01, space, numBox);
	for (int i = 0; i < numBox; i++)
	{
		gLink[SDA20D_Index::LINK_LEFT_GRIPPER].AddCollision(&gCollision[m_numCollision]);
		gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
		gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	}
	//double cylheight = 0.0765;
	//boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.04, 0.0)), 0.05, cylheight, 0.01, space, numBox);
	//for (int i = 0; i < numBox; i++)
	//{
	//	gLink[SDA20D_Index::LINK_LEFT_GRIPPER].AddCollision(&gCollision[m_numCollision]);
	//	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	//	gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
	//	gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	//}
	//int numBox2 = 10;
	//boxSet = makeCylinderWithBoxes(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0765, 0.0)), 0.09, 0.002, 0.01, space, numBox2);
	//for (int i = 0; i < numBox2; i++)
	//{
	//	gLink[SDA20D_Index::LINK_LEFT_GRIPPER].AddCollision(&gCollision[m_numCollision]);
	//	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	//	gCollision[m_numCollision].GetGeomInfo().SetDimension(boxSet[i].first);
	//	gCollision[m_numCollision++].SetLocalFrame(boxSet[i].second);
	//}

	//Vec3 box1dim = Vec3(0.03, 0.086, 0.06);
	//gLink[SDA20D_Index::LINK_LEFT_GRIPPER].AddCollision(&gCollision[m_numCollision]);
	//gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	//gCollision[m_numCollision].GetGeomInfo().SetDimension(box1dim);
	//gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, cylheight + 0.5*box1dim[1], 0.0)));

	//double gripClear = 0.01;
	//Vec3 box2dim = Vec3(0.014, 0.04, 0.005);
	//gLink[SDA20D_Index::LINK_LEFT_GRIPPER].AddCollision(&gCollision[m_numCollision]);
	//gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	//gCollision[m_numCollision].GetGeomInfo().SetDimension(box2dim);
	//gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, 0.1625 + 0.5*box2dim[1], -0.033 - 0.5*gripClear - 0.5*box2dim[2])));

	//Vec3 box3dim = Vec3(0.014, 0.035, 0.005);
	//gLink[SDA20D_Index::LINK_LEFT_GRIPPER].AddCollision(&gCollision[m_numCollision]);
	//gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	//gCollision[m_numCollision].GetGeomInfo().SetDimension(box3dim);
	//gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, 0.1675 + 0.5*box3dim[1], -0.033 + 0.5*gripClear + 0.5*box3dim[2])));

	//Vec3 box4dim = Vec3(0.014, 0.015, 0.05);
	//gLink[SDA20D_Index::LINK_LEFT_GRIPPER].AddCollision(&gCollision[m_numCollision]);
	//gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	//gCollision[m_numCollision].GetGeomInfo().SetDimension(box4dim);
	//gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, 0.175, 0.03 - 0.5*box4dim[2])));

	//Vec3 box5dim = Vec3(0.014, 0.009, 0.01);
	//gLink[SDA20D_Index::LINK_LEFT_GRIPPER].AddCollision(&gCollision[m_numCollision]);
	//gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	//gCollision[m_numCollision].GetGeomInfo().SetDimension(box5dim);
	//gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, 0.1575, -0.04125 + 0.5*box5dim[2])));

	gLink[SDA20D_Index::LINK_LEFT_GRIPPER].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(0.055, -0.085, -0.080);
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, 0.13, 0.0)));

	gLink[SDA20D_Index::LINK_LEFT_GRIPPER].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(0.03, -0.03, -0.02);
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.025, 0.04, 0.06)));

	gLink[SDA20D_Index::LINK_LEFT_GRIPPER].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(0.055, -0.020, -0.125);
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, 0.18, 0.0)));

	gLink[SDA20D_Index::LINK_LEFT_GRIPPER_L].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(0.025, 0.055, 0.030);
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, -0.0157, 0.03)));

	gLink[SDA20D_Index::LINK_LEFT_GRIPPER_L].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(0.025, 0.005, 0.038);
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, 0.0063, 0.065)));

	gLink[SDA20D_Index::LINK_LEFT_GRIPPER_U].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(0.025, 0.045, 0.05);
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, 0.0207, 0.04)));

	gLink[SDA20D_Index::LINK_LEFT_GRIPPER_U].AddCollision(&gCollision[m_numCollision]);
	gCollision[m_numCollision].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gCollision[m_numCollision].GetGeomInfo().SetDimension(0.025, 0.048, 0.036);
	gCollision[m_numCollision++].SetLocalFrame(SE3(Vec3(0.0, 0.0692, 0.066)));

#endif // SHOW_GRIPPER

	

}



void SDA20D::SetActType(srJoint::ACTTYPE actType)
{
	for (int i = 0; i < NUM_OF_RJOINT_SDA20D; i++)
		gJoint[i]->SetActType(actType);
}

void SDA20D::SetGripperActType(srJoint::ACTTYPE actType)
{
	//for (int i = 0; i < NUM_OF_GRIPERJOINT_MH12; i++)
	//	gGripJoint[i]->SetActType(actType);
}

void SDA20D::SetGripperActType(srJoint::ACTTYPE actType, vector<int> gpJointIdx)
{
	//for (unsigned int i = 0; i < gpJointIdx.size(); i++)
	//	gGripJoint[gpJointIdx[i]]->SetActType(actType);
}

void SDA20D::SetJointLimit()
{
	UpperJointLimit[SDA20D_Index::JOINT_WAIST] = DEG2RAD(180);
	double ratio = 1.0;
	UpperJointLimit[SDA20D_Index::JOINT_RIGHT_S] = ratio*DEG2RAD(180);
	UpperJointLimit[SDA20D_Index::JOINT_RIGHT_L] = ratio*DEG2RAD(90);	//DEG2RAD(110);
	UpperJointLimit[SDA20D_Index::JOINT_RIGHT_E] = ratio*DEG2RAD(170);
	UpperJointLimit[SDA20D_Index::JOINT_RIGHT_U] = ratio*DEG2RAD(130);	//1.9
	UpperJointLimit[SDA20D_Index::JOINT_RIGHT_R] = ratio*DEG2RAD(180);
	UpperJointLimit[SDA20D_Index::JOINT_RIGHT_B] = ratio*DEG2RAD(110);	//1.77
	UpperJointLimit[SDA20D_Index::JOINT_RIGHT_T] = ratio*DEG2RAD(270);	//180

	for (int i = DEGREE_OF_FREEDOM_SDA20D_RIGHTARM + DEGREE_OF_FREEDOM_SDA20D_WAIST; i < DEGREE_OF_FREEDOM_SDA20D_JOINT; i++)
		UpperJointLimit[i] = UpperJointLimit[i - DEGREE_OF_FREEDOM_SDA20D_LEFTARM];

	for (int i = 0; i < DEGREE_OF_FREEDOM_SDA20D_JOINT; i++)
		LowerJointLimit[i] = (-1) * UpperJointLimit[i];

	for (int i = 0; i < DEGREE_OF_FREEDOM_SDA20D_JOINT; i++)
		gJoint[i]->SetPositionLimit(RAD2DEG(LowerJointLimit[i]), RAD2DEG(UpperJointLimit[i]));

}

void SDA20D::SetVelocityLimit()
{
	VelocityLimit[SDA20D_Index::JOINT_WAIST] = 2.18;

	VelocityLimit[SDA20D_Index::JOINT_RIGHT_S] = 2.27;
	VelocityLimit[SDA20D_Index::JOINT_RIGHT_L] = 2.27;
	VelocityLimit[SDA20D_Index::JOINT_RIGHT_E] = 2.97;
	VelocityLimit[SDA20D_Index::JOINT_RIGHT_U] = 2.97;
	VelocityLimit[SDA20D_Index::JOINT_RIGHT_R] = 3.49;
	VelocityLimit[SDA20D_Index::JOINT_RIGHT_B] = 3.49;
	VelocityLimit[SDA20D_Index::JOINT_RIGHT_T] = 6.98;

	for (int i = DEGREE_OF_FREEDOM_SDA20D_RIGHTARM + DEGREE_OF_FREEDOM_SDA20D_WAIST; i < DEGREE_OF_FREEDOM_SDA20D_JOINT; i++)
		VelocityLimit[i] = VelocityLimit[i - DEGREE_OF_FREEDOM_SDA20D_LEFTARM];
}

void SDA20D::SetInitialConfiguration()
{
	for (int i = 0; i < DEGREE_OF_FREEDOM_SDA20D_JOINT; i++)
		gJoint[i]->m_State.m_rValue[0] = DEG2RAD(0);


	//gJoint[SDA20D_Index::JOINT_RIGHT_S]->m_State.m_rValue[0] = SR_PI_HALF;
	//gJoint[SDA20D_Index::JOINT_LEFT_S]->m_State.m_rValue[0] = SR_PI_HALF;

	//gJoint[SDA20D_Index::JOINT_RIGHT_L]->m_State.m_rValue[0] = SR_PI_HALF;
	//gJoint[SDA20D_Index::JOINT_LEFT_L]->m_State.m_rValue[0] = SR_PI_HALF;
/*
	gJoint[SDA20D_Index::JOINT_LEFT_S]->m_State.m_rValue[0] = 0.5*SR_PI_HALF;
	gJoint[SDA20D_Index::JOINT_LEFT_L]->m_State.m_rValue[0] = SR_PI_HALF;
	gJoint[SDA20D_Index::JOINT_LEFT_U]->m_State.m_rValue[0] = DEG2RAD(-20);

	gJoint[SDA20D_Index::JOINT_RIGHT_S]->m_State.m_rValue[0] = 0.7*SR_PI_HALF;
	gJoint[SDA20D_Index::JOINT_RIGHT_L]->m_State.m_rValue[0] = SR_PI_HALF;
	gJoint[SDA20D_Index::JOINT_RIGHT_E]->m_State.m_rValue[0] = SR_PI_HALF;
	gJoint[SDA20D_Index::JOINT_RIGHT_U]->m_State.m_rValue[0] = 1.1*SR_PI_HALF;*/

	KIN_UpdateFrame_All_The_Entity();
}

