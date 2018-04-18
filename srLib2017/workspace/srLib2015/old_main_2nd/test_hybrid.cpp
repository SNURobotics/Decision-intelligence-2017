#include <cstdio>

#include "myRenderer.h"
#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\indyRobotManager.h"
#include "robotManager/IndyRobot.h"
#include <time.h>
#include "robotManager\environmentBusbar.h"
#include "robotManager\environment_workcell.h"
#include "robotManager\environment_QBtech.h"
#include "robotManager\robotRRTManager.h"
#include "ForceCtrlManager\hybridPFCtrlManager.h"
#include <time.h>


// Environment
Base_HYU* busbarBase = new Base_HYU;
JigAssem_QB* jigAssem = new JigAssem_QB;
vector<Jig_HYU*> jig(4);
vector<BusBar_HYU*> busbar(8);
vector<SE3>	initSE3(8);
vector<SE3>	goalSE3(8);
vector<SE3> allSE3_busbar(2 * busbar.size());
srLink* busbarlink = new srLink;
srSystem* targetObj = new srSystem;

// Workspace
WorkCell* workCell = new WorkCell;
Eigen::VectorXd stageVal(3);
srSystem* obs = new srSystem;

// Robot
IndyRobot* robot1 = new IndyRobot;
IndyRobot* robot2 = new IndyRobot;
Eigen::VectorXd jointVal(6);
srSpace gSpace;
myRenderer* renderer;
SE3 Tbusbar2gripper = EulerZYX(Vec3(0.0, 0.0, SR_PI), Vec3(0.0, 0.0, 0.04));
indyRobotManager* rManager1;
indyRobotManager* rManager2;

hybridPFCtrlManager_6dof* hctrl = new hybridPFCtrlManager_6dof();
void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
void updateFuncTest();
void setObject(srSystem* system, Vec3 dim, SE3 T = SE3(), srSystem::BASELINKTYPE basetype = srSystem::BASELINKTYPE::FIXED);
void setHybridPFCtrl();

void environmentSetting_HYU(srSystem* object, bool connectStageBusbarBase = false);
void workspaceSetting();
void robotSetting();
void robotManagerSetting();

int main(int argc, char **argv)
{
	// environment
	workspaceSetting();
	for (unsigned int i = 0; i < jig.size(); i++)
	{
		jig[i] = new Jig_HYU;
		jig[i]->SetBaseLinkType(srSystem::FIXED);
	}
	for (unsigned int i = 0; i < busbar.size(); i++)
	{
		busbar[i] = new BusBar_HYU;
		busbar[i]->SetBaseLinkType(srSystem::FIXED);
	}
	targetObj = busbar[1];
	environmentSetting_HYU(targetObj, true);		// targetObj is assumed to be rigidly attached to robot end-effector
	robotSetting();
	//setObject(obs, Vec3(0.5, 0.5, 0.5));

	// initialize srLib
	initDynamics();

	// robot manager setting
	robotManagerSetting();
	
	// workcell robot initial config
	jointVal.setZero();
	jointVal[0] = 0.0; jointVal[1] = -SR_PI_HALF; jointVal[2] = 80.0 / 90.0*SR_PI_HALF; jointVal[3] = SR_PI_HALF;
	rManager2->setJointVal(jointVal);
	Eigen::VectorXd gripInput(2);
	gripInput[0] = -0.009;
	gripInput[1] = 0.009;
	rManager1->setGripperPosition(gripInput);
	rManager2->setGripperPosition(gripInput);

	
	int holeNum = 2;		// from 0 ~ 7
	SE3 TgoalPos = goalSE3[holeNum] * SE3(Vec3(0.01, 0.01, 0.0));
	
	// set initial config
	/////////// ÁÖÀÇ: initial busbar position and orientation should ensure contact at the beginning
	SE3 TbusbarInit = SE3(Vec3(0.01, 0.01, 0.0)) * TgoalPos * EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0));
	
	//obs->GetBaseLink()->SetFrame(SE3(Vec3(0.0, 0.0, -0.25))*TbusbarInit);

	int flag;
	Eigen::VectorXd qInit = Eigen::VectorXd::Zero(6);
	// elbow up
	qInit[1] = -0.65*SR_PI;
	qInit[2] = 0.3*SR_PI;
	qInit[3] = 0.5*SR_PI_HALF;
	Eigen::VectorXd q_config = rManager1->inverseKin(TbusbarInit * Tbusbar2gripper, rManager1->m_activeArmInfo->m_endeffector[0], true, SE3(), flag, qInit, 1000);
	cout << flag << endl;
	rManager1->setJointVal(q_config);
	Vec3 contactPoint = targetObj->GetBaseLink()->GetFrame().GetPosition();
	setHybridPFCtrl();

	// set desired trajectory (trajectory of the busbar)
	vector<SE3> Tdes(1);
	Tdes[0] = SE3(Vec3(0.0, 0.0, 0.005)) * goalSE3[holeNum];
	cout << "Tdes = " << endl << Tdes[0] << endl;
	vector<dse3> Fdes(1, dse3(0.0));		// expressed in end-effector frame
	Fdes[0][0] = 0.00;
	Fdes[0][1] = 0.0;
	Fdes[0][5] = -0.1;
	

	hctrl->isDesTrjSet = hctrl->setDesiredTraj(Tdes, Fdes);
	hctrl->setDesiredJointVal(q_config);
	rManager1->setJointValVel(q_config, Eigen::VectorXd::Zero(q_config.size()));


	rendering(argc, argv);

	return 0;
}

void rendering(int argc, char **argv)
{
	renderer = new myRenderer();

	SceneGraphRenderer::NUM_WINDOWS windows;

	windows = SceneGraphRenderer::SINGLE_WINDOWS;

	renderer->InitializeRenderer(argc, argv, windows, false);
	renderer->InitializeNode(&gSpace);
	renderer->setUpdateFunc(updateFunc);

	renderer->RunRendering();
}

void initDynamics()
{
	gSpace.SetTimestep(0.01);
	gSpace.SetGravity(0.0, 0.0, -10.0);
	gSpace.SetNumberofSubstepForRendering(1);
	gSpace.DYN_MODE_PRESTEP();
}

void updateFunc()
{
	hctrl->hybridPFControl();
	cout << "force: " << targetObj->GetBaseLink()->m_ConstraintImpulse * (1.0 / gSpace.m_Timestep_dyn_fixed) << endl;
	//cout << "error: " << Log(hctrl->T_des_trj[0] % (hctrl->m_endeffector->GetFrame()*hctrl->m_offset)) << endl;
	cout << "error: " << (hctrl->m_endeffector->GetFrame()*hctrl->m_offset).GetPosition() - hctrl->T_des_trj[0].GetPosition() << endl;
	rManager2->setJointVal(jointVal);
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();	
	
}

void setObject(srSystem* system, Vec3 dim, SE3 T, srSystem::BASELINKTYPE basetype)
{
	srLink* link1 = new srLink;
	srWeldJoint* joint1 = new srWeldJoint;
	srLink* link2 = new srLink;
	srCollision* colli2 = new srCollision;
	system->SetBaseLink(link1);
	link1->GetGeomInfo().SetDimension(Vec3(0.0));
	link2->GetGeomInfo().SetShape(srGeometryInfo::BOX);
	link2->GetGeomInfo().SetDimension(dim);
	link2->GetGeomInfo().SetColor(0.3, 0.3, 0.3);
	colli2->GetGeomInfo().SetShape(srGeometryInfo::BOX);
	colli2->GetGeomInfo().SetDimension(dim);
	link2->AddCollision(colli2);
	joint1->SetChildLink(link2);
	joint1->SetChildLinkFrame(SE3());
	joint1->SetParentLink(link1);
	joint1->SetParentLinkFrame(SE3(Vec3(0., 0.0, 0.)));
	link1->SetFrame(T);
	system->SetBaseLinkType(basetype);
	gSpace.AddSystem(system);
}

void updateFuncTest()
{
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
	Eigen::VectorXd jointVel = Eigen::VectorXd::Random(6);
	static double dt = 0.0001;
	static Eigen::MatrixXd J_bf = Eigen::MatrixXd::Zero(6, 6);
	static se3 V_bf = se3(0.0);
	
	rManager1->setJointValVel(jointVal, jointVel);
	Eigen::MatrixXd J = rManager1->getBodyJacobian(jointVal, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper));
	Eigen::MatrixXd Jdot = rManager1->getBodyJacobianDot(jointVal, jointVel, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper));
	se3 V = robot1->gMarkerLink[Indy_Index::MLINK_GRIP].m_Vel;
	Eigen::VectorXd Vnum = J*jointVel;
	cout << "Jdot_num = " << endl << (J - J_bf) / dt << endl << "------------" << endl;
	cout << "Jdot_cal = " << endl << Jdot << endl;
	jointVal += jointVel*dt;
	J_bf = J;
	V_bf = V;
}

void setHybridPFCtrl()
{
	// initial config should be aligned to the contact plane
	// assume target object is rigidly attached to robot end-effector
	hctrl->isSystemSet = hctrl->setSystem((robotManager*) rManager1, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper), targetObj->GetBaseLink());
	hctrl->setTimeStep(rManager1->m_space->m_Timestep_dyn_fixed);
	double kv_v = 0.25e2, kp_v = 0.25*kv_v*kv_v, ki_v = 0.25e3, kp_f = 1.0e-1, ki_f = 1.0e-1;
	hctrl->Kp_v = kp_v * Eigen::MatrixXd::Identity(6, 6);
	hctrl->Kv_v = kv_v * Eigen::MatrixXd::Identity(6, 6);
	hctrl->Ki_v = ki_v * Eigen::MatrixXd::Identity(6, 6);
	hctrl->Ki_f = ki_f * Eigen::MatrixXd::Identity(6, 6);
	hctrl->Kp_f = kp_f * Eigen::MatrixXd::Identity(6, 6);

	// S*V = 0 should be satisfied
	Eigen::MatrixXd S = Eigen::MatrixXd::Zero(3, 6);
	// pos controlled dir: trans x, y, rot z
	// force controlled dir: moment x, y, force z
	//S(0, 0) = 1.0;
	//S(1, 1) = 1.0;
	//S(2, 5) = 1.0;
	//hctrl->setSelectionMatrix(S);
	hctrl->setSelectionMatrix(Eigen::MatrixXd());
}


void robotSetting()
{
	gSpace.AddSystem((srSystem*)robot1);
	gSpace.AddSystem((srSystem*)robot2);
	robot1->GetBaseLink()->SetFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, SR_PI), Vec3(0.0205, 0.4005 - 0.12, 1.972)));
	robot2->GetBaseLink()->SetFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, SR_PI), Vec3(0.0205, 1.6005 + 0.12, 1.972)));
	robot1->SetActType(srJoint::ACTTYPE::TORQUE);
	robot2->SetActType(srJoint::ACTTYPE::TORQUE);
	vector<int> gpIdx(2);
	gpIdx[0] = 0;
	gpIdx[1] = 1;
	robot1->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);
	robot2->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);
	gpIdx[0] = 2;
	gpIdx[1] = 3;
	robot1->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);
	robot2->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);
}

void robotManagerSetting()
{
	// robot 1
	rManager1 = new indyRobotManager(robot1, &gSpace);

	// robot 2
	rManager2 = new indyRobotManager(robot2, &gSpace);
}

void workspaceSetting()
{
	gSpace.AddSystem(workCell);
}

void environmentSetting_HYU(srSystem* object, bool connectStageBase)
{


	SE3 Tbase = SE3(Vec3(0.025, 1.095, 1.176));
	vector<SE3> jigSE3(4);
	vector<SE3> jig2busbar(2);
	jig2busbar[0] = SE3(Vec3(0.00006, -0.0639, 0.0));
	jig2busbar[1] = SE3(Vec3(0.0, 0.0484, 0.01));
	jigSE3[0] = Tbase*SE3(Vec3(-0.1426, -0.0329, 0.01));
	jigSE3[1] = Tbase*SE3(Vec3(-0.0475, -0.0329, 0.01));
	jigSE3[2] = Tbase*SE3(Vec3(0.0475, -0.0329, 0.01));
	jigSE3[3] = Tbase*SE3(Vec3(0.1426, -0.0329, 0.01));

	busbarBase->setBaseLinkFrame(Tbase);

	for (unsigned int i = 0; i < jig.size(); i++)
	{
		jig[i]->setBaseLinkFrame(jigSE3[i]);
		if (!connectStageBase)
			gSpace.AddSystem(jig[i]);
		for (unsigned int j = 0; j < 2; j++)
		{
			busbar[2 * i + j]->setBaseLinkFrame(jigSE3[i] * jig2busbar[j]);
			goalSE3[2 * i + j] = jigSE3[i] * jig2busbar[j];
			if (object == NULL)
				gSpace.AddSystem(busbar[2 * i + j]);
		}
	}
	if (object != NULL)
	{
		srWeldJoint* wobjJoint = new srWeldJoint;
		wobjJoint->SetParentLink(&robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
		wobjJoint->SetChildLink(object->GetBaseLink());
		wobjJoint->SetParentLinkFrame(SE3());
		wobjJoint->SetChildLinkFrame(Tbusbar2gripper);
		busbarlink = object->GetBaseLink();
	}
	if (!connectStageBase)
		gSpace.AddSystem((srSystem*)busbarBase);
	else
	{
		srWeldJoint* wJoint = new srWeldJoint;
		wJoint->SetParentLink(workCell->getStagePlate());
		wJoint->SetChildLink(busbarBase->GetBaseLink());
		wJoint->SetParentLinkFrame(Tbase);
		wJoint->SetChildLinkFrame(SE3());
		vector<srWeldJoint*> wJoints(jig.size());
		for (unsigned int i = 0; i < jig.size(); i++)
		{
			wJoints[i] = new srWeldJoint;
			wJoints[i]->SetParentLink(workCell->getStagePlate());
			wJoints[i]->SetChildLink(jig[i]->GetBaseLink());
			wJoints[i]->SetParentLinkFrame(jigSE3[i]);
			wJoints[i]->SetChildLinkFrame(SE3());
		}
	}

}

void environmentSetting_HYU2(srSystem* object, bool connect)
{
	SE3 Tbase = SE3(Vec3(0.025, 1.095, 1.176));
	double z_angle = (double)rand() / RAND_MAX * 0.1;
	double x_trans = -(double)rand() / RAND_MAX * 0.1;
	double y_trans = (double)rand() / RAND_MAX * 0.1;
	SE3 Tbase2jigbase = EulerZYX(Vec3(z_angle, 0.0, 0.0), Vec3(x_trans, y_trans, 0.184));
	if (object != NULL)
	{
		srWeldJoint* wobjJoint = new srWeldJoint;
		wobjJoint->SetParentLink(&robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
		wobjJoint->SetChildLink(object->GetBaseLink());
		wobjJoint->SetParentLinkFrame(SE3());
		wobjJoint->SetChildLinkFrame(Tbusbar2gripper);
		busbarlink = object->GetBaseLink();
	}
	else
	{
		for (unsigned int i = 0; i < busbar.size(); i++)
		{
			busbar[i] = new BusBar_HYU;
			busbar[i]->SetBaseLinkType(srSystem::FIXED);
			gSpace.AddSystem(busbar[i]);
		}
	}
	
	jigAssem->SetBaseLinkType(srSystem::FIXED);
	if (!connect)
		gSpace.AddSystem((srSystem*)jigAssem);
	else
	{
		srWeldJoint* wJoint = new srWeldJoint;
		wJoint->SetParentLink(workCell->getStagePlate());
		wJoint->SetChildLink(jigAssem->GetBaseLink());
		wJoint->SetParentLinkFrame(Tbase*Tbase2jigbase);
		wJoint->SetChildLinkFrame(SE3());
	}
}