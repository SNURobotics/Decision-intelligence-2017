#include <cstdio>

#include "myRenderer.h"
#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\robotManager.h"
#include "robotManager/IndyRobot.h"
#include "robotManager\environment_QBtech.h"
#include "robotManager\robotTaskManager.h"
#include "ForceCtrlManager\impedanceCtrlManager.h"
#include "common\dataIO.h"
#include <time.h>

srSpace gSpace;
myRenderer* renderer;

// Environment
TableBusbar* table;
Base* busbarBase = new Base;
vector<Jig*> jig(4);
vector<BusBar*> busbar(8);
vector<SE3>	initSE3(8);
vector<SE3>	goalSE3(8);
vector<SE3> allSE3_busbar(initSE3.size() + goalSE3.size());
SE3 Tbusbar2gripper = EulerZYX(Vec3(0.0, SR_PI, 0.0), Vec3(0.0, 0.0, 0.04));
vector<SE3> goalWP(0);

// Robot
IndyRobot* robot1 = new IndyRobot;
srJoint::ACTTYPE actType = srJoint::ACTTYPE::HYBRID;
srJoint::ACTTYPE actType_grip = srJoint::ACTTYPE::HYBRID;
robotManager* rManager1 = new robotManager;
robotTaskManager* rTaskManager;

//IndyRobot* robot2 = new IndyRobot;
//robotManager* rManager2 = new robotManager;


srSystem* sph = new srSystem;
srSystem* obs = new srSystem;
vector<SE3> r2o(1);
impedanceCtrlManager* iManager = new impedanceCtrlManager;
void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();

void robotSetting();
void environmentSetting();
void robotManagerSetting();
void setObject(srSystem* system, Vec3 dim, SE3 T = SE3(), srSystem::BASELINKTYPE basetype = srSystem::BASELINKTYPE::FIXED);
void setImpede(SE3 TrobotInit, SE3 Tgoal, SE3 Trobot2obj, srSystem* object1);
void changeImpede(Eigen::MatrixXd cap, Eigen::MatrixXd res);
void setImpedeTraj(double step_size, double width, int rots, SE3 Tgoal0);
void setImpedeTraj2(SE3 Tgoal0);
void impedanceControl();
void torqueControl();
void accControl();

srSystem* test = new srSystem();
srLink* link1 = new srLink();
srLink* link2 = new srLink();
srCollision* coll1 = new srCollision();
srCollision* coll2 = new srCollision();
srRevoluteJoint* joint1 = new srRevoluteJoint();
srSystem* test2 = new srSystem();
srSystem* test3 = new srSystem();
srLink* link3 = new srLink();
srCollision* coll3 = new srCollision();

vector<Eigen::VectorXd> forceData(0);
vector<Eigen::VectorXd> Tdata(0);

vector<dse3> force;

void testObj();
void testObj2();
void testObj3();
void testForce();

int main(int argc, char **argv)
{
	//srand((unsigned int)time(0));
	//robotSetting();
	environmentSetting();
	//testObj3();
	
	// initialize srLib
	initDynamics();

	testForce();
	//// robotManager
	//robotManagerSetting();
	//int flag = 0;
	//Eigen::VectorXd initGuess = Eigen::VectorXd::Zero(6);
	//initGuess[0] = -0.57; initGuess[3] = -1.57; initGuess[4] = 1.53; initGuess[5] = -2.14;
	initSE3[0] = goalSE3[0]* SE3(EulerZYX(Vec3(0.25, 0.0, 0.0), Vec3(0.01, 0.01, 0.01)));
	//Eigen::VectorXd q0 = rManager1->inverseKin(initSE3[0] * Tbusbar2gripper, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flag, initGuess);
	//printf("inv kin flag: %d\n", flag);
	////q0.setZero();
	//rManager1->setJointVal(q0);
	


	// impedance control
	busbar[0]->GetBaseLink()->SetFrame(initSE3[0]);
	SE3 Tgoalpos = goalSE3[0]*SE3(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.01, 0.01, 0.01)));
	setImpede(initSE3[0], Tgoalpos, Inv(Tbusbar2gripper), busbar[0]);


	// test trajectories
	//setImpedeTraj(0.001, 0.005, 10, Tgoalpos);
	setImpedeTraj2(Tgoalpos);
	
	
	
	
	//setImpede(TrobotInit, Tgoalpos, SE3(Vec3(0.0, 0.0, 0.1)), test);

	//q << 0.008357, 0.069792, 0.634724, 1.047380, 0.113987, -0.519954;
	//dq << -0.095037, -0.374906, 1.046498, -0.329421, 0.017085, -0.350580;
	//rManager->setJointValVel(q, dq);
	gSpace.SET_USER_CONTROL_FUNCTION(impedanceControl);
	//gSpace.SET_USER_CONTROL_FUNCTION(torqueControl);
	//gSpace.SET_USER_CONTROL_FUNCTION(accControl);
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
	gSpace.SetTimestep(0.001);
	gSpace.SetGravity(0.0, 0.0, -10.0);
	gSpace.SetNumberofSubstepForRendering(10);
	gSpace.DYN_MODE_PRESTEP();
}

void updateFunc()
{
	static bool colli_bf = false;
	//Eigen::VectorXd gripInput(2);
	//gripInput[0] = 1.0;
	//gripInput[1] = -1.0;
	//rManager1->setGripperInput(0.1*gripInput);

	static int cnt = 0, stop = 0, goalIdx = 0, stopIdx = 0, ctrlIdx = 0;
	cnt++;
	//link1->AddUserExternalForce(dse3(0.0, 0.0, 0.0, 0.0, 0.0, -30.0));
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();	
	//colli_bf = gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP();
	colli_bf = gSpace.m_srDYN.RUNTIME_MARKKH();
	gSpace.m_srDYN._PRESTEP_Build_Baked_ContactConstraint________MARK7();

	static bool finished = false;
	static int idx_bf = 0;
	int idx = (cnt / 10) % force.size();

	if (idx != idx_bf)
		busbar[0]->GetBaseLink()->SetFrame(initSE3[0]);


	busbar[0]->GetBaseLink()->AddUserExternalForce(force[idx]);

	cout << "index: " << idx << endl;
	cout << "collision: " << colli_bf << endl;
	cout << "force: " << 1000.0*busbar[0]->GetBaseLink()->m_ConstraintImpulse << endl;
	if (idx != idx_bf)
		forceData.push_back(dse3toVector(1000.0*busbar[0]->GetBaseLink()->m_ConstraintImpulse));
	if (idx == force.size() - 1)
	{
		if (!finished)
		{
			saveDataToText(forceData, "../../../data/force_test5_r_0_2.txt");
			finished = true;
		}
	}
	
	
	idx_bf = idx;
	

	////////////////////////////
	//dse3 flink(0.0);
	//flink = link1->m_Inertia * link1->m_Acc - dad(link1->m_Vel, link1->m_Inertia*link1->m_Vel) - link1->m_ExtForce - link1->m_ConstraintImpulse *(1.0 / gSpace.m_Timestep_dyn_fixed);
	//flink = InvdAd(link1->m_ChildJoints[0]->GetFrame() % link1->GetFrame(), flink);

	//dse3 fjnt = link1->m_ChildJoints[0]->m_FS_Force;

	//dse3 flink3(0.0);
	//flink3 = link3->m_Inertia * link3->m_Acc - dad(link3->m_Vel, link3->m_Inertia*link3->m_Vel) - link3->m_ExtForce - link3->m_ConstraintImpulse *(1.0 /  gSpace.m_Timestep_dyn_fixed);

	//cout << "cal1: " << flink << endl;
	//cout << "cal3: " << flink3 << endl;
	//cout << "imp1: " << link1->m_ConstraintImpulse << endl;
	//cout << "imp3: " << link3->m_ConstraintImpulse << endl;
	//cout << "jnt : " << fjnt << endl;
	//if (colli_bf)
	//{
	//	int stop = 1;
	//}	
	////////////////////////////


	//cout << colli_bf << endl;
	//cout << "Fgrip from cal : " << rManager1->getForceOnGripperBase() << endl;
	//cout << "Fgrip from jnt : " << InvdAd(robot1->gLink[Indy_Index::ENDEFFECTOR].m_Frame % robot1->gWeldJoint[Indy_Index::WELDJOINT_GRIPPER]->m_ChildLink->m_Frame, robot1->gWeldJoint[Indy_Index::WELDJOINT_GRIPPER]->m_FS_Force) << endl;
	//cout << "Fgrip from jnt2: " << rManager1->readSensorValue(0) << endl;
	//cout << "Fgrip from jnt: " << robot1->gWeldJoint[Indy_Index::WELDJOINT_GRIPPER]->m_FS_Force << endl;
	//if (colli_bf)
	//{
	//	//cout << cnt << endl;
	//	//cout << link2->m_ConstraintImpulse << endl;
	//	//cout << link3->m_ConstraintImpulse << endl;

	//	//cout << robot1->gLink[Indy_Index::MLINK_GRIP].m_ConstraintImpulse << endl;
	//	
	//	//cout << jig[0]->GetBaseLink()->m_ConstraintImpulse << endl;
	//	//cout << link1->m_ConstraintImpulse << endl;
	//	//cout << link2->m_ConstraintImpulse << endl;


	//	double Vnorm = 0.0;
	//	for (int k = 0; k < 6; k++)
	//		Vnorm += busbar[0]->GetBaseLink()->m_Vel[k] * busbar[0]->GetBaseLink()->m_Vel[k];
	//	double error = 0.0;
	//	Vec3 errorVec = busbar[0]->GetBaseLink()->GetPosition() - goalWP[goalIdx].GetPosition();
	//	for (int k = 0; k < 2; k++)
	//		error += errorVec[k] * errorVec[k];
	//	if (error < 1.0e-7)
	//		stopIdx += 1;
	//	ctrlIdx++;
	//	if (stopIdx == 1000 || ctrlIdx > 1000)
	//	{
	//		cout << "Fcont: " << 1000.0*busbar[0]->GetBaseLink()->m_ConstraintImpulse << endl;
	//		cout << "Fext : " << busbar[0]->GetBaseLink()->m_ExtForce << endl;
	//		forceData.push_back(dse3toVector(1000.0*busbar[0]->GetBaseLink()->m_ConstraintImpulse));
	//		Tdata.push_back(SE3toVector(goalWP[goalIdx]));
	//		goalIdx++;
	//		static bool saved = false;
	//		if (!saved && goalIdx == goalWP.size())
	//		{
	//			saveDataToText(forceData, "../../../data/force_r_0_2.txt");
	//			saveDataToText(Tdata, "../../../data/T_r_0_2.txt");
	//			saved = true;
	//			printf("saved!!!\n");
	//		}
	//		goalIdx = min(goalIdx, goalWP.size() - 1);
	//		iManager->setDesiredTrajectory(goalWP[goalIdx]);
	//		//busbar[0]->GetBaseLink()->SetFrame(goalWP[0]);
	//		stopIdx = 0;
	//		ctrlIdx = 0;
	//	}
	//		
	//}

	

	// change desired points
	//bool useUserInput = false;
	//if (useUserInput)
	//{
	//	bool change;
	//	static double pos_x;
	//	static double pos_y;
	//	static double pos_z;
	//	if (cnt % 1000 == 0)
	//	{
	//		printf("change desired point?: ");
	//		cin >> change;
	//		if (change)
	//		{
	//			printf("\nwhere to go? (x,y,z): ");
	//			cin >> pos_x;
	//			cin >> pos_y;
	//			cin >> pos_z;
	//			iManager->Tdes_trj[0].SetPosition(iManager->Tdes_trj[0].GetPosition() + Vec3(pos_x, pos_y, pos_z));
	//			printf("\nnew Tdes: \n");
	//			cout << iManager->Tdes_trj[0] << endl;
	//			cout << goalSE3[0] << endl;
	//		}
	//	}
	//}
}


void environmentSetting()
{
	//SE3 tableCenter = SE3(Vec3(0.6, 0.0, 0.25));
	SE3 tableCenter = SE3(Vec3(0.0, 0.0, 0.0));
	Vec3 tableDim(0.6, 1.2, 0.1);
	table = new TableBusbar(tableCenter, tableDim);
	//gSpace.AddSystem((srSystem*)table);

	for (unsigned int i = 0; i < jig.size(); i++)
		jig[i] = new Jig;
	for (unsigned int i = 0; i < busbar.size(); i++)
		busbar[i] = new BusBar;
	SE3 Tbase = tableCenter*SE3(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.001)));
	vector<SE3> jigSE3(4);
	vector<SE3> jig2busbar(2);
	jig2busbar[0] = SE3(Vec3(0.00006, -0.0639, 0.0 + 0.0005));
	jig2busbar[1] = SE3(Vec3(0.0, 0.0484, 0.01 + 0.0005));
	jigSE3[0] = Tbase*SE3(Vec3(-0.1426, -0.0329, 0.01));
	jigSE3[1] = Tbase*SE3(Vec3(-0.0475, -0.0329, 0.01));
	jigSE3[2] = Tbase*SE3(Vec3(0.0475, -0.0329, 0.01));
	jigSE3[3] = Tbase*SE3(Vec3(0.1426, -0.0329, 0.01));

	busbarBase->setBaseLinkFrame(Tbase);

	for (unsigned int i = 0; i < jig.size(); i++)
	{
		jig[i]->setBaseLinkFrame(jigSE3[i]);
		jig[i]->SetBaseLinkType(srSystem::FIXED);
		gSpace.AddSystem(jig[i]);
		for (unsigned int j = 0; j < 2; j++)
		{
			initSE3[2 * i + j] = Tbase*EulerZYX(Vec3(SR_PI*0.1*i, 0.0, 0.0), Vec3(-0.2 + 0.1*i, 0.3 + 0.1*j, 0.0));
			goalSE3[2 * i + j] = (jigSE3[i] * jig2busbar[j]);
			gSpace.AddSystem(busbar[2 * i + j]);
			busbar[2 * i + j]->setBaseLinkFrame(initSE3[2 * i + j]);
			busbar[2 * i + j]->SetBaseLinkType(srSystem::DYNAMIC);
		}
	}
	gSpace.AddSystem((srSystem*)busbarBase);
	busbar[0]->m_ObjLink[0].GetGeomInfo().SetColor(0.3f, 0.0f, 0.0f, 1.0f);
	busbar[1]->m_ObjLink[0].GetGeomInfo().SetColor(0.0f, 0.3f, 0.0f, 1.0f);
	busbar[2]->m_ObjLink[0].GetGeomInfo().SetColor(0.0f, 0.0f, 0.3f, 1.0f);

	// sphere
	srLink* sphlink = new srLink();
	sphlink->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
	sphlink->GetGeomInfo().SetColor(0.3f, 0.0f, 0.0f, 1.0f);
	sphlink->GetGeomInfo().SetDimension(0.05);
	sph->SetBaseLink(sphlink);
	sphlink->SetFrame(SE3(Vec3(1.0, 0.0, 0.7)));
	sph->SetBaseLinkType(srSystem::BASELINKTYPE::FIXED);
	gSpace.AddSystem(sph);

}

void robotSetting()
{

	robot1->SetActType(actType);
	robot1->SetGripperActType(actType_grip);
	gSpace.AddSystem((srSystem*)robot1);
	//gSpace.AddSystem((srSystem*)robot2);
	robot1->GetBaseLink()->SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(-0.0, 0.0, 0.0)));
	//robot2->GetBaseLink()->SetFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.5, 0.7, 0.0)));
}

void robotManagerSetting()
{
	rManager1->setRobot((srSystem*)robot1);
	rManager1->setSpace(&gSpace);
	rManager1->setEndeffector(&robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
	vector<srJoint*> gripperJoint(2);
	gripperJoint[0] = robot1->gPjoint[Indy_Index::GRIPJOINT_L];
	gripperJoint[1] = robot1->gPjoint[Indy_Index::GRIPJOINT_U];
	rManager1->setGripper(gripperJoint, &robot1->gLink[Indy_Index::ENDEFFECTOR]);
	rManager1->setFTSensor(robot1->gWeldJoint[Indy_Index::WELDJOINT_GRIPPER]);

	rTaskManager = new robotTaskManager(rManager1);
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

void setImpede(SE3 TrobotInit, SE3 Tgoalpos, SE3 Trobot2obj, srSystem* object1)
{
	vector<srLink*> ees(1);
	vector<SE3> ofs(1, SE3());
	
	Eigen::MatrixXd mass = 1000.0 * Eigen::MatrixXd::Identity(6, 6);
	Eigen::MatrixXd res = 100.0 * Eigen::MatrixXd::Identity(6, 6);
	Eigen::MatrixXd cap = 1000.0 * Eigen::MatrixXd::Identity(6, 6);
	for (int i = 0; i < 3; i++)
		cap(i, i) = 100.0;
	cap(5, 5) = 100.0;
	for (int i = 3; i < 4; i++)
		res(i, i) = 2.0*sqrt(mass(i, i)*cap(i, i));
	
	se3 zero_se3 = se3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

	//ees[0] = rManager1->m_activeArmInfo->m_endeffector[0];
	//r2o[0] = Trobot2obj;
	//object1->GetBaseLink()->SetFrame(TrobotInit*r2o[0]);
	gSpace._KIN_UpdateFrame_All_The_Entity_All_The_Systems();
	//iManager->setSystem(rManager1, ees, ofs, r2o);
	iManager->setObject(object1);
	iManager->setInertial(mass);
	iManager->setCapacitive(cap);
	iManager->setResistive(res);
	iManager->setGravity(gSpace.m_Gravity);
	iManager->setRobotPostureAmendThreshold(0.01);
	iManager->setTimeStep(gSpace.m_Timestep_dyn_fixed);
	iManager->setDesiredTrajectory(SE3(object1->GetBaseLink()->GetOrientation(), Tgoalpos.GetPosition()), zero_se3, zero_se3);
}

void changeImpede(Eigen::MatrixXd cap, Eigen::MatrixXd res)
{
	iManager->setCapacitive(cap);
	iManager->setResistive(res);
}

void setImpedeTraj(double step_size, double width, int rots, SE3 Tgoal0)
{
	// generate rectangle spiral trajectory
	vector<SE3> Tdes(0);
	SE3 Ttemp, Ttemp2;
	goalWP.push_back(Tgoal0);
	Tdes.push_back(Tgoal0);
	Ttemp = Tgoal0;
	Ttemp2.SetOrientation(Tgoal0.GetOrientation());
	int idx_init = 0;
	Vec3 dir;
	double length;
	for (int i = 0; i < rots; i++)
	{
		// positive y dir
		dir = Vec3(0.0, 1.0, 0.0);
		length = (double)(2 * i + 1)*width;
		Ttemp.SetPosition(Ttemp.GetPosition() + length*dir);
		idx_init = Tdes.size() - 1;
		for (int j = 1; j < floor(length / step_size); j++)
		{
			Ttemp2.SetPosition(Tdes[idx_init].GetPosition() + (double) j*step_size*dir);
			Tdes.push_back(Ttemp2);
		}
		Tdes.push_back(Ttemp);
		goalWP.push_back(Ttemp);

		// positive x dir
		dir = Vec3(1.0, 0.0, 0.0);
		length = (double)(2 * i + 1)*width;
		Ttemp.SetPosition(Ttemp.GetPosition() + length*dir);
		idx_init = Tdes.size() - 1;
		for (int j = 1; j < floor(length / step_size); j++)
		{
			Ttemp2.SetPosition(Tdes[idx_init].GetPosition() + (double)j*step_size*dir);
			Tdes.push_back(Ttemp2);
		}
		Tdes.push_back(Ttemp);
		goalWP.push_back(Ttemp);

		// negative y dir
		dir = Vec3(0.0, -1.0, 0.0);
		length = (double)(2 * i + 2)*width;
		Ttemp.SetPosition(Ttemp.GetPosition() + length*dir);
		idx_init = Tdes.size() - 1;
		for (int j = 1; j < floor(length / step_size); j++)
		{
			Ttemp2.SetPosition(Tdes[idx_init].GetPosition() + (double)j*step_size*dir);
			Tdes.push_back(Ttemp2);
		}
		Tdes.push_back(Ttemp);
		goalWP.push_back(Ttemp);

		// negative x dir
		dir = Vec3(-1.0, 0.0, 0.0);
		length = (double)(2 * i + 2)*width;
		Ttemp.SetPosition(Ttemp.GetPosition() + length*dir);
		idx_init = Tdes.size() - 1;
		for (int j = 1; j < floor(length / step_size); j++)
		{
			Ttemp2.SetPosition(Tdes[idx_init].GetPosition() + (double)j*step_size*dir);
			Tdes.push_back(Ttemp2);
		}
		Tdes.push_back(Ttemp);
		goalWP.push_back(Ttemp);
	}

	//iManager->setDesiredTrajectory(Tdes);

	//// set Vdes, Vdotdes
	//vector<se3> Vdes(Tdes.size(), se3(0.0));
	//vector<se3> Vdotdes(Tdes.size(), se3(0.0));
	//iManager->setDesiredTrajectory(Tdes, Vdes, Vdotdes);
}

void setImpedeTraj2(SE3 Tgoal0)
{
	goalWP.push_back(Tgoal0);
	SO3 Rtemp;
	SE3 Ttemp = Tgoal0;
	Vec3 vtemp(0.0, 0.0, 0.0);
	double angle;
	int num = 36;
	for (int i = 0; i < num; i++)
	{
		angle = (double)2.0 * i / num * SR_PI;
		vtemp[0] = cos(angle);
		vtemp[1] = sin(angle);
		Rtemp = Exp(vtemp * SR_PI_HALF* 0.25);
		Ttemp.SetOrientation(Rtemp);
		goalWP.push_back(Ttemp);
		goalWP.push_back(Tgoal0);
	}
	//Rtemp = Exp(Vec3(1.0, -1.0, 0.0));
	//Ttemp.SetOrientation(Rtemp);
	//goalWP.push_back(Ttemp);
}

void impedanceControl()
{
	Eigen::VectorXd posInput(2);
	double input = 0.005;	// < 0.01
	posInput(0) = input;
	posInput(1) = -input;
	rManager1->setGripperPosition(posInput);
	//iManager->impedanceControl(actType);
	iManager->objectImpedanceControl();
	//object1->GetBaseLink()->AddUserExternalForce(dse3(0.1));
	//iManager->objectImpedanceControl();
	//cout << object1->GetBaseLink()->m_UserExtForce << endl;
	//object1->GetBaseLink()->m_Acc = se3(0.01);

	//object1->GetBaseLink()->AddUserExternalForce(InvdAd(iManager->objectRefFrame, -iManager->getGravitationalForce()));
	//cout << "Fg1 " << object1->GetBaseLink()->GetExtForce() + object1->GetBaseLink()->m_ChildLinks[0]->GetExtForce() << endl;
	//cout << "Fg2 " << iManager->getGravitationalForce() << endl;
	//cout << "Tb " << object1->GetBaseLink()->GetFrame() << endl;
	//cout << "T1 " << object1->m_KIN_Links[1]->GetFrame() << endl;
	//Inertia II = iManager->objectInertia;
	//for (int i = 5; i < 6; i++)
	//{
	//	((srStateJoint*)robot->m_KIN_Joints[i])->m_State.m_rValue[0] += 0.001;
	//}




	//static se3 Vbf = se3(0.0);
	//static se3 Vcur = se3(0.0);
	//static se3 Vbf_ee = se3(0.0);
	//static se3 Vcur_ee = se3(0.0);
	//static SE3 Tcur_ee = SE3();
	//static SE3 Tbf_ee = SE3();
	//Vcur = iManager->object->GetBaseLink()->m_Vel;
	//Vcur_ee = iManager->m_endeffectors[0][0]->m_Vel;
	//Tcur_ee = iManager->m_endeffectors[0][0]->GetFrame();
	//se3 Aobm, Arom;
	//double diff = 0.0;
	//if (iManager->controlStep > 0)
	//{
	//	Aobm = (Vcur - Vbf) * (1. / iManager->timeStep);
	//	
	//	Vbf = Vcur;
	//	Arom = InvAd(r2o[0], Vcur_ee - Vbf_ee) * (1. / iManager->timeStep);
	//	

	//	//cout << endl;
	//	//cout << " tau: " << rManager->getJointCommand().transpose() << endl;
	//	// print manipulability
	//	diff = 0.0;
	//	for (int i = 0; i < 6; i++)
	//		diff += (Aobm - Arom)[i] * (Aobm - Arom)[i];
	//	if (diff > 0.0001)
	//	{
	//		double manip = rManager1->manipulability(rManager1->getJointVal(), iManager->m_endeffectors[0][0]);
	//		cout << "Aobm: " << Aobm;
	//		cout << "Arom: " << Arom;
	//		cout << "   q: " << rManager1->getJointVal().transpose() << endl;
	//		cout << "  dq: " << rManager1->getJointVel().transpose() << endl;
	//		
	//		//printf("manipulability: %f\n", rManager->manipulability(rManager->getJointVal(), ee));
	//		//cout << rManager->getSpaceJacobian(rManager->getJointVal(), ee) << endl;
	//	}
	//		
	//	//cout << "Vmes: " << Log(Tbf_ee%Tcur_ee) * (1. / iManager->timeStep);
	//	//cout << "Vrob: " << Vcur_ee << endl;
	//	//cout << "Arob: " << (Vcur_ee - Vbf_ee) * (1. / iManager->timeStep) << endl;
	//	
	//	Vbf_ee = Vcur_ee;
	//	Tbf_ee = Tcur_ee;
	//	
	//}


	//cout << object1->GetBaseLink()->m_Vel << endl;
	//double dist = Norm(object1->GetBaseLink()->GetPosition() - robot->getLink("wrist_3_link")->GetPosition());
	//if (iManager->controlStep % 2 == 0)
	//	printf("dist: %f\n", dist);
}

void torqueControl()
{
	Eigen::VectorXd ddq = Eigen::VectorXd::Random(rManager1->m_activeArmInfo->m_numJoint);
	Eigen::VectorXd tau = rManager1->inverseDyn(rManager1->getJointVal(), rManager1->getJointVel(), ddq);
	rManager1->controlJointTorque(tau);
	static Eigen::VectorXd dqcur = Eigen::VectorXd::Zero(rManager1->m_activeArmInfo->m_numJoint);
	static Eigen::VectorXd dqbf = Eigen::VectorXd::Zero(rManager1->m_activeArmInfo->m_numJoint);
	static int controlStep = 0;
	dqcur = rManager1->getJointVel();
	if (controlStep > 0)
	{
		cout << "ddqmes: " << (dqcur - dqbf).transpose() * (1. / gSpace.m_Timestep_dyn_fixed) << endl;
		//cout << "ddqme2: " << rManager->getJointAcc().transpose() << endl;
		cout << "ddqdes: " << ddq.transpose() << endl;
	}
	dqbf = dqcur;
	controlStep++;
}

void accControl()
{
	Eigen::VectorXd taum = rManager1->getJointTorque();
	Eigen::VectorXd ddq = Eigen::VectorXd::Ones(rManager1->m_activeArmInfo->m_numJoint);
	Eigen::VectorXd tau = rManager1->inverseDyn(rManager1->getJointVal(), rManager1->getJointVel(), ddq);
	rManager1->controlJointAcc(ddq);
	static int controlStep = 0;
	if (controlStep > 0)
	{
		cout << "ddqmes: " << rManager1->getJointAcc().transpose() << endl;
		cout << "taumes: " << taum.transpose() << endl;
		cout << "ddqdes " << ddq.transpose() << endl;
		cout << "taudes: " << tau.transpose() << endl;	
	}
	controlStep++;
}

void testObj()
{
	joint1->SetParentLink(link1);
	joint1->SetParentLinkFrame(SE3(Vec3(-0.5, 0.0, 0.0)));
	joint1->SetChildLink(link2);
	joint1->SetChildLinkFrame(SE3(Vec3(0.5, 0.0, 0.0)));

	joint1->SetActType(srJoint::HYBRID);

	joint1->SetPositionLimit(-360, 360);

	link1->AddCollision(coll1);
	link2->AddCollision(coll2);
	link1->GetGeomInfo().SetColor(0.3, 0.3, 0.3);
	link2->GetGeomInfo().SetColor(0.3, 0.3, 0.3);
	test->SetBaseLink(link1);
	test->SetBaseLinkType(srSystem::BASELINKTYPE::FIXED);
	link1->SetFrame(SE3(Vec3(0.0, 0.0, 0.0)));
	test->SetSelfCollision(true);
	gSpace.AddSystem(test);


	link3->AddCollision(coll3);
	link3->GetGeomInfo().SetColor(0.5, 0.5, 0.5);
	test2->SetBaseLink(link3);
	gSpace.AddSystem(test2);
	link3->SetFrame(SE3(Vec3(0.0, -0.15, 0.0)));
	test2->SetBaseLinkType(srSystem::BASELINKTYPE::DYNAMIC);
}

void testObj2()
{
	link1->AddCollision(coll1);
	link2->AddCollision(coll2);
	link3->AddCollision(coll3);

	test->SetBaseLink(link1);
	test2->SetBaseLink(link2);
	test3->SetBaseLink(link3);

	link1->GetGeomInfo().SetColor(0.3, 0.3, 0.3);
	link2->GetGeomInfo().SetColor(0.3, 0.3, 0.3);
	link3->GetGeomInfo().SetColor(0.3, 0.3, 0.3);

	link2->SetFrame(SE3(Vec3(0.0, 0.0, 0.2)));
	link3->SetFrame(SE3(Vec3(0.0, 0.0, 0.3000001)));
	test->SetBaseLinkType(srSystem::BASELINKTYPE::FIXED);

	gSpace.AddSystem(test);
	gSpace.AddSystem(test2);
	gSpace.AddSystem(test3);
}

void testObj3()
{
	test->SetBaseLink(link1);
	link1->AddCollision(coll1);
	link1->GetGeomInfo().SetColor(0.0, 0.3, 0.0);
	link3->AddCollision(coll3);
	link3->GetGeomInfo().SetColor(0.3, 0.0, 0.0);
	srWeldJoint* wJoint = new srWeldJoint;
	wJoint->SetParentLink(link1);
	wJoint->SetParentLinkFrame(SE3(Vec3(0.0, 0.0, 0.05)));
	wJoint->SetChildLink(link3);
	wJoint->SetChildLinkFrame(SE3(Vec3(0.0, 0.0, -0.05)));
	
	gSpace.AddSystem(test);

	test2->SetBaseLink(link2);
	link2->AddCollision(coll2);
	link2->GetGeomInfo().SetColor(0.3, 0.3, 0.3);
	gSpace.AddSystem(test2);

	link2->SetFrame(SE3(Vec3(0.0,0.0,-0.01)) * goalSE3[0]);
	link1->SetFrame(SE3(Vec3(0.0, 0.0, 0.2)) * goalSE3[0]);
	test2->SetBaseLinkType(srSystem::BASELINKTYPE::FIXED);

	vector<srLink> linkt;
	linkt.resize(2);
	srLink linktt;
	linkt[0] = linktt;
	linkt[1] = linktt;
	srLink* linkt2 = new srLink();

	double size = 0.1;
	link2->GetGeomInfo().SetDimension(Vec3(size));
	coll2->GetGeomInfo().SetDimension(Vec3(size));
}

void testForce()
{
	vector<Vec3> pos(0);
	for (int i = 0; i < 5; i++)
		pos.push_back(Vec3(0.0225, -0.025 + 0.01*i, 0.0));

	for (int i = 0; i < 5; i++)
		pos.push_back(Vec3(0.0225 - 0.009*i, 0.025, 0.0));

	for (int i = 0; i < 5; i++)
		pos.push_back(Vec3(-0.0225, 0.025 - 0.01*i, 0.0));

	for (int i = 0; i < 5; i++)
		pos.push_back(Vec3(-0.0225 + 0.009*i, -0.025, 0.0));
	force.resize(pos.size());
	double fz = -10.0;
	for (unsigned int i = 0; i < force.size(); i++)
	{
		force[i][0] = pos[i][1] * fz;
		force[i][1] = - pos[i][0] * fz;
		force[i][2] = 0.0;
		force[i][3] = 0.0;
		force[i][4] = 0.0;
		force[i][5] = fz;
	}
}
