#include <cstdio>

#include "myRenderer.h"
#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\indyRobotManager.h"
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
srJoint::ACTTYPE actType = srJoint::ACTTYPE::TORQUE;
srJoint::ACTTYPE actType_grip = srJoint::ACTTYPE::HYBRID;
indyRobotManager* rManager1;
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
srPrismaticJoint* pjoint1 = new srPrismaticJoint();
srPrismaticJoint* pjoint2 = new srPrismaticJoint();
srWeldJoint* wjoint1 = new srWeldJoint();

vector<Eigen::VectorXd> forceData(0);
vector<Eigen::VectorXd> Tdata(0);

vector<dse3> force;

void testObj();
void testObj4();		// self colli test b/w hybrid joint and torque joint
void testForce();

// simulation setting
bool useFTsensor = true;
bool simulateInteraction = true;

int main(int argc, char **argv)
{
	//srand((unsigned int)time(0));
	robotSetting();
	environmentSetting();
	//testObj();
	
	// initialize srLib
	initDynamics();

	
	// robotManager
	robotManagerSetting();
	int flag = 0;
	Eigen::VectorXd initGuess = Eigen::VectorXd::Zero(6);
	initGuess[0] = -0.57; initGuess[3] = -1.57; initGuess[4] = 1.53; initGuess[5] = -2.14;
	initSE3[0] = goalSE3[0]* SE3(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.01, 0.01, 0.01 + 0.05)));
	Eigen::VectorXd q0 = rManager1->inverseKin(initSE3[0] * Tbusbar2gripper, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flag, initGuess);
	//printf("inv kin flag: %d\n", flag);
	//q0.setZero();
	rManager1->setJointVal(q0);
	
	Eigen::VectorXd gripPos(2);
	gripPos[0] = -0.008;
	gripPos[1] = 0.008;
	rManager1->setGripperPosition(gripPos);

	// impedance control
	busbar[0]->GetBaseLink()->SetFrame(initSE3[0]);
	SE3 Tgoalpos = goalSE3[0];// *SE3(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.01, 0.01, 0.01)));
	setImpede(initSE3[0] * Tbusbar2gripper, Tgoalpos, Inv(Tbusbar2gripper), busbar[0]);

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
	gSpace.SetGravity(0.0, 0.0, -0.0);
	gSpace.SetNumberofSubstepForRendering(1);
	gSpace.DYN_MODE_PRESTEP();
}

void updateFunc()
{
	static bool colli_bf = false;
	if (!simulateInteraction)
	{
		Eigen::VectorXd gripInput(2);
		gripInput[0] = -1.0;
		gripInput[1] = 1.0;
		rManager1->setGripperInput(1.0*gripInput);
	}
	
	//cout << rManager1->getGripperPosition().transpose() << endl;
	//static double a = 0;
	//a += 0.01;
	//Eigen::VectorXd posInput(2);
	//posInput[0] = -0.009 - 0.1*sin(a);
	//posInput[1] = 0.009;
	//for (unsigned int i = 0; i < rManager1->m_gripperInfo->m_gripJoint.size(); i++)
	//	((srStateJoint*)rManager1->m_gripperInfo->m_gripDummyJoint[i])->m_State.m_rCommand = 0.0;
	//for (unsigned int i = 0; i < rManager1->m_gripperInfo->m_gripJoint.size(); i++)
	//	((srStateJoint*)rManager1->m_gripperInfo->m_gripJoint[i])->m_State.m_rValue[0] = posInput(i);


	//static int cnt = 0, stop = 0, goalIdx = 0, stopIdx = 0, ctrlIdx = 0;
	//cnt++;
	static se3 Vbf(0.0);
	static se3 Vbf_ee(0.0);
	static SE3 r2obj;
	static se3 Acur_mes;
	static se3 Acur;
	static se3 Aee_mes;
	if (true)
	{
		// print impedance control status
		r2obj = rManager1->m_activeArmInfo->m_endeffector[0]->GetFrame() % busbar[0]->GetBaseLink()->GetFrame();
		Acur_mes = (busbar[0]->GetBaseLink()->GetVel() - Vbf) * (1.0 / iManager->timeStep);
		Acur = busbar[0]->GetBaseLink()->GetAcc();
		Aee_mes = InvAd(r2obj, (rManager1->m_activeArmInfo->m_endeffector[0]->GetVel() - Vbf_ee) * (1.0 / iManager->timeStep));
		cout << "Aee : " << Aee_mes;
		cout << "Ames: " << Acur_mes;
		cout << "Acur: " << Acur;
		
		//cout << r2obj;
		iManager->impedanceControl(actType, false, simulateInteraction);
		
		int stop = 1;
	}
	Vbf = busbar[0]->GetBaseLink()->GetVel();
	Vbf_ee = rManager1->m_activeArmInfo->m_endeffector[0]->GetVel();
	//Eigen::VectorXd q0 = Eigen::VectorXd::Zero(6);
	//rManager1->setJointVal(q0);


	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();	
	colli_bf = gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP();

	//if (colli_bf)
	//{
	//	Eigen::VectorXd acc = Eigen::VectorXd::Zero(6);
	//	acc[0] = 0.1;
	//	rManager1->controlJointAcc(acc);
	//}

	//colli_bf = gSpace.m_srDYN.RUNTIME_MARKKH();
	//gSpace.m_srDYN._PRESTEP_Build_Baked_ContactConstraint________MARK7();


	//// test
	//cout << "bf" <<  wjoint1->m_FS_Force;
	//link3->AddUserExternalForce(dse3(0.1));
	//gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
	//cout << wjoint1->m_FS_Force;
	////cout << colli_bf << endl;

}


void environmentSetting()
{
	SE3 tableCenter = SE3(Vec3(0.6, 0.0, 0.25));
	//SE3 tableCenter = SE3(Vec3(0.0, 0.0, 0.0));
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
	vector<int> gpIdx(2);
	gpIdx[0] = 0;
	gpIdx[1] = 1;
	robot1->SetGripperActType(srJoint::ACTTYPE::TORQUE, gpIdx);
	gpIdx[0] = 2;
	gpIdx[1] = 3;
	robot1->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);
	gSpace.AddSystem((srSystem*)robot1);
	//gSpace.AddSystem((srSystem*)robot2);
	robot1->GetBaseLink()->SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(-0.0, 0.0, 0.0)));
	//robot2->GetBaseLink()->SetFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.5, 0.7, 0.0)));
}

void robotManagerSetting()
{
	rManager1 = new indyRobotManager(robot1, &gSpace);
	//rManager1->setRobot((srSystem*)robot1);
	//rManager1->setSpace(&gSpace);
	//rManager1->setEndeffector(&robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
	//vector<srJoint*> gripperJoint(2);
	//gripperJoint[0] = robot1->gPjoint[Indy_Index::GRIPJOINT_L];
	//gripperJoint[1] = robot1->gPjoint[Indy_Index::GRIPJOINT_U];
	//rManager1->setGripper(gripperJoint, &robot1->gLink[Indy_Index::ENDEFFECTOR]);
	//rManager1->setFTSensor(robot1->gWeldJoint[Indy_Index::WELDJOINT_GRIPPER]);

	//rTaskManager = new robotTaskManager(rManager1);
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

	ees[0] = rManager1->m_activeArmInfo->m_endeffector[0];
	r2o[0] = Trobot2obj;
	object1->GetBaseLink()->SetFrame(TrobotInit*r2o[0]);
	gSpace._KIN_UpdateFrame_All_The_Entity_All_The_Systems();
	iManager->setSystem(rManager1, ees, ofs, r2o);
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
	iManager->impedanceControl(actType);
	//iManager->objectImpedanceControl();
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
	wjoint1->SetParentLink(link1);
	wjoint1->SetParentLinkFrame(SE3());
	wjoint1->SetChildLink(link2);
	wjoint1->SetChildLinkFrame(SE3());

	pjoint2->SetParentLink(link2);
	pjoint2->SetParentLinkFrame(SE3());
	pjoint2->SetChildLink(link3);
	pjoint2->SetChildLinkFrame(SE3(Vec3(0.0, 0.0, -0.1)));
	pjoint2->SetActType(srJoint::TORQUE);

	coll1->SetLocalFrame(SE3());
	coll2->SetLocalFrame(SE3(Vec3(0.0, 0.0, 0.0)));
	link2->AddCollision(coll1);
	link3->AddCollision(coll2);

	//link1->GetGeomInfo().SetColor(0.3, 0.3, 0.3);
	link1->GetGeomInfo().SetDimension(Vec3(0.0, 0.0, 0.0));
	link2->GetGeomInfo().SetColor(0.3, 0.3, 0.3);
	link3->GetGeomInfo().SetColor(0.3, 0.3, 0.3);
	pjoint2->m_State.m_rValue[0] = 0.01;
	test->SetBaseLink(link1);
	test->SetBaseLinkType(srSystem::BASELINKTYPE::KINEMATIC);
	test->SetSelfCollision(true);
	gSpace.AddSystem(test);
}

void testObj4()
{
	pjoint1->SetParentLink(link1);
	pjoint1->SetParentLinkFrame(SE3());
	pjoint1->SetChildLink(link2);
	pjoint1->SetChildLinkFrame(SE3());
	pjoint1->SetActType(srJoint::HYBRID);

	pjoint2->SetParentLink(link2);
	pjoint2->SetParentLinkFrame(SE3());
	pjoint2->SetChildLink(link3);
	pjoint2->SetChildLinkFrame(SE3(Vec3(0.0, 0.0, -0.1)));
	pjoint2->SetActType(srJoint::TORQUE);

	coll1->SetLocalFrame(SE3());
	coll2->SetLocalFrame(SE3(Vec3(0.0, 0.0, 0.0)));
	link2->AddCollision(coll1);
	link3->AddCollision(coll2);

	//link1->GetGeomInfo().SetColor(0.3, 0.3, 0.3);
	link1->GetGeomInfo().SetDimension(Vec3(0.0, 0.0, 0.0));
	link2->GetGeomInfo().SetColor(0.3, 0.3, 0.3);
	link3->GetGeomInfo().SetColor(0.3, 0.3, 0.3);
	pjoint2->m_State.m_rValue[0] = 0.01;
	test->SetBaseLink(link1);
	test->SetBaseLinkType(srSystem::BASELINKTYPE::KINEMATIC);
	test->SetSelfCollision(true);
	gSpace.AddSystem(test);
}
