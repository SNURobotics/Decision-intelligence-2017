#include <cstdio>

#include "serverRenderer.h"
#include "simulationEnvSetting.h"
#include "robotManager\robotRRTManager.h"
#include "ForceCtrlManager\hybridPFCtrlManager.h"
#include "Math\Spline.h"
#include "common\dataIO.h"
#include <time.h>
#include <direct.h>

// Environment
srLink* busbarlink = new srLink;
srSystem* targetObj = new srSystem;
int holeNum = 6;		// from 0 ~ 7

srSystem* obs = new srSystem;

Eigen::VectorXd jointVal(6);

hybridPFCtrlManager_6dof* hctrl = new hybridPFCtrlManager_6dof();
vector<SE3> Tdes;

// save variables
vector<Eigen::VectorXd> contactFTrj(0);
vector<Eigen::VectorXd> sensorFTrj(0);
vector<Eigen::VectorXd> robotEndSE3Trj_robotbase(0);
vector<Eigen::VectorXd> busbarSE3Trj_robotbase(0);
vector<Eigen::VectorXd> jointTrj(0);
vector<Eigen::VectorXd> loadJointVal(0);
int dataSaving_nStep = 5;
SE3 goalJigSE3;
SE3 holeSE3;
SE3 initOffsetSE3fromHole;


serverRenderer* renderer;

void rendering(int argc, char **argv);
void updateFunc();
void updateFuncPegInHole();
void updateFuncRandom();
void updateFuncTest();
void updateFuncTest2();
void updateFuncDefault();
void updateFuncLoadJointVal();
void setObject(srSystem* system, Vec3 dim, SE3 T = SE3(), srSystem::BASELINKTYPE basetype = srSystem::BASELINKTYPE::FIXED);
void connectBusbarToRobotRigidly(srSystem* object, int robotNum);
void setHybridPFCtrl();
void initializePositionCtrl();
void generateRefTraj(SE3 init, Vec3 goal);
void environmentSetting_HYU2(srSystem* object, bool connectStageBusbarBase = false);
void setInitialConfig();
SE3 setRandomDesSE3(SE3 holeSE3, double xrange = 0.04, double yrange = 0.04, double zrotrange = 1.0);

SE3 busbar_holeSE3;
vector<SE3> finalOffset(4);

int main(int argc, char **argv)
{
	srand(time(NULL));
	// environment
	workspaceSetting();
	busbar.resize(2);
	for (unsigned int i = 0; i < busbar.size(); i++)
	{
		busbar[i] = new BusBar_HYU;
		busbar[i]->SetBaseLinkType(srSystem::FIXED);
	}
	busbar[1]->setBaseLinkFrame(SE3(Vec3(0.0, 0.0, -0.5)));
	//gSpace.AddSystem((srSystem*)busbar[1]);
	targetObj = busbar[0];
	environmentSetting_HYU2(false, true);		// targetObj is assumed to be rigidly attached to robot end-effector
	connectBusbarToRobotRigidly(targetObj, 1);
	robotSetting();
	//setObject(obs, Vec3(0.5, 0.5, 0.5));

	// initialize srLib
	initDynamics();

	busbarlink = targetObj->GetBaseLink()->m_ChildLinks[0];
	// robot manager setting
	robotManagerSetting();
	
	// workcell robot initial config
	rManager2->setJointVal(robot2->homePos);
	Eigen::VectorXd gripInput(2);
	gripInput[0] = -0.009;
	gripInput[1] = 0.009;
	rManager1->setGripperPosition(gripInput);
	rManager2->setGripperPosition(gripInput);

	
	busbar_holeSE3 = jigAssem->GetBaseLink()->GetFrame() * jigAssem->holeCenter[holeNum] * Thole2busbar;

	setInitialConfig();




	/////////////////////////////// read from text
	//printf("datanum:");		//12, 15, 18, 21, 24
	//int datanum;
	//cin >> datanum;
	//string dir_folder = "../../../data/HYU_data2/failure_data" + to_string(datanum);
	//loadJointVal = loadDataFromText(dir_folder + "/jointValTraj.txt", 6);
	/////////////////////////////// plot contact
	//hctrl->T_des_trj[0] = SE3(Vec3(0.0, 0.0, 0.0))*TbusbarInit;
	//hctrl->SelectMtx = Eigen::VectorXd();
	//////////////////////////////////////////////////////////

	rendering(argc, argv);

	return 0;
}

void rendering(int argc, char **argv)
{
	renderer = new serverRenderer();

	SceneGraphRenderer::NUM_WINDOWS windows;

	windows = SceneGraphRenderer::SINGLE_WINDOWS;

	renderer->InitializeRenderer(argc, argv, windows, false);
	renderer->InitializeNode_1st(&gSpace);
	renderer->InitializeNode_2nd();
	//renderer->setUpdateFunc(updateFunc);
	//renderer->setUpdateFunc(updateFuncLoadJointVal);
	//renderer->setUpdateFunc(updateFuncTest2);
	renderer->setUpdateFunc(updateFuncPegInHole);
	renderer->RunRendering();
}


void updateFunc()
{
	static int folder_num = 100;
	static unsigned int taskIdx = 0;
	static bool isDataSaved = false;
	static bool collideStarted = false;
	static int cnt = 0;
	static int cnt_f = 0;
	static int cnt_f_temp = 0;
	static int randomIdx = 0;
	static int cnt_random = 0;
	static bool insertInitiated = false;
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
	cnt++;
	hctrl->hybridPFControl();
	
	rManager2->setJointVal(robot2->homePos);

	// task0: go to initial contact point
	if (taskIdx == 0 && Norm(hctrl->T_des_trj[0].GetPosition() - busbarlink->GetPosition()) < 0.0001)
	{
		taskIdx = 10;
		//fake_z = 0.01 * abs(hctrl->Fext_des_trj[0][5]) / hctrl->Kp_v(5, 5);
		//hctrl->T_des_trj[0] = SE3(Vec3(0.0, 0.0, -fake_z))*hctrl->T_des_trj[0];
	}
	if (taskIdx == 10)
		cnt_f_temp++;
	//cnt_f_temp = 51;
	// task1: make initial contact
	if (taskIdx == 10 && cnt_f_temp > 50)
	{
		taskIdx = 1;
		Eigen::MatrixXd S2 = Eigen::MatrixXd::Zero(3, 6);
		S2(0, 0) = 1.0;
		S2(1, 1) = 1.0;
		S2(2, 5) = 1.0;
		hctrl->setSelectionMatrix(S2);
	}
	if (taskIdx == 1 && abs(targetObj->GetBaseLink()->m_ConstraintImpulse[5] * (1.0 / gSpace.m_Timestep_dyn_fixed) + hctrl->Fext_des_trj[0][5]) < 0.3*abs(hctrl->Fext_des_trj[0][5]))
	{
		collideStarted = true;
	}
	if (collideStarted)
		cnt_f++;
	// task: random move
	if (taskIdx == 1 && cnt_f > 50)
	{
		if (cnt_random % 200 == 0)
		{
			randomIdx++;
			hctrl->T_des_trj[0] = setRandomDesSE3(busbar_holeSE3, 0.01, 0.01, 0.25);
			hctrl->X_int = se3(0.0);
			Eigen::MatrixXd S2 = Eigen::MatrixXd::Zero(1, 6);
			S2(0, 5) = 1.0;
			Eigen::MatrixXd S = Eigen::MatrixXd::Zero(3, 6);
			S(0, 0) = 1.0; S(1, 1) = 1.0; S(2, 5) = 1.0;
			hctrl->setSelectionMatrix(S2);
		}
		cnt_random++;
	}
	//// task2: go to point before goal
	//if (randomIdx == 5 && taskIdx == 1)
	//{
	//	Eigen::MatrixXd S2 = Eigen::MatrixXd::Zero(1, 6);
	//	S2(0, 5) = 1.0;
	//	Eigen::MatrixXd S = Eigen::MatrixXd::Zero(3, 6);
	//	S(0, 0) = 1.0; S(1, 1) = 1.0; S(2, 5) = 1.0;
	//	//hctrl->setSelectionMatrix(S);
	//	hctrl->setSelectionMatrix(S2);
	//	//hctrl->setSelectionMatrix(Eigen::MatrixXd());
	//	taskIdx = 2;
	//	hctrl->T_des_trj[0] = finalOffset[0] * jigAssem->GetBaseLink()->GetFrame() * jigAssem->holeCenter[holeNum] * Thole2busbar;
	//	cnt_f_temp = 0;
	//}
	//if (taskIdx == 2)
	//	cnt_f_temp++;
	//// task3: go to goal location
	//if (taskIdx == 2 && Norm(hctrl->T_des_trj[0].GetPosition() - busbarlink->GetPosition()) < 0.001 && cnt_f_temp > 50)
	//{
	//	taskIdx = 3;
	//	Eigen::MatrixXd S2 = Eigen::MatrixXd::Zero(1, 6);
	//	S2(0, 5) = 1.0;
	//	hctrl->setSelectionMatrix(S2);
	//	//hctrl->setSelectionMatrix(Eigen::MatrixXd());
	//	taskIdx = 3;
	//	hctrl->T_des_trj[0] = jigAssem->GetBaseLink()->GetFrame() * jigAssem->holeCenter[holeNum] * Thole2busbar;
	//	cnt_f_temp = 0;
	//}
	//if (taskIdx == 3)
	//	cnt_f_temp++;
	Vec3 err = hctrl->T_des_trj[0].GetPosition() - busbarlink->GetPosition();
	if (err[2] > 0.003)
		insertInitiated = true;

	// task4: go to final pos
	if (insertInitiated || (taskIdx == 3 && Norm(hctrl->T_des_trj[0].GetPosition() - busbarlink->GetPosition()) < 0.001 && cnt_f_temp > 50))
	{
		taskIdx = 4;
		hctrl->SelectMtx = Eigen::MatrixXd();
		hctrl->T_des_trj[0] = SE3(Vec3(0.0, 0.0, -0.025)) * jigAssem->GetBaseLink()->GetFrame() * jigAssem->holeCenter[holeNum] * Thole2busbar;
	}

	// gather data /////////////////////////////
	if (cnt % dataSaving_nStep == 1)
	{
		sensorFTrj.push_back(dse3toVector(rManager1->readSensorValue()));
		contactFTrj.push_back(dse3toVector(busbarlink->m_ConstraintImpulse * (1.0 / gSpace.m_Timestep_dyn_fixed)));
		robotEndSE3Trj_robotbase.push_back(SE3toVectorXd(robot1->GetBaseLink()->GetFrame() % robot1->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame()));
		busbarSE3Trj_robotbase.push_back(SE3toVectorXd(robot1->GetBaseLink()->GetFrame() % busbarlink->GetFrame()));
		jointTrj.push_back(rManager1->getJointVal());

		cout << taskIdx << endl;
		cout << "force: " << targetObj->GetBaseLink()->m_ConstraintImpulse * (1.0 / gSpace.m_Timestep_dyn_fixed) << endl;
		//cout << "error: " << Log(hctrl->T_des_trj[0] % (hctrl->m_endeffector->GetFrame()*hctrl->m_offset)) << endl;
		cout << "error to des : " << (hctrl->m_endeffector->GetFrame()*hctrl->m_offset).GetPosition() - hctrl->T_des_trj[0].GetPosition() << endl;
		cout << "error to hole: " << (hctrl->m_endeffector->GetFrame()*hctrl->m_offset).GetPosition() - busbar_holeSE3.GetPosition() << endl;
	}


	if (taskIdx == 4 && !isDataSaved && abs(hctrl->T_des_trj[0].GetPosition()[2] - (hctrl->m_endeffector->GetFrame()*hctrl->m_offset).GetPosition()[2]) < 0.0005)
	{
		vector<Eigen::VectorXd> setting_robotbase(0);
		setting_robotbase.push_back(SE3toVectorXd(robot1->GetBaseLink()->GetFrame() % goalJigSE3));
		setting_robotbase.push_back(SE3toVectorXd(robot1->GetBaseLink()->GetFrame() % holeSE3));
		setting_robotbase.push_back(SE3toVectorXd(initOffsetSE3fromHole));
		string dir_folder = "../../../data/HYU_data2/failure_data" + to_string(folder_num);
		if (folder_num < 51 && _mkdir(dir_folder.c_str()) == 0)
		{
			string dir_temp = dir_folder;
			saveDataToText(setting_robotbase, dir_temp.append("/setting_robotbase").append(".txt"));
			dir_temp = dir_folder;
			saveDataToText(sensorFTrj, dir_temp.append("/sensorValTraj").append(".txt"));
			dir_temp = dir_folder;
			saveDataToText(contactFTrj, dir_temp.append("/contactFValTraj").append(".txt"));
			dir_temp = dir_folder;
			saveDataToText(jointTrj, dir_temp.append("/jointValTraj").append(".txt"));
			dir_temp = dir_folder;
			saveDataToText(robotEndSE3Trj_robotbase, dir_temp.append("/robotEndTraj_robotbase").append(".txt"));
			dir_temp = dir_folder;
			saveDataToText(busbarSE3Trj_robotbase, dir_temp.append("/busbarTraj_robotbase").append(".txt"));
			isDataSaved = true;
		}
		
	}
	if (isDataSaved)
	{
		setInitialConfig();
		rManager1->controlJointTorque(Eigen::VectorXd::Zero(6));
		taskIdx = 0;
		isDataSaved = false;
		collideStarted = false;
		cnt = 0;
		cnt_f = 0;
		cnt_f_temp = 0;
		randomIdx = 0;
		cnt_random = 0;
		insertInitiated = false;
		folder_num++;
		sensorFTrj.resize(0);
		contactFTrj.resize(0);
		jointTrj.resize(0);
		robotEndSE3Trj_robotbase.resize(0);
		busbarSE3Trj_robotbase.resize(0);
	}
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
	static int cnt = 0;
	if (cnt == 0)
		jointVal = Eigen::VectorXd::Random(6);
	cnt++;
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
	Eigen::VectorXd jointVel = Eigen::VectorXd::Random(6);
	static double dt = 0.0001;
	static Eigen::MatrixXd J_bf = Eigen::MatrixXd::Zero(6, 6);
	static se3 V_bf = se3(0.0);
	
	rManager1->setJointValVel(jointVal, jointVel);
	Eigen::MatrixXd J = rManager1->getBodyJacobian(jointVal, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper_new));
	Eigen::MatrixXd Jdot = rManager1->getBodyJacobianDot(jointVal, jointVel, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper_new));
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
	hctrl->isSystemSet = hctrl->setSystem((robotManager*) rManager1, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper_new), busbarlink);
	hctrl->setTimeStep(rManager1->m_space->m_Timestep_dyn_fixed);
	double kv_v = 0.25e2, kp_v = 0.25*kv_v*kv_v, ki_v = 0.25e3, kp_f = 1.0e-1, ki_f = 5.0e-1;
	hctrl->setGain(kv_v, kp_v, ki_v, kp_f, ki_f);
	//hctrl->Kp_v = kp_v * Eigen::MatrixXd::Identity(6, 6);
	//hctrl->Kv_v = kv_v * Eigen::MatrixXd::Identity(6, 6);
	//hctrl->Ki_v = ki_v * Eigen::MatrixXd::Identity(6, 6);
	//hctrl->Ki_f = ki_f * Eigen::MatrixXd::Identity(6, 6);
	//hctrl->Kp_f = kp_f * Eigen::MatrixXd::Identity(6, 6);

	// S*V = 0 should be satisfied
	// pos controlled dir: trans x, y, rot z
	// force controlled dir: moment x, y, force z
	Eigen::MatrixXd S = Eigen::MatrixXd::Zero(3, 6);
	S(0, 0) = 1.0;
	S(1, 1) = 1.0;
	S(2, 5) = 1.0;
	Eigen::MatrixXd S2 = Eigen::MatrixXd::Zero(1, 6);
	S2(0, 5) = 1.0;
	
	//hctrl->setSelectionMatrix(S);					// control rot_z, pos_xy, m_xy, f_z
	hctrl->setSelectionMatrix(S2);					// control rot_xyz, pos_xy, f_z
	//hctrl->setSelectionMatrix(Eigen::MatrixXd());	// control rot_xyz, pos_xyz
}



void updateFuncRandom()
{
	static int cnt = 0;
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
	cnt++;





}

void updateFuncTest2()
{
	static int cnt = 0;
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
	
	if (cnt == 1)
	{
		//Eigen::MatrixXd S = Eigen::MatrixXd::Zero(1, 6);
		//S(0, 5) = 1;
		//hctrl->SelectMtx = S;
		//hctrl->T_des_trj[0] = SE3(Vec3(0.01, 0.0, 0.0))*hctrl->T_des_trj[0];
	}
	cnt++;
	hctrl->hybridPFControl();
	rManager2->setJointVal(robot2->homePos);
	if (cnt % 100 == 0)
	{
		printf("simulation step: %d\n", cnt);
		printf("contact force: ");
		cout << hctrl->m_contactLinks[0]->m_ConstraintImpulse * (1.0 / gSpace.m_Timestep_dyn_fixed) << endl;
		//cout << robot1->gMarkerLink[Indy_Index::MLINK_GRIP].m_ConstraintImpulse << endl;
		//printf("sensor value: ");
		//cout << rManager1->readSensorValue() << endl;
		printf("tau: ");
		cout << rManager1->getJointCommand().transpose() << endl;
		printf("tau_g: ");
		Eigen::VectorXd tau_g = rManager1->inverseDyn(rManager1->getJointVal(), Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6));
		cout << tau_g.transpose() << endl;
		printf("tau_s: ");
		Eigen::MatrixXd Jb = rManager1->getBodyJacobian(rManager1->getJointVal(), &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper_new));
		cout << (tau_g + Jb.transpose() * dse3toVector(hctrl->Fext_des_trj[0])).transpose() << endl;
	}
}

void updateFuncDefault()
{
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
	rManager2->setJointVal(robot2->homePos);
	rManager1->setJointVal(robot1->homePos);
}

void updateFuncLoadJointVal()
{
	static int cnt = 0;
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
	rManager2->setJointVal(robot2->homePos);
	rManager1->setJointVal(loadJointVal[cnt % loadJointVal.size()]);
	cnt++;

}

void generateRefTraj(SE3 init, Vec3 goal)
{
	double t_f = 10.0;
	vector<double> tspan(0);
	double t = 0.0;
	tspan.push_back(t);
	while (1)
	{
		t += gSpace.m_Timestep_dyn_fixed;
		tspan.push_back(t);
		if (t > t_f)
			break;
	}
	tspan.push_back(t_f);
	cubicSpline* cspline = new cubicSpline;

	vector<double> time_cp(3);
	vector<Eigen::VectorXd> cp(3);
	time_cp[0] = tspan[0];
	time_cp[1] = 0.5*(tspan[0] + t_f);
	time_cp[2] = t_f;
	cp[0] = Vec3toVector(init.GetPosition());
	cp[1] = Vec3toVector(0.5*(init.GetPosition() + goal));
	cp[2] = Vec3toVector(goal);
	cspline->interpolation(time_cp, cp);
	
	// set Tdes
	Tdes.resize(tspan.size());
	Eigen::VectorXd temp;
	for (unsigned int i = 0; i < Tdes.size(); i++)
	{
		temp = cspline->getPosition(tspan[i]);
		Tdes[i] = SE3(init.GetOrientation(), Vec3(temp[0], temp[1], temp[2]));
	}
}

void setInitialConfig()
{
	SE3 TgoalPos = jigAssem->GetBaseLink()->GetFrame() * jigAssem->holeCenter[holeNum] * Thole2busbar;

	// set initial config
	double xpos = (double)0.04*rand() / RAND_MAX - 0.02;
	double ypos = (double)0.04*rand() / RAND_MAX - 0.02;
	double zpos = (double)0.1*rand() / RAND_MAX;
	double xrot = (double)0.05*rand() / RAND_MAX - 0.025;
	double yrot = (double)0.05*rand() / RAND_MAX - 0.025;
	double zrot = (double)0.05*rand() / RAND_MAX - 0.025;
	SE3 initPosOffset = SE3(Vec3(xpos, ypos, 0.01));
	SE3 TbusbarInit = initPosOffset * TgoalPos * EulerZYX(Vec3(zrot, 0.0, 0.0), Vec3(0.0, 0.0, 0.0));
	int flag;

	// initial condition 1
	Eigen::VectorXd qInit = Eigen::VectorXd::Zero(6);
	// elbow up
	qInit[1] = -0.65*SR_PI; 	qInit[2] = 0.3*SR_PI; 	qInit[3] = 0.5*SR_PI_HALF;
	// initial condition 2
	Eigen::VectorXd qInit2 = Eigen::VectorXd::Zero(6);
	qInit2[0] = -0.224778; qInit2[1] = -1.91949; qInit2[2] = -0.384219; qInit2[3] = 1.5708; qInit2[4] = -0.73291; qInit2[5] = 1.79557;
	Eigen::VectorXd q_config = rManager1->inverseKin(TbusbarInit * Tbusbar2gripper_new, rManager1->m_activeArmInfo->m_endeffector[0], true, SE3(), flag, robot1->qInvKinInit, 1000);
	cout << flag << endl;
	rManager1->setJointVal(q_config);

	setHybridPFCtrl();

	// set desired trajectory (trajectory of the busbar)
	//generateRefTraj(TbusbarInit, TgoalPos.GetPosition());
	Tdes.resize(1);
	TgoalPos = SE3(Vec3(initPosOffset.GetPosition()[0], initPosOffset.GetPosition()[1], 0.0)) * TgoalPos;
	Tdes[0] = TgoalPos;

	vector<dse3> Fdes(1, dse3(0.0));		// expressed in end-effector frame
	Fdes[0][0] = 0.00;
	Fdes[0][1] = 0.0;
	Fdes[0][5] = -50.0;


	hctrl->isDesTrjSet = hctrl->setDesiredTraj(Tdes, Fdes);
	hctrl->setDesiredJointVal(q_config);
	hctrl->F_int = dse3(0.0);
	hctrl->X_int = se3(0.0);
	rManager1->setJointValVelAcc(q_config, Eigen::VectorXd::Zero(q_config.size()), Eigen::VectorXd::Zero(q_config.size()));

	// saving setting
	goalJigSE3 = jigAssem->GetBaseLink()->GetFrame();
	holeSE3 = jigAssem->GetBaseLink()->GetFrame() * jigAssem->holeCenter[holeNum];
	initOffsetSE3fromHole = holeSE3 % TbusbarInit;
}

SE3 setRandomDesSE3(SE3 holeSE3, double xrange, double yrange, double zrotrange)
{
	// random x, y pos, z rot
	double xpos = (double)xrange*rand() / RAND_MAX - 0.5*xrange;
	double ypos = (double)yrange*rand() / RAND_MAX - 0.5*yrange;
	double zrot = (double)zrotrange*rand() / RAND_MAX - 0.5*zrotrange;
	
	SE3 Trandom = SE3(Vec3(xpos, ypos, 0.0)) * holeSE3 * EulerZYX(Vec3(zrot, 0.0, 0.0), Vec3(0.0, 0.0, 0.0));
	return Trandom;
}

void connectBusbarToRobotRigidly(srSystem * object, int robotNum)
{
	srWeldJoint* wobjJoint = new srWeldJoint;
	if (robotNum == 1)
		wobjJoint->SetParentLink(&robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
	else
		wobjJoint->SetParentLink(&robot2->gMarkerLink[Indy_Index::MLINK_GRIP]);
	wobjJoint->SetChildLink(object->GetBaseLink());
	wobjJoint->SetParentLinkFrame(SE3());
	wobjJoint->SetChildLinkFrame(Tbusbar2gripper_new);
	busbarlink = object->GetBaseLink();
}

void initializePositionCtrl()
{
	hctrl->X_int = se3(0.0);
	hctrl->SelectMtx = Eigen::MatrixXd();		// position control
}

void updateFuncPegInHole()
{
	/////////////////////////////////////////// initial setting ////////////////////////////////////////////////////
	static unsigned int taskIdx = 0;								// index of task
	static double moveRange = 0.03;									// range of movement

	
	static vector<int> moveIdx(10, -1);								// direction to move busbar for each task (-1: ignore, 9: x, 10: y, 11: z)
	moveIdx[2] = 10;
	moveIdx[4] = 9;

	static vector<int> forceCheckIdx(10, -1);						// direction to check force for each task (-1: ignore, 3: f_x, 4: f_y, 5: f_z)
	forceCheckIdx[2] = 4;
	forceCheckIdx[4] = 3;

	static vector<Vec3> moveDir(moveIdx.size(), Vec3(0.0));			// direction to move busbar for each task
	for (unsigned int i = 0; i < moveIdx.size(); i++)
	{
		if (moveIdx[i] == 9)
			moveDir[i] = Vec3(1.0, 0.0, 0.0);
		else if (moveIdx[i] == 10)
			moveDir[i] = Vec3(0.0, 1.0, 0.0);
		else if (moveIdx[i] == 11)
			moveDir[i] = Vec3(0.0, 0.0, 1.0);
	}

	static vector<bool> dirChanged(moveIdx.size(), false);			// denote if moving direction is changed

	static vector<int> task_cnt(moveIdx.size(), 0);					// count control step for each task

	static int cnt = 0;												// count of simulation step
	static SE3 initialGoal;											// initial goal of position control
	static SE3 currentGoal;											// current goal of position control
	static double sign = -1.0;										// sign of force check direction (1: +dir, -1: -dir), change value when direction changes
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	

	// forward simulation step (should not be changed)
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();						
	if (cnt == 0)
		initialGoal = hctrl->T_des_trj[0];
	cnt++;

	// hybrid position/force control (give joint command to robot)
	hctrl->hybridPFControl();
	Eigen::VectorXd tau = rManager1->getJointCommand();
	// cout << tau.transpose() << endl;		// print joint command

	// hold robot2 in home position
	rManager2->setJointVal(robot2->homePos);
	task_cnt[taskIdx]++;


	/////////////////////////////////////////////////////// SELECT DESIRED POSITION (AND FORCE) FOR PEG-IN-HOLE TASK ///////////////////////////////////////////////////////
	// numbers in stop conditions are parameters to be tuned... (except taskIdx)
	// task0: go to initial contact point
	if (taskIdx == 0 && task_cnt[taskIdx] > 10 && (Norm(hctrl->T_des_trj[0].GetPosition() - busbarlink->GetPosition()) < 0.0001 || abs(busbarlink->m_ConstraintImpulse[5]) / gSpace.m_Timestep_dyn_fixed > 0.01 ))
	{
		taskIdx++;
		// task 1: tilt end-effector along x-axis
		currentGoal = initialGoal * EulerZYX(Vec3(0.0, 0.0, DEG2RAD(7.0)), Vec3(0.0, 0.0, 0.0));
		hctrl->T_des_trj[0] = currentGoal;
		initializePositionCtrl();		// only position control
	}
	
	if (taskIdx == 1 && task_cnt[taskIdx] > 100 && Norm(Log(Inv(busbarlink->GetFrame().GetOrientation()) * hctrl->T_des_trj[0].GetOrientation())) < 0.03/* && Norm(busbarlink->GetPosition() - hctrl->T_des_trj[0].GetPosition()) < 0.05*/)
	{
		taskIdx++;
		// task 2: move back and forth along y-axis and stop if force in y-axis is big
		currentGoal = SE3(moveRange * moveDir[taskIdx] + Vec3(0.0, 0.0, -0.001)) * currentGoal;
		hctrl->T_des_trj[0] = currentGoal;
		initializePositionCtrl();		// only position control
	}
	if (taskIdx == 2 && task_cnt[taskIdx] > 100 && !dirChanged[taskIdx] && (task_cnt[taskIdx] > 5000 || abs(busbarlink->GetFrame()[moveIdx[taskIdx]] - currentGoal[moveIdx[taskIdx]]) < 0.001))
	{
		// task 2-1: change direction
		currentGoal = SE3(-2.0*moveRange * moveDir[taskIdx]) * currentGoal;
		hctrl->T_des_trj[0] = currentGoal;
		initializePositionCtrl();		// only position control
		task_cnt[2] = 0;
		dirChanged[taskIdx] = true;
		sign = 1.0;
	}
	if (taskIdx == 2 && task_cnt[taskIdx] > 100 && sign * (busbarlink->m_ConstraintImpulse[forceCheckIdx[taskIdx]]) / gSpace.m_Timestep_dyn_fixed > 1.0)
	{
		taskIdx++;
		// task 3: tilt end-effector to be near to original orientation while maintaining current position
		currentGoal.SetOrientation(initialGoal.GetOrientation() * RotX(DEG2RAD(-3.0)).GetOrientation());
		currentGoal.SetPosition(busbarlink->GetFrame().GetPosition());
		hctrl->T_des_trj[0] = currentGoal;
		initializePositionCtrl();		// only position control
	}
	if (taskIdx == 3 && task_cnt[taskIdx] > 10 && Norm(Log(Inv(busbarlink->GetFrame().GetOrientation()) * hctrl->T_des_trj[0].GetOrientation())) < 0.01)
	{
		taskIdx++;
		// task 4: move back and forth along x-axis and stop if force in x-axis is big
		currentGoal = SE3(moveRange * moveDir[taskIdx]) * currentGoal;
		hctrl->T_des_trj[0] = currentGoal;
		initializePositionCtrl();		// only position control
		sign = -1.0;
	}
	if (taskIdx == 4 && task_cnt[taskIdx] > 100 && !dirChanged[taskIdx] && (task_cnt[taskIdx] > 5000 || abs(busbarlink->GetFrame()[moveIdx[taskIdx]] - currentGoal[moveIdx[taskIdx]]) < 0.001))
	{
		// task 4-1: change direction
		currentGoal = SE3(-2.0*moveRange * moveDir[taskIdx]) * currentGoal;
		hctrl->T_des_trj[0] = currentGoal;
		initializePositionCtrl();		// only position control
		task_cnt[4] = 0;
		dirChanged[taskIdx] = true;
		sign = 1.0;
	}
	if (taskIdx == 4 && task_cnt[taskIdx] > 100 && sign * (busbarlink->m_ConstraintImpulse[forceCheckIdx[taskIdx]]) / gSpace.m_Timestep_dyn_fixed > 1.0)
	{
		taskIdx++;
		// task 5: rotate to original orientation while maintaining current position
		currentGoal.SetOrientation(initialGoal.GetOrientation());
		currentGoal.SetPosition(busbarlink->GetFrame().GetPosition());
		hctrl->T_des_trj[0] = currentGoal;
		initializePositionCtrl();		// only position control
	}
	if (taskIdx == 5 && task_cnt[taskIdx] > 10 && distSE3(currentGoal, busbarlink->GetFrame()))
	{
		taskIdx++;
		// task 6: insert peg
		currentGoal = SE3(Vec3(0.0, 0.0, -0.03)) * currentGoal;
		hctrl->T_des_trj[0] = currentGoal;
		initializePositionCtrl();		// only position control
	}
	if (taskIdx == 6 && task_cnt[taskIdx] > 10 && abs(busbarlink->m_ConstraintImpulse[5]) / gSpace.m_Timestep_dyn_fixed > 1.0)
	{
		taskIdx++;
		printf("task finished\n");
		currentGoal = busbarlink->GetFrame();
		hctrl->T_des_trj[0] = currentGoal;
		initializePositionCtrl();		// only position control
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}
