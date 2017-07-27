#include <cstdio>

#include "serverRenderer.h"
#include "simulationEnvSetting.h"
//#include "2ndRenderer.h"

#include "ForceCtrlManager\hybridPFCtrlManager.h"

Eigen::VectorXd jointVal(6);
Eigen::VectorXd jointVal_add(6);
Eigen::VectorXd jointAcc(6);
Eigen::VectorXd jointVel(6);

serverRenderer* renderer;

// Planning
vector<vector<Eigen::VectorXd>> traj(0);
vector<vector<SE3>>	Ttraj(0);


vector<bool> attachObject;

vector<vector<int>> idxTraj(0);
vector<vector<int>> totalFlag(0);
vector<Eigen::VectorXd> initPos(0);
vector<Eigen::VectorXd> goalPos(0);
vector<SE3> wayPoints(0);

Eigen::VectorXd homePos = Eigen::VectorXd::Zero(6);


// Measure F/T sensor
dse3 Ftsensor;
Eigen::VectorXd ftsensor(6);

// save data
vector<vector<Eigen::VectorXd>> FTtraj;
vector<vector<Eigen::VectorXd>> TtrajVec;
vector<vector<Eigen::VectorXd>> TtrajFromJoint(2);

vector<vector<Eigen::VectorXd>> busbarTraj;
vector<Eigen::VectorXd> goalJigLocation(1);

srJoint::ACTTYPE actType = srJoint::ACTTYPE::HYBRID;
hybridPFCtrlManager_6dof* hctrl = new hybridPFCtrlManager_6dof;

void rendering(int argc, char **argv);
void updateFunc();
void updateFuncInput();
void updateFuncTestSensor();
void updateFuncTestSensorToRobot();
void updateFuncLoadJointValAttachStatus();

void setHybridPFCtrl();

Vec2 goalLocation; // busbar insertion location

vector<int> flags(0);
vector<int> flags_add(0);

double planning = 0;
vector<Eigen::VectorXd> loadJointVal(0);
vector<Eigen::VectorXd> loadAttachStatus(0);
vector<Eigen::VectorXd> testJointVal(0);
vector<Eigen::VectorXd> testJointVal2(0);
vector<vector<Eigen::VectorXd>> testJointValVec(2);

Eigen::VectorXd testjointvalue(6);
vector<SE3> busbarSE3set(0);
vector<SphereMarker*> sph(0);
int main(int argc, char **argv)
{
	srand(time(NULL));
	double height = 0.0;

	// Robot home position
	robotSetting(height);
	// environment
	workspaceSetting(height);
	//objectSetting();

	environmentSetting_HYU2(true, true);

	////////////////////////////////////////////////////////////////////////////////////////
	bool testworkspace = true;
	bool testMaxTorque = false;
	bool testRobot1 = true;
	bool testBoth = true;
	double yoffset;
	bool useSphere = true;
	if (testRobot1)
		yoffset = -0.8;
	else
		yoffset = -0.4;
	SE3 Tbase;
	if (workcell_mode == 1)
		Tbase = SE3(Vec3(0.025, 1.095, 1.176));		// when stage attached
	else if (workcell_mode == 2)
		Tbase = SE3(Vec3(0.025, 1.095, 0.910 + 0.099 + 0.009));	// when only stage4 is used
	else
		Tbase = SE3(Vec3(0.025, 1.095, 0.910 + 0.009));		// when stage removed
	SE3 Tbase2jigbase = EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.184));
	int nx = 8; 	int ny = 10; 	int nz = 5;
	double range_x = 1.4; double range_y = 1.2; double range_z = height + 0.5;
	double bin_x = range_x / (double)nx; double bin_y = range_y / (double)ny; double bin_z = range_z / (double)nz;
	busbar.resize(nx*ny*nz);
	sph.resize(nx*ny*nz);
	flags.resize(busbar.size());
	flags_add.resize(busbar.size());
	if (!useSphere)
	{
		for (unsigned int i = 0; i < busbar.size(); i++)
		{
			busbar[i] = new BusBar_HYU;
			busbar[i]->SetBaseLinkType(srSystem::FIXED);
			gSpace.AddSystem(busbar[i]);
		}
	}
	else
	{
		for (unsigned int i = 0; i < sph.size(); i++)
		{
			sph[i] = new SphereMarker(0.03);
			sph[i]->SetBaseLinkType(srSystem::FIXED);
			gSpace.AddSystem(sph[i]);
		}
	}
	
	int l = 0;
	for (int i = 0; i < nx; i++)
	{
		for (int j = 0; j < ny; j++)
		{
			for (int k = 0; k < nz; k++, l++)
			{
				busbarSE3set.push_back(SE3(Vec3((double)i*bin_x - 0.5*range_x + Trobotbase1[9], (double)j*bin_y + yoffset, (double)k*bin_z - 0.05)) * Tbase * Tbase2jigbase);
				if (!useSphere)
					busbar[l]->setBaseLinkFrame(SE3(Vec3((double)i*bin_x - 0.5*range_x + Trobotbase1[9], (double)j*bin_y + yoffset, (double)k*bin_z - 0.05 - 10.0)) * Tbase * Tbase2jigbase);
				else
					sph[l]->setBaseLinkFrame(SE3(Vec3((double)i*bin_x - 0.5*range_x + Trobotbase1[9], (double)j*bin_y + yoffset, (double)k*bin_z - 0.05 - 10.0)) * Tbase * Tbase2jigbase);
			}
		}
	}


	//busbar.resize(16);
	//busbarSE3set.resize(busbar.size());
	//flags.resize(busbar.size());
	//for (unsigned int i = 0; i < busbar.size()/2; i++)
	//{
	//	busbar[2*i] = new BusBar_HYU;
	//	busbar[2*i+1] = new BusBar_HYU;
	//	gSpace.AddSystem(busbar[2*i]);
	//	gSpace.AddSystem(busbar[2 * i + 1]);
	//	busbar[2 * i]->setBaseLinkFrame(SE3(Vec3(0.0, 0.0, -0.025 - 10.0))*Tbase*Tbase2jigbase*jigAssem->holeCenter[i]);
	//	busbar[2 * i + 1]->setBaseLinkFrame(SE3(Vec3(0.0, 0.0, -0.025 - 15.0))*Tbase*Tbase2jigbase*jigAssem->holeCenter[i]);
	//	busbarSE3set[2 * i] = Tbase*Tbase2jigbase*jigAssem->holeCenter[i];
	//	busbarSE3set[2 * i + 1] = SE3(Vec3(0.0, 0.0, -0.025))*Tbase*Tbase2jigbase*jigAssem->holeCenter[i];
	//}

	
	///////////////////////////////////////////////////////////////////
	

	// initialize srLib
	initDynamics();

	// robot manager setting
	robotManagerSetting();
	rManager1->setJointVal(robot1->homePos);
	rManager2->setJointVal(robot2->homePos);

	cout << gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP() << endl;
	Eigen::VectorXd checkPos1 = Eigen::VectorXd::Zero(6);
	checkPos1[1] = -SR_PI_HALF; checkPos1[2] = DEG2RAD(0.0); checkPos1[3] = SR_PI_HALF; checkPos1[4] = DEG2RAD(-90);

	Eigen::VectorXd checkPos2 = Eigen::VectorXd::Zero(6);
	checkPos2[1] = DEG2RAD(30 - 30); checkPos2[2] = DEG2RAD(-220 + 30); checkPos2[3] = DEG2RAD(90); checkPos2[4] = DEG2RAD(-100);

	//////// test inverse kin of robot1
	Eigen::VectorXd maxTorque2Pos;
	Eigen::VectorXd maxTorque3Pos;
	Eigen::VectorXd maxTorque2 = Eigen::VectorXd();
	Eigen::VectorXd maxTorque3 = Eigen::VectorXd();
	Eigen::VectorXd testJointTorque;
	Eigen::VectorXd Zerovec = Eigen::VectorXd::Zero(6);
	indyRobotManager* mainRobotManager;
	indyRobotManager* subRobotManager;
	IndyRobot* mainRobot;
	IndyRobot* subRobot;
	if (testRobot1)
	{
		mainRobotManager = rManager1;
		mainRobot = robot1;
		subRobotManager = rManager2;
		subRobot = robot2;
	}
	else
	{
		mainRobotManager = rManager2;
		mainRobot = robot2;
		subRobotManager = rManager1;
		subRobot = robot1;
	}
	if (testworkspace)
	{
		for (unsigned int i = 0; i < busbar.size(); i++)
		{
			jointVal = mainRobotManager->inverseKin(busbarSE3set[i] * Tbusbar2gripper_new, &mainRobot->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flags[i], mainRobot->qInvKinInit);
			testJointVal.push_back(jointVal);
			//if (flags[i] != 0)
			//	jointVal = rManager1->inverseKin(busbar[i]->GetBaseLink()->GetFrame() * Tbusbar2gripper_new, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flags[i], qInit);
			mainRobotManager->setJointVal(jointVal);

			if (testBoth)
			{
				jointVal_add = subRobotManager->inverseKin(busbarSE3set[i] * Tbusbar2gripper_new, &subRobot->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flags_add[i], subRobot->qInvKinInit);
				subRobotManager->setJointVal(subRobot->homePos);
			}

			bool isColli = mainRobotManager->checkCollision();
			
			if (flags[i] == 0 && !isColli)
			{
				if (testBoth && flags_add[i] == 0)
				{
					mainRobotManager->setJointVal(mainRobot->homePos);
					subRobotManager->setJointVal(jointVal_add);
					bool isColli_add = subRobotManager->checkCollision();
					if (!isColli_add)
					{
						if (!useSphere)
							busbar[i]->m_ObjLink[0].GetGeomInfo().SetColor(1.0, 1.0, 0.0);
						else
							sph[i]->m_ObjLink[0].GetGeomInfo().SetColor(1.0, 1.0, 0.0);
					}
					else
					{
						if (!useSphere)
							busbar[i]->m_ObjLink[0].GetGeomInfo().SetColor(0.0, 1.0, 0.0);
						else
							sph[i]->m_ObjLink[0].GetGeomInfo().SetColor(0.0, 1.0, 0.0);
					}	
				}
				else
				{
					if (!useSphere)
						busbar[i]->m_ObjLink[0].GetGeomInfo().SetColor(0.0, 1.0, 0.0);
					else
						sph[i]->m_ObjLink[0].GetGeomInfo().SetColor(0.0, 1.0, 0.0);
				}
					

				// test gravitational torque at each position
				if (testMaxTorque)
				{
					testJointTorque = mainRobotManager->inverseDyn(jointVal, Zerovec, Zerovec);
					if (maxTorque2.size() == 0)
					{
						maxTorque2 = testJointTorque;
						maxTorque3 = testJointTorque;
						maxTorque2Pos = jointVal;
						maxTorque3Pos = jointVal;
					}
					else
					{
						if (abs(testJointTorque[1]) > abs(maxTorque2[1]))
						{
							maxTorque2 = testJointTorque;
							maxTorque2Pos = jointVal;
						}
						if (abs(testJointTorque[2]) > abs(maxTorque3[2]))
						{
							maxTorque3 = testJointTorque;
							maxTorque3Pos = jointVal;
						}
					}
				}
			}	
			if (!useSphere)
			{
				if (flags[i] == 1 && !isColli)
					busbar[i]->m_ObjLink[0].GetGeomInfo().SetColor(0.0, 0.0, 0.1);
				if (flags[i] == 2 || isColli)
					busbar[i]->m_ObjLink[0].GetGeomInfo().SetColor(1.0, 0.0, 0.0);
			}
			else
			{
				if (flags[i] == 1 && !isColli)
					sph[i]->m_ObjLink[0].GetGeomInfo().SetColor(0.0, 0.0, 0.1);
				if (flags[i] == 2 || isColli)
					sph[i]->m_ObjLink[0].GetGeomInfo().SetColor(1.0, 0.0, 0.0);
			}
		}
	}
	
	if (testworkspace && !testMaxTorque)
	{
		if (!useSphere)
		{
			for (unsigned int i = 0; i < busbar.size(); i++)
				busbar[i]->setBaseLinkFrame(busbarSE3set[i]);
		}
		else
		{
			for (unsigned int i = 0; i < sph.size(); i++)
				sph[i]->setBaseLinkFrame(busbarSE3set[i]);
		}
	}

	if (testMaxTorque)
	{
		robot1->homePos = maxTorque2Pos;
		robot2->homePos = maxTorque3Pos;
		printf("tau[2] max: ");
		cout << maxTorque2.transpose() << endl;
		printf("tau[3] max: ");
		cout << maxTorque3.transpose() << endl;
		printf("tau[2] max jointval: ");
		cout << maxTorque2Pos.transpose() << endl;
		printf("tau[3] max jointval: ");
		cout << maxTorque3Pos.transpose() << endl;
	}
	Eigen::VectorXd gripInput(2);
	gripInput[0] = -0.005;
	gripInput[1] = 0.005;
	rManager1->setGripperPosition(gripInput);
	rManager2->setGripperPosition(gripInput);

	///////////////////////////////////////////// check gravity torque
	//Eigen::VectorXd checkVel = 1.0*Eigen::VectorXd::Ones(6);
	//
	//
	//Eigen::VectorXd tau1 = rManager1->inverseDyn(checkPos1, checkVel, Zerovec);
	//Eigen::VectorXd tau2 = rManager2->inverseDyn(checkPos2, checkVel, Zerovec);
	//
	//robot1->homePos = checkPos1;
	//robot2->homePos = checkPos2;
	//printf("tau1 : ");
	//cout << tau1.transpose() << endl << endl;
	//printf("tau2 : ");
	//cout << tau2.transpose() << endl << endl;


	//Eigen::VectorXd checkPos3;
	//int flagg;
	//checkPos3 = rManager2->inverseKin(jigAssem->getBaseLinkFrame() * jigAssem->holeCenter[4] * Tbusbar2gripper_new, &robot2->gLink[Indy_Index::MLINK_GRIP], true, SE3(), flagg, robot2->qInvKinInit);
	//cout << flagg << endl;
	//Eigen::MatrixXd J2 = rManager2->getBodyJacobian(checkPos2, &robot2->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper_new));
	//Eigen::VectorXd F2(6);
	//F2[0] = 0.0; F2[1] = 0.0; F2[2] = 0.0; F2[3] = 5.0; F2[4] = 5.0; F2[5] = 50.0;
	//Eigen::VectorXd tau_add = J2.transpose() * F2;
	//printf("tau_add : ");
	//cout << tau_add.transpose() << endl;
	//printf("total torque: ");
	//cout << (tau2 - tau_add).transpose() << endl;
	///////////////////////////////////////////////////////////////////

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

	//renderer->setUpdateFunc(updateFuncTestSensorToRobot);
	//renderer->setUpdateFunc(updateFunc);
	//if (planning)
	//	renderer->setUpdateFunc(updateFuncPlanning);
	//else
	//	renderer->setUpdateFunc(updateFuncInput);
	//renderer->setUpdateFunc(updateFuncPlanning);
	//renderer->setUpdateFunc(updateFuncLoadJointValAttachStatus);

	renderer->setUpdateFunc(updateFunc);
	renderer->RunRendering();
}

void updateFunc()
{
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
	rManager1->setJointVal(robot1->homePos);
	rManager2->setJointVal(robot2->homePos);
}


void updateFuncInput()
{
	gSpace.m_Gravity = Vec3(0.0, 0.0, 0.0);
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
	static int cnt = 0;
	int taskNum;
	if (cnt % 300 == 0)
	{
		printf("enter number: ");
		cin >> taskNum;
		if (taskNum < 0)
		{
			rManager1->setJointVal(initPos[0]);
			taskNum = 0;
		}
		else
			rManager1->setJointVal(goalPos[taskNum % goalPos.size()]);
		if (attachObject[taskNum % goalPos.size()])
			busbar[0]->setBaseLinkFrame(robot1->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame() / Tbusbar2gripper_new);
		else
			busbar[0]->setBaseLinkFrame(initBusbar);
		cout << "colli: " << gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP() << endl;
	}
	cnt++;
}

void updateFuncTestSensor()
{
	static Eigen::VectorXd testjointVel = Eigen::VectorXd::Ones(6);
	static int cnt = 0;
	if (cnt == 0)
	{
		rManager1->setJointValVel(homePos, testjointVel);
		jointVal = homePos;
		jointVel = testjointVel;
	}
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();

	jointVal = rManager1->getJointVal();
	jointVel = rManager1->getJointVel();
	
		
	cnt++;
	Eigen::VectorXd jointAcc = Eigen::VectorXd::Ones(6);
	Eigen::VectorXd tau = rManager1->inverseDyn(jointVal, jointVel, jointAcc);
	rManager1->controlJointTorque(tau);
	SE3 renderEndeff = rManager1->forwardKin(jointVal, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
	busbar[0]->setBaseLinkFrame(renderEndeff*Inv(Tbusbar2gripper_new));
	// read sensor value
	dse3 Ftsensor = rManager1->readSensorValue();
	dse3 Fr(0.0);
	se3 g(0.0);
	se3 Vdot(0.0);
	se3 V(0.0);
	V = Vectortose3(rManager1->getBodyJacobian(jointVal, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper_new)) * jointVel);
	if (cnt == 0)
		jointAcc = rManager1->getJointAcc();
	Vdot = Vectortose3(rManager1->getBodyJacobian(jointVal, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper_new)) * jointAcc +
		rManager1->getBodyJacobianDot(jointVal, jointVel, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper_new)) * jointVel);
	for (int i = 0; i < 3; i++)
		g[i + 3] = gSpace.m_Gravity[i];
	Fr = busbar[0]->GetBaseLink()->m_Inertia * Vdot - dad(V, busbar[0]->GetBaseLink()->m_Inertia * V);
	dse3 Fr_g = - (busbar[0]->GetBaseLink()->m_Inertia*InvAd(busbar[0]->GetBaseLink()->GetFrame(), g));
	se3 g_bus = InvAd(busbar[0]->GetBaseLink()->GetFrame(), g);
	SE3 Tbus = busbar[0]->GetBaseLink()->GetFrame();
	Fr += Fr_g;
	dse3 Fr_busbar = InvdAd((rManager1->m_ftSensorInfo[0]->m_sensorLocJoint->GetFrame() * rManager1->m_ftSensorInfo[0]->m_offset) % renderEndeff * Inv(Tbusbar2gripper_new), Fr);
	ftsensor = dse3toVector(Ftsensor + Fr_busbar);
	

	cout << "q: " << jointVal.transpose() << endl;
	cout << "a: " << rManager1->getJointAcc().transpose() << endl;
	cout << "f: " << ftsensor.transpose() << endl;
	cout << "busbar: " << endl << busbar[0]->GetBaseLink()->GetFrame() << endl;
	cout << "V: " << V << endl;
	cout << "A: " << Vdot << endl;
}

void setHybridPFCtrl()
{
	// initial config should be aligned to the contact plane
	// assume target object is rigidly attached to robot end-effector
	vector<srLink*> contactLinks(2);
	contactLinks[0] = &robot1->gLink[Indy_Index::GRIPPER_FINGER_L];
	contactLinks[1] = &robot1->gLink[Indy_Index::GRIPPER_FINGER_U];
	hctrl->isSystemSet = hctrl->setSystem((robotManager*)rManager1, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], SE3(), contactLinks);
	hctrl->setTimeStep(rManager1->m_space->m_Timestep_dyn_fixed);
	double kv_v = 0.25e2, kp_v = 0.25*kv_v*kv_v, ki_v = 0.0e3, kp_f = 1.0e-1, ki_f = 1.0e-1;
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

	//hctrl->setSelectionMatrix(S);
	hctrl->setSelectionMatrix(Eigen::MatrixXd());	//Eigen::MatrixXd(), S
}

void updateFuncTestSensorToRobot()
{
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
	hctrl->hybridPFControl();

	cout << "q: " << rManager1->getJointVal().transpose() << endl;
	cout << "F: " << rManager1->readSensorValue() << endl;
	//cout << rManager1->m_ftSensorInfo[0]->m_sensorLocJoint->GetFrame() << endl;
	cout << "Trob: " << endl << Trobotbase1 % rManager1->m_activeArmInfo->m_endeffector[0]->GetFrame() << endl;
	SE3 Tdes = Trobotbase1 % hctrl->T_des_trj[0];
	cout << "Tdes: " << endl << Trobotbase1 % hctrl->T_des_trj[0] << endl;


	SE3 Toffset = rManager1->m_activeArmInfo->m_endeffector[0]->GetFrame() % hctrl->T_des_trj[0];
	cout << "off: " <<  Toffset[11] << endl;
	cout << "des: " << Tdes[11] << endl;
	int stop = 1;
}

void updateFuncLoadJointValAttachStatus()
{
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
	static int cnt = 0;
	///////////////////////////// read from text
	if (cnt == 0)
	{
		string dir_folder = "../../../data/communication_test";
		loadJointVal = loadDataFromText(dir_folder + "/testJointValTraj_robot2.txt", 6);
	}
	
	rManager1->setJointVal(jointVal);
	rManager2->setJointVal(loadJointVal[cnt % loadJointVal.size()]);
	cnt++;
	if (loadAttachStatus.size() > 0)
	{
		if (abs(loadAttachStatus[cnt % loadAttachStatus.size()][0]) < DBL_EPSILON)
		{
		}
		else
		{
			busbar[0]->setBaseLinkFrame(rManager1->forwardKin(loadJointVal[cnt % loadJointVal.size()],
				&robot1->gMarkerLink[Indy_Index::MLINK_GRIP])*Inv(Tbusbar2gripper_new));
		}
	}

}
