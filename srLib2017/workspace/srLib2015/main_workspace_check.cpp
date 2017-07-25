#include <cstdio>

#include "serverRenderer.h"
#include "simulationEnvSetting.h"
//#include "2ndRenderer.h"

#include "ForceCtrlManager\hybridPFCtrlManager.h"

Eigen::VectorXd jointVal(6);
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

double planning = 0;
vector<Eigen::VectorXd> loadJointVal(0);
vector<Eigen::VectorXd> loadAttachStatus(0);
vector<Eigen::VectorXd> testJointVal(0);
vector<Eigen::VectorXd> testJointVal2(0);
vector<vector<Eigen::VectorXd>> testJointValVec(2);

Eigen::VectorXd testjointvalue(6);
vector<SE3> busbarSE3set(0);
int main(int argc, char **argv)
{
	srand(time(NULL));
	double height = 0.2;

	// Robot home position
	robotSetting(height);
	// environment
	workspaceSetting(height);
	//objectSetting();

	environmentSetting_HYU2(true, true);

	////////////////////////////////////////////////////////////////////////////////////////
	bool testRobot1 = true;
	double yoffset;
	if (testRobot1)
		yoffset = -0.6;
	else
		yoffset = 0.1;
	SE3 Tbase;
	if (workcell_mode == 1)
		Tbase = SE3(Vec3(0.025, 1.095, 1.176));		// when stage attached
	else if (workcell_mode == 2)
		Tbase = SE3(Vec3(0.025, 1.095, 0.910 + 0.099 + 0.009));	// when only stage4 is used
	else
		Tbase = SE3(Vec3(0.025, 1.095, 0.910 + 0.009));		// when stage removed
	SE3 Tbase2jigbase = EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.184));
	int nx = 8;
	int ny = 10;
	int n = 5;
	double bin = 0.5 / (double)n;
	busbar.resize(nx*ny*n);
	flags.resize(busbar.size());
	for (unsigned int i = 0; i < busbar.size(); i++)
	{
		busbar[i] = new BusBar_HYU;
		busbar[i]->SetBaseLinkType(srSystem::FIXED);
		gSpace.AddSystem(busbar[i]);
	}
	int l = 0;
	for (int i = 0; i < nx; i++)
	{
		for (int j = 0; j < ny; j++)
		{
			for (int k = 0; k < n; k++, l++)
			{
				busbarSE3set.push_back(SE3(Vec3((double)i*bin - 0.4 + Trobotbase1[9], (double)j*bin - 0.2 + yoffset, (double)k*bin - 0.05)) * Tbase * Tbase2jigbase);
				busbar[l]->setBaseLinkFrame(SE3(Vec3((double)i*bin - 0.4 + Trobotbase1[9], (double)j*bin - 0.2 + yoffset, (double)k*bin - 0.05 - 10.0)) * Tbase * Tbase2jigbase);
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

	//////// test inverse kin of robot1
	if (testRobot1)
	{
		for (unsigned int i = 0; i < busbar.size(); i++)
		{
			jointVal = rManager1->inverseKin(busbarSE3set[i] * Tbusbar2gripper_new, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flags[i], robot1->qInvKinInit);
			testJointVal.push_back(jointVal);
			//if (flags[i] != 0)
			//	jointVal = rManager1->inverseKin(busbar[i]->GetBaseLink()->GetFrame() * Tbusbar2gripper_new, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flags[i], qInit);
			rManager1->setJointVal(jointVal);
			bool isColli = rManager1->checkCollision();
			
			if (flags[i] == 0 && !isColli)
				busbar[i]->m_ObjLink[0].GetGeomInfo().SetColor(0.0, 1.0, 0.0);
			if (flags[i] == 1 && !isColli)
				busbar[i]->m_ObjLink[0].GetGeomInfo().SetColor(0.0, 0.0, 0.1);
			if (flags[i] == 2 || isColli)
				busbar[i]->m_ObjLink[0].GetGeomInfo().SetColor(1.0, 0.0, 0.0);
		}
	}
	else
	{
		for (unsigned int i = 0; i < busbar.size(); i++)
		{
			jointVal = rManager2->inverseKin(busbarSE3set[i] * Tbusbar2gripper_new, &robot2->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flags[i], robot2->qInvKinInit);

			//if (flags[i] != 0)
			//	jointVal = rManager2->inverseKin(busbar[i]->GetBaseLink()->GetFrame() * Tbusbar2gripper_new, &robot2->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flags[i], qInit);
			rManager2->setJointVal(jointVal);
			bool isColli = rManager2->checkCollision();
			if (flags[i] == 0 && !isColli)
				busbar[i]->m_ObjLink[0].GetGeomInfo().SetColor(0.0, 1.0, 0.0);
			if (flags[i] == 1 && !isColli)
				busbar[i]->m_ObjLink[0].GetGeomInfo().SetColor(0.0, 0.0, 0.1);
			if (flags[i] == 2 || isColli)
				busbar[i]->m_ObjLink[0].GetGeomInfo().SetColor(1.0, 0.0, 0.0);
		}
	}
	

	for (unsigned int i = 0; i < busbar.size(); i++)
		busbar[i]->setBaseLinkFrame(busbarSE3set[i]);
	//cout << jointVal.transpose() << endl;
	////// test inverse kin of robot2
	//Eigen::VectorXd qInit = Eigen::VectorXd::Zero(6);
	//// elbow up
	//qInit[1] = -0.65*SR_PI;
	//qInit[2] = 0.3*SR_PI;
	//qInit[3] = 0.5*SR_PI_HALF;
	//Eigen::VectorXd qInit2 = Eigen::VectorXd::Zero(6);
	//qInit2[0] = -0.224778; qInit2[1] = -1.91949; qInit2[2] = -0.384219; qInit2[3] = 1.5708; qInit2[4] = -0.73291; qInit2[5] = 1.79557;

	

	// workcell robot initial config
	//jointVal.setZero();
	//jointVal[0] = 0.0; jointVal[1] = -SR_PI_HALF; jointVal[2] = 80.0 / 90.0*SR_PI_HALF; jointVal[3] = SR_PI_HALF;
	//rManager2->setJointVal(jointVal);
	//rManager1->setJointVal(homePos);
	Eigen::VectorXd gripInput(2);
	gripInput[0] = -0.005;
	gripInput[1] = 0.005;
	rManager1->setGripperPosition(gripInput);
	rManager2->setGripperPosition(gripInput);

	



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
