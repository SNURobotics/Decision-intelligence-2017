#include <cstdio>

#include "myRenderer.h"
#include "NTdemoEnvSetting_4th.h"
//#include "2ndRenderer.h"

#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\MH12RobotManager.h"
#include "robotManager\MH12Robot.h"
#include <time.h>

// srSpace and renderer
srSpace gSpace;
myRenderer* renderer;

// Robot
MH12Robot* MHRobot = new MH12Robot;
MH12RobotManager* rManager1;

Eigen::VectorXd qval;

// demo environment
demoEnvironment* demoEnv;
demoTaskManager* demoTask;

void initDynamics();
void loadDataFromFile(string loc, char* visiondata);
void rendering(int argc, char **argv);
void updateFunc();
void envSetting();
void MHRobotSetting();
void MHRobotManagerSetting();
void URrrtSetting();
int activeJointIdx =0;
vector<Eigen::VectorXd> traj(0);

int main(int argc, char **argv)
{
	////////////////////////////////////////////////////////////////
	////////////////////// initialize //////////////////////////////
	////////////////////////////////////////////////////////////////
	// set the number of objects in demoEnvirionment function
	demoEnv = new demoEnvironment(5);
	// add robot to system
    MHRobotSetting();
	// add bin and objects to system
	envSetting();
	initDynamics();
	MHRobotManagerSetting();
	demoTask = new demoTaskManager(demoEnv, rManager1);

	// dummy for NULL
	char dummy_NULLobj[] = "d-1.0d-1.0d-1.0d-1.0d-1.0d-1.0d-1.0d-1.0d-1.0d-1.0d-1.0d-1.0d-1.0d-1.0d-1.0d-1.0d";

	// dummy for front side
	char dummy_oneobj[] = "d0.0d0.0d0.0d1.0d0.0d0.0d0.0d-1.0d0.0d0.0d0.0d-1.0d-0.02d-0.05d-0.005d0.005d";

	// dummy for back side
	//char dummy_oneobj[] = "d0.0d0.0d1.0d1.0d0.0d0.0d0.0d1.0d0.0d0.0d0.0d1.0d1d-0.02d0.0d-0.0004d";

	char dummy[1000];
	strcpy(dummy, "Vd-1"); strcat(dummy, dummy_oneobj);
	strcat(dummy, "-1"); strcat(dummy, dummy_oneobj);
	strcat(dummy, "1"); strcat(dummy, dummy_oneobj);
	strcat(dummy, "-1"); strcat(dummy, dummy_oneobj);
	//strcat(dummy, "3"); strcat(dummy, dummy_oneobj);
	//strcat(dummy, "4"); strcat(dummy, dummy_oneobj);

	////////////////////////////////////////////////////////////////
	////////////////////// adjust below ////////////////////////////
	////////////////////////////////////////////////////////////////
	// load vision data and update demoTask (fix data location, or directly insert string data)
	char visiondataSKKU[1000];
	strcpy(visiondataSKKU, "V9.d0.8925d6.67e-002d0.d-1.d-1.d-1.d-1.d-1.d-1.d-1.d-1.d-1.d-1.d-1.d-1.d-1.d1.d0.7477d0.1229d3.842e-002d-0.6935d-0.7142d-9.464e-002d0.7192d-0.6941d-3.197e-002d-4.286e-002d-9.023e-002d0.995d-2.e-002d-2.e-002d0.d0.d");
	//loadDataFromFile("D:/������Ʈ/Pose_example.txt", visiondataSKKU);
	//loadDataFromFile("C:\Users\robotics\Documents\Decision-intelligence-2017\srLib2017\workspace\robot\poseData", visiondataSKKU);
	demoTask->updateEnv(visiondataSKKU);
	//demoTask->updateEnv(dummy);
	demoTask->setObjectNum();



	bool collision = rManager1->checkCollision();

	if (!collision)
		cout << "collision free!" << endl;
	else
		cout << "collision OCCURED!" << endl;


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
	gSpace.SetGravity(0.0, 0.0, -0.0);
	gSpace.SetNumberofSubstepForRendering(1);
	gSpace.DYN_MODE_PRESTEP();
}

void updateFunc()
{

	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();

	static int cnt = 0;
	static int trajcnt = 0;
	cnt++;

	if (cnt % 10 == 0)
		trajcnt++;
	if (traj.size() > 0)
		rManager1->setJointVal(traj[trajcnt % traj.size()]);

	int stop = 1;
}


void MHRobotSetting()
{
	gSpace.AddSystem((srSystem*)MHRobot);
	MHRobot->GetBaseLink()->SetFrame(demoEnv->Trobotbase * demoEnv->Trobotbase2link1);
	MHRobot->SetActType(srJoint::ACTTYPE::HYBRID);

	vector<int> gpIdx(2);
	gpIdx[0] = 0;
	gpIdx[1] = 1;
	MHRobot->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);
}

void MHRobotManagerSetting()
{
	rManager1 = new MH12RobotManager(MHRobot, &gSpace);
}

void envSetting()
{
	demoEnv->setEnvironmentInSrSpace(&gSpace);
}

void loadDataFromFile(string loc, char* visiondata)
{
	std::ifstream fin;
	fin.open(loc);

	int i = 0;

	if (fin.is_open())
	{
		while (!fin.eof())
		{
			fin >> visiondata[i];
			i++;
		}
	}
	fin.close();
}