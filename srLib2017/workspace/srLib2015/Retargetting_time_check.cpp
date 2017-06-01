#include <cstdio>

#include "myRenderer.h"
//#include "2ndRenderer.h"

#include "common\dataIO.h"
#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\indyRobotManager.h"
#include "robotManager/IndyRobot.h"
#include <time.h>
#include "robotManager\environmentBusbar.h"
#include "robotManager\environment_QBtech.h"
#include "robotManager\environment_workcell.h"
#include "robotManager\robotRRTManager.h"
#include "ForceCtrlManager\hybridPFCtrlManager.h"
#include <fstream>
#include <iostream>

using namespace std;

// Environment
Base_HYU* busbarBase = new Base_HYU;
JigAssem_QB* jigAssem = new JigAssem_QB;
vector<Jig_HYU*> jig(4);
vector<SE3> jigSE3(4);
vector<SE3> jig2busbar(2);
//vector<BusBar_HYU*> busbar(8);
//vector<SE3>	initSE3(8);
//vector<SE3>	goalSE3(8);
//vector<SE3> allSE3_busbar(2 * busbar.size());

vector<BusBar_HYU*> busbar(1);
vector<SE3>	initSE3(2);
vector<SE3>	goalSE3(2);
vector<SE3> allSE3_busbar(2 * initSE3.size());

// Workspace
WorkCell* workCell = new WorkCell();
Eigen::VectorXd stageVal(3);

// Robot
IndyRobot* robot1 = new IndyRobot;
IndyRobot* robot2 = new IndyRobot;
Eigen::VectorXd jointVal(6);
Eigen::VectorXd jointAcc(6);
Eigen::VectorXd jointVel(6);
srSpace gSpace;
myRenderer* renderer;
SE3 Tbusbar2gripper = EulerZYX(Vec3(0.0, 0.0, SR_PI), Vec3(0.0, 0.0, 0.04));
SE3 Tbusbar2gripper_new = EulerZYX(Vec3(SR_PI_HALF, 0.0, SR_PI), Vec3(0.0, 0.0, 0.04));
SE3 Thole2busbar = EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.0, 0.0, 0.0));

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

SE3 initBusbar;

// Measure F/T sensor
dse3 Ftsensor;
Eigen::VectorXd ftsensor(6);

// save data
vector<vector<Eigen::VectorXd>> FTtraj;
vector<vector<Eigen::VectorXd>> TtrajVec;
vector<vector<Eigen::VectorXd>> busbarTraj;
vector<Eigen::VectorXd> goalJigLocation(1);

indyRobotManager* rManager1;
indyRobotManager* rManager2;
robotRRTManager* RRTManager1 = new robotRRTManager;
srJoint::ACTTYPE actType = srJoint::ACTTYPE::HYBRID;
hybridPFCtrlManager_6dof* hctrl = new hybridPFCtrlManager_6dof;
void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
void updateFuncInput();
void updateFuncPlanning();
void updateFuncTestSensor();
void updateFuncTestSensorToRobot();
void updateFuncLoadJointValAttachStatus();
void environmentSetting_HYU(bool connect, int startLocation, Vec2 goalLocation);
void environmentSetting_HYU2(bool connect);
void workspaceSetting();
void robotSetting();
void robotManagerSetting();
void RRT_problemSetting_HYU();
void RRT_problemSetting();
void RRT_problemSetting(Eigen::VectorXd init, vector<SE3> wayPoints, vector<bool> includeOri, vector<bool> attachObject);
void RRTSolve();
void RRTSolve_HYU(vector<bool> attachObject, vector<double> stepsize);
void rrtSetting();
void setHybridPFCtrl();

Vec2 goalLocation; // busbar insertion location

vector<int> flags(0);

double planning = 0;
vector<Eigen::VectorXd> loadJointVal(0);
vector<Eigen::VectorXd> loadAttachStatus(0);
SE3 Twaypoint;
SE3 Trobotbase1;
Eigen::VectorXd testWaypoint;
vector<Eigen::VectorXd> testJointVal(0);
Eigen::VectorXd testjointvalue(6);

int num_task = 0;
int num_success_task = 0;


int main(int argc, char **argv)
{
	srand(time(NULL));
	// Robot home position
	homePos[1] = -SR_PI_HALF; homePos[3] = SR_PI_HALF; homePos[4] = SR_PI; 
	homePos[4] = -0.5 * SR_PI;		// match to robot workcell

	// environment
	workspaceSetting();
	int startLocation =0;
	goalLocation = Vec2(1, 1);
	environmentSetting_HYU2(false);
	//jigAssem->setBaseLinkFrame(SE3(Vec3(0.0, 0.0, -0.5)));
	////////////////////////////////////////////// for testing workspace
	//SE3 Tbase = SE3(Vec3(0.025, 1.095, 1.176));		// when stage attached
	//SE3 Tbase = SE3(Vec3(0.025, 1.095, 0.910 + 0.009));		// when stage removed
	//SE3 Tbase2jigbase = EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.184));

	//int n = 5;
	//double bin = 0.4 / (double)n;
	//busbar.resize(n*n*n);
	//flags.resize(busbar.size());
	//for (unsigned int i = 0; i < busbar.size(); i++)
	//{
	//	busbar[i] = new BusBar_HYU;
	//	busbar[i]->SetBaseLinkType(srSystem::FIXED);
	//	gSpace.AddSystem(busbar[i]);
	//}
	//int l = 0;
	//for (int i = 0; i < n; i++)
	//{
	//	for (int j = 0; j < n; j++)
	//	{
	//		for (int k = 0; k < n; k++, l++)
	//		{
	//			busbar[l]->setBaseLinkFrame(SE3(Vec3((double)i*bin - 0.2, (double)j*bin - 0.2, (double)k*bin)) * Tbase);
	//		}
	//	}
	//}


	//busbar.resize(16);
	//flags.resize(busbar.size());
	//for (unsigned int i = 0; i < busbar.size()/2; i++)
	//{
	//	busbar[2*i] = new BusBar_HYU;
	//	busbar[2*i+1] = new BusBar_HYU;
	//	gSpace.AddSystem(busbar[2*i]);
	//	gSpace.AddSystem(busbar[2 * i + 1]);
	//	busbar[2*i]->setBaseLinkFrame(Tbase*Tbase2jigbase*jigAssem->holeCenter[i]);
	//	busbar[2*i+1]->setBaseLinkFrame(SE3(Vec3(0.0, 0.0, -0.025))*Tbase*Tbase2jigbase*jigAssem->holeCenter[i]);
	//}

	
	///////////////////////////////////////////////////////////////////

	robotSetting();
	Trobotbase1 = robot1->GetBaseLink()->GetFrame();


	// initialize srLib
	initDynamics();

	// robot manager setting
	robotManagerSetting();



	////// test inverse kin
	//Eigen::VectorXd qInit = Eigen::VectorXd::Zero(6);
	//// elbow up
	//qInit[1] = -0.65*SR_PI;
	//qInit[2] = 0.3*SR_PI;
	//qInit[3] = 0.5*SR_PI_HALF;
	//for (unsigned int i = 0; i < busbar.size(); i++)
	//{
	//	jointVal = rManager1->inverseKin(busbar[i]->GetBaseLink()->GetFrame() * Tbusbar2gripper_new, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flags[i], qInit);
	//	if (flags[i] == 0)
	//		busbar[i]->GetBaseLink()->GetGeomInfo().SetColor(1.0, 0.0, 0.0);
	//	if (flags[i] == 1)
	//		busbar[i]->GetBaseLink()->GetGeomInfo().SetColor(0.0, 1.0, 0.0);
	//	if (flags[i] == 2)
	//		busbar[i]->GetBaseLink()->GetGeomInfo().SetColor(0.0, 0.0, 0.1);
	//}



	// workcell robot initial config
	jointVal.setZero();
	jointVal[0] = 0.0; jointVal[1] = -SR_PI_HALF; jointVal[2] = 80.0 / 90.0*SR_PI_HALF; jointVal[3] = SR_PI_HALF;
	rManager2->setJointVal(jointVal);
	rManager1->setJointVal(homePos);
	Eigen::VectorXd gripInput(2);
	gripInput[0] = -0.009;
	gripInput[1] = 0.009;
	rManager1->setGripperPosition(gripInput);
	rManager2->setGripperPosition(gripInput);

	// rrt
	rrtSetting();

	

	///////////////////////////////////////////////// Workcell parctice trajectory ////////////////////////////////////////////
	//// hard traj ////////////////////////////////////////////////////////////////////
	//Eigen::VectorXd temp = homePos;
	//temp[4] = SR_PI;
	//wayPoints.resize(0);
	//SE3 tmpSE3 = SE3(Vec3(0.0, -0.02, -0.13)) * rManager1->forwardKin(temp)[0] * Inv(Tbusbar2gripper);
	//wayPoints.push_back(tmpSE3);
	//wayPoints.push_back(SE3(Vec3(0.0, 0.0, 0.1)) * tmpSE3);
	//vector<bool> includeOri(wayPoints.size(), true);
	//attachObject.resize(wayPoints.size());
	//for (unsigned int i = 0; i < attachObject.size(); i++)
	//	attachObject[i] = false;
	//RRT_problemSetting(homePos, wayPoints, includeOri, attachObject);
	//vector<double> stepsize(wayPoints.size(), 0.1);
	////RRTSolve_HYU(attachObject, stepsize);

	//printf("do planning?: ");
	//cin >> planning;
	//if (planning)
	//	RRTSolve_HYU(attachObject, stepsize);
	//////////////////////////////////////////////////////////////////////
	
	// hard traj 2 ////////////////////////////////////////////////////////////////////
	//Eigen::VectorXd temp = init;
	//wayPoints.resize(0);
	//SE3 tmpSE3 = SE3(Vec3(0.3, -0.1, 0.03))*rManager1->forwardKin(temp)[0];
	//wayPoints.push_back(tmpSE3);
	//tmpSE3 = SE3(Vec3(0.0, 0.3, 0.0))*tmpSE3;
	//wayPoints.push_back(tmpSE3);
	//tmpSE3 = SE3(Vec3(-0.3, 0.0, 0.0))*tmpSE3;
	//wayPoints.push_back(tmpSE3);
	//tmpSE3 = SE3(Vec3(0.0,-0.3, 0.0))*tmpSE3;
	//wayPoints.push_back(tmpSE3);
	//tmpSE3 = SE3(Vec3(0.3, 0.0, 0.0))*tmpSE3;
	//wayPoints.push_back(tmpSE3);

	//vector<bool> includeOri(2, true);
	//RRT_problemSetting(init, wayPoints, includeOri);
	//vector<bool> attachObject(wayPoints.size() + 1, false);
	//RRTSolve_HYU(attachObject);

	////////////////////////////////////////////////////////////////////
	
	// stage Edge (follow horizontal edge)////////////////////////////////////////////////////////////////////
	//Eigen::VectorXd temp = homePos;
	//wayPoints.resize(0);
	//temp[4] = 0.0;
	//Vec3 homePos3 = rManager1->forwardKin(temp)[0].GetPosition();
	//Vec3 XYZFrontEdge(0.025 + 0.25 , 1.095 - 0.25, 1.176 + 0.005);
	//Vec3 XYZBackEdge(0.025 - 0.25, 1.095 - 0.25, 1.176 + 0.005);
	//SE3 tmpSE3 = SE3(Vec3(XYZFrontEdge[0] - homePos3[0], XYZFrontEdge[1] - homePos3[1], XYZFrontEdge[2] - homePos3[2]))*rManager1->forwardKin(temp)[0];
	//tmpSE3 = tmpSE3*Inv(Tbusbar2gripper);
	////SE3 tmpSE3 = rManager1->forwardKin(temp)[0];
	//wayPoints.push_back(tmpSE3);
	//tmpSE3 = SE3(Vec3(-0.1, 0.0, 0.0))*tmpSE3;
	//wayPoints.push_back(tmpSE3);
	//tmpSE3 = SE3(Vec3(-0.1, 0.0, 0.0))*tmpSE3;
	//wayPoints.push_back(tmpSE3);
	//tmpSE3 = SE3(Vec3(-0.1, 0.0, 0.0))*tmpSE3;
	//wayPoints.push_back(tmpSE3);
	//tmpSE3 = SE3(Vec3(-0.1, 0.0, 0.0))*tmpSE3;
	//wayPoints.push_back(tmpSE3);
	//tmpSE3 = SE3(Vec3(-0.1, 0.0, 0.0))*tmpSE3;
	//wayPoints.push_back(tmpSE3);

	//vector<bool> includeOri(wayPoints.size(), true);
	//
	//attachObject.resize(wayPoints.size());
	//for (unsigned int i = 0; i < attachObject.size(); i++)
	//	attachObject[i] = false;
	//RRT_problemSetting(homePos, wayPoints, includeOri, attachObject);
	//vector<double> stepsize(wayPoints.size(), 0.1);
	////RRTSolve_HYU(attachObject,stepsize);
	//printf("do planning?: ");
	//cin >> planning;
	//if (planning)
	//	RRTSolve_HYU(attachObject, stepsize);
	////////////////////////////////////////////////////////////////////

	// stage Edge (follow long path)////////////////////////////////////////////////////////////////////
	//Eigen::VectorXd temp = init;
	//wayPoints.resize(0);
	//temp[4] = 0.0;
	//Vec3 homePos = rManager1->forwardKin(temp)[0].GetPosition();
	//Vec3 XYZFrontEdge(0.025 + 0.25, 1.095 - 0.25, 1.176 + 0.005);
	//Vec3 XYZBackEdge(0.025 - 0.25, 1.095 - 0.25, 1.176 + 0.005);
	//SE3 tmpSE3 = SE3(Vec3(XYZFrontEdge[0] - homePos[0], XYZFrontEdge[1] - homePos[1], XYZFrontEdge[2] - homePos[2]))*rManager1->forwardKin(temp)[0];
	////SE3 tmpSE3 = rManager1->forwardKin(temp)[0];
	//tmpSE3 = SE3(Vec3(0.0, 0.1, 0.0))*tmpSE3;
	//wayPoints.push_back(tmpSE3);
	//tmpSE3 = SE3(Vec3(0.0, -0.1, 0.0))*tmpSE3;
	//wayPoints.push_back(tmpSE3);
	//tmpSE3 = SE3(Vec3(-0.1, 0.0, 0.0))*tmpSE3;
	//wayPoints.push_back(tmpSE3);
	//tmpSE3 = SE3(Vec3(-0.1, 0.0, 0.0))*tmpSE3;
	//wayPoints.push_back(tmpSE3);
	//tmpSE3 = SE3(Vec3(-0.1, 0.0, 0.0))*tmpSE3;
	//wayPoints.push_back(tmpSE3);
	//tmpSE3 = SE3(Vec3(-0.1, 0.0, 0.0))*tmpSE3;
	//wayPoints.push_back(tmpSE3);
	//tmpSE3 = SE3(Vec3(-0.1, 0.0, 0.0))*tmpSE3;
	//wayPoints.push_back(tmpSE3);
	//tmpSE3 = SE3(Vec3(0.0, 0.1, 0.0))*tmpSE3;
	//wayPoints.push_back(tmpSE3);


	//vector<bool> includeOri(wayPoints.size(), true);
	//RRT_problemSetting(init, wayPoints, includeOri);
	//vector<bool> attachObject(wayPoints.size() + 1, false);
	//RRTSolve_HYU(attachObject);

	////////////////////////////////////////////////////////////////////



	// easy traj ////////////////////////////////////////////////////////////////////
	//wayPoints.resize(0);
	//wayPoints.push_back(SE3(Vec3(0.025, 0.85-0.12, 1.176)));
	//wayPoints.push_back(SE3(Vec3(0.025, 1.095 - 0.1-0.12, 1.176)));
	//vector<bool> includeOri(2, true);
	//vector<bool> attachObject(wayPoints.size() + 1, false);
	//RRTSolve_HYU(attachObject);
	////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



	//initSE3[0] = rManager1->forwardKin(initPos[0])[0] * Inv(Tbusbar2gripper);
	//goalSE3[0] = SE3(Vec3(0.025, 0.85, 1.176));
	//initSE3[1] = goalSE3[0];
	//goalSE3[1] = SE3(Vec3(0.025, 1.095 - 0.1, 1.176));
	//int flag;
	//Eigen::VectorXd qInit = Eigen::VectorXd::Zero(6);
	//// elbow up
	//qInit[1] = -0.65*SR_PI;
	//qInit[2] = 0.3*SR_PI;
	//goalPos[0] = rManager1->inverseKin(goalSE3[0] * Tbusbar2gripper, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], false, SE3(), flag, qInit);
	//cout << flag << endl;

	//RRT_problemSetting_HYU();

	

	/////////////////////////////////////////////////////////////////////// 170530 success data for success rate check
	
	
	int nWay = 3;
	vector<bool> includeOri(nWay, true);
	wayPoints.resize(nWay);
	int holeNum = 4;
	int experiment_num = 5000;
	for (int i = 0; i < experiment_num; i++)
	{
		double tran_x = (double)rand() / RAND_MAX * 0.15;
		double tran_y = -(double)rand() / RAND_MAX * 0.15;
		double tran_z = (double)rand() / RAND_MAX * 0.05;
		initBusbar = SE3(Vec3(0.0 + tran_x, -0.4 + tran_y, 0.06 + tran_z)) * jigAssem->GetBaseLink()->GetFrame();
		initSE3[0] = initBusbar;
		busbar[0]->GetBaseLink()->SetFrame(initBusbar);
		wayPoints[0] = initBusbar;
		wayPoints[1] = SE3(Vec3(0.0, 0.0, 0.02)) * jigAssem->GetBaseLink()->GetFrame() * jigAssem->holeCenter[holeNum] * Thole2busbar;
		wayPoints[2] = SE3(Vec3(0.0, 0.0, -0.025)) * wayPoints[1];
		//wayPoints[1] = SE3(Vec3(0.0, 0.0, 0.01)) * initBusbar;
		//wayPoints[2] = SE3(Vec3(0.0, 0.0, 0.02)) * initBusbar;

		attachObject.resize(nWay);
		attachObject[0] = false;
		attachObject[1] = true;
		attachObject[2] = true;
		//attachObject[3] = true;

		vector<double> stepsize(nWay, 0.8);
		//stepsize[2] = 0.005;

		RRT_problemSetting(homePos, wayPoints, includeOri, attachObject);

		busbar[0]->setBaseLinkFrame(initBusbar);
		RRTSolve_HYU(attachObject, stepsize);
		//initPos[0] = homePos;
		//for (unsigned int j = 0; j < initPos.size() - 1; j++)
		//{
		//	goalPos[j] = initPos[j];
		//	goalPos[j][5] = initPos[j][5] + 1.2;
		//	initPos[j + 1] = goalPos[j];

		//}
		//goalPos[goalPos.size() - 1] = homePos;

		//printf("do planning?: ");
		//cin >> planning;
		//if (planning)
		//	RRTSolve_HYU(attachObject, stepsize);
		cout << "Experiment number: " << i << endl;
	}

	cout << "Number of task: " << num_task << endl;
	cout << "Number of success task: " << num_success_task << endl;

	string dir_save_result = "../../../data/success_rate/retarget_step08.txt";


	std::ofstream fout;
	fout.open(dir_save_result);
	fout << "Number of task: " << num_task << '\t' << '\n';
	fout << "Number of success task: " << num_success_task;
	fout.close();
	//double tran_x = (double)rand() / RAND_MAX * 0.15;
	//double tran_y = -(double)rand() / RAND_MAX * 0.15;
	//double tran_z = (double)rand() / RAND_MAX * 0.05;
	//initBusbar = SE3(Vec3(0.0 + tran_x, -0.4 + tran_y,  0.06+tran_z)) * jigAssem->GetBaseLink()->GetFrame();
	//initSE3[0] = initBusbar;
	//busbar[0]->GetBaseLink()->SetFrame(initBusbar);
	//wayPoints[0] = initBusbar;
	//wayPoints[1] = SE3(Vec3(0.0, 0.0, 0.02)) * jigAssem->GetBaseLink()->GetFrame() * jigAssem->holeCenter[holeNum] * Thole2busbar;
	//wayPoints[2] = SE3(Vec3(0.0, 0.0, -0.025)) * wayPoints[1];
	////wayPoints[1] = SE3(Vec3(0.0, 0.0, 0.01)) * initBusbar;
	////wayPoints[2] = SE3(Vec3(0.0, 0.0, 0.02)) * initBusbar;

	//attachObject.resize(nWay);
	//attachObject[0] = false;
	//attachObject[1] = true;
	//attachObject[2] = true;
	////attachObject[3] = true;

	//vector<double> stepsize(nWay, 0.05);
	//stepsize[2] = 0.005;

	//RRT_problemSetting(homePos, wayPoints, includeOri, attachObject);
	//
	//busbar[0]->setBaseLinkFrame(initBusbar);
	////initPos[0] = homePos;
	////for (unsigned int j = 0; j < initPos.size() - 1; j++)
	////{
	////	goalPos[j] = initPos[j];
	////	goalPos[j][5] = initPos[j][5] + 1.2;
	////	initPos[j + 1] = goalPos[j];

	////}
	////goalPos[goalPos.size() - 1] = homePos;

	//printf("do planning?: ");
	//cin >> planning;
	//if (planning)
	//	RRTSolve_HYU(attachObject, stepsize);
	//
	//int flag;
	//Eigen::VectorXd qInit = Eigen::VectorXd::Zero(6);
	//// elbow up
	//qInit[1] = -0.65*SR_PI;
	//qInit[2] = 0.3*SR_PI;
	//qInit[3] = 0.5*SR_PI_HALF;
	//jointVal = rManager1->inverseKin(wayPoints[0] * Tbusbar2gripper, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flag, qInit, 2000);
	//cout << flag << endl;
	
	/////////////////////////////////////////////////////////////////////////////////////////////////


	//FTtraj.resize(initPos.size());
	//TtrajVec.resize(initPos.size());
	//busbarTraj.resize(initPos.size());
	//for (unsigned int i = 0; i < traj.size(); i++)
	//{
	//	printf("%d-th traj length: %d\n", i, traj[i].size());
	//}
	
	///////////////////////////// read from text
	//string dir_folder = "../../../data/workcell_test_data3";
	//loadJointVal = loadDataFromText(dir_folder + "/jointValTrj-.txt", 6);


	//////////////////////////////////////////////////////////////////////////// 170523 test communication
	/////////////////////////////// read from text
	//string dir_folder = "../../../data/communication_test";
	//loadJointVal = loadDataFromText(dir_folder + "/jointValTraj.txt", 6);
	//loadAttachStatus = loadDataFromText(dir_folder + "/attachTraj.txt", 1);

	/////////////////////////////// check waypoint
	//Eigen::VectorXd qInit2 = Eigen::VectorXd::Zero(6);
	//qInit2[0] = -0.224778; qInit2[1] = -1.91949; qInit2[2] = -0.384219; qInit2[3] = 1.5708; qInit2[4] = -0.73291; qInit2[5] = 1.79557;
	//SE3 Tway1;
	//SE3 Tway2;
	//SE3 Tway3;
	//Tway1[0] = -0.064028; Tway1[1] = -0.951107; Tway1[2] = -0.302153;
	//Tway1[3] = 0.645112; Tway1[4] = -0.270455; Tway1[5] = 0.714622;
	//Tway1[6] = -0.761401; Tway1[7] = -0.149167; Tway1[8] = 0.630887;
	//Tway1[9] = -0.593602; Tway1[10] = -0.150047; Tway1[11] = 0.779937;

	//Tway2[0] = -0.124107; Tway2[1] = -0.992242; Tway2[2] = -0.007254;
	//Tway2[3] = 0.990070; Tway2[4] = -0.124315; Tway2[5] = 0.065634;
	//Tway2[6] = -0.066026; Tway2[7] = 0.000964; Tway2[8] = 0.997817;
	//Tway2[9] = -0.461765; Tway2[10] = -0.004162; Tway2[11] = 0.788315;

	//Tway3[0] = 0.000000; Tway3[1] = -1.000000; Tway3[2] = -0.000000;
	//Tway3[3] = 1.000000; Tway3[4] = 0.000000; Tway3[5] = 0.0;
	//Tway3[6] = 0.0; Tway3[7] = 0.0; Tway3[8] = 1.0;
	//Tway3[9] = -0.440500; Tway3[10] = -0.004500; Tway3[11] = 0.779000;
	//Twaypoint = Tway3;
	//int flagtemp;
	//testWaypoint = rManager1->inverseKin(Trobotbase * Twaypoint, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flagtemp, qInit2);
	//cout << "waypoint inv kin flag: " << flagtemp << endl;

	//cout << Trobotbase * Tway1 << endl;
	//cout << Trobotbase * Tway2 << endl;
	//cout << Trobotbase * Tway3 << endl;

	////busbar[0]->setBaseLinkFrame(Trobotbase*Twaypoint*Inv(Tbusbar2gripper));
	//initBusbar = SE3(Vec3(0.0, -0.4, 0.05)) * jigAssem->GetBaseLink()->GetFrame();
	//busbar[0]->setBaseLinkFrame(initBusbar);
	////cout << Trobotbase % initBusbar << endl;
	////cout << jigAssem->GetBaseLink()->GetFrame() << endl;
	//////////////////////////////////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////////////// 170523 workspace end-effector matching
	//Eigen::VectorXd qInit2 = Eigen::VectorXd::Zero(6);
	//qInit2[0] = -0.224778; qInit2[1] = -1.91949; qInit2[2] = -0.384219; qInit2[3] = 1.5708; qInit2[4] = -0.73291; qInit2[5] = 1.79557;
	//double x1 = 0.75 - 0.49;
	//double x2 = x1 - 0.48;
	//double xtemp = 0.5*(0.44) - 0.005;
	//SE3 Twayconv1 = SE3(Vec3(x1, 0.5*2.068 - 0.29972, 0.5*(0.1511) + 1.03555 - 0.03));
	//SE3 Twayconv2 = SE3(Vec3(x2, 0.5*2.068 - 0.29972, 0.5*(0.1511) + 1.03555 - 0.03));
	//int flag;
	//SE3 Ttemp;
	//testWaypoint = rManager1->inverseKin(Twayconv1 * Tbusbar2gripper, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flag, qInit2);
	//busbar[0]->setBaseLinkFrame(SE3(Vec3(0.0,0.0,-0.5)));
	//cout << flag << endl;
	//
	//unsigned int Ndata = 40;
	//Ttemp = Twayconv1;
	//for (unsigned int i = 0; i < Ndata; i++)
	//{
	//	Ttemp[9] = (x1*(double)(Ndata - 1 - i) + x2*(double)i) / (Ndata - 1);
	//	testWaypoint = rManager1->inverseKin(Ttemp * Tbusbar2gripper, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flag, qInit2);
	//	cout << flag << endl;
	//	testJointVal.push_back(testWaypoint);
	//}
	//cout << Trobotbase % Twayconv1 << endl;

	//string dir_folder = "../../../data/communication_test";
	//// save
	//string dir_temp = dir_folder;
	//saveDataToText(testJointVal, dir_temp.append("/testJointValTraj").append(".txt"));
	//wayPoints.resize(1);
	//wayPoints[0] = Twayconv1;
	//vector<bool> includeOri(1, true);
	//attachObject.resize(1);
	//attachObject[0] = false;
	//RRT_problemSetting(homePos, wayPoints, includeOri, attachObject);
	//vector<double> stepsize(wayPoints.size(), 0.1);
	//RRTSolve_HYU(attachObject, stepsize);


	//cout << traj[0].size() << endl;

	////RRTManager->setStartandGoal(homePos, testJointVal[0]);

	////vector<bool> feas(2);
	////feas = RRTManager->checkFeasibility(homePos, testJointVal[0]);
	////cout << feas[0] << feas[1] << endl;
	////RRTManager->execute(0.1);
	////vector<Eigen::VectorXd> tempTraj = RRTManager->extractPath();

	//// save
	//dir_temp = dir_folder;
	//saveDataToText(traj[0], dir_temp.append("/testJointValTraj0").append(".txt"));
	/////////////////////////////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////////// 170524 test robot HW matching
	//testjointvalue[0] = 5.268; testjointvalue[1] = -97.95; testjointvalue[2] = 18.47; testjointvalue[3] = 90.0; testjointvalue[4] = 10.59; testjointvalue[5] = -1.48;
	//testjointvalue *= SR_PI / 180.0;
	//rManager1->setJointVal(testjointvalue);
	/////////////////////////////////////////////////////////////////////////////////////////////////
	//testjointvalue[0] = -45.469; testjointvalue[1] = -98.083; testjointvalue[2] = 16.538; testjointvalue[3] = 90.0; testjointvalue[4] = 10.566; testjointvalue[5] = 44.116;
	//testjointvalue *= SR_PI / 180.0;
	//rManager1->setJointVal(testjointvalue);
	/////////////////////////////////////////////////////////////////////////////////////////////////
	//testjointvalue[0] = -51.876; testjointvalue[1] = -91.79; testjointvalue[2] = -15.676; testjointvalue[3] = 90.0; testjointvalue[4] = -14.205; testjointvalue[5] = 44.251;
	//testjointvalue *= SR_PI / 180.0;
	//rManager1->setJointVal(testjointvalue);
	/////////////////////////////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////////// 170524 test robot HW matching

	//double x1 = 0.75 - 0.49;
	//double x2 = x1 - 0.48;
	//SE3 Twayconv1 = SE3(Vec3(0.5*(x1 + x2), 0.5*2.068 - 0.29972 - 0.05 - 0.15, 0.5*(0.1511) + 1.03555 - 0.03));
	//Eigen::VectorXd qInit2 = Eigen::VectorXd::Zero(6);
	//qInit2[0] = -0.224778; qInit2[1] = -1.91949; qInit2[2] = -0.384219; qInit2[3] = 1.5708; qInit2[4] = -0.73291; qInit2[5] = 1.79557;
	//int flag;
	//testjointvalue = rManager1->inverseKin(Twayconv1 * Tbusbar2gripper, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flag, qInit2);
	//cout << flag << endl;
	//cout << Twayconv1 * Tbusbar2gripper << endl;
	//rManager1->setJointVal(testjointvalue);
	//vector<SE3> Tdestrj(1);
	//vector<dse3> fdestrj(1);
	//Tdestrj[0] = SE3(Vec3(0.0, 0.0, -0.0095 - 0.01))*Twayconv1 * Tbusbar2gripper;	// -0.0095: when contact start
	//fdestrj[0] = dse3(0.0);
	//setHybridPFCtrl();
	//hctrl->setDesiredTraj(Tdestrj, fdestrj);



	///////////////////////////////////////////////////////////////////////////////////////////////// 170525 test waypoint
	//SE3 Twaypoint;
	////Twaypoint[9] = -0.593602; Twaypoint[10] = -0.150047; Twaypoint[11] = 0.779937;
	////Twaypoint[0] = -0.064028; Twaypoint[3] = 0.645112; Twaypoint[6] = -0.761401;
	////Twaypoint[1] = -0.951107; Twaypoint[4] = -0.270455; Twaypoint[7] = -0.149167;
	////Twaypoint[2] = -0.302153; Twaypoint[5] = 0.714622; Twaypoint[8] = 0.630887;
	//Twaypoint[9] = -0.461765; Twaypoint[10] = -0.004162; Twaypoint[11] = 0.788315;
	//Twaypoint[0] = -0.124107; Twaypoint[3] = 0.990070; Twaypoint[6] = -0.066026;
	//Twaypoint[1] = -0.992242; Twaypoint[4] = -0.124315; Twaypoint[7] = 0.000964;
	//Twaypoint[2] = -0.007254; Twaypoint[5] = 0.065634; Twaypoint[8] = 0.997817;
	//int flag;
	//Eigen::VectorXd qInit2 = Eigen::VectorXd::Zero(6);
	//qInit2[0] = -0.224778; qInit2[1] = -1.91949; qInit2[2] = -0.384219; qInit2[3] = 1.5708; qInit2[4] = -0.73291; qInit2[5] = 1.79557;
	//testjointvalue = rManager1->inverseKin(Trobotbase * Twaypoint, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flag, qInit2);
	//cout << flag << endl;
	//busbar[0]->setBaseLinkFrame(Trobotbase * Twaypoint*Inv(Tbusbar2gripper_new));
	/////////////////////////////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////// 170526 change ftsensor weld joint location
	//for (int i = 0; i < 10; i++)
	//{
	//	//cout << Trobotbase % robot1->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame() << endl;
	//	Eigen::VectorXd random = Eigen::VectorXd::Random(6);
	//	rManager1->setJointVal(random);
	//	cout << robot1->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame() << endl;
	//	cout << rManager1->m_ftSensorInfo[0]->m_sensorLocJoint->GetFrame() << endl << endl;
	//}
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//rendering(argc, argv);

	return 0;
}

void rendering(int argc, char **argv)
{
	renderer = new myRenderer();

	SceneGraphRenderer::NUM_WINDOWS windows;

	windows = SceneGraphRenderer::SINGLE_WINDOWS;

	renderer->InitializeRenderer(argc, argv, windows, false);
	renderer->InitializeNode(&gSpace);

	renderer->setUpdateFunc(updateFuncTestSensorToRobot);
	//if (planning)
	//	renderer->setUpdateFunc(updateFuncPlanning);
	//else
	//	renderer->setUpdateFunc(updateFuncInput);

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
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
	
	rManager2->setJointVal(jointVal);
	static int cnt = 0;
	rManager1->setJointVal(homePos);
	//rManager1->setJointVal(testjointvalue);
	//rManager1->setJointVal(testWaypoint);
	//rManager1->setJointVal(traj[0][cnt%(traj[0].size()-1)]);
	cnt++;
	//if (loadJointVal.size() > 0)
	//	rManager1->setJointVal(loadJointVal[cnt % loadJointVal.size()]);
	//else
	//	rManager1->setJointVal(homePos);
	//cnt++;



	//static int taskNum = 0;
	//rManager1->setJointVal(goalPos[taskNum % goalPos.size()]);
	//if (cnt % 1000 == 0)
	//{
	//	taskNum++;
	//	cout << taskNum % goalPos.size() << endl;
	//}
		


	//static double alpha = 0.0;
	//static int cnt = 0;
	//alpha += 0.01;

	//Eigen::VectorXd gripInput(2);
	//gripInput[0] = -0.009;
	//gripInput[1] = 0.009;
	//rManager1->setGripperPosition(gripInput);
	//rManager2->setGripperPosition(gripInput);


	//// control acceleration
	////rManager1->controlJointAcc(acc);

	//// busbar movement
	////busbar[0]->setBaseLinkFrame(rManager1->m_activeArmInfo->m_endeffector[0]->GetFrame() * Inv(Tbusbar2gripper));

	//// read sensor value
	////dse3 Ftsensor = rManager1->readSensorValue();
	////Eigen::VectorXd ftsensor = dse3toVector(Ftsensor);

	//// move stage
	////((srStateJoint*)workCell->pJoint[0])->m_State.m_rValue[0] = 0.05*sin(alpha);
	////((srStateJoint*)workCell->pJoint[1])->m_State.m_rValue[0] = 0.05*sin(2.0*alpha);
	////((srStateJoint*)workCell->rJoint[0])->m_State.m_rValue[0] = alpha;

	////test->GetBaseLink()->SetFrame(robot1->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame());
	////test2->GetBaseLink()->SetFrame(robot1->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame()*SE3(Vec3(0.0,0.0,0.03)));

	////if (cnt == 0)
	////	cout << robot1->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame() << endl;
	////cnt++;

	////static int temp = 0;
	////if (cnt % 30 == 0)
	////	temp++;
	////int idx = temp % initPos.size();
	////rManager1->setJointVal(initPos[idx]);
	//int idx = 1;
	//int objNum = idx / 2;

	//for (unsigned int j = 0; j < goalSE3.size(); j++)
	//{
	//	if (j < objNum)
	//		busbar[j]->setBaseLinkFrame(goalSE3[j]);
	//	else
	//		busbar[j]->setBaseLinkFrame(initSE3[j]);
	//	if (j == objNum && idx % 2 == 1)
	//		busbar[j]->setBaseLinkFrame(goalSE3[j]);
	//}
	//if (idx % 2 == 0)
	//	busbar[objNum]->GetBaseLink()->SetFrame(robot1->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame() / Tbusbar2gripper);

	////cout << "taskIdx: " << idx << endl;
	//cout << gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP() << endl;

}

void updateFuncPlanning()
{
	
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();

	Eigen::VectorXd gripInput(2);
	gripInput[0] = -0.009;
	gripInput[1] = 0.009;
	rManager1->setGripperPosition(gripInput);
	rManager2->setGripperPosition(gripInput);
	rManager2->setJointVal(jointVal);

	static int cnt = 0;
	static int writeCnt = 0;
	static int writeTaskCnt = 0;

	static int trjIdx = 0;
	static int taskIdx = 0;

	int idx = taskIdx % traj.size();
	if (idx == 0 && cnt > 0)
		cnt = 0;
	if (cnt == 0)
	{
		for (unsigned int i = 0; i < busbar.size(); i++)
			busbar[i]->setBaseLinkFrame(initSE3[i]);
		cnt++;
	}
	
	// Calculate acc
	if (trjIdx == traj[idx].size() - 1)
		jointAcc = (traj[idx][trjIdx ] - 2.0*traj[idx][trjIdx - 1] + traj[idx][trjIdx - 2]) / (gSpace.m_Timestep_dyn_fixed*gSpace.m_Timestep_dyn_fixed);
	else if (trjIdx == traj[idx].size() - 2)
		jointAcc = (traj[idx][trjIdx+1] - 2.0*traj[idx][trjIdx] + traj[idx][trjIdx - 1]) / (gSpace.m_Timestep_dyn_fixed*gSpace.m_Timestep_dyn_fixed);
	else
		jointAcc = (traj[idx][trjIdx + 2] - 2.0*traj[idx][trjIdx + 1] + traj[idx][trjIdx]) / (gSpace.m_Timestep_dyn_fixed*gSpace.m_Timestep_dyn_fixed);
	
	SE3 renderEndeff = rManager1->forwardKin(traj[idx][trjIdx], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);


	// Calculate init vel
	if (trjIdx < traj[idx].size() - 1)
	{
		jointVel = (traj[idx][trjIdx + 1] - traj[idx][trjIdx]) / gSpace.m_Timestep_dyn_fixed;
		rManager1->setJointValVel(traj[idx][trjIdx], jointVel);
	}
	else
		jointVel = (traj[idx][trjIdx] - traj[idx][trjIdx]) / gSpace.m_Timestep_dyn_fixed;
	
	// control acceleration
	//rManager1->controlJointAcc(jointAcc);

	

	Eigen::VectorXd tau = rManager1->inverseDyn(traj[idx][trjIdx], jointVel, jointAcc);
	//rManager1->setJointVal(traj[idx][trjIdx]);
	rManager1->controlJointTorque(tau);

	//busbar movement
	if (attachObject[taskIdx % traj.size()])
	{
		busbar[0]->setBaseLinkFrame(rManager1->m_activeArmInfo->m_endeffector[0]->GetFrame() * Inv(Tbusbar2gripper_new));
		busbar[0]->setBaseLinkFrame(renderEndeff * Inv(Tbusbar2gripper_new));
	}
	else
		busbar[0]->setBaseLinkFrame(initBusbar);

	// read sensor value
	Ftsensor = rManager1->readSensorValue();
	dse3 Fr(0.0);
	se3 g(0.0);
	se3 Vdot(0.0);
	se3 V(0.0);
	V = Vectortose3( rManager1->getBodyJacobian(traj[idx][trjIdx], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper_new)) * jointVel);
	Vdot = Vectortose3(rManager1->getBodyJacobian(traj[idx][trjIdx], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper_new)) * jointAcc +
		rManager1->getBodyJacobianDot(traj[idx][trjIdx], jointVel, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper_new)) * jointVel);
	for (int i = 0; i < 3; i++)
		g[i + 3] = gSpace.m_Gravity[i];
	Fr = busbar[0]->GetBaseLink()->m_Inertia * Vdot - dad(V, busbar[0]->GetBaseLink()->m_Inertia * V) - busbar[0]->GetBaseLink()->m_Inertia*InvAd(busbar[0]->GetBaseLink()->GetFrame(), g);
	
	if (attachObject[taskIdx % traj.size()])
		ftsensor = dse3toVector(Ftsensor + InvdAd((rManager1->m_ftSensorInfo[0]->m_sensorLocJoint->GetFrame() * rManager1->m_ftSensorInfo[0]->m_offset) % renderEndeff * Inv(Tbusbar2gripper_new), Fr));
	else
		ftsensor = dse3toVector(Ftsensor);


	//cout << "traj: " << traj[idx][trjIdx].transpose() << endl;
	//if (trjIdx < traj[idx].size()-1)
	//	cout << "trav: " << (traj[idx][trjIdx+1]-traj[idx][trjIdx]).transpose() << endl;
	//else
	//	cout << "trav: " << (traj[idx][trjIdx] - traj[idx][trjIdx]).transpose() << endl;
	//cout << "pos:  " << rManager1->getJointVal().transpose() << endl;
	//cout << "vel:  " << rManager1->getJointVel().transpose() << endl;
	//cout << "acc:  " << rManager1->getJointAcc().transpose() << endl;
	//cout << "ctrl: " << jointAcc.transpose() << endl << endl;
	//cout << "FT sensor:  " << ftsensor.transpose() << endl;
	//cout << "Ttraj:  " << Ttraj[idx][trjIdx] << endl << endl;




	

	//rManager1->setJointVal(traj[idx][trjIdx]);
	//int objNum = idx / 2;

	//for (unsigned int j = 0; j < goalSE3.size(); j++)
	//{
	//	if (j < objNum)
	//		busbar[j]->setBaseLinkFrame(goalSE3[j]);
	//	else
	//		busbar[j]->setBaseLinkFrame(initSE3[j]);
	//	if (j == objNum && idx % 2 == 1)
	//		busbar[j]->setBaseLinkFrame(goalSE3[j]);
	//}
	//if (idx % 2 == 0)
	//	busbar[objNum]->GetBaseLink()->SetFrame(robot1->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame() * Inv(Tbusbar2gripper));
	/*

	if (idxTraj[idx][trjIdx] != 100)
	busbar[objNum]->setBaseLinkFrame(robot1->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame() / Tbusbar2gripper);*/






	if (writeTaskCnt < traj.size())
	{
		if (writeCnt < traj[idx].size())
		{
			
			// make vector<> format
     		FTtraj[idx].push_back(ftsensor);
			// See from the robot base like frame
			TtrajVec[idx].push_back(SE3toVectorXd(Inv(robot1->GetBaseLink()->GetFrame())*Ttraj[idx][trjIdx]));
     		busbarTraj[idx].push_back(SE3toVectorXd(Inv(robot1->GetBaseLink()->GetFrame())*(busbar[0]->GetBaseLink()->GetFrame())));
			goalJigLocation[0] = SE3toVectorXd(Inv(robot1->GetBaseLink()->GetFrame())*jigAssem->GetBaseLink()->GetFrame());
			
			writeCnt++;
			
		}
		if (writeCnt == traj[idx].size())
		{
			string dir_folder = "../../../data/HYU_success3";
			// save
			string dir_temp = dir_folder;
			saveDataToText(traj[idx], dir_temp.append("/jointValTraj").append(to_string(idx)).append(".txt"));
			dir_temp = dir_folder;
			saveDataToText(FTtraj[idx], dir_temp.append("/sensorValTraj").append(to_string(idx)).append(".txt"));
			dir_temp = dir_folder;
			saveDataToText(TtrajVec[idx], dir_temp.append("/robotEndTraj_robotbase").append(to_string(idx)).append(".txt"));
			dir_temp = dir_folder;
			saveDataToText(busbarTraj[idx], dir_temp.append("/busbarTraj_robotbase").append(to_string(idx)).append(".txt"));
			dir_temp = dir_folder;
			saveDataToText(goalJigLocation, dir_temp.append("/setting_robotbase").append(".txt"));

			writeCnt = 0;
			writeTaskCnt++;

			printf("%d-th task write done!! \n", idx);
		}
		
	}
		

	trjIdx++;

	
	if (trjIdx == traj[idx].size())
	{
		trjIdx = 0;

		taskIdx++;
		cout << "taskIdx: " << taskIdx % traj.size() << endl;
	}
	if (gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP())
	{
		/*cout << taskIdx << endl;
		cout << traj[idx][trjIdx].transpose() << endl;*/
	}
	cout << gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP() << endl;
}

void environmentSetting_HYU(bool connect, int startLocation, Vec2 goalLocation) // startLocation: x location according to jig number / goalLocation[0]: jig number, goalLocation[1]: front or back
{
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
		
	SE3 Tbase = EulerZYX(Vec3(SR_PI / 6, 0.0, 0.0), Vec3(0.025, 1.095, 1.176));
		
	//SE3 Tbase = SE3(Vec3(0.025, 1.095, 1.176));

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
		if (!connect)
			gSpace.AddSystem(jig[i]);
		//for (unsigned int j = 0; j < 2; j++)
		//{
		//	busbar[2 * i + j]->setBaseLinkFrame(jigSE3[i] * jig2busbar[j]);
		//	gSpace.AddSystem(busbar[2 * i + j]);
		//}
	}
	if (!connect)
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


	SE3 initSE3_x = jigSE3[startLocation] * jig2busbar[0];
	initSE3[0] = EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(initSE3_x.GetPosition()[0], 0.654, 1.1111 + 0.0001));

	goalSE3[0] = jigSE3[goalLocation[0]] * jig2busbar[goalLocation[1]] * EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.10));

	initSE3[1] = goalSE3[0];
	goalSE3[1] = jigSE3[goalLocation[0]] * jig2busbar[goalLocation[1]];

	busbar[0]->GetBaseLink()->SetFrame(initSE3[0]);
	gSpace.AddSystem(busbar[0]);

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
	busbar[0]->setBaseLinkFrame(renderEndeff*Inv(Tbusbar2gripper));
	// read sensor value
	dse3 Ftsensor = rManager1->readSensorValue();
	dse3 Fr(0.0);
	se3 g(0.0);
	se3 Vdot(0.0);
	se3 V(0.0);
	V = Vectortose3(rManager1->getBodyJacobian(jointVal, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper)) * jointVel);
	if (cnt == 0)
		jointAcc = rManager1->getJointAcc();
	Vdot = Vectortose3(rManager1->getBodyJacobian(jointVal, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper)) * jointAcc +
		rManager1->getBodyJacobianDot(jointVal, jointVel, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper)) * jointVel);
	for (int i = 0; i < 3; i++)
		g[i + 3] = gSpace.m_Gravity[i];
	Fr = busbar[0]->GetBaseLink()->m_Inertia * Vdot - dad(V, busbar[0]->GetBaseLink()->m_Inertia * V);
	dse3 Fr_g = - (busbar[0]->GetBaseLink()->m_Inertia*InvAd(busbar[0]->GetBaseLink()->GetFrame(), g));
	se3 g_bus = InvAd(busbar[0]->GetBaseLink()->GetFrame(), g);
	SE3 Tbus = busbar[0]->GetBaseLink()->GetFrame();
	Fr += Fr_g;
	dse3 Fr_busbar = InvdAd((rManager1->m_ftSensorInfo[0]->m_sensorLocJoint->GetFrame() * rManager1->m_ftSensorInfo[0]->m_offset) % renderEndeff * Inv(Tbusbar2gripper), Fr);
	ftsensor = dse3toVector(Ftsensor + Fr_busbar);
	

	cout << "q: " << jointVal.transpose() << endl;
	cout << "a: " << rManager1->getJointAcc().transpose() << endl;
	cout << "f: " << ftsensor.transpose() << endl;
	cout << "busbar: " << endl << busbar[0]->GetBaseLink()->GetFrame() << endl;
	cout << "V: " << V << endl;
	cout << "A: " << Vdot << endl;
}

void environmentSetting_HYU2(bool connect)
{
	//SE3 Tbase = SE3(Vec3(0.025, 1.095, 1.176));		// when stage attached
	SE3 Tbase = SE3(Vec3(0.025, 1.095, 0.910 + 0.009));		// when stage removed
	double z_angle = (double)rand() / RAND_MAX * 0.0;
	double x_trans = -(double)rand() / RAND_MAX * 0.1;
	double y_trans = (double)rand() / RAND_MAX * 0.1;
	//SE3 Tbase2jigbase = EulerZYX(Vec3(z_angle, 0.0, 0.0), Vec3(x_trans, y_trans, 0.184));
	SE3 Tbase2jigbase = EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.184));
	for (unsigned int i = 0; i < busbar.size(); i++)
	{
		busbar[i] = new BusBar_HYU;
		busbar[i]->SetBaseLinkType(srSystem::FIXED);
		busbar[i]->setBaseLinkFrame(SE3(Vec3(0.0, 0.0, 10.0*(i+1))));
		gSpace.AddSystem(busbar[i]);
	}
	jigAssem->SetBaseLinkType(srSystem::FIXED);
	jigAssem->setBaseLinkFrame(Tbase*Tbase2jigbase);
	if (!connect)
		gSpace.AddSystem((srSystem*)jigAssem);
	else
	{
		srWeldJoint* wJoint = new srWeldJoint;
		wJoint->SetParentLink(workCell->GetBaseLink()); // removed stage
		//wJoint->SetParentLink(workCell->getStagePlate());
		wJoint->SetChildLink(jigAssem->GetBaseLink());
		wJoint->SetParentLinkFrame(Tbase*Tbase2jigbase);
		wJoint->SetChildLinkFrame(SE3());
	}
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

void rrtSetting()
{
	vector<srStateJoint*> planningJoints(6);
	for (unsigned int i = 0; i < planningJoints.size(); i++)
		planningJoints[i] = (srStateJoint*)robot1->gJoint[i];
	RRTManager1->setSystem(planningJoints);
	RRTManager1->setSpace(&gSpace);
	RRTManager1->setStateBound(robot1->getLowerJointLimit(), robot1->getUpperJointLimit());
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

void RRT_problemSetting()
{
	Eigen::VectorXd qInit = Eigen::VectorXd::Zero(6);
	qInit[1] = 0.65*SR_PI;
	qInit[2] = -1.1*SR_PI;

	for (unsigned int i = 0; i < initSE3.size(); i++)
	{
		allSE3_busbar[2 * i] = initSE3[i];
		allSE3_busbar[2 * i + 1] = goalSE3[i];
	}

	int flag;
	initPos.push_back(rManager1->inverseKin(allSE3_busbar[0] * Tbusbar2gripper, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flag, qInit));
	cout << initPos[0].transpose() << endl;
	printf("%d-th init inv kin flag: %d\n", 0, flag);
	for (unsigned int i = 1; i < allSE3_busbar.size(); i++)
	{
		qInit = initPos[i - 1];
		goalPos.push_back(rManager1->inverseKin(allSE3_busbar[i] * Tbusbar2gripper, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flag, qInit));
		cout << goalPos[i - 1].transpose() << endl;
		initPos.push_back(goalPos[i - 1]);
		if (i % 2 == 0)
			printf("%d-th init inv kin flag: %d\n", i / 2, flag);
		else
			printf("%d-th goal inv kin flag: %d\n", i / 2, flag);
	}
}

void RRT_problemSetting_HYU()
{
	Eigen::VectorXd qInit = Eigen::VectorXd::Zero(6);
	// elbow up
	qInit[1] = -0.65*SR_PI;
	qInit[2] = 0.3*SR_PI;
	// elbow down
	//qInit[2] = 0.6*SR_PI_HALF;
	//qInit[4] = 0.6*SR_PI_HALF;
	//qInit[3] = SR_PI_HALF;
	for (unsigned int i = 0; i < initSE3.size(); i++)
	{
		allSE3_busbar[2 * i] = initSE3[i];
		allSE3_busbar[2 * i + 1] = goalSE3[i];
	}

	int flag;
	initPos.push_back(rManager1->inverseKin(allSE3_busbar[0] * Tbusbar2gripper, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flag, qInit,1500));
	cout << initPos[0].transpose() << endl;
	printf("%d-th init inv kin flag: %d\n", 0, flag);

	qInit = initPos[0];
	goalPos.push_back(rManager1->inverseKin(allSE3_busbar[1] * Tbusbar2gripper, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flag, qInit));
	cout << goalPos[0].transpose() << endl;
	initPos.push_back(goalPos[0]);
	printf("%d-th init inv kin flag: %d\n", 1, flag);

	qInit = initPos[1];
	goalPos.push_back(rManager1->inverseKin(allSE3_busbar[3] * Tbusbar2gripper, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flag, qInit));
	cout << goalPos[1].transpose() << endl;

	//for (unsigned int i = 1; i < allSE3_busbar.size(); i++)
	//{
	//	qInit = initPos[i - 1];
	//	goalPos.push_back(rManager1->inverseKin(allSE3_busbar[i] * Tbusbar2gripper, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flag, qInit));
	//	cout << goalPos[i - 1].transpose() << endl;
	//	initPos.push_back(goalPos[i - 1]);
	//	if (i % 2 == 0)
	//		printf("%d-th init inv kin flag: %d\n", i / 2, flag);
	//	else
	//		printf("%d-th goal inv kin flag: %d\n", i / 2, flag);
	//}


}
void RRTSolve()
{
	int nDim = 6;
	vector<Eigen::VectorXd> tempTraj;
	vector<int> tempIdxTraj(0);
	vector<SE3> tempTtraj(0);
	int start = 0;		//  >= 0
	int end = wayPoints.size();		// <= 15
	vector<bool> feas(2);
	unsigned int objNum;
	bool isAttached = false;

	traj.resize(0);
	Ttraj.resize(0);
	idxTraj.resize(0);
	for (int i = start; i < end; i++)
	{
		objNum = i / 2;
		RRTManager1->setStartandGoal(initPos[i], goalPos[i]);

		for (unsigned int j = 0; j < goalSE3.size(); j++)
		{
			if (j < objNum)
				busbar[j]->setBaseLinkFrame(goalSE3[j]);
			else
				busbar[j]->setBaseLinkFrame(initSE3[j]);
		}
		if (i % 2 == 0)
		{
			isAttached = true;
			RRTManager1->attachObject(busbar[objNum], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper));
			printf("%d-th busbar moving...\n", objNum);
		}
		else
		{
			isAttached = false;
			RRTManager1->detachObject();
			busbar[objNum]->setBaseLinkFrame(goalSE3[objNum]);
			printf("%d-th busbar moved, reaching to next one...\n", objNum);
		}

		feas = RRTManager1->checkFeasibility(initPos[i], goalPos[i]);
		cout << feas[0] << feas[1] << endl;
		RRTManager1->execute(0.05);
		tempTraj = RRTManager1->extractPath();

		// check collision
		for (unsigned int j = 0; j < tempTraj.size(); j++)
			if (RRTManager1->setState(tempTraj[j]))
				cout << "collide at " << j << "th point!!!" << endl;

		traj.push_back(tempTraj);

		tempIdxTraj.resize(tempTraj.size());
		for (unsigned int j = 0; j < tempIdxTraj.size(); j++)
		{
			if (isAttached)
				tempIdxTraj[j] = objNum;
			else
				tempIdxTraj[j] = 100;
		}

		idxTraj.push_back(tempIdxTraj);

		tempTtraj.resize(tempTraj.size());
		for (unsigned int i = 0; i < traj.size(); i++)
		{
			for (unsigned int j = 0; j < traj[i].size(); j++)
				tempTtraj[j] = rManager1->forwardKin(traj[i][j], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
			Ttraj.push_back(tempTtraj);
		}
			


		
	}
}


void RRT_problemSetting(Eigen::VectorXd init, vector<SE3> wayPoints, vector<bool> includeOri, vector<bool> attachObject)
{
	Eigen::VectorXd qInit = Eigen::VectorXd::Zero(6);
	// elbow up
	qInit[1] = -0.65*SR_PI;
	qInit[2] = 0.3*SR_PI;
	qInit[3] = 0.5*SR_PI_HALF;
	// elbow down
	//qInit[2] = 0.6*SR_PI_HALF;
	//qInit[4] = 0.6*SR_PI_HALF;
	//qInit[3] = SR_PI_HALF;
	
	Eigen::VectorXd qInit2 = Eigen::VectorXd::Zero(6);
	qInit2[0] = -0.224778; qInit2[1] = -1.91949; qInit2[2] = -0.384219; qInit2[3] = 1.5708; qInit2[4] = -0.73291; qInit2[5] = 1.79557;

	int flag;
	initPos.resize(0);
	goalPos.resize(0);
	initPos.push_back(init);
	vector<bool> feas(2);
	for (unsigned int i = 0; i < wayPoints.size(); i++)
	{
		//qInit = initPos[i];
		goalPos.push_back(rManager1->inverseKin(wayPoints[i] * Tbusbar2gripper_new, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], includeOri[i], SE3(), flag, qInit2));
		//goalPos.push_back(rManager1->inverseKin(wayPoints[i], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], includeOri[i], SE3(), flag, qInit)); // ONLY END-EFFECTOR POS/ORI
		if (flag != 0)
			goalPos[i] = rManager1->inverseKin(wayPoints[i] * Tbusbar2gripper_new, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], includeOri[i], SE3(), flag, qInit);
		if (flag != 0)
			goalPos[i] = rManager1->inverseKin(wayPoints[i] * Tbusbar2gripper_new, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], includeOri[i], SE3(), flag, initPos[i]);
		printf("%d-th init inv kin flag: %d\n", i, flag);
		cout << goalPos[i].transpose() << endl;
		if (i < wayPoints.size() - 1)
			initPos.push_back(goalPos[i]);
		if (attachObject[i])
			RRTManager1->attachObject(busbar[0], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper_new));
		else
			RRTManager1->detachObject();


		feas = RRTManager1->checkFeasibility(initPos[i], goalPos[i]);
		cout << feas[0] << feas[1] << endl;
	}
}

void RRTSolve_HYU(vector<bool> attachObject, vector<double> stepsize)
{
	int nDim = 6;
	vector<Eigen::VectorXd> tempTraj;
	vector<int> tempIdxTraj(0);
	vector<SE3> tempTtraj(0);
	int start = 0;		//  >= 0
	int end = stepsize.size(); 		// <= 15
	vector<bool> feas(2);

	traj.resize(0);
	Ttraj.resize(0);
	idxTraj.resize(0);
	
	clock_t begin_time, end_time;


	for (int i = start; i < end; i++)
	{
		
		RRTManager1->setStartandGoal(initPos[i], goalPos[i]);
		
		cout << "initpos:  " << initPos[i].transpose() << endl;
		cout << "goalPos:  " << goalPos[i].transpose() << endl << endl;;
		
		if (attachObject[i])
			RRTManager1->attachObject(busbar[0], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper_new));
		else
			RRTManager1->detachObject();


		feas = RRTManager1->checkFeasibility(initPos[i], goalPos[i]);
		cout << feas[0] << feas[1] << endl;
		if (feas[0] == 1 || feas[1] == 1)
			continue;
		else
		{
			begin_time = clock(); 
			RRTManager1->execute(stepsize[i]);
			tempTraj = RRTManager1->extractPath();
			end_time = clock();

			cout << "way point RRT time: " << end_time - begin_time << endl;

			// check collision
			for (unsigned int j = 0; j < tempTraj.size(); j++)
				if (RRTManager1->setState(tempTraj[j]))
					cout << "collide at " << j << "th point!!!" << endl;

			traj.push_back(tempTraj);


			tempTtraj.resize(tempTraj.size());


			for (unsigned int j = 0; j < traj[i].size(); j++)
				tempTtraj[j] = rManager1->forwardKin(traj[i][j], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
			Ttraj.push_back(tempTtraj);

			num_task += 1;
			if (end_time - begin_time <= 1000)
			{
				num_success_task += 1;
			}
		}
		//begin_time = clock();
		//RRTManager->execute(stepsize[i]);
		//tempTraj = RRTManager->extractPath();
		//end_time = clock();
		//
		//cout << "way point RRT time: " << end_time - begin_time << endl;

		//// check collision
		//for (unsigned int j = 0; j < tempTraj.size(); j++)
		//	if (RRTManager->setState(tempTraj[j]))
		//		cout << "collide at " << j << "th point!!!" << endl;

		//traj.push_back(tempTraj);


		//tempTtraj.resize(tempTraj.size());

	
		//for (unsigned int j = 0; j < traj[i].size(); j++)
		//	tempTtraj[j] = rManager1->forwardKin(traj[i][j], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
		//Ttraj.push_back(tempTtraj);
	}
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
	rManager1->setJointVal(loadJointVal[cnt % loadJointVal.size()]);

	if (abs(loadAttachStatus[cnt % loadAttachStatus.size()][0]) < DBL_EPSILON)
	{ }
	else
	{
		busbar[0]->setBaseLinkFrame(rManager1->forwardKin(loadJointVal[cnt % loadJointVal.size()], 
			&robot1->gMarkerLink[Indy_Index::MLINK_GRIP])*Inv(Tbusbar2gripper_new));
	}
}
