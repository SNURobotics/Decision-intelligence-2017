///////////////////////////////////////////////////////////////////////////////////
// 4차년도 동영상 촬영 main
// 시나리오: 작업 대상물 3개를 bin 안에 랜덤하게 놓은 후 제일 위에 물체부터 집어서 밖으로 빼는 작업
///////////////////////////////////////////////////////////////////////////////////

#include <cstdio>
#include "myRenderer.h"
//#include "2ndRenderer.h"

#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\UR3RobotManager.h"
#include "robotManager\UR3Robot.h"
#include "robotManager\environment_4th.h"
#include "robotManager\robotRRTManager.h"
#include "sort_external_lib.h"

#include <time.h>
#include <random>


// Robot
UR3Robot* URRobot = new UR3Robot;
UR3RobotManager* rManager1;
srJoint::ACTTYPE actType = srJoint::ACTTYPE::TORQUE;
SE3 Trobotbase1;
Eigen::VectorXd homePos;

// Planning
robotRRTManager* RRTManager = new robotRRTManager;
vector<vector<Eigen::VectorXd>> traj(0);
vector<bool> attachObject;
vector<SE3> wayPoints(0);
vector<Eigen::VectorXd> initPos(0);
vector<Eigen::VectorXd> goalPos(0);
Eigen::VectorXd jointVal(6);
Eigen::VectorXd jointAcc(6);
Eigen::VectorXd jointVel(6);


// object
Bin* bin = new Bin(0.01);
int nWorkingObj = 3;
vector<workingObject*> workingObj(nWorkingObj);
vector<SE3> initSE3(nWorkingObj);
SE3 TworkingObj2robotEE = EulerZYX(Vec3(0.0, 0.0, SR_PI), Vec3(0.0, 0., 0.03));
vector<size_t> positionZ_ordered_index; // z축 position 높은 순으로 정렬된 index -> positionZ_ordered_index[i]가 꺼내야하는 object 순서

// srlib
srSpace gSpace;
myRenderer* renderer;

void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
void updateFuncPlanning();
void URrobotSetting();
void URrobotManagerSetting();
void URrrtSetting();
int activeJointIdx =0;
pair<vector<int>, vector<vector<bool>>>  RRT_problemSetting(Eigen::VectorXd init, vector<SE3> wayPoints, vector<bool> includeOri, vector<bool> attachObject, vector<size_t> objectIdx);
void RRTSolve_HYU(vector<bool> attachObject, vector<double> stepsize, vector<size_t> objectIdx);

int main(int argc, char **argv)
{

	// robot and object system setting
    URrobotSetting();

	gSpace.AddSystem(bin);
	for (int i = 0; i < workingObj.size(); i++)
	{
		workingObj[i] = new workingObject;
		gSpace.AddSystem(workingObj[i]);
	}

	initDynamics();

	URrobotManagerSetting();

	rManager1->setGripperDistance(0.01);

	// initial robot home position
	homePos.setZero(6);
	homePos[1] = -SR_PI_HALF;
	homePos[3] = -SR_PI_HALF;
	rManager1->setJointVal(homePos);

	// bin location
	bin->setBaseLinkFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.0, 0.5, 0.3)));

	// randomly generate working object location
	vector<pair<double, double>> randNumRangeSet_obj;
	randNumRangeSet_obj.push_back(make_pair(-0.135 + 0.07 , 0.135 - 0.07));// x range
	randNumRangeSet_obj.push_back(make_pair(-0.205 + 0.07, 0.205 - 0.07));// y range
	randNumRangeSet_obj.push_back(make_pair(0.003, 0.003 + 0.114));// z range
	randNumRangeSet_obj.push_back(make_pair(-10.0*SR_PI / 180.0, 10.0*SR_PI / 180.0));// theta x range
	randNumRangeSet_obj.push_back(make_pair(-10.0*SR_PI / 180.0, 10.0*SR_PI / 180.0));// theta y range
	randNumRangeSet_obj.push_back(make_pair(0.0, 2 * SR_PI));// theta z range 
	double randNumSet_obj[6];

	vector<pair<double, double>> randNumRangeSet_bin;
	randNumRangeSet_bin.push_back(make_pair(-0.5, 0.5));// x range
	randNumRangeSet_bin.push_back(make_pair(-0.5, 0.5));// y range
	randNumRangeSet_bin.push_back(make_pair(0.0, 0.5));// z range
	randNumRangeSet_bin.push_back(make_pair(0.0, 2 * SR_PI));// theta z range 
	double randNumSet_bin[4];
	bool isNoCollision= false;
	bool isInvKinSolved = false;
	bool isRRTSetting = false;

	int nWay = 2 * nWorkingObj;
	vector<double> stepsize(nWay, 0.05);
	while (1)
	{
		
		// bin placement
		std::random_device rd;
		std::mt19937 eng(rd());
		for (int i = 0; i < 4; i++)
		{
			std::uniform_real_distribution<> distr(randNumRangeSet_bin[i].first, randNumRangeSet_bin[i].second);
			randNumSet_bin[i] = distr(eng);
		}
		bin->setBaseLinkFrame(EulerZYX(Vec3(randNumSet_bin[3], 0.0, 0.0), Vec3(randNumSet_bin[0], randNumSet_bin[1], randNumSet_bin[2])));

		// obj placement
		for (int k = 0; k < workingObj.size(); k++)
		{
			std::random_device rd;
			std::mt19937 eng(rd());
			for (int i = 0; i < 6; i++)
			{
				std::uniform_real_distribution<> distr(randNumRangeSet_obj[i].first, randNumRangeSet_obj[i].second);
				randNumSet_obj[i] = distr(eng);
			}
			SE3 Tbin2obj = EulerZYX(Vec3(randNumSet_obj[5], randNumSet_obj[4], randNumSet_obj[3]), Vec3(randNumSet_obj[0], randNumSet_obj[1], randNumSet_obj[2]));
			workingObj[k]->setBaseLinkFrame(bin->getBaseLinkFrame()*Tbin2obj);
		}
		
		cout << "Random object location collision check (if 0, pass): " << gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP() << endl;
	

		if (!gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP())
		{
			isNoCollision = true;
			// initial Position setting
			for (int i = 0; i < workingObj.size(); i++)
				initSE3[i] = workingObj[i]->getBaseLinkFrame();

			// ------------------------- Planning setting ------------------------------
			// Object order setting (top object first)
			vector<double> workingObjs_positionZ;
			for (int i = 0; i < workingObj.size(); i++)
			{
				workingObjs_positionZ.push_back(workingObj[i]->getBaseLinkFrame().GetPosition()[2]);
			}
			vector<double> workingObjs_positionZ_sorted;

			sort_external_reverse(workingObjs_positionZ, workingObjs_positionZ_sorted, positionZ_ordered_index);

			// way point setting 
			vector<bool> includeOri(nWay, true);
			wayPoints.resize(nWay);
			attachObject.resize(nWay);
			for (int i = 0; i < workingObj.size(); i++)
			{
				wayPoints[2 * i] = workingObj[positionZ_ordered_index[i]]->getBaseLinkFrame() * TworkingObj2robotEE;
				attachObject[2 * i] = false;
				SE3 Tbin2objPlacement = EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(-0.4, 0.12 - 0.12*i, 0.1));
				wayPoints[2 * i + 1] = Tbin2objPlacement * TworkingObj2robotEE;
				attachObject[2 * i + 1] = true;
			}
			URrrtSetting();
			pair<vector<int>, vector<vector<bool>>> RRT_setting_output;
			RRT_setting_output = RRT_problemSetting(homePos, wayPoints, includeOri, attachObject, positionZ_ordered_index);
			
			// inverse kinematics check
			int flagCnt = 0;
			for (int i = 0; i < RRT_setting_output.first.size(); i++)
			{
				flagCnt += RRT_setting_output.first[i];
			}
			if (flagCnt == 0)
				isInvKinSolved = true;
			int feasCnt = 0;
			for (int i = 0; i < RRT_setting_output.second.size(); i++)
			{
				for (int j = 0; j < RRT_setting_output.second[i].size(); j++)
				{
					feasCnt += RRT_setting_output.second[i][j];
				}
			}
			if (feasCnt == 0)
				isRRTSetting = true;

			if (isNoCollision && isInvKinSolved && isRRTSetting)
			{
				break;
			}
			else
			{
				isNoCollision = false;
				isInvKinSolved = false;
				isRRTSetting = false;
			}
		}
	}

	RRTSolve_HYU(attachObject, stepsize, positionZ_ordered_index);


	int flag;
	
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
	//renderer->setUpdateFunc(updateFunc);
	renderer->setUpdateFunc(updateFuncPlanning);

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

	static double JointVal = 0;
	//((srStateJoint*)URRobot->m_KIN_Joints[activeJointIdx])->m_State.m_rValue[0] = JointVal;
	//((srStateJoint*)URRobot->m_KIN_Joints[0])->m_State.m_rValue[0] = JointVal;
	JointVal += 0.01;

	static int cnt = 0;
	static int trajcnt = 0;
	cnt++;

	//if (cnt % 10 == 0)
	//	trajcnt++;
	//if (traj.size() > 0)
	//	rManager1->setJointVal(traj[trajcnt % traj.size()]);

	
	//cout << URRobot->gMarkerLink[UR3_Index::MLINK_GRIP].GetFrame() << endl;
	//cout << URRobot->gLink[UR3_Index::GRIPPER].GetFrame() << endl;
	//rManager1->setJointVal(qval);

	// check inv dyn with gripper
	//Eigen::VectorXd gripInput(2);
	//gripInput[0] = -1.5;
	//gripInput[1] = 1.5;
	//rManager1->setGripperInput(gripInput);
	//rManager2->setGripperInput(gripInput);
	////cout << gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP() << endl;
	//Eigen::VectorXd acc(6);
	//acc.setRandom();
	//Eigen::VectorXd tau = rManager1->inverseDyn(rManager1->getJointVal(), rManager1->getJointVel(), acc);
	//rManager1->controlJointAcc(acc);
	//rManager2->controlJointTorque(tau);

	//cout << "robot2 acc:  " << rManager2->getJointAcc().transpose() << endl << endl;
	////cout << "robot1 tau:  " << rManager1->getJointTorque().transpose() << endl;
	//cout << "desired acc: " << acc.transpose() << endl;
	////cout << "desired tau: " << tau.transpose() << endl;


	// check sensor val
	//Eigen::VectorXd acc(6);
	//acc.setZero();
	//rManager1->controlJointAcc(acc);
	//Eigen::VectorXd tau = rManager1->inverseDyn(rManager1->getJointVal(), rManager1->getJointVel(), acc);
	//rManager2->controlJointTorque(tau);
	//cout << "Fsensor1: " << rManager1->readSensorValue() << endl;
	//cout << "Fext1:    " << rManager1->m_ftSensorInfo[0]->getExtForce() << endl;
	//cout << "Fsensor2: " << rManager2->readSensorValue() << endl;
	//cout << "Fext2:    " << rManager2->m_ftSensorInfo[0]->getExtForce() << endl;
	int stop = 1;
}


void URrobotSetting()
{
	gSpace.AddSystem((srSystem*)URRobot);
	URRobot->GetBaseLink()->SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	URRobot->SetActType(srJoint::ACTTYPE::HYBRID);

	vector<int> gpIdx(2);
	gpIdx[0] = 0;
	gpIdx[1] = 1;
	URRobot->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);

	Trobotbase1 = URRobot->GetBaseLink()->GetFrame() * URRobot->TsrLinkbase2robotbase;
	//robot1->SetActType(srJoint::ACTTYPE::HYBRID);
	//robot2->SetActType(srJoint::ACTTYPE::TORQUE);
	//vector<int> gpIdx(2);
	//gpIdx[0] = 0;
	//gpIdx[1] = 1;
	//robot1->SetGripperActType(srJoint::ACTTYPE::TORQUE, gpIdx);
	//robot2->SetGripperActType(srJoint::ACTTYPE::TORQUE, gpIdx);
	//gpIdx[0] = 2;
	//gpIdx[1] = 3;
	//robot1->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);
	//robot2->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);
}

void URrobotManagerSetting()
{

	//rManager2 = new robotManager();
	//rManager2->setRobot((srSystem*)robot2);
	//rManager2->setSpace(&gSpace);
	//rManager2->setEndeffector(&robot2->gMarkerLink[Indy_Index::MLINK_GRIP]);

	//// gripper setting
	//vector<srJoint*> gripperJoint(2);
	//gripperJoint[0] = robot2->gPjoint[Indy_Index::GRIPJOINT_L];
	//gripperJoint[1] = robot2->gPjoint[Indy_Index::GRIPJOINT_U];
	//vector<srJoint*> gripperDummyJoint(2);
	//gripperDummyJoint[0] = robot2->gPjoint[Indy_Index::GRIPJOINT_L_DUMMY];
	//gripperDummyJoint[1] = robot2->gPjoint[Indy_Index::GRIPJOINT_U_DUMMY];
	//rManager2->setGripper(gripperJoint, gripperDummyJoint);

	//// sensor setting
	//rManager2->setFTSensor(robot2->gWeldJoint[Indy_Index::WELDJOINT_GRIPPER]);

	rManager1 = new UR3RobotManager(URRobot, &gSpace);
}

void URrrtSetting()
{
	RRTManager->setSpace(&gSpace);
	vector<srStateJoint*> planningJoint(6);
	for (int i = 0; i < 6; i++)
		planningJoint[i] = (srStateJoint*)URRobot->gJoint[i];
	RRTManager->setSystem(planningJoint);
	RRTManager->setStateBound(URRobot->getLowerJointLimit(), URRobot->getUpperJointLimit());
}

pair<vector<int>, vector<vector<bool>>>  RRT_problemSetting(Eigen::VectorXd init, vector<SE3> wayPoints, vector<bool> includeOri, vector<bool> attachObject, vector<size_t> objectIdx)
{
	//Eigen::VectorXd qInit = Eigen::VectorXd::Zero(6);
	//// elbow up
	//qInit[1] = -0.65*SR_PI;
	//qInit[2] = 0.3*SR_PI;
	//qInit[3] = 0.5*SR_PI_HALF;
	//// elbow down
	////qInit[2] = 0.6*SR_PI_HALF;
	////qInit[4] = 0.6*SR_PI_HALF;
	////qInit[3] = SR_PI_HALF;
	//
	//Eigen::VectorXd qInit2 = Eigen::VectorXd::Zero(6);
	//qInit2[0] = -0.224778; qInit2[1] = -1.91949; qInit2[2] = -0.384219; qInit2[3] = 1.5708; qInit2[4] = -0.73291; qInit2[5] = 1.79557;

	int flag;
	initPos.resize(0);
	goalPos.resize(0);
	initPos.push_back(init);
	vector<bool> feas(2);

	vector<int> flagSet;
	vector<vector<bool>> feasSet;

	for (unsigned int i = 0; i < wayPoints.size(); i++)
	{
		//qInit = initPos[i];
		goalPos.push_back(rManager1->inverseKin(wayPoints[i], &URRobot->gMarkerLink[UR3_Index::MLINK_GRIP], includeOri[i], SE3(), flag, URRobot->qInvKinInit));
		//goalPos.push_back(rManager1->inverseKin(wayPoints[i], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], includeOri[i], SE3(), flag, qInit)); // ONLY END-EFFECTOR POS/ORI
		if (flag != 0)
			goalPos[i] = rManager1->inverseKin(wayPoints[i], &URRobot->gMarkerLink[UR3_Index::MLINK_GRIP], includeOri[i], SE3(), flag);
		if (flag != 0)
			goalPos[i] = rManager1->inverseKin(wayPoints[i], &URRobot->gMarkerLink[UR3_Index::MLINK_GRIP], includeOri[i], SE3(), flag, initPos[i]);
		printf("%d-th init inv kin flag: %d\n", i, flag);
		cout << goalPos[i].transpose() << endl;
		if (i < wayPoints.size() - 1)
			initPos.push_back(goalPos[i]);
		if (attachObject[i])
			RRTManager->attachObject(workingObj[objectIdx[(i-1)/2]], &URRobot->gMarkerLink[UR3_Index::MLINK_GRIP], Inv(TworkingObj2robotEE));
		else
			RRTManager->detachObject();


		feas = RRTManager->checkFeasibility(initPos[i], goalPos[i]);
		cout << feas[0] << feas[1] << endl;
		flagSet.push_back(flag);
		feasSet.push_back(feas);

	}

	pair<vector<int>, vector<vector<bool>>> output(flagSet, feasSet);
	return output;

}


void RRTSolve_HYU(vector<bool> attachObject, vector<double> stepsize, vector<size_t> objectIdx)
{
	int nDim = 6;
	vector<Eigen::VectorXd> tempTraj;
	int start = 0;		//  >= 0
	int end = stepsize.size(); 		// <= 15
	vector<bool> feas(2);

	traj.resize(0);
	clock_t begin_time, end_time;

	double total_time = 0;

	for (int i = start; i < end; i++)
	{
		begin_time = clock();
		RRTManager->setStartandGoal(initPos[i], goalPos[i]);

		cout << "initpos:  " << initPos[i].transpose() << endl;
		cout << "goalPos:  " << goalPos[i].transpose() << endl << endl;;

		if (attachObject[i])
			RRTManager->attachObject(workingObj[objectIdx[(i - 1) / 2]], &URRobot->gMarkerLink[UR3_Index::MLINK_GRIP], Inv(TworkingObj2robotEE));
		else
			RRTManager->detachObject();


		feas = RRTManager->checkFeasibility(initPos[i], goalPos[i]);
		cout << feas[0] << feas[1] << endl;
		RRTManager->execute(stepsize[i]);
		tempTraj = RRTManager->extractPath();
		end_time = clock();

		cout << "way point RRT time: " << end_time - begin_time << endl;
		total_time += end_time - begin_time;
		// check collision
		for (unsigned int j = 0; j < tempTraj.size(); j++)
			if (RRTManager->setState(tempTraj[j]))
				cout << "collide at " << j << "th point!!!" << endl;

		traj.push_back(tempTraj);
	}
	cout << "Average RRT time: " << total_time / (double)nWorkingObj << endl;
	ofstream myfile;
	myfile.open("RRT_time.txt");
	myfile << total_time / (double)nWorkingObj;
	myfile.close();
}


void updateFuncPlanning()
{

	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();

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
		for (unsigned int i = 0; i < workingObj.size(); i++)
			workingObj[i]->setBaseLinkFrame(initSE3[i]);
		cnt++;
	}
	rManager1->setGripperDistance(0.01);

	// Calculate acc
	if (trjIdx == traj[idx].size() - 1)
		jointAcc = (traj[idx][trjIdx] - 2.0*traj[idx][trjIdx - 1] + traj[idx][trjIdx - 2]) / (gSpace.m_Timestep_dyn_fixed*gSpace.m_Timestep_dyn_fixed);
	else if (trjIdx == traj[idx].size() - 2)
		jointAcc = (traj[idx][trjIdx + 1] - 2.0*traj[idx][trjIdx] + traj[idx][trjIdx - 1]) / (gSpace.m_Timestep_dyn_fixed*gSpace.m_Timestep_dyn_fixed);
	else
		jointAcc = (traj[idx][trjIdx + 2] - 2.0*traj[idx][trjIdx + 1] + traj[idx][trjIdx]) / (gSpace.m_Timestep_dyn_fixed*gSpace.m_Timestep_dyn_fixed);

	SE3 renderEndeff = rManager1->forwardKin(traj[idx][trjIdx], &URRobot->gMarkerLink[UR3_Index::MLINK_GRIP]);


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


	/////////////////////
	//busbar movement
	if (attachObject[taskIdx % traj.size()])
	{
		workingObj[positionZ_ordered_index[(taskIdx % traj.size() -1 )/2]]->setBaseLinkFrame(rManager1->m_activeArmInfo->m_endeffector[0]->GetFrame() * Inv(TworkingObj2robotEE));
		//busbar[0]->setBaseLinkFrame(renderEndeff * Inv(Tbusbar2gripper_new));
	}
	////////////////////////////////////////////////////

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
