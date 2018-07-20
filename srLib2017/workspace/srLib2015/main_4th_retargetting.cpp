#include <cstdio>

#include "myRenderer.h"
//#include "2ndRenderer.h"

#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\UR5RobotManager.h"
#include "robotManager\UR5Robot.h"
#include "robotManager\environment_4th.h"
#include <time.h>
#include "common\dataIO.h"
#include "retargettingEnvSetting.h"


// Robot
// Robot
UR5Robot* URRobot = new UR5Robot;
UR5RobotManager* rManager1;

//// Objects
//TableRetarget* tableRetarget = new TableRetarget();
//BlueMaleConnector* blueMaleConnector = new BlueMaleConnector();
//BlueFemaleConnector* blueFemaleConnector = new BlueFemaleConnector();
//RedFemaleConnector* redFemaleConnector = new RedFemaleConnector();
//RedMaleConnector* redMaleConnector = new RedMaleConnector();

// environment setting
retargetEnvironment* retargetEnv;



// Given traj
vector<Eigen::VectorXd> right_wrist_data;


srSpace gSpace;
myRenderer* renderer;

SE3 Trobotbase1;
void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
void URRobotSetting();
void URRobotManagerSetting();
void envSetting();
vector<Eigen::VectorXd> generateRetargetPath(string data_txt_path, int data_col_Num);



int main(int argc, char **argv)
{



	retargetEnv = new retargetEnvironment();

	// add robot to system
	URRobotSetting();
	// add environment to system
	envSetting();
	
	initDynamics();
	URRobotManagerSetting();

	// data preperation
	// right wrist
	string data_txt_path = "D:/Google_Drive/판단지능_ksh_local/4차년도/재평가 관련/생기원 경로/result1/right_wrist.txt";
	int dataColNum = 3;
	right_wrist_data = generateRetargetPath(data_txt_path, dataColNum);

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

	//static double JointVal = 0;
	//((srStateJoint*)MHRobot->m_KIN_Joints[activeJointIdx])->m_State.m_rValue[0] = JointVal;
	//((srStateJoint*)URRobot->m_KIN_Joints[5])->m_State.m_rValue[0] = JointVal;
	//JointVal += 0.01;
	//obs->GetBaseLink()->SetFrame(URRobot->gMarkerLink[UR5_Index::MLINK_GRIP].GetFrame());
	static int cnt = 0;
	static int trajcnt = 0;
	cnt++;

	if (cnt % 10 == 0)
		trajcnt++;
	if (right_wrist_data.size() > 0)
	rManager1->setJointVal(right_wrist_data[trajcnt % right_wrist_data.size()]);


	//cout << MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP].GetFrame() << endl;
	//cout << MHRobot->gLink[MH12_Index::GRIPPER].GetFrame() << endl;
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
	//int stop = 1;
}


void URRobotSetting()
{
	gSpace.AddSystem((srSystem*)URRobot);
	URRobot->GetBaseLink()->SetFrame(retargetEnv->Trobotbase);
	URRobot->SetActType(srJoint::ACTTYPE::HYBRID);

	vector<int> gpIdx(2);
	gpIdx[0] = 0;
	gpIdx[1] = 1;
	URRobot->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);


}

void URRobotManagerSetting()
{

	rManager1 = new UR5RobotManager(URRobot, &gSpace);

}

vector<Eigen::VectorXd> generateRetargetPath(string data_txt_path, int data_col_num)
{
	vector<Eigen::VectorXd> data_from_txt = loadDataFromText(data_txt_path, data_col_num);
	vector < Eigen::VectorXd > robotTraj(0);
	int invKinFlag;
	for (int i_frame = 0; i_frame < 155 /*data_from_txt.size()*/; i_frame++)
	{
		Vec3 dataXYZ_Vec3_txt;

		for (int i_coord = 0; i_coord < data_from_txt[i_frame].size(); i_coord++)
		{
			dataXYZ_Vec3_txt[i_coord] = data_from_txt[i_frame][i_coord];
		}

		SE3 dataXYZ_SE3_ori = SE3();
		dataXYZ_SE3_ori.SetPosition(dataXYZ_Vec3_txt);

		//cout << retargetEnv->Tworld2camera << endl;
		//SE3 AAA = retargetEnv->Tworld2camera*dataXYZ_Vec3_txt;
		//cout << AAA << endl;
		SE3 dataXYZfromWorld = retargetEnv->Tworld2camera*dataXYZ_SE3_ori;
		//cout << dataXYZfromWorld << endl;

		robotTraj.push_back(rManager1->inverseKin(dataXYZfromWorld, &URRobot->gMarkerLink[UR5_Index::MLINK_GRIP], false, SE3(), invKinFlag, URRobot->qInvKinInit));
		cout << "inverse kinematics flag: "<< invKinFlag << endl;
	}




	return robotTraj;
}


void envSetting()
{
	retargetEnv->setEnvironmentInSrSpace(&gSpace);
}
