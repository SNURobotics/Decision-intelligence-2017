#include <cstdio>

#include "myRenderer.h"
//#include "2ndRenderer.h"

#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\SDA20DRobotManager.h"
#include "robotManager\SDA20DRobot.h"
#include "robotManager\environment_4th.h"
#include <time.h>
#include <ctime>

#define GRID_NUM		8
#define TOTAL_DOF		7
#define MAX_ITER		100
//#define SOLVE_INVKIN
// Robot
SDA20D* sdaRobot = new SDA20D;
vector<SDA20DManager*> rManagers(8);
Bin* bin = new Bin(0.01);

Eigen::VectorXd qval;

srSpace gSpace;
myRenderer* renderer;

rrtManager* RRTManager = new rrtManager;

srLink* ee = new srLink;
srSystem* obs = new srSystem;
srJoint::ACTTYPE actType = srJoint::ACTTYPE::TORQUE;
SE3 Trobotbase1;
void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
void sdaRobotSetting();
void sdaRobotManagerSetting(int robotMode);
void sdarrtSetting();
void setSelectionMtx();
Eigen::VectorXd dropIndexOfVector(Eigen::VectorXd input, int dropidx);
int totalDof = TOTAL_DOF;
int gridNum = GRID_NUM;
vector<Eigen::MatrixXd> S(TOTAL_DOF);
int activeJointIdx = 0;

vector<Eigen::VectorXd> traj(0);


int main(int argc, char **argv)
{
	setSelectionMtx();
    sdaRobotSetting();

	//ee->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
	//ee->GetGeomInfo().SetDimension(0.03);
	//ee->GetGeomInfo().SetColor(1.0, 0.0, 0.0);
	//obs->SetBaseLink(ee);
	//obs->SetBaseLinkType(srSystem::FIXED);
	//gSpace.AddSystem(obs);
	//gSpace.AddSystem(bin);

	initDynamics();
	
	sdaRobotManagerSetting(SDA20DManager::MoveRightArmOnly);

	///////////////////////////////////////////////////////////////////////////
	/////////////////////// make grid of joint space //////////////////////////
	///////////////////////////////////////////////////////////////////////////
	
	Eigen::MatrixXd grid(totalDof, gridNum);
	//bool collision[GRID_NUM][GRID_NUM][GRID_NUM][GRID_NUM][GRID_NUM][GRID_NUM];
	//bool collision_fault[TOTAL_DOF - 1][GRID_NUM][GRID_NUM][GRID_NUM][GRID_NUM][GRID_NUM][GRID_NUM];
	
	//double manips[GRID_NUM][GRID_NUM][GRID_NUM][GRID_NUM][GRID_NUM][GRID_NUM];
	//double manips_fault[TOTAL_DOF][GRID_NUM][GRID_NUM][GRID_NUM][GRID_NUM][GRID_NUM][GRID_NUM];
	//double manips_fault_ratio[TOTAL_DOF][GRID_NUM][GRID_NUM][GRID_NUM][GRID_NUM][GRID_NUM][GRID_NUM];
	
	//int invKinFlags_fault[TOTAL_DOF - 1][GRID_NUM][GRID_NUM][GRID_NUM][GRID_NUM][GRID_NUM][GRID_NUM];
	//int invKinFlags_faultLast[GRID_NUM][GRID_NUM][GRID_NUM][GRID_NUM][GRID_NUM][GRID_NUM][GRID_NUM];
	for (int i = 0; i < totalDof; i++)
	{
		double bin = (double)(rManagers[0]->m_upperJointLimit[i] - rManagers[0]->m_lowerJointLimit[i]) / (gridNum + 1);
		for (int j = 0; j < gridNum; j++)
		{
			grid(i, j) = (j + 1) * bin + rManagers[0]->m_lowerJointLimit[i];
		}
	}

	///////////////////////////////////////////////////////////////////////////
	/////////////////////// investigate fault cases ///////////////////////////
	///////////////////////////////////////////////////////////////////////////
	vector<int> iters(7);
#ifdef SOLVE_INVKIN
	vector<int> Nsuccess(totalDof, 0);		// success
	vector<int> Nfailure1(totalDof, 0);	// exceed joint limit
	vector<int> Nfailure2(totalDof, 0);	// exceed maximum iter
	vector<int> Nfailure3(totalDof, 0);	// collision
#endif

	vector<double> mean_ManipRatio(totalDof, 0.0);
	vector<double> sum_ManipFault(totalDof, 0.0);
	double sum_Manip = 0.0;
	int NinvestigateAll = 0;
	int Nneglect = 0;
	int Ninvestigate_exceptLast = 0;
	double workSpaceVol_anglewise[TOTAL_DOF][GRID_NUM];
	double workSpaceVolRatio_anglewise[TOTAL_DOF][GRID_NUM];
	double workSpaceVol[TOTAL_DOF + 1];
	double workSpaceVolRatio[TOTAL_DOF];

	// initialize
	for (int i = 0; i < totalDof + 1; i++)
	{
		workSpaceVol[i] = 0.0;
		for (int j = 0; j < gridNum; j++)
			workSpaceVol_anglewise[i][j] = 0.0;
	}

	std::clock_t start = clock();
	for (iters[5] = 0; iters[5] < gridNum; iters[5]++)
	{
		printf("iters5: %d\n", iters[5]);
		for (iters[4] = 0; iters[4] < gridNum; iters[4]++)
		{
			printf("  iters4: %d\n", iters[4]);
			for (iters[3] = 0; iters[3] < gridNum; iters[3]++)
			{
				for (iters[2] = 0; iters[2] < gridNum; iters[2]++)
				{
					for (iters[1] = 0; iters[1] < gridNum; iters[1]++)
					{
						for (iters[0] = 0; iters[0] < gridNum; iters[0]++)
						{
							Eigen::VectorXd jointAll(totalDof);
							for (int k = 0; k < totalDof - 1; k++)
								jointAll[k] = grid(k, iters[k]);
							jointAll[totalDof - 1] = 0.0;
							// solve forward kin for normal case
							SE3 tempSE3 = rManagers[0]->forwardKin(jointAll, &sdaRobot->gMarkerLink[SDA20D_Index::MLINK_RIGHT_T]);

							// check collision
							bool isColli = rManagers[0]->checkCollision();
							//collision[iters[0]][iters[1]][iters[2]][iters[3]][iters[4]][iters[5]] = isColli;
							// calculate manipulability
							double maniptemp = rManagers[0]->manipulability(jointAll, &sdaRobot->gMarkerLink[SDA20D_Index::MLINK_RIGHT_T], robotManager::VOL);
							//manips[iters[0]][iters[1]][iters[2]][iters[3]][iters[4]][iters[5]] = maniptemp;
							if (isColli || maniptemp < 1e-15)
							{
								Nneglect += 1;
							}
							else
							{
								Ninvestigate_exceptLast += 1;
								workSpaceVol[TOTAL_DOF] += maniptemp * (double) gridNum;
								for (int i = 0; i < TOTAL_DOF; i++)
								{
									double maniptemp_i = rManagers[0]->manipulability(jointAll, &sdaRobot->gMarkerLink[SDA20D_Index::MLINK_RIGHT_T], robotManager::VOL, S[i]);
									workSpaceVol[i] += maniptemp_i * (double)gridNum;
									if (i < TOTAL_DOF - 1)
										workSpaceVol_anglewise[i][iters[i]] += maniptemp_i * (double)gridNum;
									else
									{
										for (int j = 0; j < gridNum; j++)
											workSpaceVol_anglewise[i][j] += maniptemp_i;
									}
								}
							}
#ifdef SOLVE_INVKIN
							for (iters[6] = 0; iters[6] < gridNum; iters[6]++)
							{
								Eigen::VectorXd jointAll(totalDof);
								for (int k = 0; k < totalDof; k++)
									jointAll[k] = grid(k, iters[k]);
								// solve forward kin for normal case
								SE3 tempSE3 = rManagers[0]->forwardKin(jointAll, &sdaRobot->gMarkerLink[SDA20D_Index::MLINK_RIGHT_T]);

								// check collision
								bool isColli = rManagers[0]->checkCollision();
								//collision[iters[0]][iters[1]][iters[2]][iters[3]][iters[4]][iters[5]] = isColli;
								// calculate manipulability
								double maniptemp = rManagers[0]->manipulability(jointAll, &sdaRobot->gMarkerLink[SDA20D_Index::MLINK_RIGHT_T], robotManager::VOL);
								//manips[iters[0]][iters[1]][iters[2]][iters[3]][iters[4]][iters[5]] = maniptemp;
								if (isColli || maniptemp < 1e-15)
								{
									Nneglect += 1;
									break;
								}
									
								
								if (iters[6] == gridNum / 2)
								{
									Ninvestigate_exceptLast += 1;
									sum_Manip += maniptemp;
									for (int i = 0; i < TOTAL_DOF; i++)
									{
										double maniptemp_i = rManagers[0]->manipulability(jointAll, &sdaRobot->gMarkerLink[SDA20D_Index::MLINK_RIGHT_T], robotManager::VOL, S[i]);
										//manips_fault[i][iters[0]][iters[1]][iters[2]][iters[3]][iters[4]][iters[5]] = maniptemp_i;
										//manips_fault_ratio[i][iters[0]][iters[1]][iters[2]][iters[3]][iters[4]][iters[5]] = maniptemp_i / maniptemp;
										double maniptemp_ratio = maniptemp_i / maniptemp;
										mean_ManipRatio[i] += maniptemp_ratio;
										sum_ManipFault[i] += maniptemp_i;
									}

									// solve inverse kin for fault cases (except last joint)
									for (int k = 0; k < totalDof - 1; k++)
									{
										int flag;
										rManagers[0]->setJointVal(Eigen::VectorXd::Zero(totalDof));
										Eigen::VectorXd qtemp = rManagers[k + 1]->inverseKin(tempSE3, &sdaRobot->gMarkerLink[SDA20D_Index::MLINK_RIGHT_T], true, SE3(), flag, rManagers[k + 1]->qInvKinInitActiveJoint, MAX_ITER, robotManager::QP, robotManager::DG);
										//invKinFlags_fault[k][iters[0]][iters[1]][iters[2]][iters[3]][iters[4]][iters[5]] = flag;
										bool isColli_k = rManagers[k + 1]->checkCollision();
										//collision_fault[k][iters[0]][iters[1]][iters[2]][iters[3]][iters[4]][iters[5]] = isColli_k;
										if (flag == robotManager::SOLVED && !isColli_k)
											Nsuccess[k] += 1;
										if (flag == robotManager::SOLVED_BUT_EXCEED_JOINT_LIM)
											Nfailure1[k] += 1;
										if (flag == robotManager::EXCEED_MAX_ITER)
											Nfailure2[k] += 1;
										if (flag != robotManager::EXCEED_MAX_ITER && isColli_k)
											Nfailure3[k] += 1;
									}
								}
								// total number of investigation
								NinvestigateAll += 1;
								// solve inverse kin for fault cases (last joint)
								int flag;
								rManagers[0]->setJointVal(Eigen::VectorXd::Zero(totalDof));
								Eigen::VectorXd qtemp = rManagers[totalDof]->inverseKin(tempSE3, &sdaRobot->gMarkerLink[SDA20D_Index::MLINK_RIGHT_T], true, SE3(), flag, rManagers[totalDof]->qInvKinInitActiveJoint, MAX_ITER, robotManager::QP, robotManager::DG);
								//invKinFlags_faultLast[iters[0]][iters[1]][iters[2]][iters[3]][iters[4]][iters[5]][iters[6]] = flag;
								bool isColli_k = rManagers[totalDof]->checkCollision();
								if (flag == robotManager::SOLVED && !isColli_k)
									Nsuccess[totalDof - 1] += 1;
								if (flag == robotManager::SOLVED_BUT_EXCEED_JOINT_LIM)
									Nfailure1[totalDof - 1] += 1;
								if (flag == robotManager::EXCEED_MAX_ITER)
									Nfailure2[totalDof - 1] += 1;
								if (flag != robotManager::EXCEED_MAX_ITER && isColli_k)
									Nfailure3[totalDof - 1] += 1;
#endif // SOLVE_INVKIN
						}
					}
				}
			}
		}
	}
	cout << "elapsed time for investigation: " << (std::clock() - start) / (double)CLOCKS_PER_SEC << endl;
	///////////////////////////////////////////////////////////////////////////
	///////////////////////////// statistics //////////////////////////////////
	///////////////////////////////////////////////////////////////////////////


	for (int i = 0; i < totalDof; i++)
	{
		workSpaceVolRatio[i] = workSpaceVol[i] / workSpaceVol[totalDof];
		for (int j = 0; j < gridNum; j++)
		{
			workSpaceVolRatio_anglewise[i][j] = workSpaceVol_anglewise[i][j] / workSpaceVol[i];
		}
	}

#ifdef SOLVE_INVKIN
	vector<int> Ninvestigate(totalDof, Ninvestigate_exceptLast);
	
	vector<double> mean_ManipRatio2(totalDof, 0.0);
	Ninvestigate[totalDof - 1] = NinvestigateAll;
	for (int i = 0; i < TOTAL_DOF; i++)
	{
		mean_ManipRatio[i] /= Ninvestigate_exceptLast;
		mean_ManipRatio2[i] = sum_ManipFault[i] / sum_Manip;
	}
	vector<double> successRatio(totalDof, 0.0);
	vector<double> f1Ratio(totalDof, 0.0);
	vector<double> f2Ratio(totalDof, 0.0);
	vector<double> f3Ratio(totalDof, 0.0);
	for (int i = 0; i < TOTAL_DOF; i++)
	{
		successRatio[i] = (double)Nsuccess[i] / Ninvestigate[i];
		f1Ratio[i] = (double)Nfailure1[i] / Ninvestigate[i];
		f2Ratio[i] = (double)Nfailure2[i] / Ninvestigate[i];
		f3Ratio[i] = (double)Nfailure3[i] / Ninvestigate[i];
	}
#endif // SOLVE_INVKIN
	//sdarrtSetting();

	//Eigen::VectorXd qTemp = Eigen::VectorXd::Random(rManagers[0]->m_activeArmInfo->m_numJoint);
	//Eigen::VectorXd qTemp2 = Eigen::VectorXd::Random(rManagers[0]->m_activeArmInfo->m_numJoint);

	//SE3 Ttemp = rManagers[0]->forwardKin(qTemp, &sdaRobot->gMarkerLink[SDA20D_Index::MLINK_RIGHT_T]);
	//int flag;


	//std::clock_t start = std::clock();
	//Ttemp = Ttemp * SE3(Vec3(100.0, 0.0, 0.0));
	//Eigen::VectorXd q = rManagers[0]->inverseKin(Ttemp, &sdaRobot->gMarkerLink[SDA20D_Index::MLINK_RIGHT_T], true, SE3(), flag, rManagers[0]->qInvKinInitActiveJoint, 500, robotManager::QP, robotManager::DG);
	//cout << "elapsed time for invKin: " << (std::clock() - start) / (double)CLOCKS_PER_SEC << endl;

	//start = std::clock();
	//double mainp = rManagers[0]->manipulability(qTemp, &sdaRobot->gMarkerLink[SDA20D_Index::MLINK_RIGHT_T], robotManager::VOL);
	//cout << "elapsed time for manip: " << (std::clock() - start) / (double)CLOCKS_PER_SEC << endl;

	//cout << qTemp.transpose() << endl;
	//cout << Ttemp << endl;
	//cout << q.transpose() << endl;
	//SE3 T = rManagers[0]->forwardKin(q, &sdaRobot->gMarkerLink[SDA20D_Index::MLINK_RIGHT_T]);
	//cout << T << endl;
	//RRTManager->setStartandGoal(qTemp, qTemp2);
	//cout << RRTManager->setState(qTemp) << RRTManager->setState(qTemp2);
	//RRTManager->execute(0.1);
	////rManager1->setJointVal(qval);
	//obs->GetBaseLink()->SetFrame(sdaRobot->gMarkerLink[SDA20D_Index::MLINK_RIGHT_T].GetFrame());
	//bin->setBaseLinkFrame(SE3(Vec3(1.0, 0.0, 0.0)));
	//if (RRTManager->isExecuted())
	//{
	//	traj = RRTManager->extractPath();
	//}
	
	//rManagers[0]->setJointVal(q);
	
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

	static double JointVal = 0;
	//((srStateJoint*)sdaRobot->m_KIN_Joints[activeJointIdx])->m_State.m_rValue[0] = JointVal;
	//((srStateJoint*)sdaRobot->m_KIN_Joints[5])->m_State.m_rValue[0] = JointVal;
	//JointVal += 0.01;
	obs->GetBaseLink()->SetFrame(sdaRobot->gMarkerLink[SDA20D_Index::MLINK_RIGHT_T].GetFrame());
	static int cnt = 0;
	static int trajcnt = 0;
	cnt++;

	if (cnt % 10 == 0)
		trajcnt++;
	if (traj.size() > 0)
		trajcnt++;
		//rManager1->setJointVal(traj[trajcnt % traj.size()]);

	
	//cout << sdaRobot->gMarkerLink[MH12_Index::MLINK_GRIP].GetFrame() << endl;
	//cout << sdaRobot->gLink[MH12_Index::GRIPPER].GetFrame() << endl;
	//rManager1->setJointVal(qval);


	int stop = 1;
}


void sdaRobotSetting()
{
	gSpace.AddSystem((srSystem*)sdaRobot);
	sdaRobot->GetBaseLink()->SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	sdaRobot->SetActType(srJoint::ACTTYPE::HYBRID);

	vector<int> gpIdx(2);
	gpIdx[0] = 0;
	gpIdx[1] = 1;
	sdaRobot->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);

	Trobotbase1 = sdaRobot->GetBaseLink()->GetFrame() * sdaRobot->TsrLinkbase2robotbase;
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

void sdaRobotManagerSetting(int robotMode)
{
	// set non-operating joint
	vector<srJoint*> excludeJoints(0);
	for (int i = 0; i < 8; i++)
	{
		if (i > 0)
		{
			excludeJoints.resize(1);
			excludeJoints[0] = sdaRobot->gJoint[i];
		}
		rManagers[i] = new SDA20DManager(sdaRobot, &gSpace, robotMode, excludeJoints);
	}
}

//void sdarrtSetting()
//{
//	RRTManager->setSpace(&gSpace);
//	vector<srStateJoint*> planningJoint(0);
//	for (int i = 0; i < rManager1->m_activeArmInfo->m_numJoint; i++)
//		planningJoint.push_back( (srStateJoint*) rManager1->m_activeArmInfo->m_activeJoint[i]);
//	RRTManager->setSystem(planningJoint);
//	RRTManager->setStateBound(VecToVector(rManager1->m_lowerJointLimit), VecToVector(rManager1->m_upperJointLimit));
//}
Eigen::VectorXd dropIndexOfVector(Eigen::VectorXd input, int dropidx)
{
	if (dropidx >= input.size())
		return Eigen::VectorXd();
	Eigen::VectorXd output(input.size() - 1);

	for (int i = 0, k = 0; i < input.size(); i++)
	{
		if (i != dropidx)
		{
			output[k] = input[i];
			k++;
		}
	}
}

void setSelectionMtx()
{
	Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(TOTAL_DOF, TOTAL_DOF);
	for (int i = 0; i < TOTAL_DOF; i++)
	{
		S[i] = Eigen::MatrixXd::Zero(TOTAL_DOF, TOTAL_DOF - 1);
		for (int j = 0; j < TOTAL_DOF - 1; j++)
		{
			if (j < i)
				S[i].col(j) = identity.col(j);
			else
				S[i].col(j) = identity.col(j + 1);
		}
	}
}
