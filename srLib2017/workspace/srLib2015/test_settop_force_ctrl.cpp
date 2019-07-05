#include <cstdio>

#include "myRenderer.h"
//#include "2ndRenderer.h"

#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\UR3RobotManager.h"
#include "robotManager\UR3Robot.h"
#include "robotManager\UR5RobotManager.h"
#include "robotManager\UR5Robot.h"
#include "robotManager\environment_5th.h"
#include <time.h>
#include "robotManager\robotRRTManager.h"
#include "ForceCtrlManager\hybridPFCtrlManager.h"
#include "common/dataIO.h"


srSpace gSpace;
myRenderer* renderer;
// Robot
UR3Robot* ur3 = new UR3Robot;
UR3RobotManager* ur3Manager;
UR5Robot* ur5 = new UR5Robot;
UR5RobotManager* ur5Manager;
robotRRTManager* ur3RRTManager = new robotRRTManager;
robotRRTManager* ur5RRTManager = new robotRRTManager;

hybridPFCtrlManager_6dof* hctrl = new hybridPFCtrlManager_6dof();
vector<SE3> Tdes;

HDMI* hdmi = new HDMI();
Power* power = new Power();
Settop* settop = new Settop();
Soldering* soldering = new Soldering();
PCB* pcb = new PCB();
PCBJig* pcbjig = new PCBJig();
Tape* tape = new Tape();
BoxForTape* boxfortape = new BoxForTape();

srJoint::ACTTYPE actType = srJoint::ACTTYPE::TORQUE;
SE3 T_ur3base;
SE3 T_ur5base;

void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
void URrobotSetting();
void URrobotManagerSetting();
void URrrtSetting();
void settopEnvSetting();
void setHybridPFCtrl(Vec3 initPosOffset = Vec3(0.0));
void setHybridPFCtrl_2nd(Vec3 posOffset = Vec3(0.0));
double norm_dse3(dse3 input);
// void tempObjectSetting();
Eigen::VectorXd qval;

Eigen::VectorXd point0;
Eigen::VectorXd point1;
Eigen::VectorXd point2;
Eigen::VectorXd point3;
vector<Eigen::VectorXd> ur5traj1(0);
vector<Eigen::VectorXd> ur5traj2(0);
vector<Eigen::VectorXd> ur5traj3(0);
vector<SE3> objTraj(0);
vector<Eigen::VectorXd> tempTraj(0);
srLink* ee = new srLink;
// srSystem* obs = new srSystem;

Eigen::VectorXd ur3_invkinInit = Eigen::VectorXd::Zero(6);
Eigen::VectorXd ur5_invkinInit = Eigen::VectorXd::Zero(6);
SE3 Tobs2robot = SE3();
SE3 Tsettop = EulerXYZ(Vec3(SR_PI_HALF, 0, SR_PI_HALF), Vec3(-0.25, -0.2, 0));
SE3 Tur52settop = EulerXYZ(Vec3(SR_PI_HALF, 0, SR_PI_HALF), Vec3(0.05, 0, 0));
SE3 Tur32settop_init = EulerZYX(Vec3(SR_PI_HALF, SR_PI_HALF, 0.0), Vec3(0.0, 0.0, 0.08));
SE3 Tsettop2hdmi_init = EulerZYX(Vec3(SR_PI, 0.0, 0.0), Vec3(0.047 + 0.0275, -0.02, 0.0));
SE3 Tsettop2power_init = EulerZYX(Vec3(SR_PI, 0.0, 0.0), Vec3(0.047 + 0.017, 0.017, 0.0));
#ifdef GRASP_HDMI
SE3 Tsettop2obj = Tsettop2hdmi_init;
SE3 Tee2contact = EulerZYX(Vec3(-SR_PI_HALF, -SR_PI_HALF, 0.0), Vec3(0.0, 0.0, 0.0275));
SE3 Tobj2contact = SE3(Vec3(0.0275,0.0,0.0));
SE3 Tsettop2contactGoal = SE3(Vec3(-0.009, 0.0, 0.0)) * Tsettop2hdmi_init * Tobj2contact;
#endif // GRASP_HDMI
#ifdef GRASP_POWER
SE3 Tsettop2obj = Tsettop2power_init;
SE3 Tee2contact = EulerZYX(Vec3(-SR_PI_HALF, -SR_PI_HALF, 0.0), Vec3(0.0, 0.0, 0.017));
SE3 Tobj2contact = SE3(Vec3(0.017, 0.0, 0.0));
SE3 Tsettop2contactGoal = SE3(Vec3(-0.006, 0.0, 0.0)) * Tsettop2power_init * Tobj2contact;
#endif // GRASP_POWER

vector<dse3> Fdes;
vector<Eigen::VectorXd> GripTraj(0);
vector<Eigen::VectorXd> makeGriptraj(double gripangle, Eigen::VectorXd currentPos);

int main(int argc, char **argv)
{

	///////////////////////// test file read /////////////////////////
	string str = "../../../data/environment_setting/test.txt";
	vector<int> lineNums(3);
	lineNums[0] = 1;
	lineNums[1] = 3;
	lineNums[2] = 5;
	vector<vector<double>> poss = loadDataFromTextSpecifiedLines(str, lineNums);
	//////////////////////////////////////////////////////////////////


	srand(NULL);
	// robot, object, environment settings (including AddSystem) should come before initDynamics()
    URrobotSetting();
	settopEnvSetting();
	initDynamics();

	// robotManager setting should come after initDynamics()
	URrobotManagerSetting();

	ur3Manager->setGripperDistance(0.01);
	qval.setZero(6);
	qval[1] = -SR_PI_HALF;
	qval[3] = -SR_PI_HALF;
	qval[4] = SR_PI_HALF;
	ur3Manager->setJointVal(qval);
	
	// rrtSetting should come after robotManager setting
	URrrtSetting();
	
	// place object in space
	// obs->GetBaseLink()->SetFrame(EulerXYZ(Vec3(0, 0, -SR_PI / 2), Vec3(-0.5, -0.8, 0.12)));
	cout << ur5Manager->forwardKin(qval, &ur5->gMarkerLink[UR5_Index::MLINK_GRIP]) << endl;

	int flag = 0;
	
	ur5_invkinInit[0] = -1.754548; ur5_invkinInit[1] = 2.409134; ur5_invkinInit[2] = -1.813758;
	ur5_invkinInit[3] = -2.546216; ur5_invkinInit[4] = -1.754548; ur5_invkinInit[5] = 3.141593;
	point2 = ur5Manager->inverseKin(Tsettop / Tur52settop, &ur5->gMarkerLink[UR5_Index::MLINK_GRIP], true, SE3(), flag, ur5_invkinInit);
	cout << point2.transpose() << endl;
	cout << flag << endl;
	
	ur5RRTManager->attachObject(settop, &ur5->gMarkerLink[UR5_Index::MLINK_GRIP], Tur52settop);		// attaching object occurs here
	ur5Manager->setJointVal(point2);
	settop->setBaseLinkFrame(Tsettop);

	cout << ur3->gMarkerLink[UR3_Index::MLINK_GRIP].GetFrame() << endl;
	

	ur3_invkinInit[0] = -2.796488; ur3_invkinInit[1] = -SR_PI_HALF; ur3_invkinInit[2] = 1.787395;
	ur3_invkinInit[3] = -1.75; ur3_invkinInit[4] = -1.570796; ur3_invkinInit[5] = 1.915901;

	Eigen::VectorXd q = ur3Manager->inverseKin(SE3(Vec3(0.0,0.0,-0.005)) * Tsettop * Tsettop2obj, &ur3->gLink[UR3_Index::OBJECT], true, SE3(), flag, ur3_invkinInit);
	//ur3Manager->setJointVal(Eigen::VectorXd::Zero(6));
	cout << q.transpose() << endl;
	cout << flag << endl;
	ur3Manager->setJointVal(q);
	SE3 Tur32hdmi = EulerZYX(Vec3(-SR_PI_HALF, -SR_PI_HALF, 0.0), Vec3(0.0, 0.0, 0.0));
	//hdmi->setBaseLinkFrame(ur3->gMarkerLink[UR3_Index::MLINK_GRIP].GetFrame()*EulerXYZ(Vec3(0, 0, 0), Vec3(0.0, 0.0, 0.1)));
	//power->setBaseLinkFrame(ur3->gMarkerLink[UR3_Index::MLINK_GRIP].GetFrame()*Tur32hdmi);
	//hdmi->setBaseLinkFrame(SE3(Vec3(0.0, 0.1, 0.0)));
	//power->setBaseLinkFrame(SE3(Vec3(0.0, 0.1, 0.1)));

	Vec3 initPosOffset(0.0);
	initPosOffset[0] = 0.0025;
	initPosOffset[1] = 0.0025;
	initPosOffset[2] = 0.05;
	setHybridPFCtrl(initPosOffset);
	cout << ur3->gMarkerLink[UR3_Index::MLINK_GRIP].GetFrame() * Tee2contact << endl;
	cout << Tsettop * Tsettop2contactGoal << endl;

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
	//GripTraj = makeGriptraj(30, tempTraj.back());
	//ur5traj.insert(ur5traj.end(), GripTraj.begin(), GripTraj.end());
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();

	static int cnt = 0;
	cnt++;
	ur3Manager->setGripperDistance(0.01);
	if (cnt == 1)
		cout << gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP();

	// hybrid position force control
	clock_t start = clock();
	hctrl->hybridPFControl();


	static bool contactOccurred = false;


	// output current contact force
	srLink* contactLink = &ur3->gLink[UR3_Index::OBJECT];
	SE3 Ttemp = ur3->gMarkerLink[UR3_Index::MLINK_GRIP].GetFrame() * Tee2contact;
	dse3 Fext = -InvdAd(Ttemp % contactLink->GetFrame(), contactLink->m_ConstraintImpulse) * (1.0 / ur3Manager->m_space->m_Timestep_dyn_fixed);
	
	if (norm_dse3(Fext) > 1e-5 && !contactOccurred)
	{
		contactOccurred = true;
		setHybridPFCtrl_2nd();
	}

	// calculate error rate
	double Fdes_norm = norm_dse3(Fdes[Fdes.size() - 1]);
	dse3 Fdiff = Fext - Fdes[Fdes.size() - 1];
	double error_rate = norm_dse3(Fdiff) / Fdes_norm;
	double error_rate_Fx = Fdiff[3] / Fdes[Fdes.size() - 1][3];
	double error_rate_Fnorm = (norm_dse3(Fext) - Fdes_norm) / Fdes_norm;
	
	double calc_time = (clock() - start) / (double)CLOCKS_PER_SEC;
	cout << Fext << "calculation time: " << calc_time << "(sec),     error rate: " << error_rate_Fnorm << endl;
}


void URrobotSetting()
{

	// ur3 setting
	gSpace.AddSystem((srSystem*)ur3);
	ur3->GetBaseLink()->SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	ur3->SetActType(srJoint::ACTTYPE::TORQUE);

	vector<int> gpIdx(2);
	gpIdx[0] = 0;
	gpIdx[1] = 1;
	ur3->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);

	T_ur3base = ur3->GetBaseLink()->GetFrame() * ur3->TsrLinkbase2robotbase;

	// ur5 setting
	gSpace.AddSystem((srSystem*)ur5);
	ur5->GetBaseLink()->SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(-1.0, 0.0, 0.0)));
	ur5->SetActType(srJoint::ACTTYPE::HYBRID);
	// add gripper setting for ur5 later
	//vector<int> gpIdx(2);
	//gpIdx[0] = 0;
	//gpIdx[1] = 1;
	//ur5->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);
	T_ur5base = ur5->GetBaseLink()->GetFrame() * ur5->TsrLinkbase2robotbase;
}

void URrobotManagerSetting()
{
	ur3Manager = new UR3RobotManager(ur3, &gSpace);
	ur5Manager = new UR5RobotManager(ur5, &gSpace);
}

void URrrtSetting()
{
	ur3RRTManager->setSpace(&gSpace);
	vector<srStateJoint*> ur3planningJoint(6);
	for (int i = 0; i < 6; i++)
		ur3planningJoint[i] = (srStateJoint*)ur3->gJoint[i];
	ur3RRTManager->setSystem(ur3planningJoint);
	ur3RRTManager->setStateBound(ur3->getLowerJointLimit(), ur3->getUpperJointLimit());

	ur5RRTManager->setSpace(&gSpace);
	vector<srStateJoint*> ur5planningJoint(6);
	for (int i = 0; i < 6; i++)
		ur5planningJoint[i] = (srStateJoint*)ur5->gJoint[i];
	ur5RRTManager->setSystem(ur5planningJoint);
	ur5RRTManager->setStateBound(ur5->getLowerJointLimit(), ur5->getUpperJointLimit());
}

//void tempObjectSetting()
//{
//	double dim = 0.05;
//	ee->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
//	ee->GetGeomInfo().SetDimension(dim);
//	ee->GetGeomInfo().SetColor(1.0, 0.0, 0.0);
//	srCollision* tempCol = new srCollision;
//	tempCol->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
//	tempCol->GetGeomInfo().SetDimension(dim);
//	ee->AddCollision(tempCol);
//	settop->SetBaseLink(ee);
//	settop->SetBaseLinkType(srSystem::FIXED);
//	gSpace.AddSystem(settop);
//}

vector<Eigen::VectorXd> makeGriptraj(double gripangle, Eigen::VectorXd currentPos)
{
	vector<Eigen::VectorXd> gripTraj(0);
	for (int i = 0; i < 10; i++)
	{
		Eigen::VectorXd tempPos  = currentPos;
		cout << tempPos << endl;
		tempPos[6] += gripangle / 180 * SR_PI / 10;
		tempPos[10] += gripangle / 180 * SR_PI / 10;
		tempPos[14] += gripangle / 180 * SR_PI / 10;
		gripTraj.push_back(tempPos);
	}
	return gripTraj;
}

void settopEnvSetting()
{
	settop->setBaseLinkFrame(SE3(Vec3(-0.5, -0.3, 0)));
	soldering->setBaseLinkFrame(EulerXYZ(Vec3(0, SR_PI / 2, 0), Vec3(-0.5, -0.8, 0.12)));
	pcb->setBaseLinkFrame(EulerXYZ(Vec3(0, SR_PI / 2, 0), Vec3(-0.2, 0.5, 0)));
	pcbjig->setBaseLinkFrame(EulerXYZ(Vec3(0, 0, 0), Vec3(-1, -1, 0.31)));
	tape->setBaseLinkFrame(EulerXYZ(Vec3(0, 0, -SR_PI_HALF), Vec3(-0.5, 0.5, 0)));
	boxfortape->setBaseLinkFrame(EulerXYZ(Vec3(0, 0, -SR_PI_HALF), Vec3(-0.4, -0.5, 0)));

	gSpace.AddSystem(hdmi);
	gSpace.AddSystem(power);
	gSpace.AddSystem(settop);
	//gSpace.AddSystem(soldering);
	//gSpace.AddSystem(pcb);
	//gSpace.AddSystem(pcbjig);
	//gSpace.AddSystem(tape);
	//gSpace.AddSystem(boxfortape);
}

void setHybridPFCtrl(Vec3 initPosOffset)
{
	// initial config should be aligned to the contact plane
	// assume target object is rigidly attached to robot end-effector
	hctrl->isSystemSet = hctrl->setSystem((robotManager*)ur3Manager, &ur3->gMarkerLink[UR3_Index::MLINK_GRIP], Tee2contact, &ur3->gLink[UR3_Index::OBJECT]);
	hctrl->setTimeStep(ur3Manager->m_space->m_Timestep_dyn_fixed);
	double kv_v = 0.25e2, kp_v = 0.25*kv_v*kv_v, ki_v = 0.25e3, kp_f = 1.0e-1, ki_f = 1.0e-1;
	hctrl->setGain(kv_v, kp_v, ki_v, kp_f, ki_f);

	// S*V = 0 should be satisfied in contact frame
	// pos controlled dir: trans y, z, rot x
	// force controlled dir: moment y, z, force x
	Eigen::MatrixXd S = Eigen::MatrixXd::Zero(3, 6);
	S(0, 1) = 1.0;
	S(1, 2) = 1.0;
	S(2, 3) = 1.0;

	//hctrl->setSelectionMatrix(S);
	hctrl->setSelectionMatrix(Eigen::MatrixXd());	//Eigen::MatrixXd(), S


	// set desired trajectory (trajectory of the contact frame)
	//SE3 TgoalPos = Tsettop * Tsettop2obj * Tobj2contact;
	//TgoalPos = SE3(initPosOffset) * TgoalPos;
	Tdes.resize(1);
	//Tdes[0] = TgoalPos;
	Tdes[0] = SE3(Vec3(initPosOffset[0], initPosOffset[1], 0.0)) * Tsettop * Tsettop2contactGoal;

	Fdes.resize(1);
	Fdes[0] = dse3(0.0);		// expressed in contact frame
	Fdes[0][1] = 0.0;
	Fdes[0][2] = 0.0;
	Fdes[0][3] = 1.0;
	int flag;
	Eigen::VectorXd q_config = ur3Manager->inverseKin(SE3(Vec3(initPosOffset)) * Tsettop * Tsettop2obj, &ur3->gLink[UR3_Index::OBJECT], true, SE3(), flag, ur3_invkinInit);
	hctrl->isDesTrjSet = hctrl->setDesiredTraj(Tdes, Fdes);
	hctrl->setDesiredJointVal(q_config);
	hctrl->F_int = dse3(0.0);
	hctrl->X_int = se3(0.0);
	ur3Manager->setJointValVelAcc(q_config, Eigen::VectorXd::Zero(q_config.size()), Eigen::VectorXd::Zero(q_config.size()));
}

void setHybridPFCtrl_2nd(Vec3 posOffset)
{
	double kv_v = 0.25e2, kp_v = 0.25*kv_v*kv_v, ki_v = 0.25e3, kp_f = 1.0e-1, ki_f = 1.0e-1;
	hctrl->setGain(kv_v, kp_v, ki_v, kp_f, ki_f);
	// S*V = 0 should be satisfied in contact frame
	// pos controlled dir: trans y, z, rot x
	// force controlled dir: moment y, z, force x
	Eigen::MatrixXd S = Eigen::MatrixXd::Zero(3, 6);
	S(0, 1) = 1.0;
	S(1, 2) = 1.0;
	S(2, 3) = 1.0;
	hctrl->setSelectionMatrix(S);

	// set desired trajectory (trajectory of the contact frame)
	SE3 TgoalPos;
	TgoalPos = Tsettop * Tsettop2obj * Tobj2contact;
	Tdes.resize(1);
	TgoalPos = SE3(Vec3(posOffset[0], posOffset[1], 0.0)) * TgoalPos;
	//Tdes[0] = TgoalPos;
	Tdes[0] = SE3(Vec3(posOffset[0], posOffset[1], 0.0)) * Tsettop * Tsettop2contactGoal;

	Fdes.resize(1);
	Fdes[0] = dse3(0.0);		// expressed in contact frame
	Fdes[0][1] = 0.0;
	Fdes[0][2] = 0.0;
	Fdes[0][3] = 1.0;
	int flag;
	hctrl->isDesTrjSet = hctrl->setDesiredTraj(Tdes, Fdes);
	hctrl->F_int = dse3(0.0);
	hctrl->X_int = se3(0.0);
	Eigen::VectorXd q_config = ur3Manager->getJointVal();
	ur3Manager->setJointValVelAcc(q_config, Eigen::VectorXd::Zero(q_config.size()), Eigen::VectorXd::Zero(q_config.size()));
}

double norm_dse3(dse3 input)
{
	double output = 0.0;
	for (int i = 0; i < 6; i++)
		output += input[i] * input[i];
	return sqrt(output);
}
