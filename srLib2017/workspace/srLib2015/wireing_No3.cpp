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


srSpace gSpace;
myRenderer* renderer;

// Robot
UR3Robot* ur3 = new UR3Robot;
UR3RobotManager* ur3Manager;
UR5Robot* ur5 = new UR5Robot;
UR5RobotManager* ur5Manager;

// Floor
srLink* floor_link = new srLink;
srSystem* Floor = new srSystem;
srCollision* floor_colli = new srCollision;
void setFloor();

// RRTManager
robotRRTManager* ur3RRTManager = new robotRRTManager;
robotRRTManager* ur5RRTManager = new robotRRTManager;

//HDMI* hdmi = new HDMI();
//Power* power = new Power();
//Settop* settop = new Settop();
//Soldering* soldering = new Soldering();
//PCB* pcb = new PCB();
//PCBJig* pcbjig = new PCBJig();
Tape* tape = new Tape();
BoxForTape* boxfortape = new BoxForTape();
//WireBlock* wireBlock = new WireBlock();


srJoint::ACTTYPE actType = srJoint::ACTTYPE::TORQUE;
SE3 T_ur3base;
SE3 T_ur5base;

void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
void URrobotSetting();
void URrobotManagerSetting();
void URrrtSetting();
void tempObjectSetting();
Eigen::VectorXd robustInverseKinematics(SE3 finalpos, Eigen::VectorXd original, int maxiter);
Eigen::VectorXd robustInverseKinematics_UR5(SE3 finalpos, Eigen::VectorXd original, int maxiter);
Eigen::VectorXd qval;

Eigen::VectorXd point0;
Eigen::VectorXd point1;
Eigen::VectorXd point2;
Eigen::VectorXd point3;
Eigen::VectorXd point4;
Eigen::VectorXd point5;
vector<Eigen::VectorXd> ur5traj1(0);
vector<Eigen::VectorXd> ur5traj2(0);
vector<Eigen::VectorXd> ur5traj3(0);
vector<Eigen::VectorXd> ur5traj3_2(0);
vector<Eigen::VectorXd> ur5traj4(0);
vector<Eigen::VectorXd> ur5traj5(0);
vector<Eigen::VectorXd> ur5traj6(0);
vector<Eigen::VectorXd> ur3traj1(0);
vector<Eigen::VectorXd> ur3traj2(0);
vector<Eigen::VectorXd> ur3traj3(0);
vector<Eigen::VectorXd> ur3traj4(0);
vector<Eigen::VectorXd> ur3traj5(0);
vector<Eigen::VectorXd> ur3traj6(0);

vector<SE3> objTraj(0);
vector<Eigen::VectorXd> tempTraj(0);
srLink* ee = new srLink;
//srSystem* obs = new srSystem;
SE3 Tobs2robot1 = SE3();
SE3 Tobs2robot2 = SE3();
SE3 Tobs2robot3 = SE3();
SE3 Tobs2robot4 = SE3();
SE3 Tobs2robot5 = SE3();

int loopNumFlag = 0;

// Grip angle setting
double graspAngle = 0.1;
double idleAngle = -0.825148; // for UR3
Eigen::VectorXd idlePos = Eigen::VectorXd::Zero(6);
Eigen::VectorXd gripPos = Eigen::VectorXd::Zero(6);

Eigen::VectorXd UR5angle;
Eigen::VectorXd UR3angle;

vector<Eigen::VectorXd> GripTraj(0);
vector<Eigen::VectorXd> makeGriptraj(double gripangle, Eigen::VectorXd currentPos);

Lines* wire1 = new Lines();
Lines* wire2 = new Lines();
Lines* wire3 = new Lines();
Lines* wire4 = new Lines();
vector<Vec3> wireNodes(0);

int main(int argc, char **argv)
{
	srand(NULL);
	// robot, object, environment settings should come before initDynamics()
	URrobotSetting();
	//tempObjectSetting();
	//gSpace.AddSystem(hdmi);
	//gSpace.AddSystem(power);
	//gSpace.AddSystem(settop);
	//gSpace.AddSystem(soldering);
	//gSpace.AddSystem(pcb);
	//gSpace.AddSystem(pcbjig);
	gSpace.AddSystem(tape);
	gSpace.AddSystem(boxfortape);
	//gSpace.AddSystem(wireBlock);

	setFloor();

	initDynamics();

	// robotManager setting should come after initDynamics()
	URrobotManagerSetting();

	ur3Manager->setGripperDistance(0.0);
	qval.setZero(6);
	qval[1] = -SR_PI_HALF;
	qval[3] = -SR_PI_HALF;
	qval[4] = SR_PI_HALF;
	ur3Manager->setJointVal(qval);

	// rrtSetting should come after robotManager setting
	URrrtSetting();

	// place object in space
	//obs->GetBaseLink()->SetFrame(EulerXYZ(Vec3(0, 0, -SR_PI_HALF), Vec3(-0.5, -0.8, 0.12)));
	//cout << ur3Manager->forwardKin(qval, &ur3->gMarkerLink[UR3_Index::MLINK_GRIP]) << endl;

	idlePos(0) = idleAngle;
	idlePos(1) = idleAngle;
	idlePos(2) = -idleAngle;
	idlePos(3) = -idleAngle;
	idlePos(4) = -idleAngle;
	idlePos(5) = idleAngle;

	gripPos(0) = graspAngle;
	gripPos(1) = graspAngle;
	gripPos(2) = -graspAngle;
	gripPos(3) = -graspAngle;
	gripPos(4) = -graspAngle;
	gripPos(5) = graspAngle;

	Vec3 marginPos = Vec3();
	string in_line;
	ifstream in("../../../data/environment_setting/wireing_rod_position.txt");
	int i = 0;
	while (getline(in, in_line)) {
		marginPos[i] = stod(in_line);
		i++;
	}
	in.close();

	//hdmi->setBaseLinkFrame(EulerXYZ(Vec3(0, 0, SR_PI_HALF), Vec3(-0.2, -0.5, 0)));
	//power->setBaseLinkFrame(EulerXYZ(Vec3(0, 0, SR_PI_HALF), Vec3(-0.3, -0.5, 0)));
	//settop->setBaseLinkFrame(SE3(Vec3(-0.5, -0.3, 0)));
	//soldering->setBaseLinkFrame(ur3Manager->forwardKin(qval, &ur3->gMarkerLink[UR3_Index::MLINK_GRIP]));
	//pcb->setBaseLinkFrame(EulerXYZ(Vec3(0, SR_PI_HALF, 0), Vec3(-0.0, 0.35, 0.12)));
	//pcbjig->setBaseLinkFrame(EulerXYZ(Vec3(0, 0, 0), Vec3(-0.5, -0.35, 0.31)));
	tape->setBaseLinkFrame(EulerXYZ(Vec3(0, 0, 0), Vec3(-0.4, 0.3, 0.2)));
	boxfortape->setBaseLinkFrame(EulerXYZ(Vec3(0, 0, 0), marginPos));
	//wireBlock->setBaseLinkFrame(EulerXYZ(Vec3(0, 0, 0), Vec3(-0.6, -0.2, 0)));

	UR3angle = Eigen::VectorXd::Zero(6);
	in = ifstream("../../../data/environment_setting/wireing_No1_output.txt");
	i = 0;
	while (getline(in, in_line)) {
		UR3angle[i] = stod(in_line);
		i++;
	}
	in.close();
	/*UR3angle[0] = 1.075181;
	UR3angle[1] = -2.303650;
	UR3angle[2] = -2.052102;
	UR3angle[3] = -1.927433;
	UR3angle[4] = -2.066412;
	UR3angle[5] = -0.000000;*/

	UR5angle = Eigen::VectorXd::Zero(6);
	in = ifstream("../../../data/environment_setting/wireing_No2_output.txt");
	i = 0;
	while (getline(in, in_line)) {
		UR5angle[i] = stod(in_line);
		i++;
	}
	in.close();
	/*UR5angle[0] = 0.613712;
	UR5angle[1] = 0.550476;
	UR5angle[2] = 1.288623;
	UR5angle[3] = -0.268302;
	UR5angle[4] = -1.570796;
	UR5angle[5] = 0.613712;*/

	ur3Manager->setJointVal(UR3angle);
	ur5Manager->setJointVal(UR5angle);
	Eigen::VectorXd temp(6);
	temp << -1.1, -1.1, 1.1, 1.1, 1.1, -1.1;
	ur5Manager->setGripperPosition(temp);

	// T from wire zig frame
	Tobs2robot1 = EulerXYZ(Vec3(SR_PI_HALF, 0, SR_PI_HALF), Vec3(0.03, -0.020, 0.085));
	Tobs2robot2 = EulerXYZ(Vec3(-SR_PI_HALF, -SR_PI_HALF, SR_PI_HALF), Vec3(0.070, -0.015, 0.01));
	Tobs2robot3 = EulerXYZ(Vec3(SR_PI_HALF, 0, -SR_PI_HALF), Vec3(-0.0, -0.00, 0.120));

	//Tobs2robot4 = EulerXYZ(Vec3(0, -SR_PI_HALF, SR_PI), Vec3(0.020, 0.035, 0.120));
	//Tobs2robot4 = EulerXYZ(Vec3(0, SR_PI_HALF, 0), Vec3(0.020, 0.035, 0.120));
	//Tobs2robot5 = EulerXYZ(Vec3(-SR_PI_HALF, 0, SR_PI_HALF), Vec3(0.115, 0.005, 0.120));
	//cout << Tobs2robot4 << endl;

	//wire->addPoint(Vec3());
	wire1->setColor(0, 0, 0, 0.9);
	wire2->setColor(0, 0, 0, 0.9);
	wire3->setColor(0, 0, 0, 0.9);
	wire4->setColor(0, 0, 0, 0.9);
	wire1->setLineWidth(3);
	wire2->setLineWidth(3);
	wire3->setLineWidth(3);
	wire4->setLineWidth(3);


	in = ifstream("../../../data/environment_setting/wireing_wire.txt");
	i = 0;
	Vec3 tempwire = Vec3();
	while (getline(in, in_line)) {
		tempwire[i % 3] = stod(in_line);
		if (i % 3 == 2) wireNodes.push_back(tempwire);
		i++;
	}
	in.close();
	////wireNodes.push_back(Vec3(-0.210, -0.505, 0.049));
	//wireNodes.push_back(Vec3(-0.466, -0.505, 0.059));
	//wireNodes.push_back(Vec3(-0.466, -0.495, 0.060));
	//wireNodes.push_back(Vec3(-0.334, -0.495, 0.059));
	//wireNodes.push_back(Vec3(-0.334, -0.505, 0.060));
	//wireNodes.push_back(Vec3(-0.334, -0.505, 0.060));
	//wireNodes.push_back(Vec3(-0.466, -0.505, 0.061));
	//wireNodes.push_back(Vec3(-0.466, -0.495, 0.061));
	//wireNodes.push_back(Vec3(-0.334, -0.495, 0.060));
	//wireNodes.push_back(Vec3(-0.334, -0.505, 0.060));
	//wireNodes.push_back(Vec3(-0.334, -0.505, 0.060));
	//wireNodes.push_back(Vec3(-0.466, -0.505, 0.059));
	//wireNodes.push_back(Vec3(-0.466, -0.495, 0.059));

	/////////////// RRT planning to reach object (point0 -> point1) ///////////////
	clock_t start = clock();
	point0 = UR3angle;
	int flag = 0;
	ur3Manager->setGripperPosition(gripPos);
	point1 = robustInverseKinematics(boxfortape->GetBaseLink()->GetFrame() * Tobs2robot1, point0, 20);

	//cout << wireBlock->GetBaseLink()->GetFrame() * Tobs2robot1 << endl;
	//cout << ur3->gMarkerLink[UR3_Index::MLINK_GRIP].GetFrame() << endl;
	
	ur3RRTManager->setStartandGoal(point0, point1);
	ur3RRTManager->execute(0.1);
	ur3traj1 = ur3RRTManager->extractPath(20);

	// set object trajectory
	for (unsigned int i = 0; i < ur3traj1.size(); i++)
	{
		ur3RRTManager->setState(ur3traj1[i]);
		SE3 gripSE3 = ur3Manager->forwardKin(ur3traj1[i], &ur3->gMarkerLink[UR3_Index::MLINK_GRIP], SE3());
	}
	double time1 = (clock() - start) / (double)CLOCKS_PER_SEC;
	double error1 = (point1 - ur3traj1[ur3traj1.size() - 1]).norm() / point1.norm();
	//cout << "time for planning: " << time1 << endl;
	//////////////////////////////////////////////////////////////

	/////////////////// RRT planning for ur3 with object attached (point1 -> point2) ///////////////
	start = clock();
	ur3Manager->setGripperPosition(idlePos);
	point2 = robustInverseKinematics(tape->GetBaseLink()->GetFrame() * Tobs2robot2, point1, 20);

	ur3RRTManager->setStartandGoal(point1, point2);
	ur3RRTManager->execute(0.1);
	ur3traj2 = ur3RRTManager->extractPathOptimal();
	for (unsigned int i = 0; i < 3; i++) {
		Tobs2robot2 = EulerXYZ(Vec3(0.0, 0.0, 0.0), Vec3(-0.008, -0.00, -0.00)) * Tobs2robot2;
		//point2 = robustInverseKinematics(tape->GetBaseLink()->GetFrame() * Tobs2robot2, point2, 15);
		point2 = ur3Manager->inverseKin(tape->GetBaseLink()->GetFrame() * Tobs2robot2, &ur3->gMarkerLink[UR3_Index::MLINK_GRIP], true, SE3(), flag, point2);
		ur3traj2.push_back(point2);
	}
	// set object trajectory
	for (unsigned int i = 0; i < ur3traj2.size(); i++)
	{
		ur3RRTManager->setState(ur3traj2[i]);
	}
	double time2 = (clock() - start) / (double)CLOCKS_PER_SEC;
	double error2 = (point2 - ur3traj2[ur3traj2.size() - 1]).norm() / point2.norm();
	//cout << "time for planning: " << time2 << endl;
	///////////////////////////////////////////////////////////////////////////

	////////////////// RRT planning for ur5 with object detached (point2 -> point3) ///////////////
	start = clock();
	ur5Manager->setGripperPosition(temp);
	point3 = robustInverseKinematics_UR5(tape->GetBaseLink()->GetFrame() * Tobs2robot3, UR5angle, 20);

	ur5RRTManager->setStartandGoal(UR5angle, point3);
	ur5RRTManager->execute(0.1);
	//ur5traj3 = ur5RRTManager->extractPath(20);
	ur5traj3 = ur5RRTManager->extractPathOptimal();
	for (unsigned int i = 0; i < 3; i++) {
		Tobs2robot3 = EulerXYZ(Vec3(), Vec3(-0.0, -0.00, -0.005)) * Tobs2robot3;
		//point3 = robustInverseKinematics_UR5(tape->GetBaseLink()->GetFrame() * Tobs2robot3, point3, 10);
		point3 = ur5Manager->inverseKin(tape->GetBaseLink()->GetFrame() * Tobs2robot3, &ur5->gMarkerLink[UR5_Index::MLINK_GRIP], true, SE3(), flag, point3);
		//cout << ur5Manager->getBodyJacobian(point3, &ur5->gMarkerLink[UR5_Index::MLINK_GRIP], SE3()) << endl;
		ur5traj3.push_back(point3);
	}
	ur5traj3_2.clear();
	for (unsigned int i = 0; i < 3; i++) {
		ur5traj3_2.push_back(point3);
	}
	for (unsigned int i = 0; i < 6; i++) {
		Tobs2robot3 = EulerXYZ(Vec3(), Vec3(-0.01, -0.00, 0.01)) * Tobs2robot3;
		//point3 = robustInverseKinematics_UR5(tape->GetBaseLink()->GetFrame() * Tobs2robot3, point3, 10);
		point3 = ur5Manager->inverseKin(tape->GetBaseLink()->GetFrame() * Tobs2robot3, &ur5->gMarkerLink[UR5_Index::MLINK_GRIP], true, SE3(), flag, point3);
		//cout << ur5Manager->getBodyJacobian(point3, &ur5->gMarkerLink[UR5_Index::MLINK_GRIP], SE3()) << endl;
		ur5traj3_2.push_back(point3);
	}

	// set object trajectory
	for (unsigned int i = 0; i < ur5traj3.size(); i++)
	{
		ur5RRTManager->setState(ur5traj3[i]);
	}
	
	double time3 = (clock() - start) / (double)CLOCKS_PER_SEC;
	double error3 = (point3 - ur5traj3_2[ur5traj3_2.size() - 1]).norm() / point3.norm();
	//cout << "time for planning: " << time3 << endl;
	ur5Manager->setJointVal(UR5angle);
	///////////////////////////////////////////////////////////////////////////////

	////////////////// RRT planning for ur3 with object detached (point3 -> point4) ///////////////
	start = clock();
	ur3Manager->setGripperPosition(gripPos);
	//point4 = robustInverseKinematics(boxfortape->GetBaseLink()->GetFrame() * Tobs2robot4, point3);
	
	ur3RRTManager->setStartandGoal(point2, qval);
	ur3RRTManager->execute(0.1);
	//ur3traj4 = ur3RRTManager->extractPath(20);
	ur3traj4 = ur3RRTManager->extractPathOptimal();
	// set object trajectory
	for (unsigned int i = 0; i < ur3traj4.size(); i++)
	{
		ur3RRTManager->setState(ur3traj4[i]);
	}
	double time4 = (clock() - start) / (double)CLOCKS_PER_SEC;
	double error4 = (qval - ur3traj4[ur3traj4.size() - 1]).norm() / qval.norm();
	//cout << "time for planning: " << time4 << endl;
	///////////////////////////////////////////////////////////////////////////////

	cout << fixed;
	cout.precision(2);
	cout << endl;
	cout << "8. Grab wire using UR3" << endl;
	cout << "time for planning: " << time1 << " error:" << error1 << ", ";
	if (error1 * 100 <= 5 && time1 <= 0.6) cout << "success";
	else cout << "fail";
	cout << endl << endl;

	cout << "9. Put wire into wire tie machine" << endl;
	cout << "time for planning: " << time2 << " error:" << error2 << ", ";
	if (error2 * 100 <= 5 && time2 <= 0.6) cout << "success";
	else cout << "fail";
	cout << endl << endl;

	cout << "10. Tie wire with UR5" << endl;
	cout << "time for planning: " << time3 << " error:" << error3 << ", ";
	if (error3 * 100 <= 5 && time3 <= 0.6) cout << "success";
	else cout << "fail";
	cout << endl << endl;

	cout << "11. Unfold UR3 gripper & move to zero configuration" << endl;
	cout << "time for planning: " << time4 << " error:" << error4 << ", ";
	if (error4 * 100 <= 5 && time4 <= 0.6) cout << "success";
	else cout << "fail";
	cout << endl << endl;

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

	ur3Manager->setJointVal(UR3angle);
	ur5Manager->setJointVal(UR5angle);
	Eigen::VectorXd temp(6);
	temp << -1.1, -1.1, 1.1, 1.1, 1.1, -1.1;
	ur5Manager->setGripperPosition(temp);

	renderer->setUpdateFunc(updateFunc);

	renderer->addNode(wire1);
	renderer->addNode(wire2);
	renderer->addNode(wire3);
	renderer->addNode(wire4);

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
	//ur3traj.insert(ur3traj.end(), GripTraj.begin(), GripTraj.end());
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();

	static int cnt = 0;
	static int trajcnt = 0;
	cnt++;

	if (cnt % 10 == 0) {
		trajcnt++;

		// plot planned trajectory
		Eigen::VectorXd tempPos = Eigen::VectorXd::Zero(6);
		if (trajcnt == 1)
		{
			wire1->clearPoints();
			wire2->clearPoints();
			wire3->clearPoints();
			wire4->clearPoints();
			Eigen::VectorXd tempPos = Eigen::VectorXd::Zero(6);
			ur3Manager->setGripperPosition(gripPos);
		}
		if (trajcnt < ur3traj1.size())
		{
			ur3Manager->setGripperPosition(gripPos);
			wire1->clearPoints();
			for (int i = 0; i < wireNodes.size(); i++) wire1->addPoint(wireNodes[i]);
			wire1->glRender();

			ur3Manager->setJointVal(ur3traj1[trajcnt % ur3traj1.size()]);
			//wireBlock->GetBaseLink()->SetFrame(objTraj[trajcnt % ur5traj1.size()]);
			if (trajcnt == ur3traj1.size() - 1) {
				ur3Manager->setGripperPosition(idlePos);

				wire1->clearPoints();
				for (int i = 0; i < wireNodes.size(); i++) {
					wireNodes[i] = Inv(ur3->gMarkerLink[UR3_Index::MLINK_GRIP].GetFrame()) * (Vec3(0.03, -0.02, 0.026) + wireNodes[i]);
					wire1->addPoint(ur3->gMarkerLink[UR3_Index::MLINK_GRIP].GetFrame() * wireNodes[i]);
				}
				wire1->glRender();
			}
		}
		else if (trajcnt < ur3traj1.size() + ur3traj2.size())
		{	
			ur3Manager->setGripperPosition(idlePos);
			ur3Manager->setJointVal(ur3traj2[(trajcnt - ur3traj1.size()) % ur3traj2.size()]);

			wire1->clearPoints();
			//SE3 wireTransformation = EulerXYZ(Vec3(0, 0, 0), Vec3(-0.03, 0.02, -0.06)) * ur3->gMarkerLink[UR3_Index::MLINK_GRIP].GetFrame() * EulerXYZ(Vec3(0, SR_PI_HALF, SR_PI_HALF), Vec3(0, 0, 0)) * Inv(boxfortape->getBaseLinkFrame());
			for (int i = 0; i < wireNodes.size(); i++) wire1->addPoint(ur3->gMarkerLink[UR3_Index::MLINK_GRIP].GetFrame() * wireNodes[i]);
			wire1->glRender();

			if (trajcnt == ur3traj1.size() + ur3traj2.size() - 1) {
				ur3Manager->setGripperPosition(idlePos);
			}
		}
		else if (trajcnt < ur3traj1.size() + ur3traj2.size() + ur5traj3.size())
		{
			ur5Manager->setJointVal(ur5traj3[(trajcnt - ur3traj1.size() - ur3traj2.size()) % ur5traj3.size()]);
			ur3Manager->setJointVal(ur3traj2[ur3traj2.size() - 1]);
			if (trajcnt == ur3traj1.size() + ur3traj2.size() + ur5traj3.size() - 1) {
				tape->re.m_State.m_rValue[0] = -0.5;
				ur3Manager->setGripperPosition(gripPos);
			}
		}
		else if (trajcnt < ur3traj1.size() + ur3traj2.size() + ur5traj3.size() + ur5traj3_2.size())
		{
			ur5Manager->setJointVal(ur5traj3_2[(trajcnt - ur3traj1.size() - ur3traj2.size() - ur5traj3.size()) % ur5traj3_2.size()]);
			ur3Manager->setJointVal(ur3traj2[ur3traj2.size() - 1]);
			if (trajcnt == ur3traj1.size() + ur3traj2.size() + ur5traj3.size() + ur5traj3_2.size() - 1) {
				ur3Manager->setGripperPosition(gripPos);
			}
		}
		else if (trajcnt < ur3traj1.size() + ur3traj2.size() + ur5traj3.size() + ur5traj3_2.size() + ur3traj4.size())
		{
			ur3Manager->setGripperPosition(gripPos);
			ur3Manager->setJointVal(ur3traj4[(trajcnt - ur3traj1.size() - ur3traj2.size() - ur5traj3.size() - ur5traj3_2.size()) % ur3traj4.size()]);
			if (trajcnt == ur3traj1.size() + ur3traj2.size() + ur5traj3.size() + ur5traj3_2.size() + ur3traj4.size() - 1) {
				ur3Manager->setGripperPosition(idlePos);
			}
		}
	}
}


void URrobotSetting()
{

	// ur3 setting
	gSpace.AddSystem((srSystem*)ur3);
	ur3->GetBaseLink()->SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	ur3->SetActType(srJoint::ACTTYPE::HYBRID);

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

	ur3RRTManager->printIter = false;
	ur3RRTManager->printFinish = false;
	ur3RRTManager->printDist = false;
	ur5RRTManager->printIter = false;
	ur5RRTManager->printFinish = false;
	ur5RRTManager->printDist = false;
}

void tempObjectSetting()
{
	double dim = 0.05;
	ee->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
	ee->GetGeomInfo().SetDimension(dim);
	ee->GetGeomInfo().SetColor(1.0, 0.0, 0.0);
	srCollision* tempCol = new srCollision;
	tempCol->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
	tempCol->GetGeomInfo().SetDimension(dim);
	ee->AddCollision(tempCol);
	//obs->SetBaseLink(ee);
	//obs->SetBaseLinkType(srSystem::FIXED);
	//gSpace.AddSystem(obs);
}

vector<Eigen::VectorXd> makeGriptraj(double gripangle, Eigen::VectorXd currentPos)
{
	vector<Eigen::VectorXd> gripTraj(0);
	for (int i = 0; i < 10; i++)
	{
		Eigen::VectorXd tempPos = currentPos;
		cout << tempPos << endl;
		tempPos[6] += gripangle / 180 * SR_PI / 10;
		tempPos[10] += gripangle / 180 * SR_PI / 10;
		tempPos[14] += gripangle / 180 * SR_PI / 10;
		gripTraj.push_back(tempPos);
	}
	return gripTraj;
}

void setFloor()
{
	floor_link->GetGeomInfo().SetShape(srGeometryInfo::BOX);
	Vec3 obs_size = Vec3(5.0, 5.0, 0.05);
	Vec3 obs_col_size = Vec3(5.0, 5.0, 0.05);
	floor_link->GetGeomInfo().SetDimension(obs_size);
	floor_link->GetGeomInfo().SetColor(0.4, 0.4, 0.4);
	floor_colli->GetGeomInfo().SetShape(srGeometryInfo::BOX);
	floor_colli->GetGeomInfo().SetDimension(obs_col_size);
	floor_link->AddCollision(floor_colli);
	Floor->SetBaseLink(floor_link);
	Floor->SetBaseLinkType(srSystem::FIXED);
	gSpace.AddSystem(Floor);
	Floor->GetBaseLink()->SetFrame(SE3(Vec3(0.0, 0.0, -0.05)));
}

Eigen::VectorXd robustInverseKinematics(SE3 finalpos, Eigen::VectorXd original, int maxiter)
{
	srand((unsigned int)time(NULL));

	int flag = 0;
	Eigen::VectorXd lower = ur3->getLowerJointLimit();
	Eigen::VectorXd upper = ur3->getUpperJointLimit();
	Eigen::VectorXd initial = Eigen::VectorXd::Zero(lower.size());
	Eigen::VectorXd currentpos = ur3Manager->getJointVal();

	vector<Eigen::VectorXd> solList(0);

	for (int i = 0; i < maxiter; i++) {
		Eigen::VectorXd tempsol;
		if (i == 0) initial = original;
		else {
			for (int j = 0; j < lower.size(); j++) {
				int temp = int((upper[j] - lower[j]) * 1000);
				initial[j] = double(rand() % temp) / 1000 + lower[j];
			}
		}
		tempsol = ur3Manager->inverseKin(finalpos, &ur3->gMarkerLink[UR3_Index::MLINK_GRIP], true, SE3(), flag, initial);
		//cout << "inverse kinematics flag: " << flag << endl;

		ur3Manager->setJointVal(tempsol);
		if (!ur3Manager->checkCollision() && flag == 0) {
			solList.push_back(tempsol);
		}
	}
	if (solList.size() == 0) {
		cout << "Cannot find solution..." << endl;
		ur3Manager->setJointVal(currentpos);
		return Eigen::VectorXd::Zero(lower.size());
	}
	else {
		double temp = 10000;
		double newsize = 0;
		Eigen::VectorXd sol = Eigen::VectorXd::Zero(lower.size());
		Eigen::VectorXd weight = ur3Manager->getBodyJacobian(currentpos, &ur3->gMarkerLink[UR3_Index::MLINK_GRIP], SE3()).colwise().squaredNorm();
		for (int i = 0; i < solList.size(); i++) {
			//newsize = (solList[i] - original).squaredNorm();
			newsize = (solList[i] - original).transpose().cwiseAbs() * weight;
			if (temp > newsize) {
				temp = newsize;
				sol = solList[i];
			}
		}
		return sol;
	}
}

Eigen::VectorXd robustInverseKinematics_UR5(SE3 finalpos, Eigen::VectorXd original, int maxiter)
{
	srand((unsigned int)time(NULL));

	int flag = 0;
	Eigen::VectorXd lower = ur5->getLowerJointLimit();
	Eigen::VectorXd upper = ur5->getUpperJointLimit();
	Eigen::VectorXd initial = Eigen::VectorXd::Zero(lower.size());
	Eigen::VectorXd currentpos = ur5Manager->getJointVal();

	vector<Eigen::VectorXd> solList(0);

	for (int i = 0; i < maxiter; i++) {
		Eigen::VectorXd tempsol;
		if (i == 0) initial = original;
		else {
			for (int j = 0; j < lower.size(); j++) {
				int temp = int((upper[j] - lower[j]) * 1000);
				initial[j] = double(rand() % temp) / 1000 + lower[j];
			}
		}
		tempsol = ur5Manager->inverseKin(finalpos, &ur5->gMarkerLink[UR5_Index::MLINK_GRIP], true, SE3(), flag, initial);
		//cout << "inverse kinematics flag: " << flag << endl;

		ur5Manager->setJointVal(tempsol);
		if (!ur5Manager->checkCollision() && flag == 0) {
			solList.push_back(tempsol);
			//ur5Manager->setJointVal(currentpos);
			//return tempsol;
		}
	}
	ur5Manager->setJointVal(currentpos);
	if (solList.size() == 0) {
		cout << "Cannot find solution..." << endl;
		return Eigen::VectorXd::Zero(lower.size());
	}
	else {
		double temp = 10000;
		double newsize = 0;
		Eigen::VectorXd sol = Eigen::VectorXd::Zero(lower.size());
		Eigen::VectorXd weight = ur5Manager->getBodyJacobian(currentpos, &ur5->gMarkerLink[UR5_Index::MLINK_GRIP], SE3()).colwise().squaredNorm();
		for (int i = 0; i < solList.size(); i++) {
			//newsize = (solList[i] - original).squaredNorm();
			newsize = (solList[i] - original).transpose().cwiseAbs() * weight;
			if (temp > newsize) {
				temp = newsize;
				sol = solList[i];
			}
		}
		return sol;
	}
}