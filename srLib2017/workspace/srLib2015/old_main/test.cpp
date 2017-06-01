#include <cstdio>

#include "myRenderer.h"
#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\robotManager.h"
#include <time.h>

srSpace gSpace;
srSpace gSpace2;
myRenderer* renderer;
gamasot::srRobot* robot = new gamasot::srRobot;
gamasot::srRobot* robot2 = new gamasot::srRobot;
robotManager* rManager = new robotManager;
robotManager* rManager2 = new robotManager;

srLink* link1 = new srLink;
srWeldJoint* joint1 = new srWeldJoint;
srRevoluteJoint* joint2 = new srRevoluteJoint;
srLink* link2 = new srLink;

srLink* link1_2 = new srLink;
srWeldJoint* joint1_2 = new srWeldJoint;
srRevoluteJoint* joint2_2 = new srRevoluteJoint;
srLink* link2_2 = new srLink;

void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
void testJb();
void testInvKin();
Eigen::VectorXd inverseDyn2(const Eigen::VectorXd & jointVal, const Eigen::VectorXd & jointVel, const Eigen::VectorXd & jointAcc, robotManager* rManager);
int main(int argc, char **argv)
{
	
	srand((unsigned int)time(0));

	link1->GetGeomInfo().SetShape(srGeometryInfo::BOX);
	link1->GetGeomInfo().SetDimension(Vec3(0.1, 0.1, 0.1));
	link1->SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(5.0, 0.0, 0.0)));

	//robot1->SetBaseLink(link1);
	//robot1->SetBaseLinkType(srSystem::FIXED);
	//gSpace.AddSystem(robot1);
	gamasot::urdf2srRobot(robot, "../../../workspace/robot", "ur10", "ur10_robot", true);
	gamasot::urdf2srRobot(robot2, "../../../workspace/robot", "ur10", "ur10_robot", true);
	
	//joint1->SetChildLink(link1);
	//joint1->SetChildLinkFrame(SE3());
	//joint1->SetParentLink(robot->getLink("wrist_2_link"));
	//joint1->SetParentLinkFrame(SE3(Vec3(0.,0.10,0.)));

	//joint2->SetChildLink(link2);
	//joint2->SetChildLinkFrame(SE3());
	//joint2->SetParentLink(link1);
	//joint2->SetParentLinkFrame(SE3());

	//joint1_2->SetChildLink(link1_2);
	//joint1_2->SetChildLinkFrame(SE3());
	//joint1_2->SetParentLink(robot2->getLink("wrist_2_link"));
	//joint1_2->SetParentLinkFrame(SE3(Vec3(0., 0.10, 0.)));

	//joint2_2->SetChildLink(link2_2);
	//joint2_2->SetChildLinkFrame(SE3());
	//joint2_2->SetParentLink(link1_2);
	//joint2_2->SetParentLinkFrame(SE3());
		  
	gSpace.AddSystem((srSystem*)robot);
	//gSpace2.AddSystem((srSystem*)robot2);

	vector<double> ulim(6, 360.0);
	vector<double> llim(6, -360.0);
	robot->setStateJointLimit(ulim, llim);
	robot2->setStateJointLimit(ulim, llim);
	// initialize srLib
	initDynamics();

	
	// robotManager
	rManager->setRobot(robot);
	rManager->setSpace(&gSpace);
	vector<srLink*> endeffector1(2);
	endeffector1[0] = robot->getLink("wrist_3_link");
	endeffector1[1] = link2;
	rManager->m_activeArmInfo->setActiveArmInfo((srSystem*)robot, endeffector1);


	// robotManager copy
	//rManager2->setRobot(robot2);
	//rManager2->setSpace(&gSpace2);
	//vector<srLink*> endeffector2(2);
	//endeffector2[0] = robot2->getLink("wrist_3_link");
	//endeffector2[1] = link2_2;
	//rManager2->m_activeArmInfo->setActiveArmInfo((srSystem*)robot2, endeffector2);
	//rManager->setEndeffector("wrist_3_link", "wrist_3_link");
	//rManager->setEndeffector(link2, link2_2);

	//int a = rManager->m_activeArmInfo->m_numJoint;
	//int b = rManager2->m_activeArmInfo->m_numJoint;


	//Eigen::VectorXd q(a);
	//Eigen::VectorXd dq(a);
	//Eigen::VectorXd ddq(a);
	//for (int i = 0; i < q.size(); i++)
	//{
	//	q(i) = (double)rand() / RAND_MAX;
	//	dq(i) = (double)rand() / RAND_MAX;
	//	ddq(i) = (double)rand() / RAND_MAX;
	//}

	////cout << Jbnum << endl;

	//cout << rManager->forwardKin(q, link2) << endl;
	//Eigen::VectorXd torque;

	//time_t start, end;
	//time(&start);
	//for (int i = 0; i < 100; i++)
	//	torque = rManager->inverseDyn(q, dq, ddq);
	//time(&end);
	//cout << "torque: " << endl << torque << endl;
	//cout << "time: " << end - start << endl;


	// check time for inverse dynamics
	//Eigen::VectorXd q = Eigen::VectorXd::Random(a);
	//Eigen::VectorXd dq = Eigen::VectorXd::Random(a);
	//Eigen::VectorXd ddq = Eigen::VectorXd::Random(a);

	//clock_t start, end;
	//start = clock();
	//Eigen::VectorXd tau;
	//for (int i = 0; i < 10000; i++)
	//	tau = rManager->inverseDyn(q, dq, ddq);

	//end = clock();
	//printf("t = %f\n", (double)(end - start) / CLOCKS_PER_SEC);

	//srLink* linki;
	//srLink* linki_;
	//vector<dse3> F(q.size()+1, dse3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
	//Eigen::VectorXd tau(q.size());
	//for (int i = rManager->m_robotCopy->m_KIN_Links.get_size() - 1; i > -1; i--)
	//{
	//	linki = rManager->m_robotCopy->m_KIN_Links[i];
	//	F[i] = linki->m_Inertia * linki->m_Acc - dad(linki->m_Vel, linki->m_Inertia*linki->m_Vel) - linki->m_ExtForce;
	//	if (i < q.size())
	//		F[i] += InvdAd(rManager->m_robotCopy->m_KIN_Links[i + 1]->m_MexpSq, F[i + 1]);
	//	if (i > 0)
	//		tau(i - 1) = F[i] * Ad(linki->m_ParentJoint->m_ChildLinkToJoint,  se3(0.,0.,1.,0.,0.,0.));
	//}
	//cout << "torque: " << endl << tau << endl;
	//for (int i = 0; i < rManager->m_robotCopy->m_KIN_Links.get_size(); i++)
	//{
	//	linki = rManager->m_robotCopy->m_KIN_Links[i];
	//	linki_ = rManager->m_robot->m_KIN_Links[i];
	//	//cout << i << "th V: " << linki->m_Vel << endl;
	//	//cout << i << "th Vdot: " << linki->m_Acc << endl;
	//	//cout << "MexpSq: " << linki->m_MexpSq << endl;
	//	//cout << "T: " << linki->m_Frame << endl;
	//	//cout << i << "th F: " << F[i] << endl;
	//	cout << i << "th Fext: " << linki->m_ExtForce << endl;
	//	//if (i > 0)
	//	//	cout << i << "th S: " << Ad(linki->m_ParentJoint->m_ChildLinkToJoint, se3(0., 0., 1., 0., 0., 0.)) << endl;
	//	//cout << "Ffs: " << linki->m_AInertia * linki->m_Acc + linki->m_Bias << endl;
	//}
	//Eigen::VectorXd torque2;
	//time(&start);
	//for (int i = 0; i < 100; i++)
	//	torque2 = inverseDyn2(q, dq, ddq, rManager2);
	//time(&end);
	//cout << "torque: " << endl << torque2 << endl;
	//cout << "time: " << end - start << endl;
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
	gSpace.SetGravity(0.0, 0.0, -1.0);
	gSpace.SetNumberofSubstepForRendering(10);

	gSpace.DYN_MODE_PRESTEP();

	gSpace2.SetTimestep(0.0000001);
	gSpace2.SetGravity(0.0, 0.0, -1.0);
	gSpace2.SetNumberofSubstepForRendering(10);

	gSpace2.DYN_MODE_PRESTEP();
}

void updateFunc()
{
	for (int i = 0; i < 1; i++)
	{
		((srStateJoint*)robot->m_KIN_Joints[i])->m_State.m_rValue[0] += 0.001;
	}
	
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
}

void testJb()
{
	int a = rManager->m_activeArmInfo->m_numJoint;
	int b = rManager2->m_activeArmInfo->m_numJoint;


	Eigen::VectorXd q(a);
	Eigen::VectorXd dq(a);
	for (int i = 0; i < q.size(); i++)
	{
		q(i) = (double)rand() / RAND_MAX;
		dq(i) = (double)rand() / RAND_MAX;
	}

	vector<srLink*> links(0);
	vector<SE3> offsets(2, SE3());
	links.push_back(link2);
	links.push_back(robot->getLink("wrist_3_link"));
	Eigen::MatrixXd Jb = rManager->getBodyJacobian(q, links, offsets);
	Eigen::MatrixXd Jbdot = rManager->getBodyJacobianDot(q, dq, links, offsets);
	cout << q << endl;
	cout << Jbdot << endl;


	vector<SE3> T = rManager->forwardKin(q, links, offsets);
	Eigen::MatrixXd Jbdotnum(6 * links.size(), q.cols());
	double eps = 0.001;
	Eigen::MatrixXd Jbtemp = rManager->getBodyJacobian(q + eps*dq, links, offsets);
	Jbdotnum = (Jbtemp - Jb) / eps;
	cout << Jbdotnum << endl;

	cout << (Jbdot - Jbdotnum).norm() << endl;

	vector<SE3> Ttemp;
	Eigen::VectorXd temp;
	for (int i = 0; i < q.size(); i++)
	{
		q(i) += eps;
		Ttemp = rManager->forwardKin(q, links, offsets);
		for (unsigned int j = 0; j < Ttemp.size(); j++)
			cout << (Jb.block(6 * j, i, 6, 1) - se3toVector(Log(Inv(T[j])*Ttemp[j])) / eps).norm() << endl;

		q(i) -= eps;
	}
}

void testInvKin()
{
	int a = rManager->m_activeArmInfo->m_numJoint;
	int b = rManager2->m_activeArmInfo->m_numJoint;

	Eigen::VectorXd q(a);
	Eigen::VectorXd dq(a);
	for (int i = 0; i < q.size(); i++)
	{
		q(i) = (double)rand() / RAND_MAX;
		dq(i) = (double)rand() / RAND_MAX;
	}

	vector<srLink*> links(0);

	links.push_back(link2);
	links.push_back(robot->getLink("wrist_3_link"));

	vector<SE3> offsets(links.size(), SE3());
	vector<SE3> T = rManager->forwardKin(q, links, offsets);

	vector<bool> includeOri(T.size(), true);
	int flag;
	Eigen::VectorXd qinit(a);
	for (int i = 0; i < qinit.size(); i++)
	{
		qinit(i) = (double)rand() / RAND_MAX;
	}
	Eigen::VectorXd qnum = rManager->inverseKin(T, links, includeOri, offsets, flag, qinit, 500);
	cout << "q: " << endl << q << endl;
	cout << "qinit: " << endl << qinit << endl;
	cout << "qnum: " << endl << qnum << endl;
}


Eigen::VectorXd inverseDyn2(const Eigen::VectorXd & jointVal, const Eigen::VectorXd & jointVel, const Eigen::VectorXd & jointAcc, robotManager* rManager)
{
	// use copy space
	bool isHybrid = true;
	for (int i = 0; i < rManager->m_activeArmInfo->m_numJoint; i++)
	{
		if (rManager->m_activeArmInfo->m_activeJoint[i]->GetActType() != srJoint::HYBRID)
		{
			isHybrid = false;
			cout << "check joint acttype for inverse dynamics!!!" << endl;
		}
	}

	Eigen::VectorXd jointTorque = Eigen::VectorXd::Zero(rManager->m_activeArmInfo->m_numJoint);

	if (isHybrid == true)
	{
		srStateJoint* curJoint;
		if (jointVal.size() != rManager->m_activeArmInfo->m_numJoint || jointVel.size() != rManager->m_activeArmInfo->m_numJoint || jointAcc.size() != rManager->m_activeArmInfo->m_numJoint)
			cout << "Check number of joints !!!" << endl;

		for (int i = 0; i < rManager->m_activeArmInfo->m_numJoint; i++)
		{
			curJoint = (srStateJoint*)rManager->m_activeArmInfo->m_activeJoint[i];
			curJoint->m_State.m_rValue[0] = jointVal[i];
			curJoint->m_State.m_rValue[1] = jointVel[i];
			//curJoint->m_State.m_rValue[2] = 0.0;
			curJoint->m_State.m_rValue[3] = 0.0;
			curJoint->m_State.m_rCommand = jointAcc[i];
		}
		rManager->m_robot->KIN_UpdateFrame_All_The_Entity();
		rManager->m_robot->DIFFKIN_LinkVelocityPropagation();
		rManager->m_space->DYN_MODE_RUNTIME_SIMULATION_LOOP();
		//cout << "q = " << getJointVal(m_robotCopy) << endl;
		//cout << "dotq = " << getJointVel(m_robotCopy) << endl;
		//cout << "dotdotq = " << getJointAcc(m_robotCopy) << endl;
		for (int i = 0; i < rManager->m_activeArmInfo->m_numJoint; i++)
		{
			curJoint = (srStateJoint*)rManager->m_activeArmInfo->m_activeJoint[i];
			jointTorque(i) = curJoint->m_State.m_rValue[3];
		}
	}
	return jointTorque;
}