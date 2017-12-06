#include "serverPlanningSetting.h"
#include "robotManager/rrtVectorFields.h"
#include "myRenderer.h"
#include <time.h>

myRenderer* renderer;
vector<Eigen::VectorXd> traj1(0);
vector<Eigen::VectorXd> traj2(0);
// rendering functions
void rendering(int argc, char **argv);
void updateFunc();
int main(int argc, char **argv)
{
	srand(time(NULL));
	robot1Setting();
	initDynamics();
	robotManager1Setting();


	rrtSetting();

	//singularityAvoidanceVectorField* singAvoidVF = new singularityAvoidanceVectorField;
	//singAvoidVF->setRobotEndeffector(rManager1, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);

	workspaceConstantPositionVectorField* workspaceVF = new workspaceConstantPositionVectorField;
	workspaceVF->setRobotEndeffector(rManager1, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
	Eigen::VectorXd workspaceVec = Eigen::VectorXd::Zero(3);
	workspaceVec[2] = 1.0;
	workspaceVF->setWorkspaceVector(workspaceVec);
	workspaceVF->_fixOri = true;

	double epsilon = 1e-6;
	Eigen::VectorXd testPos(6);
	testPos.setRandom();
	Eigen::VectorXd testVel(6);
	testVel.setZero();

	// set init and goal pos for singavoid vf rrt
	//Eigen::VectorXd singPos(6);
	//singPos.setZero();
	//singPos[1] = -SR_PI_HALF; singPos[2] = DEG2RAD(0.0); singPos[3] = SR_PI_HALF; singPos[4] = DEG2RAD(0);
	//rManager1->setJointVal(singPos);
	//cout << rManager1->manipulability(singPos, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]) << endl;
	//cout << rManager1->manipulabilityGradient(singPos, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]).transpose() << endl;
	//
	//
	//Eigen::VectorXd singPos2(6);
	//singPos2.setZero();
	//singPos2[1] = -SR_PI_HALF; singPos2[2] = DEG2RAD(0.0); singPos2[3] = SR_PI_HALF; singPos2[4] = DEG2RAD(0);
	//
	//Eigen::VectorXd initPos = singPos;
	//initPos[2] -= 0.5;
	//cout << initPos.transpose() << endl;
	//cout << rManager1->manipulability(initPos, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]) << endl;
	//cout << initPos.transpose() << endl;
	//Eigen::VectorXd goalPos = singPos;
	//goalPos[2] += 0.5;
	//cout << rManager1->manipulability(goalPos, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]) << endl;

	//Eigen::MatrixXd Jtest = rManager1->getBodyJacobian(goalPos, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
	//Eigen::JacobiSVD<Eigen::MatrixXd> svdtest(Jtest, Eigen::ComputeThinU | Eigen::ComputeThinV);
	//cout << "U: " << endl << svdtest.matrixU() << endl;
	//cout << "V: " << endl << svdtest.matrixV() << endl;
	//cout << "S: " << svdtest.singularValues().transpose() << endl;

	// set init and goal pos for workspace vf rrt

	//rManager1->setJointVal(robot1->homePos);
	SE3 tempSE3 = rManager1->forwardKin(robot1->homePos, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
	//cout << tempSE3 << endl;
	SE3 tempSE3_1 = EulerZYX(Vec3(0.0, SR_PI, 0.0), Vec3(-0.3, 0.8, 1.1));
	SE3 tempSE3_2 = EulerZYX(Vec3(0.0, SR_PI, 0.0), Vec3(-0.7, 0.2, 1.1));
	int flag1, flag2;
	Eigen::VectorXd initPos = rManager1->inverseKin(tempSE3_1, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flag1, robot1->homePos);
	Eigen::VectorXd goalPos = rManager1->inverseKin(tempSE3_2, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flag2, robot1->homePos);
	cout << flag1 << endl;
	cout << flag2 << endl;
	goalPos[0] -= 0.5;
	rManager1->setJointVal(initPos);

	// do planning
	vector<bool> feas = RRTManager1->checkFeasibility(initPos, goalPos);
	RRTManager1->setStartandGoal(initPos, goalPos);
	RRTManager1->execute(0.1);
	traj1 = RRTManager1->extractPath(200);
	rManager1->setJointVal(initPos);

	
	// do planning (vf rrt)
	//RRTManager1->addVectorField(singAvoidVF);
	RRTManager1->addVectorField(workspaceVF);
	RRTManager1->setVectorFieldWeight(1.0);
	RRTManager1->setStartandGoal(initPos, goalPos);
	RRTManager1->execute(0.1);
	traj2 = RRTManager1->extractPath(200);
	
	// check cost
	double cost1 = 0.0;
	for (unsigned int i = 0; i < traj1.size(); i++)
	{
		//cost1 += rManager1->manipulability(traj1[i], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
		cost1 += rManager1->forwardKin(traj1[i], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP])[11];
	}
	cout << "cost1: " << (double) cost1 / traj1.size() << endl;
	
	double cost2 = 0.0;
	for (unsigned int i = 0; i < traj2.size(); i++)
	{
		//cost2 += rManager1->manipulability(traj2[i], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
		cost2 += rManager1->forwardKin(traj2[i], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP])[11];
	}
	cout << "cost2: " << (double) cost2 / traj2.size() << endl;





	rendering(argc, argv);

	/////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////
	// check jacobian derivative
	//Eigen::MatrixXd Jb = rManager1->getBodyJacobian(testPos, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
	//Eigen::MatrixXd Jbdot_num;
	//Eigen::MatrixXd Jbdot_ana;
	//cout << "body jacobian: " << endl;
	//cout << Jb << endl << endl;
	//Eigen::JacobiSVD<Eigen::MatrixXd> svd(Jb, Eigen::ComputeThinV);
	//Eigen::VectorXd s = svd.singularValues();
	//Eigen::MatrixXd V = svd.matrixV();
	//Eigen::JacobiSVD<Eigen::MatrixXd> svd2(Jb.transpose()*Jb, Eigen::ComputeThinV);
	//Eigen::VectorXd s2 = svd2.singularValues();
	//Eigen::MatrixXd V2 = svd2.matrixV();
	//Eigen::VectorXd eigGrad(6);
	//cout << "singval: " << s.transpose() << endl;
	//cout << "singvec: " << endl << V << endl;
	//cout << "singval: " << s2.transpose() << endl;
	//cout << "singvec: " << endl << V2 << endl;
	//for (int i = 0; i < 6; i++)
	//{
	//	if (i > 0)
	//	{
	//		testPos[i - 1] -= epsilon;
	//		testVel[i - 1] = 0;
	//	}
	//	testPos[i] += epsilon;
	//	testVel[i] = 1;
	//	cout << "joint " << i << endl;
	//	Eigen::MatrixXd Jbtemp = rManager1->getBodyJacobian(testPos, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
	//	Eigen::JacobiSVD<Eigen::MatrixXd> svd(Jbtemp, Eigen::ComputeThinV);
	//	Eigen::VectorXd stemp = svd.singularValues();
	//	//Jbdot_num = (Jbtemp - Jb) / epsilon;
	//	//cout << "numerical deriv: " << endl;
	//	//cout << Jbdot_num << endl;
	//	Jbdot_ana = rManager1->getBodyJacobianDot(testPos, testVel, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
	//	cout << "analytical deriv: " << endl;
	//	cout << Jbdot_ana << endl;
	//}

	/////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////
	// check manipulability gradient
	//for (int testIdx = 0; testIdx < 10; testIdx++)
	//{
	//	testPos.setRandom();
	//	cout << "test pos: " << testPos.transpose() << endl;
	//	double manip0 = rManager1->manipulability(testPos, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
	//	
	//	Eigen::VectorXd num_grad(6);
	//	for (int i = 0; i < 6; i++)
	//	{
	//		if (i > 0)
	//			testPos[i - 1] -= epsilon;
	//		testPos[i] += epsilon;
	//		double temp = rManager1->manipulability(testPos, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
	//		num_grad[i] = (temp - manip0) / epsilon;
	//	}
	//	Eigen::VectorXd ana_grad = rManager1->manipulabilityGradient(testPos, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
	//	
	//	cout << "num grad: " << num_grad.transpose() << endl;
	//	cout << "ana grad: " << ana_grad.transpose() << endl << endl;
	//	testPos[5] -= epsilon;
	//	double manip1 = rManager1->manipulability(testPos + ana_grad, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
	//	cout << manip0 << endl;
	//	cout << manip1 << endl;
	//}
	



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

void updateFunc()
{
	static int cnt = 0, trjIdx = 0, trj = 0;
	cnt++;
	if (traj1.size() > 0 && trj%2 == 0)
	{
		if (cnt % 20 == 0)
		{	
			trjIdx++;
			if (trjIdx == 1)
				cout << endl << "trajectory 1" << endl;
			rManager1->setJointVal(traj1[(trjIdx - 1) % traj1.size()]);
			//cout << rManager1->manipulability(traj1[(trjIdx - 1) % traj1.size()], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]) << endl;
			cout << rManager1->forwardKin(traj1[(trjIdx - 1) % traj1.size()], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP])[11] << endl;
			if (trjIdx  == traj1.size() + 1)
			{
				trjIdx = 0;
				trj += 1;
			}
		}
	}
	if (traj2.size() > 0 && trj%2 == 1)
	{
		if (cnt % 20 == 0)
		{
			trjIdx++;
			if (trjIdx == 1)
				cout << endl << "trajectory 2" << endl;
			rManager1->setJointVal(traj2[(trjIdx - 1) % traj2.size()]);
			//cout << rManager1->manipulability(traj2[(trjIdx - 1) % traj2.size()], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]) << endl;
			cout << rManager1->forwardKin(traj2[(trjIdx - 1) % traj2.size()], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP])[11] << endl;
			if (trjIdx == traj2.size() + 1)
			{
				trjIdx = 0;
				trj += 1;
			}
		}
	}
}