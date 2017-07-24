#include <cstdio>

#include "serverRenderer.h"
#include "serverPlanningSetting.h"
#include <fstream>
#include <iostream>

#include <Winsock2.h>
#include <stdlib.h>
#include <stdio.h>
#include "Eigen/Dense"

#include <stdlib.h>
#include <vector>

// memory leakaage check
#include <crtdbg.h>


vector<vector<bool>> attachObjectWaypoint(2);
bool checkTorque = false;

// save initial and final busbar SE(3)
vector<bool> initialPlanning(2, true);
bool initialPlanning_twoArm = true;
vector<bool> startPlanningFromCurRobotState(2, false);
bool startPlanningFromCurRobotState_twoArm = false;

// Two arm flag
bool isTwoArm = true;
bool planTwoArmOkay = false;
vector<bool> planningInitUpdated(2, false);
Eigen::VectorXd planningInit_twoArm(12);

void communicationFunc(int argc, char **argv);		// run on the other thread

													// 서버 초기화
Server serv = Server::Server();
dataset hyu_dataset;
vector<desired_dataset> hyu_desired_dataset;
vision_data skku_dataset;
robot_current_data robot_state;

//char hyu_data[30000];
char hyu_data_flag;
bool useSleep = false;


int main(int argc, char **argv)
{
	Eigen::initParallel();
	bool useVision = false;
	if (useVision)
		useNoVisionTestSetting = false;
	srand(time(NULL));
	// Robot home position
	robotSetting();
	// environment
	workspaceSetting();
	objectSetting();
	////////////////////////////////////////////// setting environment (replacable from vision data)
	if (!useVision)
	{
		environmentSetting_HYU2(true);				// temporary environment setting
		initDynamics();								// initialize srLib				
		isSystemAssembled = true;
		robotManagerSetting();						// robot manager setting

													// workcell robot initial config
		rManager1->setJointVal(robot1->homePos);
		rManager2->setJointVal(robot2->homePos);
		Eigen::VectorXd gripInput(2);
		gripInput[0] = -0.005;
		gripInput[1] = 0.005;
		rManager1->setGripperPosition(gripInput);
		rManager2->setGripperPosition(gripInput);

		cout << Trobotbase2 % robot2->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame() << endl;


		// rrt
		rrtSetting();

		serv.SendMessageToClient("Ld0d");
		if (useSleep)
			Sleep(50);
	}


	communicationFunc(argc, argv);

	
	//만약 while 루프를 돌리지 않을 경우 무한정 서버를 기다리는 함수, 실제 사용하지는 않는다.
	//serv.WaitServer(); 

	// 서버를 종료 시킴
	serv.~Server();

	return 0;
}

void communicationFunc(int argc, char **argv)
{
	while (TRUE) {

		//Receiving data from HYU client
		char* hyu_data = serv.RecevData();
		//strcpy(hyu_data, "");
		//strcat(hyu_data, serv.RecevData());

		hyu_data_flag = hyu_data[0];

		//printf(&hyu_data_flag);
		//cout << endl;
		//serv.SendMessageToClient("G");

		// 데이터 전송
		if (hyu_data_flag == 'I')
		{
			serv.SendMessageToClient("I");
			if (useSleep)
				Sleep(50);
		}
		else if (hyu_data_flag == 'D')
		{
			serv.SendMessageToClient("D");
			if (useSleep)
				Sleep(50);
		}
		else if (hyu_data_flag == 'F')
		{
			serv.SendMessageToClient("F");
			if (useSleep)
				Sleep(50);
		}

		else if (hyu_data_flag == 'V')
		{
			//cout << "Vision communicate called" << endl;

			// vision data
			char* copy = (char*)malloc(sizeof(char)*(strlen(hyu_data) + 1));
			for (unsigned int p = 0; p <= strlen(hyu_data); p++)
				copy[p] = hyu_data[p];
			serv.SendMessageToClient(copy);
			Sleep(50);
			printf("%s\n", hyu_data);
			readSKKUvision(hyu_data, skku_dataset);

			int bNum = 0;
			int cNum = 0;
			setEnviromentFromVision(skku_dataset, bNum, cNum);		// should be called later than robotSetting


			if (!isSystemAssembled)
			{
				if (isJigConnectedToWorkCell)
					connectJigToWorkCell();
				else
					gSpace.AddSystem((srSystem*)jigAssem);
				/////////////////////////////////////// after setting environment
				initDynamics();								// initialize srLib

				isSystemAssembled = true;
				robotManagerSetting();						// robot manager setting

															// workcell robot initial config
				rManager2->setJointVal(robot2->homePos);
				rManager1->setJointVal(robot1->homePos);
				Eigen::VectorXd gripInput(2);
				gripInput[0] = -0.005;
				gripInput[1] = 0.005;
				rManager1->setGripperPosition(gripInput);
				rManager2->setGripperPosition(gripInput);

				// rrt
				rrtSetting();
			}

			// lift objects if collision occur
			printf("initial collision check: %d\n", (int)gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP());
			bool liftObjects = false;
			if (liftObjects)
			{
				Vec3 delta_z = Vec3(0.0, 0.0, 0.0001);
				int liftIter = 0;
				while (gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP())
				{
					for (int i = 0; i < bNum; i++)
					{
						busbar[i]->GetBaseLink()->SetPosition(busbar[i]->GetBaseLink()->GetPosition() + delta_z);
						busbar[i]->KIN_UpdateFrame_All_The_Entity();
					}
					for (int i = 0; i < cNum; i++)
					{
						ctCase[i]->GetBaseLink()->SetPosition(ctCase[i]->GetBaseLink()->GetPosition() + delta_z);
						ctCase[i]->KIN_UpdateFrame_All_The_Entity();
					}
					liftIter++;
				}
				char liftBuf[20];
				sprintf(liftBuf, "Ld%fd", -(double)liftIter * delta_z[2]);
				serv.SendMessageToClient(liftBuf);
				if (useSleep)
					Sleep(50);
				printf(liftBuf);
				printf("\n");
			}
			else
			{
				serv.SendMessageToClient("Ld0d");
				if (useSleep)
					Sleep(50);
			}
			//////////////////////////////////////////////////////////////////////
			//m.lock();
			//m.unlock();
			free(copy);
		}
		else if (hyu_data_flag == 'G') {

			//char* send_data;
			//send_data = getSimulationState(objects);
			//serv.SendMessageToClient(send_data);	
			printf(hyu_data);
			serv.SendMessageToClient(hyu_data);
			if (useSleep)
				Sleep(50);
		}
		else if (hyu_data_flag == 'R')
		{
			// Robot cur data
			char* copy = (char*)malloc(sizeof(char)*(strlen(hyu_data) + 1));
			for (unsigned int p = 0; p <= strlen(hyu_data); p++)
				copy[p] = hyu_data[p];
			serv.SendMessageToClient(copy);
			if (useSleep)
				Sleep(50);
			printf("%s\n", hyu_data);
			int robotFlag = 0;
			robotFlag = readRobotCurState(hyu_data, robot_state);

			if (robotFlag == 1)
			{
				//rManager1->setJointVal(robot_state.robot_joint);

				serv.SendMessageToClient("T1");
				if (useSleep)
					Sleep(50);
			}
			else if (robotFlag == 2)
			{
				//rManager2->setJointVal(robot_state.robot_joint);
				//char pbuffer[100];
				//char tmp_buffer[255];
				//int digit_num = 5;
				//SE3 robot2StateFromRobot1;
				//for (int i = 0; i < 9; i++)
				//	robot2StateFromRobot1[i] = robot_state.robot_rot[i];
				//for (int i = 0; i < 3; i++)
				//	robot2StateFromRobot1[9 + i] = robot_state.robot_pos[i];
				//robot2StateFromRobot1 = Trobotbase1 % Trobotbase2 * robot2StateFromRobot1;

				//char *temp_data = strtok(copy, "d");

				//char* copy2 = (char*)malloc(sizeof(char) * 30000);
				//memset(copy2, NULL, sizeof(char) * 30000);
				//strcat(copy2, temp_data);
				//strcat(copy2, "d");
				//for (int i = 0; i < 12; i++)
				//{
				//	temp_data = strtok(NULL, "d");
				//	strcpy(pbuffer, "");
				//	strcat(pbuffer, _gcvt(robot2StateFromRobot1[i], digit_num, tmp_buffer));
				//	strcat(copy2, pbuffer);
				//	strcat(copy2, "d");
				//}
				//while (temp_data != NULL)
				//{
				//	temp_data = strtok(NULL, "d");
				//	strcat(copy2, temp_data);
				//	strcat(copy2, "d");
				//}
				//serv.SendMessageToClient(copy2);
				//if (useSleep)
				//	Sleep(50);


				serv.SendMessageToClient("T2");
				if (useSleep)
					Sleep(50);
			}
			else
				printf("Wrong robot flag is given (Flag = 'R')!!!!!!\n");

			//cout << robot_state.robot_joint.transpose() << endl;

			//m.lock();
			//isVision = false;
			//isHYUPlanning = false;
			//isRobotState = true;
			//m.unlock();
			free(copy);
		}
		else if (hyu_data_flag == 'S') {
			char* copy = (char*)malloc(sizeof(char)*(strlen(hyu_data) + 1));
			for (unsigned int p = 0; p <= strlen(hyu_data); p++)
				copy[p] = hyu_data[p];

			pair<int, vector<int>> hyu_data_output = readRobotCommand(hyu_data, hyu_desired_dataset);

			if (hyu_data_output.first == 1 || hyu_data_output.first == 2)
			{
				if (hyu_data_output.second[0] == 1 || hyu_data_output.second[0] == 2)
				{
					// send to robot
					serv.SendMessageToClient(copy);
					if (useSleep)
						Sleep(50);
					printf(copy);
					printf("\n");
					if (hyu_data_output.second[0] == 1) // when gripper input comes
						gripObjectIdx[hyu_data_output.first - 1] = getObjectIdx(hyu_data_output.first);
				}
				else
				{
					char temp_char[3];
					sprintf(temp_char, "P%d", hyu_data_output.first);
					serv.SendMessageToClient(temp_char);
					if (useSleep)
						Sleep(50);
				}
			}
			else if (hyu_data_output.first == 3)
			{
				int nway_robot1 = hyu_desired_dataset[0].robot_gripper.size();
				int nway_robot2 = hyu_desired_dataset[1].robot_gripper.size();

				int nway1;
				int nway;
				int iter = 0;
				int n_inside_way = 19;
				char plus[10];
				char nway_char[10];
				int disregardNum = 3;
				planTwoArmOkay = false;
				planTwoArmOkay = (isTwoArm == true) && (nway_robot1 == nway_robot2) && (hyu_data_output.second[0] == 0) && (hyu_data_output.second[1] == 0);
				if (planTwoArmOkay) // planning two arms simultaneously 
				{
					serv.SendMessageToClient("P1");
					Sleep(50);
					serv.SendMessageToClient("P2");
					Sleep(50);
				}
				else 
				{

					if (hyu_data_output.second[0] == 0)
					{
						serv.SendMessageToClient("P1");
						Sleep(50);
					}
					else
					{
						char* tmp_Data1 = (char*)malloc(sizeof(char) * 30000);
						memset(tmp_Data1, NULL, sizeof(char) * 30000);
						strcat(tmp_Data1, "S");
						sprintf(plus, "%dd", 1);
						strcat(tmp_Data1, plus);
						strcpy(plus, "");
						strcpy(nway_char, "");

						//char tmp_Data1[30000] = "S";
						//sprintf(plus, "%dd", 1);
						//strcat(tmp_Data1, plus);
						//strcpy(plus, "");
						//strcpy(nway_char, "");


						for (unsigned int p = 0; p <= strlen(copy); p++)
						{
							if (iter != 0 && iter != 2)
							{
								sprintf(plus, "%c", copy[p]);
								strcat(tmp_Data1, plus);
								if (iter == 1 && copy[p] != 'd')
									strcat(nway_char, plus);
								if (iter == 1 && copy[p] == 'd')
									nway = atoi(nway_char);
							}
							if (copy[p] == 'd')
								iter++;
							if (iter == nway*n_inside_way + 1 + disregardNum)
								break;
						}
						char* send_data1 = (char*)malloc(sizeof(char)*(strlen(tmp_Data1) + 1));
						strcpy(send_data1, tmp_Data1);

						serv.SendMessageToClient(send_data1);
						Sleep(50);
						printf(send_data1);
						printf("\n");
						free(send_data1);
						free(tmp_Data1);
						if (hyu_data_output.second[0] == 1) // when gripper input comes
							gripObjectIdx[0] = getObjectIdx(1);
					}

					if (hyu_data_output.second[1] == 0)
					{
						serv.SendMessageToClient("P2");
						Sleep(50);
					}

					else
					{

						char* tmp_Data2 = (char*)malloc(sizeof(char) * 30000);
						memset(tmp_Data2, NULL, sizeof(char) * 30000);
						strcat(tmp_Data2, "S");
						sprintf(plus, "%dd", 2);
						strcat(tmp_Data2, plus);
						strcpy(plus, "");
						strcpy(nway_char, "");

						//char tmp_Data2[30000] = "S";
						//sprintf(plus, "%dd", 2);
						//strcat(tmp_Data2, plus);
						//strcpy(plus, "");
						//strcpy(nway_char, "");
						iter = 0;
						for (unsigned int p = 0; p <= strlen(copy); p++)
						{
							if (iter == 1 || iter == 2)
								sprintf(plus, "%c", copy[p]);
							if (iter == 1 && copy[p] != 'd')
								strcat(nway_char, plus);
							if (iter == 1 && copy[p] == 'd')
							{
								nway1 = atoi(nway_char);
								strcpy(nway_char, "");
							}
							if (iter == 2 && copy[p] != 'd')
								strcat(nway_char, plus);
							if (iter == 2 && copy[p] == 'd')
								nway = atoi(nway_char);

							if (iter == 2 || iter >= nway1*n_inside_way + 1 + disregardNum)
							{
								sprintf(plus, "%c", copy[p]);
								strcat(tmp_Data2, plus);
							}
							if (copy[p] == 'd')
								iter++;
							if (iter == (nway + nway1)*n_inside_way + 2 + disregardNum || copy[p] == '\0')
								break;
						}
						char* send_data2 = (char*)malloc(sizeof(char)*(strlen(tmp_Data2) + 1));
						strcpy(send_data2, tmp_Data2);

						serv.SendMessageToClient(send_data2);
						Sleep(50);
						printf(send_data2);
						printf("\n");
						free(send_data2);
						free(tmp_Data2);
						if (hyu_data_output.second[1] == 1) // when gripper input comes
							gripObjectIdx[1] = getObjectIdx(2);
					}
				}
			}

			free(copy);
		}
		else if (hyu_data_flag == 'A')
		{
			char temp_char[2];
			sprintf(temp_char, "%c", hyu_data[1]);
			int robotFlag = atoi(temp_char);
			sprintf(temp_char, "%c", hyu_data[2]);
			int objectMaintainFlag = atoi(temp_char);
			if (objectMaintainFlag == 0)
			{
				// initialize all (object, robot)
				for (unsigned int i = 0; i < objects.size(); i++)
				{
					objects[i]->setBaseLinkFrame(TobjectsInitSimul[i]);
					objects[i]->KIN_UpdateFrame_All_The_Entity();
				}
				for (unsigned int i = 0; i < gripObjectIdx.size(); i++)
				{
					if (gripObjectIdx[i] != -1)
						TlastObjects_multi[i] = TobjectsInitSimul[gripObjectIdx[i]];
					gripObjectIdx[i] = -1;
				}
				if (robotFlag == 1 || robotFlag == 2)
					initialPlanning[robotFlag - 1] = true;
				else
				{
					initialPlanning[0] = true;
					initialPlanning[1] = true;
					initialPlanning_twoArm = true;
				}
			}
			else
			{
				// maintain object, initialize planning from current robot state update
				if (robotFlag == 1 || robotFlag == 2)
					startPlanningFromCurRobotState[robotFlag - 1] = true;
				else
				{
					startPlanningFromCurRobotState[0] = true;
					startPlanningFromCurRobotState[1] = true;
					startPlanningFromCurRobotState_twoArm = true;
				}
			}
				

			// to see values
			gripObjectIdx;
			TlastObjects_multi;
			TinitObjects_multi;
			int stop = 1;
		}

		else if (hyu_data_flag == 'P') {
			vector<bool> attachObject(0);		// grip status from received data
			vector<bool> waypointFlag(0);
			vector<double> stepsize(0);
			vector<bool> attachobject(0);		// grip status with disregarding unavailable points

			vector<vector<bool>> attachObject_twoArm(2);
			vector<vector<bool>> waypointFlag_twoArm(2);
			vector<vector<bool>> attachobject_twoArm(2);

			int robotFlag = 0;
			robotFlag = readRobotCurState(hyu_data, robot_state);

			if (robotFlag != 1 && robotFlag != 2)
				printf("Wrong robot Flag is given (Flag = 'P')!!!\n");
			else
			{
				// send confirming message to robot
				char temp_char[3];
				sprintf(temp_char, "T%d", robotFlag);
				serv.SendMessageToClient(temp_char);
				if (useSleep)
					Sleep(50);
				if (!planTwoArmOkay)
				{
					// RRT problem setting for single arm planning
					// decide initial point (read from robot for initial planning, use last joint val otherwise)
					Eigen::VectorXd planningInit;
					if (initialPlanning[robotFlag - 1])
						planningInit = robot_state.robot_joint;
					else
					{
						if (startPlanningFromCurRobotState[robotFlag - 1])
							planningInit = robot_state.robot_joint;
						else
							planningInit = lastJointVal_multi[robotFlag - 1];
						startPlanningFromCurRobotState[robotFlag - 1] = false;
						if (gripObjectIdx[robotFlag - 1] != -1 && distSE3(TinitObjects_multi[robotFlag - 1], TlastObjects_multi[robotFlag - 1]) > 1.0e-5)		// move busbar to its last location when releasing
						{
							objects[gripObjectIdx[robotFlag - 1]]->setBaseLinkFrame(TlastObjects_multi[robotFlag - 1]);
							objects[gripObjectIdx[robotFlag - 1]]->KIN_UpdateFrame_All_The_Entity();
						}
					}

					RRT_problemSettingFromSingleRobotCommand(hyu_desired_dataset[robotFlag - 1], attachObject, planningInit, waypointFlag, robotFlag);
					for (unsigned int i = 0; i < waypointFlag.size(); i++)
					{
						if (waypointFlag[i])
						{
							stepsize.push_back(0.1);
							attachobject.push_back(attachObject[i]);
						}
					}
					attachObjectWaypoint[robotFlag - 1] = attachObject;

					// Solve RRT
					//m.lock();
					if (goalPos.size() > 0)
					{
						RRTSolve_HYU_SingleRobot(attachobject, stepsize, robotFlag);

						char* send_data = (char*)malloc(sizeof(char) * 30000);
						memset(send_data, NULL, sizeof(char) * 30000);
						char *add = makeJointCommand_SingleRobot(renderTraj_multi[robotFlag - 1], hyu_desired_dataset[robotFlag - 1], robotFlag);
						strcat(send_data, add);
						delete(add);

						//char send_data[30000];
						//strcpy(send_data, "");
						//strcat(send_data, makeJointCommand_SingleRobot(renderTraj_multi[robotFlag - 1], hyu_desired_dataset[robotFlag - 1], robotFlag));

						//char* send_data = makeJointCommand_SingleRobot(renderTraj_multi[robotFlag - 1], hyu_desired_dataset[robotFlag - 1], robotFlag);
						serv.SendMessageToClient(send_data);
						if (useSleep)
							Sleep(50);
						printf("%s\n", send_data);
						free(send_data);
						if (checkTorque)
						{
							vector<Eigen::VectorXd> tauTrj = calculateJointTorque(renderTraj_multi[robotFlag - 1], robotFlag);
							Eigen::VectorXd maxTau = Eigen::VectorXd::Zero(6);
							for (unsigned int i = 0; i < tauTrj.size(); i++)
							{
								maxTau = maxTau.cwiseMax(tauTrj[i]);
								printf("maximum torque: \n");
								cout << maxTau.transpose() << endl;
							}
						}
						if (attachobject.size() > 0 && attachobject[attachobject.size() - 1])
							gripState_multi[robotFlag - 1] = 1;
						else
							gripState_multi[robotFlag - 1] = 0;

						initialPlanning[robotFlag - 1] = false;
					}
					else
					{
						// send not feasible flag
						serv.SendMessageToClient("W");
					}
					//m.unlock();
				}
				else
				{
					// RRT problem setting for multi arm planning
					// decide initial point (read from robot for initial planning, use last joint val otherwise)
					if (initialPlanning_twoArm)
						planningInit_twoArm.segment(6*(robotFlag - 1), 6) = robot_state.robot_joint;
					else
					{
						if (startPlanningFromCurRobotState_twoArm)
							planningInit_twoArm.segment(6 * (robotFlag - 1), 6) = robot_state.robot_joint;
						else
							planningInit_twoArm.segment(6 * (robotFlag - 1), 6) = lastJointVal_multi_twoArm.segment(6 * (robotFlag - 1), 6);
						startPlanningFromCurRobotState_twoArm = false;
						for (int robotnum = 0; robotnum < 2; robotnum++)
						{
							if (gripObjectIdx[robotnum] != -1 && distSE3(TinitObjects_multi[robotnum], TlastObjects_multi[robotnum]) > 1.0e-5)		// move busbar to its last location when releasing
							{
								objects[gripObjectIdx[robotnum]]->setBaseLinkFrame(TlastObjects_multi[robotnum]);
								objects[gripObjectIdx[robotnum]]->KIN_UpdateFrame_All_The_Entity();
							}
						}
					}
					
					planningInitUpdated[robotFlag - 1] = true;

					// start planning when both arm initial points are updated
					if (planningInitUpdated[0] && planningInitUpdated[1])
					{
						for (int i = 0; i < 2; i++)
						{
							attachObject_twoArm[i].resize(0);
							waypointFlag_twoArm[i].resize(0);
							attachobject_twoArm[i].resize(0);
						}
						RRT_problemSettingFromMultiRobotCommand(hyu_desired_dataset, attachObject_twoArm, planningInit_twoArm, waypointFlag_twoArm);
						vector<double> stepsize(0);

						for (unsigned int i = 0; i < waypointFlag_twoArm[0].size(); i++)
						{
							if (waypointFlag_twoArm[0][i] && waypointFlag_twoArm[1][i])
							{
								stepsize.push_back(0.1);
								attachobject_twoArm[0].push_back(attachObject_twoArm[0][i]);
								attachobject_twoArm[1].push_back(attachObject_twoArm[1][i]);
							}
						}

						// Solve RRT
						if (goalPos.size() > 0)
						{
							RRTSolve_HYU_multiRobot(attachobject_twoArm, stepsize);

							char* send_data = (char*)malloc(sizeof(char) * 30000);
							for (int i = 0; i < 2; i++)
							{
								memset(send_data, NULL, sizeof(char) * 30000);
								char *add = makeJointCommand_MultiRobot(renderTraj_twoArm, hyu_desired_dataset, i + 1);
								strcat(send_data, add);
								delete(add);

								//char send_data[30000];
								//strcpy(send_data, "");
								//strcat(send_data, makeJointCommand_SingleRobot(renderTraj_multi[robotFlag - 1], hyu_desired_dataset[robotFlag - 1], robotFlag));

								//char* send_data = makeJointCommand_SingleRobot(renderTraj_multi[robotFlag - 1], hyu_desired_dataset[robotFlag - 1], robotFlag);
								serv.SendMessageToClient(send_data);
								Sleep(50);
								printf("%s\n", send_data);
							}
							
							free(send_data);
							for (int robotnum = 0; robotnum < 2; robotnum++)
							{
								if (attachobject_twoArm[robotnum].size() > 0 && attachobject_twoArm[robotnum][attachobject_twoArm[robotnum].size() - 1])
									gripState_multi[robotnum] = 1;
								else
									gripState_multi[robotnum] = 0;

								initialPlanning[robotnum] = false;
							}
							initialPlanning_twoArm = false;
						}
						// initialize two arm planning conditions
						planTwoArmOkay = false;
						planningInitUpdated[0] = false;
						planningInitUpdated[1] = false;
					}
				}
			}
		}

		// _CrtDumpMemoryLeaks();
		/*hyu_data[0] = '\0';*/
		if (hyu_data[0] != '\0')
			free(hyu_data);


		hyu_data_flag = ' ';
		if (useSleep)
			Sleep(50);
	}
}

