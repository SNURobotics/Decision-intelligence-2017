/*******************************************************************/
/*                                                                 */
/*    FOR USING URDF FORMAT IN SRLIB                               */
/*    MADE BY: Keunjun Choi                                        */
/*    Date: 2014-12-15                                             */
/*                                NAMESPACE:     GAMASOT           */
/*                                                                 */
/*******************************************************************/

#ifndef __SR_URDF__
#define __SR_URDF__

//#define srLOG

#include <iostream>
//#include <boost/algorithm/string.hpp>
#include <fstream>

#include "urdf/urdf_parser/urdf_parser.h"
#include "srGamasot/srRobot.h"
#include "srGamasot/srSTL.h"

using namespace std;

namespace gamasot
{
	static string urlcon(string url, string package_url)
	{
		return package_url.append(url.c_str() + 9); // package:/ 을 url로 바꾼다.
	}

	static void urdf2srRobot(srRobot* mRobot, string package_url, string package_name, string robotname, bool collisionflag = true)
	{
#ifdef srLOG
		printf("urdf 파일을 srLib에서 쓸 수 있도록 변환시켜 줍니다.\n파일: %s\n", (package_url + "/robots/" + robotname + ".stl").c_str());
#endif

#ifdef srLOG
		printf(".urdf 파일을 읽기 시작합니다.\n");
#endif
		for (int i = package_url.length() - 1; i >= 0; i--)
		{
			if (package_url[i] == '\n' || package_url[i] == '/' || package_url[i] == '\\')
			{
				package_url.erase(i);
				continue;
			}
			break;
		}

		string xml_string;
		fstream xml_file(package_url + "/" + package_name + "/robots/" + robotname + ".urdf", fstream::in);
		if (!xml_file.good())
		{
#ifdef srLOG
			printf("파일을 열수 없습니다. 확인해주세요.\n");
#endif
			return;
		}
		while (xml_file.good())
		{
			std::string line;
			std::getline(xml_file, line);
			xml_string += (line + "\n");
		}
		xml_file.close();
#ifdef srLOG
		printf("파일을 다 읽었습니다.\n");
#endif

#ifdef srLOG
		printf(".urdf 파일을 로봇 형태로 읽어드립니다.");
#endif
		urdf::ModelInterfaceSharedPtr robot = urdf::parseURDF(xml_string);
		if (!robot)
		{
#ifdef srLOG
			printf(".urdf 파일에 형태가 맞지 않습니다. 확인해주세요.\n");
#endif
			return;
		}

#ifdef srLOG
		printf("로봇을 srRobot으로 변환합니다.\n");
#endif
		urdf::LinkConstSharedPtr root_link = robot->getRoot();

#ifdef srLOG
		printf("Link 정보를 불러옵니다.\n");
#endif
		srLink* link;
		srCollision* coll;
		double r, p, y;
		string fn;
		for (map<std::string, urdf::LinkSharedPtr >::iterator child = robot->links_.begin(); child != robot->links_.end(); child++)
		{
			urdf::LinkSharedPtr linkp = child->second;
#ifdef srLOG
			printf("Link (%s) 가 들어왔습니다.\n", (linkp->name).c_str());
#endif

			link = mRobot->getLink(linkp->name);
			coll = mRobot->getCollision(linkp->name);

#ifdef srLOG
			printf("Inertia, mass 추가\n");
#endif
			if (linkp->inertial != NULL)
			{
				// TODO: see below
				//link->SetInertia(Inertia(linkp->inertial->mass, linkp->inertial->ixx, linkp->inertial->iyy, linkp->inertial->izz));
				//link->SetInertia(Inertia(linkp->inertial->mass, linkp->inertial->ixx, 1, linkp->inertial->izz));
				link->SetInertia(Inertia(1,1,1,1));
			}

#ifdef srLOG
			printf("Visual 추가\n");
#endif
			if (linkp->visual != NULL)
			{
				switch (linkp->visual->geometry->type)
				{
				case urdf::Geometry::MESH:
					fn = ((urdf::Mesh*)linkp->visual->geometry.get())->filename;
					//boost::to_lower(fn);
					std::transform(fn.begin(), fn.end(), fn.begin(), ::tolower);
					if (fn.substr(fn.find_last_of(".")+1) == "stl")
					{
						link->GetGeomInfo().SetShape(srGeometryInfo::STL);
					}
					else
					{
						link->GetGeomInfo().SetShape(srGeometryInfo::TDS);
					}
					if (linkp->visual->material != NULL)
					{
						link->GetGeomInfo().SetColor(linkp->visual->material->color.r, linkp->visual->material->color.g, linkp->visual->material->color.b, linkp->visual->material->color.a);
					}
					link->GetGeomInfo().SetFileName((char *)urlcon(((urdf::Mesh*)linkp->visual->geometry.get())->filename, package_url).c_str());
					linkp->visual->origin.rotation.getRPY(r, p, y);
					link->GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(y, p, r), Vec3(linkp->visual->origin.position.x, linkp->visual->origin.position.y, linkp->visual->origin.position.z)));
					break;
				default:
#ifdef srLOG
					printf("Visual Geometry Type에 아직 구현되지 않은 정보가 들어왔습니다. 구현해주세요.\n");
#endif
					break;
				}
			}

#ifdef srLOG
			printf("Collision 추가\n");
#endif
			if (linkp->collision != NULL && collisionflag)
			{
				switch (linkp->collision->geometry->type)
				{
				case urdf::Geometry::MESH:
					coll = gamasot::STL2Collsion(gamasot::readSTLfile(urlcon(((urdf::Mesh*)linkp->collision->geometry.get())->filename, package_url)), "BOX");
					if (coll == NULL) break;
					coll->GetGeomInfo().SetColor(1, 0, 0, 0.1);
					coll->SetPosition(Vec3(linkp->visual->origin.position.x, linkp->visual->origin.position.y, linkp->visual->origin.position.z));
					linkp->visual->origin.rotation.getRPY(r, p, y);
					coll->GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(y, p, r), Vec3(linkp->visual->origin.position.x, linkp->visual->origin.position.y, linkp->visual->origin.position.z)));
					link->AddCollision(coll);
					break;
				default:
#ifdef srLOG
					printf("Collision Geometry Type에 아직 구현되지 않은 정보가 들어왔습니다. 구현해주세요.\n");
#endif
					break;
				}
			}

#ifdef srLOG
			printf("Link 불러오기 성공\n");
#endif
		}

#ifdef srLOG
		printf("Joint 정보를 불러옵니다.\n");
#endif
		srJoint* joint;
		Vec3 axis, oth1, oth2;
		SE3 rot;
		for (map<std::string, std::shared_ptr<urdf::Joint> >::iterator child = robot->joints_.begin(); child != robot->joints_.end(); child++)
		{
			std::shared_ptr<urdf::Joint> jointp = child->second;
#ifdef srLOG
			printf("Joint (%s) 가 들어왔습니다.\n", (jointp->name).c_str());
#endif

			switch (jointp->type)
			{
			case urdf::Joint::FIXED:
				joint = mRobot->getJoint(jointp->name, srJoint::WELD);
				joint->SetChildLink(mRobot->getLink(jointp->child_link_name));
				joint->SetParentLink(mRobot->getLink(jointp->parent_link_name));
				r = p = y = 0;
				jointp->parent_to_joint_origin_transform.rotation.getRPY(r, p, y);
				joint->SetParentLinkFrame(EulerZYX(Vec3(0, 0, 0), Vec3(jointp->parent_to_joint_origin_transform.position.x, jointp->parent_to_joint_origin_transform.position.y, jointp->parent_to_joint_origin_transform.position.z)) * EulerZYX(Vec3(y, p, r), Vec3(0, 0, 0)));
				break;

			case urdf::Joint::REVOLUTE:
				axis = Vec3(jointp->axis.x, jointp->axis.y, jointp->axis.z);
				oth1 = Vec3(jointp->axis.z, jointp->axis.z, -(jointp->axis.x + jointp->axis.y));
				oth2 = Vec3(-(jointp->axis.y + jointp->axis.z), jointp->axis.x, jointp->axis.x);
				if (oth1 == Vec3(0, 0, 0))
					oth1.Clone(oth2);
				oth1.Normalize();
				oth2 = Cross(axis, oth1);
				rot = SE3(oth1[0], oth2[0], axis[0], oth1[1], oth2[1], axis[1], oth1[2], oth2[2], axis[2]);

				joint = mRobot->getJoint(jointp->name, srJoint::REVOLUTE);
				joint->SetActType(srJoint::ACTTYPE::HYBRID);
				//joint->SetActType(srJoint::ACTTYPE::TORQUE);
				joint->SetChildLink(mRobot->getLink(jointp->child_link_name));
				joint->SetParentLink(mRobot->getLink(jointp->parent_link_name));
				if (jointp->child_link_name == "NX03_B03_HINGE-H-T-7-1")
				{
					r = r;
				}
				r = p = y = 0;
				jointp->parent_to_joint_origin_transform.rotation.getRPY(r, p, y);
				joint->SetParentLinkFrame(EulerZYX(Vec3(0, 0, 0), Vec3(jointp->parent_to_joint_origin_transform.position.x, jointp->parent_to_joint_origin_transform.position.y, jointp->parent_to_joint_origin_transform.position.z)) * EulerZYX(Vec3(y, p, r)) * Inv(rot));
				joint->SetChildLinkFrame(Inv(rot));
				break;


			default:
#ifdef srLOG
				printf("Joint Type에 아직 구현되지 않은 정보가 들어왔습니다. 구현해주세요.\n");
#endif
				break;
			}

#ifdef srLOG
			printf("Joint 불러오기 성공\n");
#endif
		}

#ifdef srLOG
		printf("srLib에서 로봇을 쓸 수 있도록 마무리 작업중입니다.\n");
#endif
		srLink* base = mRobot->getLink(robot->getRoot()->name);
		base->SetFrame(EulerZYX(Vec3(0,0,0.0)));
		mRobot->SetBaseLink(base);
		mRobot->SetBaseLinkType(srSystem::FIXED);
		mRobot->SetSelfCollision(false);

#ifdef srLOG
		printf("urdf 파일을 srLib에서 쓸 수 있도록 변환했습니다.\n");
#endif
	}
}

#endif