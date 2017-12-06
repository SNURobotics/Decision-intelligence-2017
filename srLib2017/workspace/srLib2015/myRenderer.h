#ifndef __MY_RENDERER__
#define __MY_RENDERER__

#include "srDyn/srSpace.h"
#include "SceneGraph/SceneGraphRenderer.h"

class myRenderer : public SceneGraphRenderer
{
	srSpace* targetSpace;
	void (*updatefunc)(void);

public:
	void InitializeNode(srSpace* pSapce)
	{
		targetSpace = pSapce;

		glutSetWindow(window1);
		// original
		//m_camera = new Camera(2.0, 0.4*SR_PI_HALF, 0.5*SR_PI_HALF);
		// workcell view
		Vec3 focus = Vec3(0.025, 1.095, 1.176);
		m_camera = new Camera(2.5, 0.3*SR_PI_HALF, 2.0*SR_PI_HALF, focus);
		///////////////////////////////////////////////////////////
		Light* light = new Light;
		Light* light2 = new Light;
		Grid* grid = new Grid(10, 1);
		Shader* shader = new Shader();
		Group*	nodeGroup = new Group();
		Group* coordinateGroup = new Group();
		Coordinate* coord = new Coordinate();
		coordinateGroup->addNode(coord);
		shader->setColor(0.3, 0.3, 0.3);

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		addNode(grid);
		addNode(nodeGroup);
		addNode(coordinateGroup);

		nodeGroup->addNode(shader);
		parsing(nodeGroup, coordinateGroup);

		light->setPosition(10, 30, -50, 1);
		light2->setPosition(-40, 40, 40, 1);

		m_root->addNode(m_camera);
		m_root->addNode(light);
		m_root->addNode(light2);

	}

	void parsing(Group* nodeGroup, Group* coordiGroup)
	{
		int numSystem = targetSpace->m_Systems.get_size();

		for (int i = 0; i < numSystem; i++)
		{
			for (int j = 0; j < targetSpace->m_Systems[i]->m_KIN_Joints.get_size(); j++)
			{
				if (targetSpace->m_Systems[i]->m_KIN_Joints[j]->GetType() == srJoint::JOINTTYPE::REVOLUTE)
				{
					Coordinate* coord = new Coordinate(0.1, 1, 1);
					coord->setTargetSE3Pointer(&targetSpace->m_Systems[i]->m_KIN_Joints[j]->m_Frame);
					coordiGroup->addNode(coord);
				}
			}
		}

		for (int i = 0; i < numSystem; i++){
			for (int j = 0; j < targetSpace->m_Systems[i]->m_KIN_Links.get_size(); j++){
				Group*	linkGroup = new Group();
				Shader* linkShader = new Shader();
				Leaf*	body;
				double dim[3];
				float color[4];
				targetSpace->m_Systems[i]->m_KIN_Links[j]->GetGeomInfo().GetDimension(dim[0], dim[1], dim[2]);
				targetSpace->m_Systems[i]->m_KIN_Links[j]->GetGeomInfo().GetColor(color[0], color[1], color[2], color[3]);

				linkShader->setColor(color[0], color[1], color[2], color[3]);
				srGeometryInfo::SHAPETYPE type = targetSpace->m_Systems[i]->m_KIN_Links[j]->GetGeomInfo().GetShape();
				if (type == srGeometryInfo::BOX){
					body = new Box(0.5*dim[0], 0.5*dim[1], 0.5*dim[2]);
				}
				else if (type == srGeometryInfo::CAPSULE)
					body = new Capsule(0.5*dim[0], dim[1]);
				else if (type == srGeometryInfo::CYLINDER)
					body = new Cylinder(0.5*dim[0], dim[1]);
				else if (type == srGeometryInfo::SPHERE)
					body = new Sphere(0.5*dim[0]);
				else if (type == srGeometryInfo::TDS){
					body = new TDSNode(targetSpace->m_Systems[i]->m_KIN_Links[j]->GetGeomInfo().GetFileName(),
						targetSpace->m_Systems[i]->m_KIN_Links[j]->GetGeomInfo().GetLocalFrame());
				}
				else if (type == srGeometryInfo::STL) {
					//linkGroup->addNode(linkShader);
					body = new STLNode(targetSpace->m_Systems[i]->m_KIN_Links[j]->GetGeomInfo().GetFileName(),
						targetSpace->m_Systems[i]->m_KIN_Links[j]->GetGeomInfo().GetLocalFrame());
				}
				else
					continue;

				Coordinate* coord = new Coordinate(0.1, 1, 1);
				coord->setTargetSE3Pointer(&targetSpace->m_Systems[i]->m_KIN_Links[j]->m_Frame);
				body->setTargetSE3Pointer(&targetSpace->m_Systems[i]->m_KIN_Links[j]->m_Frame);
				linkGroup->addNode(body);
				linkGroup->addNode(linkShader);
				nodeGroup->addNode(linkGroup);
			}

		}
		Group*   bigColliGroup = new Group();
		addNode(bigColliGroup);
		for (int i = 0; i < numSystem; i++){
			for (int j = 0; j < targetSpace->m_Systems[i]->m_KIN_Links.get_size(); j++){
				for (int k = 0; k < targetSpace->m_Systems[i]->m_KIN_Links[j]->m_Collisions.get_size(); k++){
					Group*	colliGroup = new Group();
					Leaf*	body;
					double dim[3];
					//float color[4];
					srGeometryInfo::SHAPETYPE type = targetSpace->m_Systems[i]->m_KIN_Links[j]->m_Collisions[k]->GetGeomInfo().GetShape();
					targetSpace->m_Systems[i]->m_KIN_Links[j]->m_Collisions[k]->GetGeomInfo().GetDimension(dim[0], dim[1], dim[2]);

					Shader* colliShader = new Shader();
					colliShader->setColor(0, 0, 0, 0.1);
					if (type == srGeometryInfo::BOX)
						body = new Box(0.5*dim[0], 0.5*dim[1], 0.5*dim[2]);
					else if (type == srGeometryInfo::CAPSULE)
						body = new Capsule(0.5*dim[0], dim[1]);
					else if (type == srGeometryInfo::CYLINDER)
						body = new Cylinder(0.5*dim[0], dim[1]);
					else if (type == srGeometryInfo::SPHERE)
						body = new Sphere(0.5*dim[0]);
					else
						continue;
					body->setTargetSE3Pointer(&targetSpace->m_Systems[i]->m_KIN_Links[j]->m_Collisions[k]->m_Frame);
					colliGroup->addNode(colliShader);
					colliGroup->addNode(body);

					bigColliGroup->addNode(colliGroup);
				}
			}
		}
	}

	void setUpdateFunc(void(*func)(void)){
		updatefunc = func;
	}

private:
	virtual void updateScene()
	{
		updatefunc();
	}
};

#endif