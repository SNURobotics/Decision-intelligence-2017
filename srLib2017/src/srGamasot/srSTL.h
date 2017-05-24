/*******************************************************************/
/*                                                                 */
/*    FOR USING STL FILES IN SRLIB                                 */
/*    MADE BY: Keunjun Choi                                        */
/*    Date: 2014-12-14                                             */
/*                                NAMESPACE:     GAMASOT           */
/*                                                                 */
/*******************************************************************/

#ifndef __SR_STL__
#define __SR_STL__

#include "LieGroup/LieGroup.h"
//#include "LieGroup/rmatrix3.h"
#include <list>
#include <string>

#include "srDyn/srSpace.h"
#include "../Eigen/core"
#include "../Eigen/dense"
using namespace std;

namespace gamasot
{
	class facet
	{
	public:
		Eigen::Vector3d normal;
		Eigen::Vector3d vertex[3];
		//RMatrix vertex[3];
		short attribute;

		facet()
		{
			normal.setZero();
			for (int i = 0; i < 3; i++)
			{
				vertex[i].setZero();
				//vertex[i] = RMatrix(3, 1);
			}
		}
	};

	typedef list < facet > STLc;

	static double readBinaryFloat(FILE* in)
	{
		float temp;
		fread((char*)&temp, sizeof(float), 1, in);
		return (double) temp;
	}

	static STLc readSTLfile(string stl_filename)
	{
		FILE *stlin = fopen(stl_filename.c_str(), "rb");
		STLc stl_content;

		if (stlin == NULL)
		{
#ifdef srLOG
			printf("파일이 존재하지 않습니다.\n(%s)\n", stl_filename.c_str());
#endif
			return stl_content;
		}

		int n;
		char temp;

#ifdef srLOG
		printf("STL 파일을 읽기 시작합니다.\n(%s)\n", stl_filename.c_str());
#endif
		for (int i = 0; i < 80; i++)
		{
			fread((char*)&temp, sizeof(char), 1, stlin);
		}
		fread((char*)&n, sizeof(int), 1, stlin);

		for (; n > 0; n--)
		{
			facet* content = new facet();
			content->normal[0] = readBinaryFloat(stlin);
			content->normal[1] = readBinaryFloat(stlin);
			content->normal[2] = readBinaryFloat(stlin);
			for (int i = 0; i < 3; i++)
			{
				content->vertex[i][0] = readBinaryFloat(stlin);
				content->vertex[i][1] = readBinaryFloat(stlin);
				content->vertex[i][2] = readBinaryFloat(stlin);
			}
			fread((char*)&content->attribute, sizeof(short), 1, stlin);
			stl_content.push_back(*content);
		}
#ifdef srLOG
		printf("STL 파일을 모두 읽었습니다.\n");
#endif

		fclose(stlin);
		return stl_content;
	}

	static srCollision* STL2Collsion(STLc stl_content, string type)
	{
#ifdef srLOG
		printf("STLc을 srCollsion으로 바꿉니다.\n");
#endif
		srCollision* _srCollision = new srCollision();

		if (type == "BOX")
		{
#ifdef srLOG
			printf("Collision Type은 BOX입니다.\n", type.c_str());
#endif
			int n;

			Eigen::Vector3d mean;
			Eigen::Matrix3d cov;
			Eigen::Matrix3d cov_eignvec;

			mean.setZero();
			cov.setZero();
			cov_eignvec.setZero();
			

			//RMatrix mean = RMatrix(3, 1);
			//RMatrix cov = RMatrix(3, 3);
			//RMatrix cov_eignvec = RMatrix(3, 3);
			//RMatrix cov_eignval = RMatrix(3, 1);

			n = 0;
			for (STLc::iterator point = stl_content.begin(); point != stl_content.end(); point++)
			{
				for (int i = 0; i < 3; i++)
				{
					mean = mean + point->vertex[i];
					cov = cov + point->vertex[i] * (point->vertex[i].transpose());
					n++;
				}
			}
			if (n == 0)
			{
#ifdef srLOG
				printf("Link의 Vertex가 0입니다.\n");
#endif
				return NULL;
			}
			mean = mean / n;
			cov = cov - mean * (mean).transpose();

#ifdef srLOG
			printf("MEAN, COVARIANCE 모두 계산 했습니다.\n");
#endif
			Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(cov);
			cov_eignvec = eigensolver.eigenvectors();


			Eigen::Vector3d W = cov_eignvec.col(0);
			Eigen::Vector3d H = cov_eignvec.col(1);
			Eigen::Vector3d D = cov_eignvec.col(2);

			W.normalize();
			H.normalize();
			D.normalize();

		//	Eig(cov, cov_eignvec, cov_eignval);
		//	cov_eignvec = ~cov_eignvec;

			//RMatrix W = RMatrix(3, 1);
			//RMatrix H = RMatrix(3, 1);
			//RMatrix D = RMatrix(3, 1);

			//W[0] = cov_eignvec[0]; W[1] = cov_eignvec[3]; W[2] = cov_eignvec[6];
			//H[0] = cov_eignvec[1]; H[1] = cov_eignvec[4]; H[2] = cov_eignvec[7];
			//D[0] = cov_eignvec[2]; D[1] = cov_eignvec[5]; D[2] = cov_eignvec[8];

			//W.Normalize();
			//H.Normalize();
			//D.Normalize();

			double wmin, hmin, dmin;
			double wmax, hmax, dmax;
			bool w, h, d;

			w = h = d = false;
			wmin = hmin = dmin = 0;
			wmax = hmax = dmax = 0;

			for (STLc::iterator point = stl_content.begin(); point != stl_content.end(); point++)
			{
				for (int i = 0; i < 3; i++)
				{
					double temp;
					temp = W.dot(point->vertex[i]);
						//(point->vertex[i] * W)[0];
					if (!w)
					{
						wmin = wmax = temp;
						w = true;
					}
					else
					{
						if (wmin > temp) wmin = temp;
						if (wmax < temp) wmax = temp;
					}
					temp = H.dot(point->vertex[i]);
					//temp = (~point->vertex[i] * H)[0];
					if (!h)
					{
						hmin = hmax = temp;
						h = true;
					}
					else
					{
						if (hmin > temp) hmin = temp;
						if (hmax < temp) hmax = temp;
					}
					temp = D.dot(point->vertex[i]);
					//temp = (~point->vertex[i] * D)[0];
					if (!d)
					{
						dmin = dmax = temp;
						d = true;
					}
					else
					{
						if (dmin > temp) dmin = temp;
						if (dmax < temp) dmax = temp;
					}
				}
			}

			SO3 rotation = SO3(W[0], H[0], D[0], W[1], H[1], D[1], W[2], H[2], D[2]);
			Vec3 origin = Inv(rotation) * Vec3((wmin + wmax) / 2, (hmin + hmax) / 2, (dmin + dmax) / 2);

#ifdef srLOG
			printf("srCollision으로 변환 중...\n");
#endif
			_srCollision->GetGeomInfo().SetShape(srGeometryInfo::BOX);
			_srCollision->GetGeomInfo().SetDimension(wmax-wmin, hmax-hmin, dmax-dmin);
			_srCollision->SetLocalFrame(SE3(Inv(rotation), origin));
#ifdef srLOG
			printf("srCollision 변환이 성공적으로 끝났습니다.\n");
#endif
		}
		else
		{
#ifdef srLOG
			printf("TYPE을 잘못 입력했습니다.\n");
#endif
		}

		return _srCollision;
	}
}

#endif