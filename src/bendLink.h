#ifndef BENDLINK_H
#define BENDLINK_H

#include <crossingNode.h>

//#define LOGOUT 1
#define POINT_MASS 0.001

class bendLink
{
public:
	// 大中小
	bendLink(crossingNode* node1, crossingNode* node2, crossingNode* node3, float kb):node1(node1), node2(node2), node3(node3), kb(kb)
	{

	}


	void solveU(double deltaT)
	{
		// u1 > u0 > u2
		Vector3d x0(node2->position.x, node2->position.y, node2->position.z), 
			x1(node1->position.x, node1->position.y, node1->position.z),
			x2(node3->position.x, node3->position.y, node3->position.z);

		double u0 = node2->u, u1 = node1->u, u2 = node3->u;

		double l1 = (x1 - x0).norm(), l2 = (x2 - x0).norm();
		//double kb = 0.01 * 3.1415 * 0.25 * 0.25;

		Vector3d d1 = (x1 - x0) / l1, d2 = (x2 - x0) / l2;
		double theta = acos((-d1.transpose() * d2)[0]);
		if(u1 > u0 && u0 > u2)
		{
			// 水平状没有力的作用
			if(theta <= 0.01)
				return;

			Matrix3d I = Matrix3d::Identity();
			Matrix3d P1 = I - d1 * d1.transpose(), P2 = I - d2 * d2.transpose();

			Vector3d Fx1 = ((-2 * kb * theta) / (l1 * (u1 - u2) * sin(theta))) * P1 * d2, 
				Fx2 = ((-2 * kb * theta) / (l2 * (u1 - u2) * sin(theta))) * P2 * d1, 
				Fx0 = -(Fx1 + Fx2);

			double Fu1 = (kb * theta * theta) / (u1*u1 - 2*u1*u2 + u2*u2),
				Fu2 = -Fu1,
				Fu0 = 0;

#ifdef LOGOUT
			cout << "theta:\n" << theta << std::endl;

			//cout << "Fx0:\n" << Fx0 << endl;
			//cout << "Fx1:\n" << Fx1 << endl;
			//cout << "Fx2:\n" << Fx2 << endl;

			//cout << "Fu0:\n" << Fu0 << endl;
			//cout << "Fu1:\n" << Fu1 << endl;
			//cout << "Fu2:\n" << Fu2 << endl;
#endif // LOGOUT

			// 更改速度
			node1->velocity.x += Fx1(0) / POINT_MASS;
			node1->velocity.y += Fx1(1) / POINT_MASS;
			node1->velocity.z += Fx1(2) / POINT_MASS;
			node1->velocityUV.x += Fu1 / POINT_MASS;

			node2->velocity.x += Fx0(0) / POINT_MASS;
			node2->velocity.y += Fx0(1) / POINT_MASS;
			node2->velocity.z += Fx0(2) / POINT_MASS;
			node2->velocityUV.x += Fu0 / POINT_MASS;

			node3->velocity.x += Fx2(0) / POINT_MASS;
			node3->velocity.y += Fx2(1) / POINT_MASS;
			node3->velocity.z += Fx2(2) / POINT_MASS;
			node3->velocityUV.x += Fu2 / POINT_MASS;

			// 改变位置
			node1->position += node1->velocity * deltaT;

			node1->u += node1->velocityUV.x * deltaT;

			node2->position += node2->velocity * deltaT;

			node2->u += node2->velocityUV.x * deltaT;	

			node3->position += node2->velocity * deltaT;

			node3->u += node2->velocityUV.x * deltaT;

			// 保存当前帧力
			node1->Fbu = Fu1;
			node2->Fbu = Fu0;
			node3->Fbu = Fu2;
		}
	}


	void solveV(double deltaT)
	{
		// u1 > u0 > u2
		Vector3d x0(node2->position.x, node2->position.y, node2->position.z), 
			x1(node1->position.x, node1->position.y, node1->position.z),
			x2(node3->position.x, node3->position.y, node3->position.z);

		double u0 = node2->v, u1 = node1->v, u2 = node3->v;

		double l1 = (x1 - x0).norm(), l2 = (x2 - x0).norm();
		//double kb = 0.01 * 3.1415 * 0.25 * 0.25;

		Vector3d d1 = (x1 - x0) / l1, d2 = (x2 - x0) / l2;
		double theta = acos((-d1.transpose() * d2)[0]);

		if(u1 > u0 && u0 > u2)
		{
			// 水平状没有力的作用
			if(theta <= 0.01)
				return;

			Matrix3d I = Matrix3d::Identity();
			Matrix3d P1 = I - d1 * d1.transpose(), P2 = I - d2 * d2.transpose();

			Vector3d Fx1 = ((-2 * kb * theta) / (l1 * (u1 - u2) * sin(theta))) * P1 * d2, 
				Fx2 = ((-2 * kb * theta) / (l2 * (u1 - u2) * sin(theta))) * P2 * d1, 
				Fx0 = -(Fx1 + Fx2);

			double Fu1 = (kb * theta * theta) / (u1*u1 - 2*u1*u2 + u2*u2),
				Fu2 = -Fu1,
				Fu0 = 0;

#ifdef LOGOUT
			cout << "theta:\n" << theta << std::endl;

			//cout << "Fx0:\n" << Fx0 << endl;
			//cout << "Fx1:\n" << Fx1 << endl;
			//cout << "Fx2:\n" << Fx2 << endl;

			//cout << "Fu0:\n" << Fu0 << endl;
			//cout << "Fu1:\n" << Fu1 << endl;
			//cout << "Fu2:\n" << Fu2 << endl;
#endif // LOGOUT

			// 更改速度
			node1->velocity.x += Fx1(0) / POINT_MASS;
			node1->velocity.y += Fx1(1) / POINT_MASS;
			node1->velocity.z += Fx1(2) / POINT_MASS;
			node1->velocityUV.y += Fu1 / POINT_MASS;

			node2->velocity.x += Fx0(0) / POINT_MASS;
			node2->velocity.y += Fx0(1) / POINT_MASS;
			node2->velocity.z += Fx0(2) / POINT_MASS;
			node2->velocityUV.y += Fu0 / POINT_MASS;

			node3->velocity.x += Fx2(0) / POINT_MASS;
			node3->velocity.y += Fx2(1) / POINT_MASS;
			node3->velocity.z += Fx2(2) / POINT_MASS;
			node3->velocityUV.y += Fu2 / POINT_MASS;

			// 改变位置
			node1->position += node1->velocity * deltaT;

			node1->v += node1->velocityUV.y * deltaT;

			node2->position += node2->velocity * deltaT;

			node2->v += node2->velocityUV.y * deltaT;	

			node3->position += node2->velocity * deltaT;

			node3->v += node2->velocityUV.y * deltaT;

			// 保存当前帧力
			node1->Fbu = Fu1;
			node2->Fbu = Fu0;
			node3->Fbu = Fu2;

		}
	}

	// 调试方法
	void draw()
	{
		if(node1 && node2 && node3)
		{
			ofSetColor(0);
			ofLine(node2->position, node1->position);
			ofLine(node2->position, node3->position);
		}
		else
			printf("bendLink: 三节点中有结点为空\n");
	}

	// 这里只能假设节点质量为M = pi * R * R * 1 * ro;
	crossingNode* node1;
	crossingNode* node2;
	crossingNode* node3;

	float kb;

	// 0:warp 1:weft
	int bType;
};

#endif