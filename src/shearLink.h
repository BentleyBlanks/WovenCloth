#ifndef SHEARLINK_H
#define SHEARLINK_H

#include <crossingNode.h>

//#define LOGOUT 1
#define SHEAR_POINT_MASS 5000

class shearLink
{
public:
	// 大中小
	shearLink(crossingNode* node0, crossingNode* node1, crossingNode* node3, float kx):node0(node0), node1(node1), node3(node3), kx(kx)
	{

	}

	void solve(double deltaT)
	{
		Vector3d x0(node0->position.x, node0->position.y, node0->position.z), 
			x1(node1->position.x, node1->position.y, node1->position.z),
			x3(node3->position.x, node3->position.y, node3->position.z);

		//double u0 = node0->u, u1 = node1->u, v3 = node3->v;

		double l1 = (x1 - x0).norm(), l3 = (x3 - x0).norm();
		Vector3d d1 = (x1 - x0) / l1, d3 = (x3 - x0) / l3;

		double fi = acos(d1.transpose() * d3);
		double L = 5;

		Matrix3d I = Matrix3d::Identity();
		Matrix3d P1 = I - d1 * d1.transpose(), P3 = I - d3 * d3.transpose();

		// pi / 2 = 1.5707963
		Vector3d Fx1 = ((kx * L * (fi - 1.5707963)) / (l1 * sin(fi))) * P1 * d3,
			Fx3 = ((kx * L * (fi - 1.5707963)) / (l3 * sin(fi))) * P3 * d1,
			Fx0 = -(Fx1 + Fx3);

		// --!"Forces on warp and weft coordinates are all zero"

#ifdef LOGOUT
		cout << "fi:\n" << fi * 180 / 3.1415 << std::endl;

		//cout << "Fx0:\n" << Fx0 << endl;
		//cout << "Fx1:\n" << Fx1 << endl;
		//cout << "Fx2:\n" << Fx3 << endl;
#endif // LOGOUT

		if(abs(fi - 1.5707963) < 0.01)
			return;

		// 更改速度
		node0->velocity.x += Fx0(0) / SHEAR_POINT_MASS;
		node0->velocity.y += Fx0(1) / SHEAR_POINT_MASS;
		node0->velocity.z += Fx0(2) / SHEAR_POINT_MASS;

		node1->velocity.x += Fx1(0) / SHEAR_POINT_MASS;
		node1->velocity.y += Fx1(1) / SHEAR_POINT_MASS;
		node1->velocity.z += Fx1(2) / SHEAR_POINT_MASS;

		node3->velocity.x += Fx3(0) / SHEAR_POINT_MASS;
		node3->velocity.y += Fx3(1) / SHEAR_POINT_MASS;
		node3->velocity.z += Fx3(2) / SHEAR_POINT_MASS;

		// 改变位置
		node0->position += node0->velocity;

		node1->position += node1->velocity;

		node3->position += node3->velocity;
	}

	// 调试方法
	void draw()
	{
		if(node1 && node0 && node3)
		{
			ofSetColor(0, 0, 0);
			ofLine(node0->position, node1->position);
			//ofSetColor(255, 0, 0);
			ofLine(node0->position, node3->position);
		}
		else
			printf("bendLink: 三节点中有结点为空\n");
	}

	double kx;
	crossingNode *node0, *node1, *node3;
};

#endif