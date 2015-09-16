#ifndef STRETCHNODE_H
#define STRETCHNODE_H

#include <crossingNode.h>
#include <global.h>
//#define LOGOUT 1

class stretchLink
{
public:
	stretchLink(crossingNode* node1, crossingNode* node2, float ks):node1(node1), node2(node2), ks(ks)
	{
		bType = false;
	}

	void solveU(double deltaT)
	{
		// matrix test
		Vector3d x0(node1->position.x, node1->position.y, node1->position.z), 
			x1(node2->position.x, node2->position.y, node2->position.z);
		double u0 = node1->u, u1 = node2->u;


		double ro = 0;
		// 静态长度
		double delta_u = u1 - u0;
		// 弹性系数
		//double ks = 0.25 * 0.25 * 3.1415 * 10000000;
		double l = (x1 - x0).norm();

		if(u1 < u0 || delta_u >= 2 * L)
			return;

		// 默认为列向量
		Vector3d w = (x1 - x0) / delta_u, d = (x1 - x0) / l;
		Matrix3d I = Matrix3d::Identity();
		Matrix3d P = I - d * d.transpose();

		MatrixXd A(8, 8), b(8, 1);

		MatrixXd M(8, 8);
		M << 2,       0,       0,        -2*w[0],                  1,       0,       0,			  -w[0],
			 0,       2,       0,        -2*w[1],                  0,       1,       0,			  -w[1],
			 0,       0,       2,        -2*w[2],                  0,       0,       1,			  -w[2],

			-2*w[0], -2*w[1], -2*w[2],   (2*w.transpose()*w)[0],  -w[0],   -w[1],   -w[2],        (w.transpose()*w)[0],

			 1,       0,       0,        -w[0],                    2,       0,       0,			  -2*w[0],
			 0,       1,       0,        -w[1],                    0,       2,       0,			  -2*w[1],
			 0,       0,       1,        -w[2],                    0,       0,       2,           -2*w[2],

			-w[0],   -w[1],   -w[2],     (w.transpose()*w)[0],    -2*w[0], -2*w[1], -2*w[2],      (2*w.transpose()*w)[0];

#ifdef LOGOUT
		//cout << "M:\n" << M << endl;
#endif // LOGOUT

		M = 0.16666666666 * ro * delta_u * M;

#ifdef LOGOUT
		//cout << "M:\n" << M << endl;
#endif // LOGOUT

		MatrixXd F_X(8, 8);
		Matrix3d Fx1_x1 =  (ks / l) * P - (ks / delta_u) * I,
			Fx0_x0 =  Fx1_x1, 
			Fx1_x0 = -Fx1_x1,
			Fx0_x1 = -Fx1_x1;

		double Fu1_u1 =  -ks * w.squaredNorm() / delta_u,
			Fu0_u0 =  Fu1_u1,
			Fu1_u0 = -Fu1_u1,
			Fu0_u1 = -Fu1_u1;

		Vector3d Fx1_u1 =  (ks * w.norm() / delta_u) * d,
			Fx0_u0 =  Fx1_u1,
			Fx1_u0 = -Fx1_u1,
			Fx0_u1 = -Fx1_u1;

		Vector3d Fu1_x1 =  (ks / delta_u) * w.transpose(),
			Fu0_x0 =  Fu1_x1,
			Fu1_x0 = -Fu1_x1,
			Fu0_x1 = -Fu1_x1;

		F_X << 
			Fx0_x0(0), Fx0_x0(1), Fx0_x0(2),    Fx0_u0[0],    Fx0_x1(0), Fx0_x1(1), Fx0_x1(2),    Fx0_u1[0],

			Fx0_x0(3), Fx0_x0(4), Fx0_x0(5),    Fx0_u0[1],    Fx0_x1(3), Fx0_x1(4), Fx0_x1(5),    Fx0_u1[1],

			Fx0_x0(6), Fx0_x0(7), Fx0_x0(8),    Fx0_u0[2],    Fx0_x1(6), Fx0_x1(7), Fx0_x1(8),    Fx0_u1[2],


			Fu0_x0[0], Fu0_x0[1], Fu0_x0[2],    Fu0_u0,		 Fu0_x1[0], Fu0_x1[1], Fu0_x1[2],    Fu0_u1,


			Fx1_x0(0), Fx1_x0(1), Fx1_x0(2),    Fx1_u0[0],	 Fx1_x1(0), Fx1_x1(1), Fx1_x1(2),    Fx1_u1[0],

			Fx1_x0(3), Fx1_x0(4), Fx1_x0(5),    Fx1_u0[1],	 Fx1_x1(3), Fx1_x1(4), Fx1_x1(5),    Fx1_u1[1],

			Fx1_x0(6), Fx1_x0(7), Fx1_x0(8),    Fx1_u0[2],	 Fx1_x1(6), Fx1_x1(7), Fx1_x1(8),    Fx1_u1[2],


			Fu1_x0[0], Fu1_x0[1], Fu1_x0[2],	Fu1_u0,		 Fu1_x1[0], Fu1_x1[1], Fu1_x1[2],    Fu1_u1;


		MatrixXd F(8, 1);

		Vector3d Fx1 = -ks * (w.norm() - 1) * d,
			Fx0 = -Fx1;
		double Fu1 = 0.5 * ks * (w.norm() * w.norm() - 1),
			Fu0 = -Fu1;
		F << Fx0[0], Fx0[1], Fx0[2], Fu0, Fx1[0], Fx1[1], Fx1[2], Fu1;

#ifdef LOGOUT
		//cout << "M:\n" << M << endl;
		//cout << "F_X:\n" << F_X << endl;
		//cout << "F:\n" << F << endl;
#endif // LOGOUT

		MatrixXd V(8, 1);
		V << node1->velocity.x, node1->velocity.y, node1->velocity.z, node1->velocityUV.x, 
			 node2->velocity.x, node2->velocity.y, node2->velocity.z, node2->velocityUV.x;

		//MatrixXd I8(8, 8);
		//I8<<
		//	1, 0, 0, 0, 0, 0, 0, 0,
		//	0, 1, 0, 0, 0, 0, 0, 0,
		//	0, 0, 1, 0, 0, 0, 0, 0,
		//	0, 0, 0, 1, 0, 0, 0, 0,
		//	0, 0, 0, 0, 1, 0, 0, 0,
		//	0, 0, 0, 0, 0, 1, 0, 0,
		//	0, 0, 0, 0, 0, 0, 1, 0,
		//	0, 0, 0, 0, 0, 0, 0, 1;

		A = M - deltaT * deltaT * F_X - deltaT * F_X;
		b = deltaT * F + deltaT * deltaT * F_X * V; 
		//A = F_X;
		//b = F;

#ifdef LOGOUT
		//cout << "A:\n" << M << endl;
		//cout << "b:\n" << b << endl;
#endif // LOGOUT

		// 求解线性方程组
		MatrixXd x(8, 1);
		//MatrixXd xk(8, 1), rk(8, 1), pk(8, 1);
		//xk << node1->velocity.x, node1->velocity.y, node1->velocity.z, node1->velocityUV.x, 
		//	node2->velocity.x, node2->velocity.y, node2->velocity.z, node2->velocityUV.x;
		//rk = b - A * xk;
		//pk = rk;
		//
		//MatrixXd rk1(8, 1);
		//double alphaK=0.0, beitaK=0.0;
		//for(int i=0; i<3; i++)
		//{
		//	MatrixXd temp = A * pk;
		//	double a = (rk.transpose() * A * rk)(0);
		//	double b = (temp.transpose() * temp)(0);
		//	if(b != 0)
		//		alphaK = a/b;
		//	else
		//	{
		//		xk << 0, 0, 0, 0, 0, 0, 0, 0;
		//		break;
		//	}
		//	xk = xk + alphaK * pk;
		//	rk1 = rk - alphaK * A * pk;
		//
		//	double c = (rk1.transpose() * A * rk1)(0);
		//	double d = (rk.transpose() * A * rk)(0);
		//	if(d != 0)
		//		beitaK = c / d;
		//	else
		//	{
		//		xk << 0, 0, 0, 0, 0, 0, 0, 0;
		//		break;
		//	}
		//	pk = rk1 + beitaK * pk;
		//}

		x = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);
		//x = A.colPivHouseholderQr().solve(b);

#ifdef LOGOUT
		std::cout << "The solution is:\n" << x << std::endl << std::endl;
#endif // _DEBUG

		// 更改速度
		if(abs(x(0)) > 0.001)
			node1->velocity.x += x(0);
		if(abs(x(1)) > 0.001)
			node1->velocity.y += x(1);
		if(abs(x(2)) > 0.001)
			node1->velocity.z += x(2);
		if(abs(x(3)) > 0.001)
			node1->velocityUV.x += x(3);

		if(abs(x(4)) > 0.001)
			node2->velocity.x += x(4);
		if(abs(x(5)) > 0.001)
			node2->velocity.y += x(5);
		if(abs(x(6)) > 0.001)
			node2->velocity.z += x(6);
		if(abs(x(7)) > 0.001)
			node2->velocityUV.x += x(7);

		//std::cout << "The node1's velocity is:\n" << node1->velocity << std::endl;

		//std::cout << "The node2's velocity is:\n" << node2->velocity << std::endl << std::endl;
		
		//if(node1->velocityUV.x>0)
		//	node1->velocityUV.x -= abs(0.3*(node1->u-20));
		//else
		//	node1->velocityUV.x += abs(0.3*(node1->u-20));

		//if(node2->velocityUV.x>0)
		//	node2->velocityUV.x -= abs(0.3*(node2->u-25));
		//else
		//	node2->velocityUV.x += abs(0.3*(node2->u-25));


		// 改变位置
		node1->position += node1->velocity * deltaT;

		node1->u += node1->velocityUV.x * deltaT;

		node2->position += node2->velocity * deltaT;

		node2->u += node2->velocityUV.x * deltaT;	

		// 保存力
		node1->Fbu = Fu0;
		node2->Fbu = Fu1;
	}

	void solveV(double deltaT)
	{
		// matrix test
		Vector3d x0(node1->position.x, node1->position.y, node1->position.z), 
			x1(node2->position.x, node2->position.y, node2->position.z);
		double u0 = node1->v, u1 = node2->v;

		double ro = 0;
		// 静态长度
		double delta_u = abs(u1 - u0);
		// 弹性系数
		//double ks = 0.25 * 0.25 * 3.1415 * 10000000;
		double l = (x1 - x0).norm();

		if(u1 < u0 || delta_u >= 2 * L)
			return;

		// 默认为列向量
		Vector3d w = (x1 - x0) / delta_u, d = (x1 - x0) / l;
		Matrix3d I = Matrix3d::Identity();
		Matrix3d P = I - d * d.transpose();

		MatrixXd A(8, 8), b(8, 1);

		MatrixXd M(8, 8);
		M << 2,       0,       0,        -2*w[0],                  1,       0,       0,			  -w[0],
			0,       2,       0,        -2*w[1],                  0,       1,       0,			  -w[1],
			0,       0,       2,        -2*w[2],                  0,       0,       1,			  -w[2],

			-2*w[0], -2*w[1], -2*w[2],   (2*w.transpose()*w)[0],  -w[0],   -w[1],   -w[2],        (w.transpose()*w)[0],

			1,       0,       0,        -w[0],                    2,       0,       0,			  -2*w[0],
			0,       1,       0,        -w[1],                    0,       2,       0,			  -2*w[1],
			0,       0,       1,        -w[2],                    0,       0,       2,           -2*w[2],

			-w[0],   -w[1],   -w[2],     (w.transpose()*w)[0],    -2*w[0], -2*w[1], -2*w[2],      (2*w.transpose()*w)[0];

#ifdef LOGOUT
		//cout << "M:\n" << M << endl;
#endif // LOGOUT

		M = 0.16666666666 * ro * delta_u * M;

#ifdef LOGOUT
		//cout << "M:\n" << M << endl;
#endif // LOGOUT

		MatrixXd F_X(8, 8);
		Matrix3d Fx1_x1 =  (ks / l) * P - (ks / delta_u) * I,
			Fx0_x0 =  Fx1_x1, 
			Fx1_x0 = -Fx1_x1,
			Fx0_x1 = -Fx1_x1;

		double Fu1_u1 =  -ks * w.squaredNorm() / delta_u,
			Fu0_u0 =  Fu1_u1,
			Fu1_u0 = -Fu1_u1,
			Fu0_u1 = -Fu1_u1;

		Vector3d Fx1_u1 =  (ks * w.norm() / delta_u) * d,
			Fx0_u0 =  Fx1_u1,
			Fx1_u0 = -Fx1_u1,
			Fx0_u1 = -Fx1_u1;

		Vector3d Fu1_x1 =  (ks / delta_u) * w.transpose(),
			Fu0_x0 =  Fu1_x1,
			Fu1_x0 = -Fu1_x1,
			Fu0_x1 = -Fu1_x1;

		F_X << 
			Fx0_x0(0), Fx0_x0(1), Fx0_x0(2),    Fx0_u0[0],    Fx0_x1(0), Fx0_x1(1), Fx0_x1(2),    Fx0_u1[0],

			Fx0_x0(3), Fx0_x0(4), Fx0_x0(5),    Fx0_u0[1],    Fx0_x1(3), Fx0_x1(4), Fx0_x1(5),    Fx0_u1[1],

			Fx0_x0(6), Fx0_x0(7), Fx0_x0(8),    Fx0_u0[2],    Fx0_x1(6), Fx0_x1(7), Fx0_x1(8),    Fx0_u1[2],


			Fu0_x0[0], Fu0_x0[1], Fu0_x0[2],    Fu0_u0,		 Fu0_x1[0], Fu0_x1[1], Fu0_x1[2],    Fu0_u1,


			Fx1_x0(0), Fx1_x0(1), Fx1_x0(2),    Fx1_u0[0],	 Fx1_x1(0), Fx1_x1(1), Fx1_x1(2),    Fx1_u1[0],

			Fx1_x0(3), Fx1_x0(4), Fx1_x0(5),    Fx1_u0[1],	 Fx1_x1(3), Fx1_x1(4), Fx1_x1(5),    Fx1_u1[1],

			Fx1_x0(6), Fx1_x0(7), Fx1_x0(8),    Fx1_u0[2],	 Fx1_x1(6), Fx1_x1(7), Fx1_x1(8),    Fx1_u1[2],


			Fu1_x0[0], Fu1_x0[1], Fu1_x0[2],	Fu1_u0,		 Fu1_x1[0], Fu1_x1[1], Fu1_x1[2],    Fu1_u1;


		MatrixXd F(8, 1);

		Vector3d Fx1 = -ks * (w.norm() - 1) * d,
			Fx0 = -Fx1;
		double Fu1 = 0.5 * ks * (w.norm() * w.norm() - 1),
			Fu0 = -Fu1;
		F << Fx0[0], Fx0[1], Fx0[2], Fu0, Fx1[0], Fx1[1], Fx1[2], Fu1;

#ifdef LOGOUT
		//cout << "M:\n" << M << endl;
		//cout << "F_X:\n" << F_X << endl;
		//cout << "F:\n" << F << endl;
#endif // LOGOUT

		MatrixXd V(8, 1);
		V << node1->velocity.x, node1->velocity.y, node1->velocity.z, node1->velocityUV.y, 
			node2->velocity.x, node2->velocity.y, node2->velocity.z, node2->velocityUV.y;

		A = M - deltaT * deltaT * F_X - deltaT * F_X;
		b = deltaT * F + deltaT * deltaT * F_X * V; 
		//A = F_X;
		//b = F;

#ifdef LOGOUT
		//cout << "A:\n" << M << endl;
		//cout << "b:\n" << b << endl;
#endif // LOGOUT

		// 求解线性方程组
		MatrixXd x(8, 1);

		x = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);
		//x = A.colPivHouseholderQr().solve(b);

#ifdef LOGOUT
		std::cout << "The solution is:\n" << x << std::endl << std::endl;
#endif // _DEBUG

		// 更改速度
		if(abs(x(0)) > 0.001)
			node1->velocity.x += x(0);
		if(abs(x(1)) > 0.001)
			node1->velocity.y += x(1);
		if(abs(x(2)) > 0.001)
			node1->velocity.z += x(2);
		if(abs(x(3)) > 0.001)
			node1->velocityUV.y += x(3);
		if(abs(x(4)) > 0.001)
			node2->velocity.x += x(4);
		if(abs(x(5)) > 0.001)
			node2->velocity.y += x(5);
		if(abs(x(6)) > 0.001)
			node2->velocity.z += x(6);
		if(abs(x(7)) > 0.001)
			node2->velocityUV.y += x(7);

		//std::cout << "The node1's velocity is:\n" << node1->velocity << std::endl;

		//std::cout << "The node2's velocity is:\n" << node2->velocity << std::endl << std::endl;

		//if(node1->velocityUV.x>0)
		//	node1->velocityUV.x -= abs(0.3*(node1->u-20));
		//else
		//	node1->velocityUV.x += abs(0.3*(node1->u-20));

		//if(node2->velocityUV.x>0)
		//	node2->velocityUV.x -= abs(0.3*(node2->u-25));
		//else
		//	node2->velocityUV.x += abs(0.3*(node2->u-25));

		//node1->velocity *= 0.95;
		//node2->velocity *= 0.95;


		// 改变位置
		node1->position += node1->velocity * deltaT;

		node1->v += node1->velocityUV.y * deltaT;

		node2->position += node2->velocity * deltaT;

		node2->v += node2->velocityUV.y * deltaT;	

		// 保存力
		node1->Fbv = Fu0;
		node2->Fbv = Fu1;
	}


	// 调试方法
	void draw()
	{
		if(node1 && node2)
		{
			ofSetColor(0);
			ofLine(node1->position, node2->position);
		}
		else
			printf("stretchLink: 两节点中有结点为空\n");
	}

	crossingNode *node1, *node2;

	// 0:warp 1:weft
	bool bType;

	float ks;
};
#endif
