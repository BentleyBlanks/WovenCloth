#ifndef CROSSINGNODE_H
#define CROSSINGNODE_H

#include <ofMain.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
using namespace Eigen;

enum yarnType
{
	// 只是为了表示上下状态
	YARN_WARP = 0,
	YARN_WEFT = 1
};

class crossingNode
{
public:
	crossingNode(ofVec3f position, float u, float v):position(position), u(u), v(v)
	{
		warpIsUp = false;
		//type = YARN_WARP;
		velocity.zero();
		velocityUV.zero();

		//Fsu = Vector3d::Zero();
		//Fsv = Vector3d::Zero();

		//Fbu = Vector3d::Zero();
		//Fbv = Vector3d::Zero();
		Fsu =  Fsv =  Fbu =  Fbv = 0.0;

		// 不动产……
		anchorU = u;
		anchorV = v;
	}

	//void setType(yarnType type)
	//{
	//	if(type >= YARN_WARP && type <= YARN_WEFT)
	//		this->type = type;
	//	else
	//		printf("crossingNode: 指定的纺线类型有误\n");
	//}

	void reset()
	{
		Fsu =  Fsv =  Fbu =  Fbv = 0.0;
	}


	// 五个自由度
	// 拉格朗日坐标
	ofVec3f position;

	// 欧拉坐标表示sliding
	float u, v;

	float anchorU, anchorV;

	ofVec3f velocity;
	ofVec2f velocityUV;

	double Fsu, Fsv, Fbu, Fbv;

	bool warpIsUp;

	// --!本例特殊化 交叉点一定交叉 不会分开
	// --!尽管和原论文中不同 但实现的方式更简单一些
	// 经纬纱的类型
	//yarnType type;
};
#endif