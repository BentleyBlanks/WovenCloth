#ifndef CROSSINGNODE_H
#define CROSSINGNODE_H

#include <ofMain.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
using namespace Eigen;

enum yarnType
{
	// ֻ��Ϊ�˱�ʾ����״̬
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

		// ����������
		anchorU = u;
		anchorV = v;
	}

	//void setType(yarnType type)
	//{
	//	if(type >= YARN_WARP && type <= YARN_WEFT)
	//		this->type = type;
	//	else
	//		printf("crossingNode: ָ���ķ�����������\n");
	//}

	void reset()
	{
		Fsu =  Fsv =  Fbu =  Fbv = 0.0;
	}


	// ������ɶ�
	// ������������
	ofVec3f position;

	// ŷ�������ʾsliding
	float u, v;

	float anchorU, anchorV;

	ofVec3f velocity;
	ofVec2f velocityUV;

	double Fsu, Fsv, Fbu, Fbv;

	bool warpIsUp;

	// --!�������⻯ �����һ������ ����ֿ�
	// --!���ܺ�ԭ�����в�ͬ ��ʵ�ֵķ�ʽ����һЩ
	// ��γɴ������
	//yarnType type;
};
#endif