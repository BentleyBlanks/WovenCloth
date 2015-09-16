#ifndef PARALLELCONTACTLINK_H
#define PARALLELCONTACTLINK_H

#include <crossingNode.h>

#define PARALLE_POINT_MASS 1

class parallelContactLink
{
public:
	parallelContactLink(crossingNode* node0, crossingNode* node1, float kc):node0(node1), node1(node1), kc(kc)
	{

	}

	void solveU(float deltaT)
	{
		double L = 5;
		double d = 4 * 2;
		double Fu0 = kc * L * (node1->u - node0->u - d),
			Fu1 = -Fu0;

		if(node1->u - node0->u > d)
			return;

		node0->velocityUV.x += Fu0 / PARALLE_POINT_MASS;
		node1->velocityUV.x += Fu1 / PARALLE_POINT_MASS;

		node0->u += node0->velocityUV.x * deltaT;
		node1->u += node1->velocityUV.x * deltaT;
	}

	void solveV(float deltaT)
	{
		double L = 5;
		double d = 4 * 0.24;
		double Fu0 = kc * L * (node1->v - node0->v - d),
			Fu1 = -Fu0;

		if(node1->v - node0->v > d)
			return;

		node0->velocityUV.y += Fu0 / PARALLE_POINT_MASS;
		node1->velocityUV.y += Fu1 / PARALLE_POINT_MASS;

		node0->v += node0->velocityUV.y * deltaT;
		node1->v += node1->velocityUV.y * deltaT;
	}

	crossingNode *node0, *node1;

	// 0:warp 1:weft
	bool bType;

	float kc;
};

#endif