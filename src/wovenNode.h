#ifndef WOVENNODE_H 
#define WOVENNODE_H

#include <crossingNode.h>

enum yarnState
{
	// 只是为了表示上下状态
	WARP_YARN_UP = 0,
	WARP_YARN_DOWN = 1
};

class wovenNode
{
public:
	wovenNode(crossingNode* warp, crossingNode* weft, yarnState state):weft(weft), warp(warp), state(state)
	{

	}

	crossingNode *weft, *warp;

	yarnState state;
};

#endif