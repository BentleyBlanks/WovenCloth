#include "ofApp.h"
#include <crossingNode.h>
#include <wovenNode.h>
#include <stretchLink.h>
#include <bendLink.h>
#include <shearLink.h>
#include <drawLink.h>
#include <parallelContactLink.h>
#include <global.h>

#define TEST 0
#ifndef TEST
#define ONCE 1
#endif // !TEST

//#define STRETCH 2
//#define BEND 3
//#define SHEAR 4
//#define FRICTION 5
#define OFFLINE_RENDERING 6


#define FRAME_NUM 200
//--------------------------------------------------------------
void ofApp::setup(){
	ofSetFrameRate(60);
	cam.setFarClip(100000000);
	cam.setOrientation(ofVec3f(45, 0, 0));
	//cam.setTarget(ofVec3f(100, 100, 200));

	font.loadFont("BuxtonSketch.ttf", 20);


#ifdef ONCE
#ifdef STRETCH
	// stretch
	node1 = new crossingNode(ofVec3f(20, 25, 0), 20, 25);
	node2 = new crossingNode(ofVec3f(25, 25, 0), 25, 25);
	
	link = new stretchLink(node1, node2, 0.25 * 0.25 * 3.1415 * 10000000);
	
	node2->velocity.y = 0.1;
	node2->velocity.x = 0.1;
#endif // STRETCH

#ifdef BEND
	//	bend
	node1 = new crossingNode(ofVec3f(25, 25, 0), 25, 25);
	node2 = new crossingNode(ofVec3f(20, 25, 0), 20, 25);
	node3 = new crossingNode(ofVec3f(15, 30, 0), 15, 30);

	bLink = new bendLink(node1, node2, node3, 0.01 * 3.1415 * 0.25 * 0.25);
#endif // BEND

#ifdef SHEAR
	// shear
	node0 = new crossingNode(ofVec3f(20, 25, 0), 20, 25);
	node1 = new crossingNode(ofVec3f(25, 25, 0), 25, 25);
	node3 = new crossingNode(ofVec3f(25, 30, 0), 22, 30);
	
	sLink = new shearLink(node0, node1, node3, 1);
#endif // SHEAR

	// friction
#ifdef FRICTION

#endif // FRICTION

#endif // ONCE

#ifdef TEST
	// 假定交叉点不会互相分离

	for(int i=0; i<width; i++)
	{
		for(int j=0; j<height; j++)
		{
			crossingNode* node1 = new crossingNode(ofVec3f(i*L - width * L / 2, j*L - height * L / 2 + 2000, -2000), i*L, j*L);

			if((i + j) % 2 == 0)
				node1->warpIsUp = true;
			else
				node1->warpIsUp = false;

			wovenCloth.push_back(node1);
		}
	}

	// parallel Contact
	for(int i=0; i < wovenCloth.size(); i++)
	{
		// 经纱
		if(i % width + 1 < width)
		{
			// ks = Y * PI * R * R
			parallelContactLink* link = new parallelContactLink(wovenCloth[i], wovenCloth[i+1], 1);
			link->bType = 0;
			parallelContactLinkList.push_back(link);
		}

		// 纬纱
		if(i + height < wovenCloth.size())
		{
			parallelContactLink* link = new parallelContactLink(wovenCloth[i], wovenCloth[i + width], 1);
			link->bType = 1;
			parallelContactLinkList.push_back(link);
		}
	}

	// stretch
	for(int i=0; i < wovenCloth.size(); i++)
	{
		// 经纱
		if(i % width + 1 < width)
		{
			// ks = Y * PI * R * R
			stretchLink* link = new stretchLink(wovenCloth[i], wovenCloth[i+1], 0.25 * 0.25 * 3.1415 * 10000000);
			link->bType = 0;
			stretchLinkList.push_back(link);
		}

		// 纬纱
		if(i + height < wovenCloth.size())
		{
			stretchLink* link = new stretchLink(wovenCloth[i], wovenCloth[i + width], 0.25 * 0.25 * 3.1415 * 10000000);
			link->bType = 1;
			stretchLinkList.push_back(link);
		}
	}

	// bend
	for(int i=0; i < wovenCloth.size(); i++)
	{		
		// 经纱
		if(i % width + 2 < width)
		{
			bendLink* link = new bendLink(wovenCloth[i], wovenCloth[i+1], wovenCloth[i+2], 0.01 * 3.1415 * 0.25 * 0.25);
			link->bType = 0;
			bendLinkList.push_back(link);
		}

		// 纬纱
		if(i + height * 2 < wovenCloth.size())
		{
			bendLink* link = new bendLink(wovenCloth[i], wovenCloth[i + width], wovenCloth[i + width * 2], 0.01 * 3.1415 * 0.25 * 0.25);
			link->bType = 1;
			bendLinkList.push_back(link);
		}
	}

	// shear
	for(int i=0; i < wovenCloth.size(); i++)
	{		
		// 经纱
		if(i - width >= 0 && i % width + 1 < width)
		{
			shearLink* link = new shearLink(wovenCloth[i], wovenCloth[i-width], wovenCloth[i+1], 0.01);
			shearLinkList.push_back(link);
		}
	}

	// drawLink
	// 只在渲染时才会有双层的概念
	for(int i=0; i < wovenCloth.size(); i++)
	{
		// 分上下层 相距2R
		// 经纱
		if(i % width + 1 < width)
		{
			crossingNode* node1 = wovenCloth[i], *node2 = wovenCloth[i+1];
			drawLink *link;
			if(node1->warpIsUp)
			{
				link = new drawLink(node1, node2, R);
				link->type = 0;
				link->state = 0;
				drawLinkList.push_back(link);
			}
			else
			{
				link = new drawLink(node1, node2, R);
				link->type = 0;
				link->state = 1;
				drawLinkList.push_back(link);
			}
		}

		// 纬纱
		if(i + height < wovenCloth.size())
		{
			crossingNode* node1 = wovenCloth[i], *node2 = wovenCloth[i + height];
			drawLink *link;
			if(node1->warpIsUp)
			{
				link = new drawLink(node1, node2, R);
				link->type = 1;
				link->state = 0;
				drawLinkList.push_back(link);
			}
			else
			{
				link = new drawLink(node1, node2, R);
				link->type = 1;
				link->state = 1;
				drawLinkList.push_back(link);
			}
		}
	}

	for(int i=0; i<FRAME_NUM; i++)
	{
		ofFbo *fbo = new ofFbo();
		fbo->allocate(ofGetWidth(), ofGetHeight());
		fboList.push_back(fbo);
	}

	// 自变量
	//wovenCloth[13]->position.z = 5;
#endif // TEST
}

//--------------------------------------------------------------
void ofApp::update(){

}

//--------------------------------------------------------------
void ofApp::draw(){
	static float pTime = 0, nowTime = 0;

	nowTime = ofGetElapsedTimef();

	static int frame = 0;
#ifdef ONCE
#ifdef STRETCH
	cam.begin();
	link->solveU(nowTime - pTime);
	//link->solveV(nowTime - pTime);

	// 绿-z
	ofSetColor(0, 255, 0);
	ofLine(ofVec3f(20, 25, 0), ofVec3f(20, 25, 1000));
	// 红-y
	ofSetColor(255, 0, 0);
	ofLine(ofVec3f(20, 25, 0), ofVec3f(20, 1025, 0));
	// 蓝-x
	ofSetColor(0, 0, 255);
	ofLine(ofVec3f(20, 25, 0), ofVec3f(1020, 25, 0));
	//ofLine(ofVec3f(20, 25, 0), ofVec3f(25, 25, 5));

	link->draw();

	cout << "length: " << (node1->position - node2->position).length() << std::endl << std::endl;
	cout << "deltaU: " << node2->u - node1->u << std::endl << std::endl;
	cam.end();
#endif

#ifdef BEND
	cam.begin();
	bLink->solveU(nowTime - pTime);

	// 绿-z
	ofSetColor(0, 255, 0);
	ofLine(ofVec3f(20, 25, 0), ofVec3f(20, 25, 1000));
	// 红-y
	ofSetColor(255, 0, 0);
	ofLine(ofVec3f(20, 25, 0), ofVec3f(20, 1025, 0));
	// 蓝-x
	ofSetColor(0, 0, 255);
	ofLine(ofVec3f(20, 25, 0), ofVec3f(1020, 25, 0));
	//ofLine(ofVec3f(20, 25, 0), ofVec3f(25, 25, 5));

	bLink->draw();

	//cout << "length: " << (node1->position - node2->position).length() << std::endl << std::endl;
	//cout << "deltaU: " << node2->u - node1->u << std::endl << std::endl;
	cam.end();

#endif // !BEND

#ifdef SHEAR
	cam.begin();
	sLink->solve(nowTime - pTime);

	// 绿-z
	ofSetColor(0, 255, 0);
	ofLine(ofVec3f(20, 25, 0), ofVec3f(20, 25, 1000));
	// 红-y
	ofSetColor(255, 0, 0);
	ofLine(ofVec3f(20, 25, 0), ofVec3f(20, 1025, 0));
	// 蓝-x
	ofSetColor(0, 0, 255);
	ofLine(ofVec3f(20, 25, 0), ofVec3f(1020, 25, 0));
	//ofLine(ofVec3f(20, 25, 0), ofVec3f(25, 25, 5));

	sLink->draw();

	cam.end();
#endif // SHEAR
#endif // !ONCE


#ifdef TEST
#ifdef OFFLINE_RENDERING
	if(frame < FRAME_NUM)
	{
		fboList[frame]->begin();

		if(wovenCloth[13]->velocity.z < 10)
			wovenCloth[13]->velocity.z += 4.0;

		//float deltaT = nowTime - pTime;
		float deltaT = 0.05;
		//cout << deltaT << endl;
#endif // OFFLINE_RENDERING
		for(int i=0; i<stretchLinkList.size(); i++)
		{
			if(stretchLinkList[i]->bType == 1)
				stretchLinkList[i]->solveU(deltaT);
			else
				stretchLinkList[i]->solveV(deltaT);
		}

		for(int i=0; i<bendLinkList.size(); i++)
		{
			if(bendLinkList[i]->bType == 1)
				bendLinkList[i]->solveU(deltaT);
			else
				bendLinkList[i]->solveV(deltaT);
		}

		for(int i=0; i<shearLinkList.size(); i++)
		{
			shearLinkList[i]->solve(deltaT);
		}

		//for(int i=0; i<parallelContactLinkList.size(); i++)
		//{
		//	if(parallelContactLinkList[i]->bType == 1)
		//		parallelContactLinkList[i]->solveU(nowTime - pTime);
		//	else
		//		parallelContactLinkList[i]->solveV(nowTime - pTime);
		//}

		for(int i=0; i < wovenCloth.size(); i++)
		{
			wovenCloth[i]->velocity -= 0.1*wovenCloth[i]->velocity;
			wovenCloth[i]->velocityUV -= 0.1*wovenCloth[i]->velocityUV;
		}

		cam.begin();
		for(int i=0; i<drawLinkList.size(); i++)
		{
			if(i != drawLinkList.size()-1)
				drawLinkList[i]->draw(false, ofToString(frame));
			else
				// 最后一次循环保存模型文件
				drawLinkList[i]->draw(true, ofToString(frame));
		}
		cam.end();

#ifdef OFFLINE_RENDERING
		fboList[frame]->end();
		frame++;

		printf("渲染百分比: %d%%\n", frame * 100 / FRAME_NUM);
	}

	pTime = nowTime;

	if(frame >= FRAME_NUM)
	{
		static int nowFrame = 0;
		fboList[nowFrame]->draw(0, 0);
		nowFrame = (nowFrame + 1) % FRAME_NUM;
	}
#endif // OFFLINE_RENDERING
#endif // TEST

	ofSetColor(0);
	font.drawString(ofToString(ofGetFrameRate()), 50, 50);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
	if(key == 'r')
	{
		//node2->velocity.x++;
		//wovenCloth[7]->weft->position.z = 30;
		wovenCloth[3]->velocity.z++;
	}
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
