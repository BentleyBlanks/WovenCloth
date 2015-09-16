#pragma once

#include "ofMain.h"

class crossingNode;
class wovenNode;
class stretchLink;
class bendLink;
class shearLink;
class drawLink;
class parallelContactLink;

class ofApp : public ofBaseApp{

public:
	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y );
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	ofEasyCam cam;

	vector<crossingNode*> wovenCloth;
	//vector<wovenNode*> wovenCloth;
	vector<stretchLink*> stretchLinkList;
	vector<bendLink*> bendLinkList;
	vector<shearLink*> shearLinkList;
	vector<drawLink*> drawLinkList;
	vector<parallelContactLink*> parallelContactLinkList;



	stretchLink *link;
	bendLink *bLink;
	shearLink *sLink;

	crossingNode* node0;
	crossingNode* node1;
	crossingNode* node2;
	crossingNode* node3;

	ofTrueTypeFont font;

	vector<ofFbo*> fboList;
};
