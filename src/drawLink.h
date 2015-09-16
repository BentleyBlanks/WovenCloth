#ifndef DRAWLINK_H
#define DRAWLINK_H

#include <crossingNode.h>
#include <global.h>

class face
{
public:
	face(ofVec3f v, ofVec3f vn):v(v), vn(vn)
	{

	}

	ofVec3f v;
	ofVec3f vn;
};

class drawLink
{
public:
	drawLink(crossingNode *node1, crossingNode *node2, float R):node1(node1), node2(node2), R(R)
	{

	}

	void draw(bool saved, string name)
	{
		static ofVec3f pos1, pos2;
		ofSetColor(79, 20, 100);
		if(type == 0)
		{
			if(state == 0)
			{
				pos1.set(node1->position.x, node1->position.y, node1->position.z - R);
				pos2.set(node2->position.x, node2->position.y, node2->position.z + R);
				ofLine(pos1, pos2);
			}
			else if(state == 1)
			{
				pos1.set(node1->position.x, node1->position.y, node1->position.z + R);
				pos2.set(node2->position.x, node2->position.y, node2->position.z - R);
				ofLine(pos1, pos2);
			}
		}
		else if(type == 1)
		{
			if(state == 0)
			{
				pos1.set(node1->position.x, node1->position.y, node1->position.z + R);
				pos2.set(node2->position.x, node2->position.y, node2->position.z - R);
				ofLine(pos1, pos2);
			}
			else if(state == 1)
			{
				pos1.set(node1->position.x, node1->position.y, node1->position.z - R);
				pos2.set(node2->position.x, node2->position.y, node2->position.z + R);
				ofLine(pos1, pos2);
			}
		}

		double R = 5;

		// 保存obj模型
		// 线缆实质为斜长方体 而非圆柱
		static vector<ofVec3f> v;
		static vector<ofVec3f> n;
		static vector<face> f;
		static int frame = 0;

		int vj = v.size(), vnj = n.size();

		ofVec3f
			p1(pos1.x-R, pos1.y+R, pos1.z), p2(pos1.x+R, pos1.y+R, pos1.z), p3(pos1.x+R, pos1.y-R, pos1.z), p4(pos1.x-R, pos1.y-R, pos1.z),
			p5(pos2.x-R, pos2.y+R, pos2.z), p6(pos2.x+R, pos2.y+R, pos2.z), p7(pos2.x+R, pos2.y-R, pos2.z), p8(pos2.x-R, pos2.y-R, pos2.z);

		ofVec3f
			// 1
			n1 = p1 - p2,  n2 = p1 - p4,  n3 = p1 - p4, n4 = p1 - p5,  n5 = p1 - p5,

			// 2
			n6 = p2 - p6,  n7 = p2 - p1,  n8 = p2 - p3,

			//3
			n9 = p3 - p4, n10 = p3 - p4, n11 = p3 - p7, n12 = p3 - p7, n13 = p3 - p2,

			//4
			n14 = p4 - p8, n15 = p4 - p1, n16 = p4 - p1, n17 = p4 - p3, n18 = p4 - p3,

			//5
			n19 = p5 - p8, n20 = p5 - p6, n21 = p5 - p6, n22 = p5 - p1, n23 = p5 - p1,

			// 6
			n24 = p6 - p2, n25 = p6 - p7 ,n26 = p6 - p7, n27 = p6 - p5, n28 = p6 - p5,

			// 7
			n29 = p7 - p3, n30 = p7 - p3, n31 = p7 - p8, n32 = p7 - p6, n33 = p7 - p6,

			// 8
			n34 = p8 - p5, n35 = p8 - p7, n36 = p8 - p4;

		// vertex
		v.push_back(p1);
		v.push_back(p2);
		v.push_back(p3);
		v.push_back(p4);


		v.push_back(p5);
		v.push_back(p6);
		v.push_back(p7);
		v.push_back(p8);

		// normal
		// 5 3 5 5 5 5 5 3 = 36个法线
		n.push_back(n1);
		n.push_back(n2);
		n.push_back(n3);
		n.push_back(n4);
		n.push_back(n5);

		n.push_back(n6);
		n.push_back(n7);
		n.push_back(n8);

		n.push_back(n9);
		n.push_back(n10);
		n.push_back(n11);
		n.push_back(n12);
		n.push_back(n13);

		n.push_back(n14);
		n.push_back(n15);
		n.push_back(n16);
		n.push_back(n17);
		n.push_back(n18);

		n.push_back(n19);
		n.push_back(n20);
		n.push_back(n21);
		n.push_back(n22);
		n.push_back(n23);

		n.push_back(n24);
		n.push_back(n25);
		n.push_back(n26);
		n.push_back(n27);
		n.push_back(n28);

		n.push_back(n29);
		n.push_back(n30);
		n.push_back(n31);
		n.push_back(n32);
		n.push_back(n33);

		n.push_back(n34);
		n.push_back(n35);
		n.push_back(n36);

		// face
		f.push_back(face(ofVec3f(vj+1, vj+4, vj+3), ofVec3f(vnj+4, vnj+14, vnj+12)));
		f.push_back(face(ofVec3f(vj+1, vj+3, vj+2), ofVec3f(vnj+5, vnj+11, vnj+6)));

		f.push_back(face(ofVec3f(vj+2, vj+3, vj+6), ofVec3f(vnj+7, vnj+10, vnj+27)));
		f.push_back(face(ofVec3f(vj+3, vj+7, vj+6), ofVec3f(vnj+9, vnj+31, vnj+28)));

		f.push_back(face(ofVec3f(vj+5, vj+6, vj+7), ofVec3f(vnj+23, vnj+24, vnj+29)));
		f.push_back(face(ofVec3f(vj+8, vj+5, vj+7), ofVec3f(vnj+36, vnj+22, vnj+30)));

		f.push_back(face(ofVec3f(vj+1, vj+5, vj+4), ofVec3f(vnj+1, vnj+21, vnj+18)));
		f.push_back(face(ofVec3f(vj+4, vj+5, vj+8), ofVec3f(vnj+17, vnj+20, vnj+35)));

		f.push_back(face(ofVec3f(vj+1, vj+2, vj+6), ofVec3f(vnj+3, vnj+8, vnj+26)));
		f.push_back(face(ofVec3f(vj+1, vj+6, vj+5), ofVec3f(vnj+2, vnj+25, vnj+19)));

		f.push_back(face(ofVec3f(vj+4, vj+7, vj+3), ofVec3f(vnj+16, vnj+33, vnj+13)));
		f.push_back(face(ofVec3f(vj+4, vj+8, vj+7), ofVec3f(vnj+15, vnj+34, vnj+32)));

		if(saved)
		{
			string temp = "D:/Program/C++/Mitsuba/woven/mesh/" + name + ".obj";

			ofstream fout(temp.c_str());

			if ( fout )
			{
				fout << "#Created by Mr.Guan" << endl;

				fout << "#vertex" << endl;
				// vertex
				for(int i=0; i<v.size(); i++)
					fout << "v " << v[i].x << " " << v[i].y << " " << v[i].z << endl;
				fout << "# " << v.size() << " vertices" << endl << endl;


				fout << "#normal" << endl;
				// vertex normal
				for(int i=0; i<n.size(); i++)
					fout << "vn " << n[i].x << " " << n[i].y << " " << n[i].z << endl;
				fout << "# " << n.size() << " normals" << endl << endl;


				fout << "#face" << endl;
				// face
				for(int i=0; i<f.size(); i++)
				{
					fout << "f " << f[i].v.x << "//" << f[i].vn.x << " "
						<< f[i].v.y << "//" << f[i].vn.y << " "
						<< f[i].v.z << "//" << f[i].vn.z << endl;
				}

				fout << "# " << f.size() << " faces" << endl;

				fout.close();

				cout << "保存文件" << name << ".obj 成功!" << endl;

				v.clear();
				f.clear();
				n.clear();
			}

		}
	}

	crossingNode *node1, *node2;

	// 0:warp is up 1:weft is up
	int state;
	// 0:warp 1:weft
	int type;

	float R;
};

#endif // !drawLink
