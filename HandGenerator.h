#pragma once

#include "Hand.h"

#define WR(x) std::cout << x << std::flush
#define WRL(x) std::cout << x << std::endl



class HandGenerator{
	class Trackbar
	{
		private:
			char _wname[1024];
			int _dim;

			int tb_id;
			int* tb_val;

			

		public:
			int id;
			float* val;

			//
			int testid;
			int testval[3];
			float testval2[3];

			Trackbar(){
	
			}
	
			Trackbar(const char* wname,int dim){
				_dim = dim;
				tb_val = new int[_dim];
				val = new float[_dim];

				strcpy(_wname, wname);
				cvNamedWindow(_wname, 1);	

				//initial value of trackbar
				tb_id = 0;
				for (int i = 0; i < _dim; i++)
					tb_val[i] = 0;
				tb_val[0] = 50;
				tb_val[1] = 50;
				tb_val[2] = 0;
				
				//
				testid = 0;
			}

			void run(){
				cvCreateTrackbar("jid", "handparam", &tb_id, _dim-1, NULL);
				cvCreateTrackbar("param", "handparam", &tb_val[tb_id], 100, NULL);

				cvCreateTrackbar("testid", "handparam", &testid, 18, NULL);
				cvCreateTrackbar("testval0", "handparam", &testval[0], 100, NULL);
				cvCreateTrackbar("testval1", "handparam", &testval[1], 100, NULL);
				cvCreateTrackbar("testval2", "handparam", &testval[2], 100, NULL);

				for (int i = 0; i < 3; i++)
					testval2[i] = -200 + (testval[i] / 100.0) * 400.0;


				for (int i = 0; i < _dim;i++)
					val[i] = (tb_val[i] / 100.0)*(boundary_max[1][i] - boundary_max[0][i]) + boundary_max[0][i];
			}
	};


public:
	Trackbar _trackbar;

	HandGenerator(){

	}

	HandGenerator(HandParameters hg){

		handParamNum = hg.handParamNum;
		

		//init trackbar
		_trackbar = Trackbar("handparam", handParamNum);
		hand_param = new float[handParamNum];
		
		//init hand
		hg.CopyTo(&hp);
		bool hand_success = hand.Init(hp);
		if (!hand_success)
		{
			WRL("ERROR: Hand loading was not successfull, please check the parameters and/or your system");
			return;
		}
		WRL("hand OK");


		//hand.initialRun();
		
		//trackbar initialization
	

		/*
		WRL("Init start");
		
		hg.CopyTo(&hp);

		bool hand_success = hand.Init(hp);
		if (!hand_success)
		{
			WRL("ERROR: Hand loading was not successfull, please check the parameters and/or your system");
			return;
		}
		WRL("hand OK");
		
		std::vector<int> temp;
		ComputePosition(&positions, temp, 0);
		WRL("positions OK");

		ComputeRotation(&rotations);
		WRL("rotations OK");
		*/
	}

	void run_gui(){
		//hand parameter.
		//tile width, tile height
		//particle index

	//--trackbar--//
		_trackbar.run();
		for (int i = 0; i < handParamNum; i++)
			hand_param[i] = _trackbar.val[i];

		
	//--temporary input of hand parameters--//
		
		float fx = 0;
		float fy = 0;
		float fz = 0;

	//--render--//
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		for (int i = 0; i < 2;i++)
		for (int j = 0; j < 2; j++){
			//wrist
			float wx = hand_param[0];
			float wy = hand_param[1];
			float wz = hand_param[2];
			float rx = hand_param[3];
			float ry = hand_param[4];
			float rz = hand_param[5];

			//finger
			float fx = _trackbar.testval2[0];
			float fy = _trackbar.testval2[1];
			float fz = _trackbar.testval2[2];
			int joint_index = _trackbar.testid;//2;

			int positions_index = 0;

			printf("jid:%d %f %f %f\n", joint_index, fx, fy, fz);
			printf("w: %f %f %f\n", wx,wy,wz);
			//-put handparam to .. using 'setJoint'.
			//hand.GetJoint(joint_index, positions_index, fx,fy,fz);
			hand.SetJoint(joint_index, positions_index,fx,fy,fz);
			
			//hand.SetJoint(0, -1,90,90,90);

			///hand.Render2tile(i, j, 640, 480, rx, ry, rz, false, wx,wy,wz);
			

			//
			//hand.SetJoint(joint_index, positions_index);
		}
	}

	void setHandPose(float* in)
	{
		for (int i = 0; i < hp.handParamNum; i++)
			hand_param[i] = in[i];
	}
	void runTile(int px,int py,float* in,std::string vistype){
		
		if (px==0 && py==0) //clear only when drawing first particle.
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		
		//printf("tile %.2f %.2f %.2f\n", in[0], in[1], in[2]);
		setPose(in);
		hand.setViewport(px, py, 128, 128);

		hand.Render(in[3], in[4],in[5], false,in[0],in[1],in[2],vistype);
	}

	void run(float* in,std::string vistype){
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		setPose(in);
		hand.setViewport(640, 480);
		hand.Render(in[3], in[4], in[5], false, in[0], in[1], in[2],vistype);
	}


private:
	Hand hand;
	HandParameters hp;

	int handParamNum;

	float* hand_param;

	void setPose(float* in){
		hand.SetJoint(0, 0, in[6], 0, in[7]);
		hand.SetJoint(1, 0, in[8], 0, 0);
		hand.SetJoint(2, 0, in[9], 0, 0);

		hand.SetJoint(4, 0, in[10], 0, in[11]);
		hand.SetJoint(5, 0, in[12], 0, 0);
		hand.SetJoint(6, 0, in[13], 0, 0);

		hand.SetJoint(8, 0, in[14], 0, in[15]);
		hand.SetJoint(9, 0, in[16], 0, 0);
		hand.SetJoint(10, 0, in[17], 0, 0);

		hand.SetJoint(12, 0, in[18], 0, in[19]);
		hand.SetJoint(13, 0, in[20], 0, 0);
		hand.SetJoint(14, 0, in[21], 0, 0);

		hand.SetJoint(16, 0, in[21], 0, in[22]);
		hand.SetJoint(17, 0, in[23], 0, 0);
		hand.SetJoint(18, 0, in[24], 0, 0);
	}
};