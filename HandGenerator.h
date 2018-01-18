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

		public:

			int fid_tb;
			int fid2_tb;
			int fval_tb[19][3];
			int wval_tb[6];

			int fid;
			int fid2;
			float fval[3];
			float wval[6];

			Trackbar(){
	
			}
	
			Trackbar(int dim){
				_dim = dim;
				cvNamedWindow("wrist", 1);	
				cvNamedWindow("finger", 1);
				cvMoveWindow("wrist", 0, 600);
				cvMoveWindow("finger", 400, 600);
				//initial value of trackbar
				fid_tb = 0;

				for (int i = 0; i < 19;i++)
				for (int j = 0; j < 3; j++)
					fval_tb[i][j] = 50;

				wval_tb[0] = 50;
				wval_tb[1] = 50;
				wval_tb[2] = 100;
				wval_tb[3] = 50;
				wval_tb[4] = 0;
				wval_tb[5] = 0;
			}
			
			void run(){
				cvCreateTrackbar("tx", "wrist", &wval_tb[0], 100, NULL);
				cvCreateTrackbar("ty", "wrist", &wval_tb[1], 100, NULL);
				cvCreateTrackbar("tz", "wrist", &wval_tb[2], 100, NULL);
				cvCreateTrackbar("rx", "wrist", &wval_tb[3], 100, NULL);
				cvCreateTrackbar("ry", "wrist", &wval_tb[4], 100, NULL);
				cvCreateTrackbar("rz", "wrist", &wval_tb[5], 100, NULL);

				cvCreateTrackbar("fid", "finger", &fid_tb,18, NULL);
				cvCreateTrackbar("fid2", "finger", &fid2_tb, 1, NULL);
				cvCreateTrackbar("fval0", "finger", &fval_tb[fid_tb][0], 100, NULL);
				cvCreateTrackbar("fval1", "finger", &fval_tb[fid_tb][1], 100, NULL);
				cvCreateTrackbar("fval2", "finger", &fval_tb[fid_tb][2], 100, NULL);

				wval[0] = -100 + (wval_tb[0] / 100.0) * 200.0;
				wval[1] = -100 + (wval_tb[1] / 100.0) * 200.0;
				wval[2] =   0 +  (wval_tb[2] / 100.0) * 300.0;
				wval[3] = -180 + (wval_tb[3] / 100.0) * 360.0;
				wval[4] = -180 + (wval_tb[4] / 100.0) * 360.0;
				wval[5] = -180 + (wval_tb[5] / 100.0) * 360.0;

				fid = fid_tb;
				fid2 = fid2_tb;
				for (int j = 0; j < 3; j++)
					fval[j] = -360 + (fval_tb[fid][j] / 100.0) * 720.0;		
			}
	};


public:
	Trackbar _trackbar;

	HandGenerator(){

	}

	HandGenerator(HandParameters hg){

		handParamNum = hg.handParamNum;
		

		//init trackbar
		_trackbar = Trackbar(handParamNum);
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

	void run_setTbarFromResult(float* hsol)
	{
		_trackbar.wval_tb[0] = (hsol[0] + 100)*(100.0 / 200.0);
		_trackbar.wval_tb[1] = (hsol[1] + 100)*(100.0 / 200.0);
		_trackbar.wval_tb[2] = (hsol[2] +   0)*(100.0 / 300.0);
		_trackbar.wval_tb[3] = (hsol[3] + 180)*(100.0 / 360.0);
		_trackbar.wval_tb[4] = (hsol[4] + 180)*(100.0 / 360.0);
		_trackbar.wval_tb[5] = (hsol[5] + 180)*(100.0 / 360.0);

		for (int i = 0; i < 5; i++)
		{
			_trackbar.fval_tb[4*i+0][0] = (hsol[6+4*i] + 360)*(100.0 / 720.0);
			_trackbar.fval_tb[4*i+0][2] = (hsol[7+4*i] + 360)*(100.0 / 720.0);
			_trackbar.fval_tb[4*i+1][0] = (hsol[8+4*i] + 360)*(100.0 / 720.0);
			_trackbar.fval_tb[4*i+2][0] = (hsol[9+4*i] + 360)*(100.0 / 720.0);
		}

	}

	void run_gui(){

	//--trackbar--//
		_trackbar.run();

	//--set parameter--//

		//wrist
		float tx = _trackbar.wval[0];
		float ty = _trackbar.wval[1];
		float tz = _trackbar.wval[2];
		float rx = _trackbar.wval[3];
		float ry = _trackbar.wval[4];
		float rz = _trackbar.wval[5];
		
		//finger
		int joint_index = _trackbar.fid;
		int positions_index = 0;// _trackbar.fid2;
		float fx = _trackbar.fval[0];
		float fy = _trackbar.fval[1];
		float fz = _trackbar.fval[2];
		
		//set joint
		//hand.GetJoint(joint_index, positions_index, fx,fy,fz);
		hand.SetJoint(joint_index, positions_index,fx,fy,fz);
	

	//--render--//
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		hand.setViewport(640, 480);
		hand.Render(rx, ry, rz, false, tx, ty, tz, "color");	
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
		
		float f0 = -360 + (_trackbar.fval_tb[0][1] / 100.0) * 720.0;
		float f1 = -360 + (_trackbar.fval_tb[4][1] / 100.0) * 720.0;
		float f2 = -360 + (_trackbar.fval_tb[8][1] / 100.0) * 720.0;
		float f3 = -360 + (_trackbar.fval_tb[12][1] / 100.0) * 720.0;
		float f4 = -360 + (_trackbar.fval_tb[16][1] / 100.0) * 720.0;

		hand.SetJoint(0, 0, in[6], f0, in[7]);
		hand.SetJoint(1, 0, in[8], 0, 0);
		hand.SetJoint(2, 0, in[9], 0, 0);

		hand.SetJoint(4, 0, in[10], f1, in[11]);
		hand.SetJoint(5, 0, in[12], 0, 0);
		hand.SetJoint(6, 0, in[13], 0, 0);

		hand.SetJoint(8, 0, in[14], f2, in[15]);
		hand.SetJoint(9, 0, in[16], 0, 0);
		hand.SetJoint(10, 0, in[17], 0, 0);

		hand.SetJoint(12, 0, in[18], f3, in[19]);
		hand.SetJoint(13, 0, in[20], 0, 0);
		hand.SetJoint(14, 0, in[21], 0, 0);

		hand.SetJoint(16, 0, in[22], f4, in[23]);
		hand.SetJoint(17, 0, in[24], 0, 0);
		hand.SetJoint(18, 0, in[25], 0, 0);

		/*
		hand.SetJoint(0, 0, in[6], 0, in[7]);
		hand.SetJoint(1, 0, in[8], 0, 0);
		hand.SetJoint(2, 0, in[9], 0, 0);

		hand.SetJoint(4, 0, in[10], -14, in[11]);
		hand.SetJoint(5, 0, in[12], 0, 0);
		hand.SetJoint(6, 0, in[13], 0, 0);

		hand.SetJoint(8, 0, in[14], 0, in[15]);
		hand.SetJoint(9, 0, in[16], 0, 0);
		hand.SetJoint(10, 0, in[17], 0, 0);

		hand.SetJoint(12, 0, in[18], 0, in[19]);
		hand.SetJoint(13, 0, in[20], 0, 0);
		hand.SetJoint(14, 0, in[21], 0, 0);

		hand.SetJoint(16, 0, in[22], 0, in[23]);
		hand.SetJoint(17, 0, in[24], 0, 0);
		hand.SetJoint(18, 0, in[25], 0, 0);
		*/
		
	}
};