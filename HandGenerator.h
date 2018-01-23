#pragma once

#include "Hand.h"

#define WR(x) std::cout << x << std::flush
#define WRL(x) std::cout << x << std::endl

const float bar_range = 100;

class HandGenerator{
	class Trackbar
	{
		private:
			char _wname[1024];
			int _dim;

		public:

			int fid_tb;
			int jid_tb;
			int fval_tb[15][3];
			int wval_tb[6];

			float fval[15][3];
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
				jid_tb = 0;

				wval_tb[0] = 99;
				wval_tb[1] = 32;
				wval_tb[2] = 42;//84;
				wval_tb[3] = 73;
				wval_tb[4] = 45;
				wval_tb[5] = 53;

				for (int i = 0; i < 15;i++)
				for (int j = 0; j < 3; j++)
					fval_tb[i][j] = 50;

			}

			
			void run(){
				cvCreateTrackbar("tx", "wrist", &wval_tb[0], bar_range, NULL);
				cvCreateTrackbar("ty", "wrist", &wval_tb[1], bar_range, NULL);
				cvCreateTrackbar("tz", "wrist", &wval_tb[2], bar_range, NULL);
				cvCreateTrackbar("rx", "wrist", &wval_tb[3], bar_range, NULL);
				cvCreateTrackbar("ry", "wrist", &wval_tb[4], bar_range, NULL);
				cvCreateTrackbar("rz", "wrist", &wval_tb[5], bar_range, NULL);

				cvCreateTrackbar("fid", "finger", &fid_tb, 4, NULL); //14
				cvCreateTrackbar("jid", "finger",  &jid_tb, 2, NULL);
				cvCreateTrackbar("fval0", "finger", &fval_tb[fid_tb*3+jid_tb][0], bar_range, NULL);
				cvCreateTrackbar("fval1", "finger", &fval_tb[fid_tb*3+jid_tb][1], bar_range, NULL);
				cvCreateTrackbar("fval2", "finger", &fval_tb[fid_tb*3+jid_tb][2], bar_range, NULL);

				// wval_tb-> wval
				// fval_tb-> fval
				// fid_tb -> fid 
				// fid2_tb -> fid2
				wval[0] = -100 + (wval_tb[0] / bar_range) * 200.0;
				wval[1] = -100 + (wval_tb[1] / bar_range) * 200.0;
				wval[2] = 0 + (wval_tb[2] / bar_range) * 600.0;
				wval[3] = -180 + (wval_tb[3] / bar_range) * 360.0;
				wval[4] = -180 + (wval_tb[4] / bar_range) * 360.0;
				wval[5] = -180 + (wval_tb[5] / bar_range) * 360.0;

				//printf("wt %f %f %f\n", wval[0], wval[1], wval[2]);
				//printf("wr %f %f %f\n", wval[3], wval[4], wval[5]);
				for (int i = 0; i < 15;i++)
				for (int j = 0; j < 3; j++)
					fval[i][j] = -180 + (fval_tb[i][j] / bar_range) * 360.0;
			}

			void save(int cid){
				char filename[100];
				char str[100];
				sprintf(filename, "save/trackbar/posture%d.txt", cid);

				FILE* fp;
				fp = fopen(filename, "w");
	
				for (int i = 0; i < 6; i++){
					sprintf(str, "%d\n", wval_tb[i]);
					fputs(str, fp);
				}
				for (int i = 0; i < 15; i++)
				for (int j = 0; j < 3; j++){
					sprintf(str, "%d\n", fval_tb[i][j]);
					fputs(str, fp);
				}

				fclose(fp);
			}
			void load(int cid){
				
				char filename[100];
				char str[100];
				sprintf(filename, "save/trackbar/posture%d.txt", cid);

				FILE* fp;
				fp = fopen(filename, "r");

				fseek(fp, 0, SEEK_SET);
				for (int i = 0; i < 6; i++){
					fgets(str, sizeof(str), fp);
					wval_tb[i] = std::stoi(str);
				}
				for (int i = 0; i < 15; i++)
				for (int j = 0; j <  3; j++){
					fgets(str, sizeof(str), fp);
					fval_tb[i][j] = std::stoi(str);
				}
				fclose(fp);
				
				
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
		_trackbar.wval_tb[0] = (hsol[0] + 100)*(bar_range / 200.0);
		_trackbar.wval_tb[1] = (hsol[1] + 100)*(bar_range / 200.0);
		_trackbar.wval_tb[2] = (hsol[2] + 0)*(bar_range / 600.0);
		_trackbar.wval_tb[3] = (hsol[3] + 180)*(bar_range / 360.0);
		_trackbar.wval_tb[4] = (hsol[4] + 180)*(bar_range / 360.0);
		_trackbar.wval_tb[5] = (hsol[5] + 180)*(bar_range / 360.0);

		for (int i = 0; i < 5; i++)
		{
			_trackbar.fval_tb[3 * i + 0][0] = (hsol[6 + 4 * i] + 180)*(bar_range / 360.0);
			_trackbar.fval_tb[3 * i + 0][2] = (hsol[7 + 4 * i] + 180)*(bar_range / 360.0);
			_trackbar.fval_tb[3 * i + 1][0] = (hsol[8 + 4 * i] + 180)*(bar_range / 360.0);
			_trackbar.fval_tb[3 * i + 2][0] = (hsol[9 + 4 * i] + 180)*(bar_range / 360.0);
		}

	}

	void run_trackbar(){
		_trackbar.run();
	}
	void save_trackbar(){
		int cid;
		printf("class index to save:\n");
		std::cin >> cid;
		_trackbar.save(cid);
	}
	void load_trackbar(){
		int cid;
		printf("class index to load:\n");
		std::cin >> cid;
		_trackbar.load(cid);

	}
	void run_gui(std::string type){

		for (int i = 0; i < 5;i++)
		for (int j = 0; j < 3;j++) {
			float tx = _trackbar.wval[0];
			float ty = _trackbar.wval[1];
			float tz = _trackbar.wval[2];
			float rx = _trackbar.wval[3];
			float ry = _trackbar.wval[4];
			float rz = _trackbar.wval[5];

			float fx = _trackbar.fval[i*3+j][0];
			float fy = _trackbar.fval[i*3+j][1];
			float fz = _trackbar.fval[i*3+j][2];

			printf("[%d][%d]=%.2f %.2f %.2f\n", i, j, fx, fy, fz);
			renderGui(tx, ty, tz, rx, ry, rz, fx, fy, fz, i*3+j, type);
		}

	}

	void setHandPose(float* in)
	{
		for (int i = 0; i < hp.handParamNum; i++)
			hand_param[i] = in[i];
	}

	void renderGui(float tx, float ty, float tz, float rx, float ry, float rz, float fx, float fy, float fz, float jid, std::string type){

		int positions_index = 0;// _trackbar.fid2;

		hand.SetJoint(jid, positions_index, fx, fy, fz);

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		hand.setViewport(640, 480);
		hand.Render(rx, ry, rz, false, tx, ty, tz, type);
	}

	void renderTile(int px,int py,float* in,std::string vistype){
		
		if (px==0 && py==0) //clear only when drawing first particle.
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		
		//printf("tile %.2f %.2f %.2f\n", in[0], in[1], in[2]);
		setPose(in);
		hand.setViewport(px, py, 128, 128);

		hand.Render(in[3], in[4],in[5], false,in[0],in[1],in[2],vistype);
	}

	void renderOrig(float* in,std::string vistype){
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
		
		float f0 = -180 + (_trackbar.fval_tb[0][1] / 100.0) * 180.0;
		float f1 = -180 + (_trackbar.fval_tb[1][1] / 100.0) * 180.0;
		float f2 = -180 + (_trackbar.fval_tb[2][1] / 100.0) * 180.0;
		float f3 = -180 + (_trackbar.fval_tb[3][1] / 100.0) * 180.0;
		float f4 = -180 + (_trackbar.fval_tb[4][1] / 100.0) * 180.0;
		
		hand.SetJoint(0, 0, in[6], 0, in[7]);
		hand.SetJoint(1, 0, in[8], 0, 0);
		hand.SetJoint(2, 0, in[9], 0, 0);

		hand.SetJoint(3, 0, in[10], 0, in[11]);
		hand.SetJoint(4, 0, in[12], 0, 0);
		hand.SetJoint(5, 0, in[13], 0, 0);

		hand.SetJoint(6, 0, in[14], 0, in[15]);
		hand.SetJoint(7, 0, in[16], 0, 0);
		hand.SetJoint(8, 0, in[17], 0, 0);

		hand.SetJoint(9, 0, in[18], 0, in[19]);
		hand.SetJoint(10, 0, in[20], 0, 0);
		hand.SetJoint(11, 0, in[21], 0, 0);

		hand.SetJoint(12, 0, in[22], -10, in[23]);
		hand.SetJoint(13, 0, in[24], 0, 0);
		hand.SetJoint(14, 0, in[25], 0, 0);
		
		/*
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

		hand.SetJoint(16, 0, in[22], 0, in[23]);
		hand.SetJoint(17, 0, in[24], 0, 0);
		hand.SetJoint(18, 0, in[25], 0, 0);
		*/
		
	}
};