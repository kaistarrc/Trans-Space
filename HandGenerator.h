#pragma once

#include "Hand.h"

#define WR(x) std::cout << x << std::flush
#define WRL(x) std::cout << x << std::endl

const float bar_range = 255.0;// 100;



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

				
				wval_tb[0] = 127;// 50;
				wval_tb[1] = 51;// 20;
				wval_tb[2] = 127;// 50;
				wval_tb[3] = 166;// 75;
				wval_tb[4] = 127;// 50;
				wval_tb[5] = 127;// 50;
				for (int i = 0; i < 15;i++)
				for (int j = 0; j < 3; j++)
					fval_tb[i][j] = 127;// 50;

				//init wval&fval
				wval[0] = -100 + (wval_tb[0] / bar_range) * 200.0;
				wval[1] = -100 + (wval_tb[1] / bar_range) * 200.0;
				wval[2] = 0 + (wval_tb[2] / bar_range) * 600.0;
				wval[3] = -180 + (wval_tb[3] / bar_range) * 360.0;
				wval[4] = -180 + (wval_tb[4] / bar_range) * 360.0;
				wval[5] = -180 + (wval_tb[5] / bar_range) * 360.0;

				for (int i = 0; i < 15; i++)
				for (int j = 0; j < 3; j++)
					fval[i][j] = -180 + (fval_tb[i][j] / bar_range) * 360.0;


			}

			//wval_tb -> wval
			//fval_tb -> fval
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


	class PosesetGenerator
	{
	public:
		Trackbar _trackbar;

		PosesetGenerator(){
		}
	

		//26,3
		//dim=the number of hand parameter(26) , controldim=the number of control paraemter for animation.
		PosesetGenerator(int dim, int controldim){
			_dim = dim;
			_trackbar = Trackbar(dim);

			classid = 0;// 0;
			poseidx = 0;
			_controlDim = controldim;
			ani_idx = new int[controldim];
			posemin = new float[controldim];

			posestep = new float[controldim];
			posenum = new int[controldim];

			////////////////////////////////////
			//--hard coding for manual input--//

			posenum_class = 4 * 4 * 4;
			for (int i = 0; i < 3; i++){
				posestep[i] = 8; //10: train , 8: test
				posenum[i] = 4;//6: train, 4: test

			}
			
			//--hard coding for manual input--//
			////////////////////////////////////

			maxNumPose = posenum_class * dim;

			//set cnn class to 0.
			_trackbar.load(0);
			_trackbar.run();

			////////////////////////////////////
			//--hard coding for manual input--//
			posemin[0] = _trackbar.wval[3] - posestep[0] * posenum[0] / 2;
			posemin[1] = _trackbar.wval[4] - posestep[1] * posenum[1] / 2;
			posemin[2] = _trackbar.wval[5] - posestep[2] * posenum[2] / 2;
			//--hard coding for manual input--//
			////////////////////////////////////
		}

		int run_cnndataset(){

			//load trackbar value, and convert it to hand parameter value.
			if (poseidx == (posenum_class* (classid + 1))){
				classid++;

				_trackbar.load(classid);
				_trackbar.run();

				////////////////////////////////////
				//--hard coding for manual input--//
				posemin[0] = _trackbar.wval[3] - posestep[0] * posenum[0] / 2;
				posemin[1] = _trackbar.wval[4] - posestep[1] * posenum[1] / 2;
				posemin[2] = _trackbar.wval[5] - posestep[2] * posenum[2] / 2;
				////////////////////////////////////
				//--hard coding for manual input--//
			}


			//
			int poseidx2 = poseidx - classid*posenum[0] * posenum[1] * posenum[2];

			for (int i = 0; i < _controlDim - 1; i++){

				ani_idx[i] = poseidx2 / pow(posenum[i], (_controlDim - 1) - i);
				poseidx2 -= pow(posenum[i], (_controlDim - 1) - i)*ani_idx[i];
			}
			ani_idx[_controlDim - 1] = poseidx2;

			////////////////////////////////////
			//--hard coding for manual input--//
			_trackbar.wval[3] = posemin[0] + ani_idx[0] * (posestep[0]);
			_trackbar.wval[4] = posemin[1] + ani_idx[1] * (posestep[1]);
			_trackbar.wval[5] = posemin[2] + ani_idx[2] * (posestep[2]);
			////////////////////////////////////
			//--hard coding for manual input--//

			poseidx++;
			

			if (poseidx == (maxNumPose + 1))
				return -1;

			return 0;
		}

		float wval_a[6];
		float wval_b[6];
		float fval_a[15][3];
		float fval_b[15][3];

		//sequence around pose(a).
		int run_sequence(){

			//pose(a)
			if (poseidx == 0){
				_trackbar.load(classid);
				_trackbar.run();

				for (int j = 0; j < 6; j++)
					wval_a[j] = _trackbar.wval[j];
				for (int i = 0; i < 15; i++)
				for (int j = 0; j < 3; j++)
					fval_a[i][j] = _trackbar.fval[i][j];
			}
			
			
			for (int j = 0; j < 6; j++)
				_trackbar.wval[j] = wval_a[j] - 10 + 20 * rand() / double(RAND_MAX);

			
			for (int i = 0; i < 15; i++)
			for (int j = 0; j < 3; j++)
				_trackbar.fval[i][j] = fval_a[i][j];
			

			//
			poseidx++;
			

			if (poseidx == 10){
				poseidx = 0;
				classid += 1;
			}
			if (classid == 25)
				return -1;

			return 0;
		}

		//sequence between pose(a) and pose(b)
		int run_sequence_backup(){

			//pose(a)
			if (poseidx == 0){
				_trackbar.load(classid);
				_trackbar.run();

				for (int j = 0; j < 6; j++)
					wval_a[j] = _trackbar.wval[j];
				for (int i = 0; i < 15; i++)
				for (int j = 0; j < 3; j++)
					fval_a[i][j] = _trackbar.fval[i][j];

				//pose(b)
				_trackbar.load(classid+1);
				_trackbar.run();
				for (int j = 0; j < 6; j++)
					wval_b[j] = _trackbar.wval[j];
				for (int i = 0; i < 15; i++)
				for (int j = 0; j < 3; j++)
					fval_b[i][j] = _trackbar.fval[i][j];
			}
			//calculate difference between pose(a) and pose(b).
			float div = 2.0;
			for (int j = 0; j < 6; j++)
				_trackbar.wval[j] = wval_a[j] + (poseidx/div)*(wval_b[j] - wval_a[j]);
			for (int i = 0; i < 15; i++)
			for (int j = 0; j < 3; j++)
				_trackbar.fval[i][j] = fval_a[i][j] + (poseidx/div)*(fval_b[i][j] - fval_a[i][j]);

			//_trackbar.run();


			poseidx++;

			if (poseidx == (int)div){
				poseidx = 0;
				classid += 1;
			}
			if (classid == 25)
				return -1;

			return 0;
		}

		int test(){
			//if (cv::waitKey(1) == 'p'){
				
			/*
				classid++;
				if (classid == 26)
					classid = 0;
			*/
			
			classid = 3;
			//classid = 7;


				printf("classid:%d\n", classid);
				_trackbar.load(classid);
				_trackbar.run();

				//_trackbar.wval[3] += 0;
				//_trackbar.fval[0][0] += 30;

			//}
			
			return 0;
		}

		int classid;

	private:
		
		int _dim;

		int poseidx;
		int _controlDim;
		int* ani_idx;
		float* posemin;

		int maxNumPose;
		float* posestep;
		int* posenum;
		int posenum_class;

	};
	


public:
	Trackbar _trackbar;
	PosesetGenerator _posesetgenerator;
	Hand hand;

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

		//pose generator for getting training/test set.
		_posesetgenerator=  PosesetGenerator(handParamNum, 3);
	
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
	void run_gui2hand(std::string type){

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
		
			//printf("[%d][%d]=%.2f %.2f %.2f\n", i, j, fx, fy, fz);
			renderGui(tx, ty, tz, rx, ry, rz, fx, fy, fz, i*3+j, type);
		}
	}
	void run_posegenerator2hand(std::string type){

		for (int i = 0; i < 5; i++)
		for (int j = 0; j < 3; j++) {

			float tx = _posesetgenerator._trackbar.wval[0];
			float ty = _posesetgenerator._trackbar.wval[1];
			float tz = _posesetgenerator._trackbar.wval[2];
			float rx = _posesetgenerator._trackbar.wval[3];
			float ry = _posesetgenerator._trackbar.wval[4];
			float rz = _posesetgenerator._trackbar.wval[5];

			float fx = _posesetgenerator._trackbar.fval[i * 3 + j][0];
			float fy = _posesetgenerator._trackbar.fval[i * 3 + j][1];
			float fz = _posesetgenerator._trackbar.fval[i * 3 + j][2];

			//printf("tx ty tz: %f %f %f\n", tx, ty, tz);
			//printf("rx ry rz: %f %f %f\n", rx, ry, rz);
			//printf("[%d][%d]=%.2f %.2f %.2f\n", i, j, fx, fy, fz);
			renderGui(tx, ty, tz, rx, ry, rz, fx, fy, fz, i * 3 + j, type);
		}
	}

	void setHandPose(float* in)
	{
		for (int i = 0; i < hp.handParamNum; i++)
			hand_param[i] = in[i];
	}

	void getHandPose(float* out){

		//wt
		for (int i = 0; i < 3; i++)
			out[i] = _posesetgenerator._trackbar.wval[i];
		//wr
		for (int i = 3; i < 6; i++)
			out[i] = _posesetgenerator._trackbar.wval[i];

		//f
		for (int i = 0; i < 5; i++){
			out[6 + 4 * i + 0] = _posesetgenerator._trackbar.fval[3 * i + 0][0];
			out[6 + 4 * i + 1] = _posesetgenerator._trackbar.fval[3 * i + 0][2];
			out[6 + 4 * i + 2] = _posesetgenerator._trackbar.fval[3 * i + 1][0];
			out[6 + 4 * i + 3] = _posesetgenerator._trackbar.fval[3 * i + 2][0];
		}

	}

	void renderGui(float tx, float ty, float tz, float rx, float ry, float rz, float fx, float fy, float fz, float jid, std::string type){

		int positions_index = 0;// _trackbar.fid2;

		hand.SetJoint(jid, positions_index, fx, fy, fz);

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		//hand.setViewport(640, 480);
		hand.setViewport(hp.width_tile*hp.particle_numx, hp.height_tile*hp.particle_numy);
		hand.Render(rx, ry, rz, false, tx, ty, tz, type);
	}

	void renderTile(int px,int py,float* in,std::string vistype){
		
		if (px==0 && py==0) //clear only when drawing first particle.
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		
		//printf("tile %.2f %.2f %.2f\n", in[0], in[1], in[2]);
		setPose(in);
		hand.setViewport(px, py, hp.width_tile, hp.height_tile);

		hand.Render(in[3], in[4],in[5], false,in[0],in[1],in[2],vistype);
	}

	void renderOrig(float* in,std::string vistype){
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		setPose(in);
		//hand.setViewport(640, 480);
		hand.setViewport(hp.width_tile*hp.particle_numx, hp.height_tile*hp.particle_numy);
		hand.Render(in[3], in[4], in[5], false, in[0], in[1], in[2],vistype);
	}

	void showJoints(cv::Mat cam_color, cv::Mat cmat){

		std::vector<float> jpos;
		hand.GetJointAllPosition(&jpos);

		int idx[20] = { 1, 2, 3, 4,
			6, 7, 8, 9,
			11, 12, 13, 14,
			16, 17, 18, 19,
			21, 22, 23, 24 };
		int color[5][3] = { { 255, 0, 0 }, { 0, 255, 0 }, { 0, 0, 255 }, { 255, 255, 0 }, { 255, 255, 255 } };

		for (int i = 0; i < 20; i++)
		{
			int jid = idx[i];

			float x = jpos[3 * jid + 0];
			float y = jpos[3 * jid + 1];
			float z = jpos[3 * jid + 2];

			float x_ = x * cmat.at<float>(0, 0) + y * cmat.at<float>(0, 1) + z * cmat.at<float>(0, 2);
			float y_ = x * cmat.at<float>(1, 0) + y * cmat.at<float>(1, 1) + z * cmat.at<float>(1, 2);
			float z_ = x * cmat.at<float>(2, 0) + y * cmat.at<float>(2, 1) + z * cmat.at<float>(2, 2);
			x_ /= z_;
			y_ /= z_;

			cv::circle(cam_color, cv::Point(x_, y_), 10, cv::Scalar(color[i / 4][0], color[i / 4][1], color[i / 4][2]), 5, 8, 0);

		}
		cv::imshow("groundtruth", cam_color);
		
		//
		/*
		for (int i = 0; i < 5; i++){
		for (int j = 0; j < 3; j++){
		float jpos[3];
		hand.GetJointPosition(i, j, jpos);

		float x_ = jpos[0] * cmat.at<float>(0, 0) + jpos[1] * cmat.at<float>(0, 1) + jpos[2] * cmat.at<float>(0, 2);
		float y_ = jpos[0] * cmat.at<float>(1, 0) + jpos[1] * cmat.at<float>(1, 1) + jpos[2] * cmat.at<float>(1, 2);
		float z_ = jpos[0] * cmat.at<float>(2, 0) + jpos[1] * cmat.at<float>(2, 1) + jpos[2] * cmat.at<float>(2, 2);
		x_ /= z_;
		y_ /= z_;
		cv::circle(cam_color, cv::Point(x_, y_), 10, cv::Scalar(255, 255, 255), 5, 8, 0);
		//printf("g[%d]: x:%f y:%f z:%f\n", i*3+j, jpos[0] - com_hand[0], jpos[1] - com_hand[1], jpos[2] - com_hand[2]);
		}
		}
		cv::imshow("groundtruth", cam_color);
		*/
	}

	void saveJoints(){
		
		//label (joints position)
		/*
		{
			FILE* fp;
			char filenamel[200];
			sprintf(filenamel, "save/cnn/test/label/label.csv");
			fp = fopen(filenamel, "a");

			char str[100];

			for (int i = 0; i < 5; i++)
			for (int j = 0; j < 3; j++){
			float jpos[3];
			hand.GetJointPosition(i, j, jpos);

			for (int k = 0; k < 3; k++)
			jpos[k] = com_hand[k] - jpos[k];

			if (i == 4 & j == 2)
				sprintf(str, "%.2f,%.2f,%.2f\n", jpos[0], jpos[1], jpos[2]);
			else
				sprintf(str, "%.2f,%.2f,%.2f,", jpos[0], jpos[1], jpos[2]);

			fputs(str, fp);
			}

			fclose(fp);
		}
		*/

		//label (26D pose)
		{
			FILE* fp;
			char filenamel[200];
			//sprintf(filenamel, "save/cnn26D/train/label/label26D.csv");
			sprintf(filenamel, "save/sequence/label/label26D.csv");
			fp = fopen(filenamel, "a");

			char str[100];
			float jpos[26];
			getHandPose(jpos);

			for (int i = 0; i < 26; i++)
			{
				if (i == 25)
					sprintf(str, "%.2f\n", jpos[i]);
				else
					sprintf(str, "%.2f,", jpos[i]);

				fputs(str, fp);
			}
			fclose(fp);
		}
	}


private:
	
	HandParameters hp;

	int handParamNum;

	float* hand_param;

	
	void setPose(float* in){
		
		float f0 = -180 + (_trackbar.fval_tb[0][1] / bar_range) * 180.0;
		float f1 = -180 + (_trackbar.fval_tb[1][1] / bar_range) * 180.0;
		float f2 = -180 + (_trackbar.fval_tb[2][1] / bar_range) * 180.0;
		float f3 = -180 + (_trackbar.fval_tb[3][1] / bar_range) * 180.0;
		float f4 = -180 + (_trackbar.fval_tb[4][1] / bar_range) * 180.0;
		
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

		hand.SetJoint(12, 0, in[22], 0, in[23]);
		//hand.SetJoint(12, 0, in[22], -30, in[23]);
		hand.SetJoint(13, 0, in[24], 0, 0);
		hand.SetJoint(14, 0, in[25], 0, 0);

	}
};