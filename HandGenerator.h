#pragma once

#include "Hand.h"

#define WR(x) std::cout << x << std::flush
#define WRL(x) std::cout << x << std::endl

const float bar_range = 255.0;// 100;

using namespace std;

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

				for (int j = 0; j < 6; j++){
					float bu = boundary_max[1][j];
					float bl = boundary_max[0][j];
					float l = bu - bl;
					wval[j] = bl + (wval_tb[j] / bar_range)*l;
				}

				for (int i = 0; i < 15; i++)
				for (int j = 0; j < 3; j++)
					fval[i][j] = -180 + (fval_tb[i][j] / bar_range) * 360.0; // due to other DoFs except for 26 DoFs.

				for (int i = 0; i < 5; i++){
					float bu0 = boundary_max[1][6 + 4 * i + 0];
					float bl0 = boundary_max[0][6 + 4 * i + 0];
					float bu1 = boundary_max[1][6 + 4 * i + 1];
					float bl1 = boundary_max[0][6 + 4 * i + 1];
					float bu2 = boundary_max[1][6 + 4 * i + 2];
					float bl2 = boundary_max[0][6 + 4 * i + 2];
					float bu3 = boundary_max[1][6 + 4 * i + 3];
					float bl3 = boundary_max[0][6 + 4 * i + 3];

					float l0 = bu0 - bl0;
					float l1 = bu1 - bl1;
					float l2 = bu2 - bl2;
					float l3 = bu3 - bl3;

					fval[3 * i + 0][0] = bl0 + (fval_tb[3 * i + 0][0] / bar_range)*l0;
					fval[3 * i + 0][2] = bl1 + (fval_tb[3 * i + 0][2] / bar_range)*l1;
					fval[3 * i + 1][0] = bl2 + (fval_tb[3 * i + 1][0] / bar_range)*l2;
					fval[3 * i + 2][0] = bl3 + (fval_tb[3 * i + 2][0] / bar_range)*l3;
				}

				/*
				wval[0] = -100 + (wval_tb[0] / bar_range) * 200.0;
				wval[1] = -100 + (wval_tb[1] / bar_range) * 200.0;
				wval[2] = 0 + (wval_tb[2] / bar_range) * 600.0;
				wval[3] = -180 + (wval_tb[3] / bar_range) * 360.0;
				wval[4] = -180 + (wval_tb[4] / bar_range) * 360.0;
				wval[5] = -180 + (wval_tb[5] / bar_range) * 360.0;

				for (int i = 0; i < 15; i++)
				for (int j = 0; j < 3; j++)
					fval[i][j] = -180 + (fval_tb[i][j] / bar_range) * 360.0;
				*/

				load(-1);
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

				
				cv::moveWindow("wrist", 640 * 3, 550);
				cv::moveWindow("finger", 640 * 3+400, 550);
				for (int j = 0; j < 6; j++){
					float bu = boundary_max[1][j];
					float bl = boundary_max[0][j];
					float l = bu - bl;
					wval[j] = bl + (wval_tb[j] / bar_range)*l;
				}

				
				for (int i = 0; i < 5; i++){
					float bu0 = boundary_max[1][6 + 4 * i + 0];
					float bl0 = boundary_max[0][6 + 4 * i + 0];
					float bu1 = boundary_max[1][6 + 4 * i + 1];
					float bl1 = boundary_max[0][6 + 4 * i + 1];
					float bu2 = boundary_max[1][6 + 4 * i + 2];
					float bl2 = boundary_max[0][6 + 4 * i + 2];
					float bu3 = boundary_max[1][6 + 4 * i + 3];
					float bl3 = boundary_max[0][6 + 4 * i + 3];

					float l0 = bu0 - bl0;
					float l1 = bu1 - bl1;
					float l2 = bu2 - bl2;
					float l3 = bu3 - bl3;

					fval[3 * i + 0][0] = bl0 + (fval_tb[3 * i + 0][0] / bar_range)*l0;
					fval[3 * i + 0][2] = bl1 + (fval_tb[3 * i + 0][2] / bar_range)*l1;
					fval[3 * i + 1][0] = bl2 + (fval_tb[3 * i + 1][0] / bar_range)*l2;
					fval[3 * i + 2][0] = bl3 + (fval_tb[3 * i + 2][0] / bar_range)*l3;
				}

				//for (int i = 0; i < 5; i++){
				//	
				//	if (i == 0)
				//		fval_tb[3 * i + 0][2] = bar_range / 2;
				//	else
				//		fval_tb[3 * i + 0][1] = bar_range / 2;

				//	fval[3 * i + 0][1] = -50 + (fval_tb[3 * i + 0][1] / bar_range) * 100;
				//}

				//printf
				//for (int i = 0; i < 5; i++)
				//	printf("fval[%d]=%f\n", i,fval[3 * i][2]);
				//printf("fval0[x]=%f\n",  fval[0][0]);
				//printf("fval0[y]=%f\n",  fval[0][1]);
				//printf("fval0[z]=%f\n", fval[0][2]);
				//printf("fval1[x]=%f\n", fval[1][0]);

				/*
				wval[0] = -100 + (wval_tb[0] / bar_range) * 200.0;
				wval[1] = -100 + (wval_tb[1] / bar_range) * 200.0;
				wval[2] = 0 + (wval_tb[2] / bar_range) * 600.0;
				wval[3] = -180 + (wval_tb[3] / bar_range) * 360.0;
				wval[4] = -180 + (wval_tb[4] / bar_range) * 360.0;
				wval[5] = -180 + (wval_tb[5] / bar_range) * 360.0;

				for (int i = 0; i < 15;i++)
				for (int j = 0; j < 3; j++)
					fval[i][j] = -180 + (fval_tb[i][j] / bar_range) * 360.0;
				*/

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
	
		float _num_between;

		//26,3
		//dim=the number of hand parameter(26) , controldim=the number of control paraemter for animation.
		PosesetGenerator(int dim, int controldim){
			_dim = dim;
			_trackbar = Trackbar(dim);

			//classid = -1;
			classid = 0;

			poseidx = 0;
			_controlDim = controldim;
			ani_idx = new int[controldim];
			posemin = new float[controldim];

			posestep = new float[controldim];
			posenum = new int[controldim];

			////////////////////////////////////
			//--hard coding for manual input--//
			for (int i = 0; i < 3; i++){
				posestep[i] = 8; //10: train , 8: test
				posenum[i] = 4;//6: train, 4: test
			}
			posenum_class = posenum[0] * posenum[1] * posenum[2];
			
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
			

			poseidx++;
			//if (poseidx == 10){
			if (poseidx == 1){
				poseidx = 0;
				classid += 1;
			}
			if (classid == 25)
				return -1;
		

			return 0;
		}

		//finger test for epfl
		int run_sequence_between_FingerTest(){

			//float num_between = 100.0;

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
				_trackbar.load(classid);
				_trackbar.run();
				for (int j = 0; j < 6; j++)
					wval_b[j] = _trackbar.wval[j];
				for (int i = 0; i < 15; i++)
				for (int j = 0; j < 3; j++)
					fval_b[i][j] = _trackbar.fval[i][j];

					//test(1): move a finger individually to check an axis.
				    //0(ok) , 3(ok), 6(ok), 9(ok), 12(ok)
					//fval_b[3][0] = -90;  //0,3,6,9,12
					//fval_b[6][0] = -90;  //0,3,6,9,12
					//fval_b[9][0] = -70;  //0,3,6,9,12
					fval_b[12][0] = -90;  //0,3,6,9,12


					//test(2): move some fingers simultaneously.
				
					//fval_b[3 + 1][0] = -90;		fval_b[3 + 2][0] = -90;
					//fval_b[6 + 1][0] = -90;		fval_b[6 + 2][0] = -90;
					//fval_b[9 + 1][0] = -90;		fval_b[9 + 2][0] = -90;
					//fval_b[12 + 1][0] = -70;	fval_b[12 + 2][0] = -90;
				
					//fval_b[3 + 0][0] = -90; 
					//fval_b[6 + 0][0] = -90; 
					//fval_b[9 + 0][0] = -90;
					//fval_b[12 + 0][0] = -90; 
			}
			//calculate difference between pose(a) and pose(b).
			for (int j = 0; j < 6; j++)
				_trackbar.wval[j] = wval_a[j] + (poseidx / _num_between)*(wval_b[j] - wval_a[j]);

			for (int i = 0; i < 5; i++){
				_trackbar.fval[3 * i + 0][0] = fval_a[3 * i + 0][0] + (poseidx / _num_between)*(fval_b[3 * i + 0][0] - fval_a[3 * i + 0][0]);
				_trackbar.fval[3 * i + 0][2] = fval_a[3 * i + 0][2] + (poseidx / _num_between)*(fval_b[3 * i + 0][2] - fval_a[3 * i + 0][2]);
				_trackbar.fval[3 * i + 1][0] = fval_a[3 * i + 1][0] + (poseidx / _num_between)*(fval_b[3 * i + 1][0] - fval_a[3 * i + 1][0]);
				_trackbar.fval[3 * i + 2][0] = fval_a[3 * i + 2][0] + (poseidx / _num_between)*(fval_b[3 * i + 2][0] - fval_a[3 * i + 2][0]);
			}

			for (int j = 0; j < 6; j++)
				printf("w[%d]=%.2f\n", j, wval_b[j] - _trackbar.wval[j]);
			for (int i = 0; i < 5; i++){
				float d0 = fval_b[3 * i + 0][0] - _trackbar.fval[3 * i + 0][0];
				float d1 = fval_b[3 * i + 0][2] - _trackbar.fval[3 * i + 0][2];
				float d2 = fval_b[3 * i + 1][0] - _trackbar.fval[3 * i + 1][0];
				float d3 = fval_b[3 * i + 2][0] - _trackbar.fval[3 * i + 2][0];
				printf("f[%d]=%.2f %.2f %.2f %.2f\n", i, d0, d1, d2, d3);
			}

			poseidx++;

			if (poseidx == _num_between)
				return -1;

	
			return 0;
		}

		//sequence between pose(a) and pose(b)
		//we consider collision by first moving from index finger to pinky, and then finally moving thumb.
		
		void updatePose(std::string type, int fr,float num_between){
			
			if (fr == 0){
				for (int i = 0; i < 15; i++)
				for (int j = 0; j < 3; j++)
					_trackbar.fval[i][j] = fval_a[i][j];
			}
			
			if (type == "thumbFirst"){
				for (int j = 0; j < 6; j++)
					_trackbar.wval[j] = wval_a[j] + (fr / (2*num_between))*(wval_b[j] - wval_a[j]);

					for (int i = 0; i < 3; i++)
					for (int j = 0; j < 3; j++)
						_trackbar.fval[i][j] = fval_a[i][j] + ((fr%(int)num_between) / num_between)*(fval_b[i][j] - fval_a[i][j]);

			}
			else if (type == "thumbLast"){
				for (int j = 0; j < 6; j++)
					_trackbar.wval[j] = wval_a[j] + (fr / (2*num_between))*(wval_b[j] - wval_a[j]);

					for (int i = 3; i < 15; i++)
					for (int j = 0; j < 3; j++)
						_trackbar.fval[i][j] = fval_a[i][j] + ((fr%(int)num_between) / num_between)*(fval_b[i][j] - fval_a[i][j]);

			}
			else if (type == "All"){
				for (int j = 0; j < 6; j++)
					_trackbar.wval[j] = wval_a[j] + (fr / (2 * num_between))*(wval_b[j] - wval_a[j]);

				for (int i = 0; i < 15; i++)
				for (int j = 0; j < 3; j++)
					_trackbar.fval[i][j] = fval_a[i][j] + (fr / (2 * num_between)) *(fval_b[i][j] - fval_a[i][j]);


			}
		}

		void updatePose_backup(std::string type,float num_between){
			if (type == "thumbFirst"){
				//calculate difference between pose(a) and pose(b).
				if (poseidx < num_between){
					for (int j = 0; j < 6; j++)
						_trackbar.wval[j] = wval_a[j] + (poseidx / num_between)*(wval_b[j] - wval_a[j]);

					if (poseidx == 0){
						for (int i = 0; i < 15; i++)
						for (int j = 0; j < 3; j++)
							_trackbar.fval[i][j] = fval_a[i][j] + (poseidx / num_between)*(fval_b[i][j] - fval_a[i][j]);
					}
					else{
						for (int i = 0; i < 3; i++)
						for (int j = 0; j < 3; j++)
							_trackbar.fval[i][j] = fval_a[i][j] + (poseidx / num_between)*(fval_b[i][j] - fval_a[i][j]);
					}
				}
				else{
					for (int i = 3; i < 15; i++)
					for (int j = 0; j < 3; j++)
						_trackbar.fval[i][j] = fval_a[i][j] + ((poseidx - num_between) / num_between)*(fval_b[i][j] - fval_a[i][j]);
				}
			}
			else if (type == "thumbLast"){
				//calculate difference between pose(a) and pose(b).
				if (poseidx < num_between){
					for (int j = 0; j < 6; j++)
						_trackbar.wval[j] = wval_a[j] + (poseidx / num_between)*(wval_b[j] - wval_a[j]);

					if (poseidx == 0){
						for (int i = 0; i < 15; i++)
						for (int j = 0; j < 3; j++)
							_trackbar.fval[i][j] = fval_a[i][j] + (poseidx / num_between)*(fval_b[i][j] - fval_a[i][j]);
					}
					else{
						for (int i = 3; i < 15; i++)
						for (int j = 0; j < 3; j++)
							_trackbar.fval[i][j] = fval_a[i][j] + (poseidx / num_between)*(fval_b[i][j] - fval_a[i][j]);
					}
				}
				else{
					for (int i = 0; i < 3; i++)
					for (int j = 0; j < 3; j++)
						_trackbar.fval[i][j] = fval_a[i][j] + ((poseidx - num_between) / num_between)*(fval_b[i][j] - fval_a[i][j]);
				}
			}
		}

		bool gotoOpenPalm = false;
		bool halfSeq = true;
		int run_sequence_between_includingOpenPalm(){

			//0: update thumb first,  1: update other fingers first, 2: update all simultaneously.
			int movingType[26] = { 2, 2, 2, 2, 0,
				2, 2, 2, 1,
				1, 2, 2, 0, 0,
				2, 2, 2, 2,
				1, 0, 2, 2, 2,
				2, 2, 2 };

			/*
			int movingType[26] = { 1,0,0,0,0,
			                       0,0,0,1,
								   1,0,0,0,0,
								   0,0,0,0,
								   1,0,0,0,0,
								   0,0,0};
			*/

			//float num_between = 50;//50.0;
			float num_seq = 2 * _num_between;

			//set 2 key poses.
			if (poseidx == 0){
				//openPalm -> ESL
				if (gotoOpenPalm == false){
					//pose(a)
					_trackbar.load(-1);
					_trackbar.run();
					for (int j = 0; j < 6; j++)
						wval_a[j] = _trackbar.wval[j];
					for (int i = 0; i < 15; i++)
					for (int j = 0; j < 3; j++)
						fval_a[i][j] = _trackbar.fval[i][j];

					//pose(b)
					if (classid == 15){
						classid += 2;
						_trackbar.load(classid);
					}
					else{
						classid += 1;
						_trackbar.load(classid);
					}

					_trackbar.run();
					for (int j = 0; j < 6; j++)
						wval_b[j] = _trackbar.wval[j];
					for (int i = 0; i < 15; i++)
					for (int j = 0; j < 3; j++)
						fval_b[i][j] = _trackbar.fval[i][j];

				}
				//ESL -> openPalm
				else{
					//pose(a)
					_trackbar.load(classid);
					_trackbar.run();
					for (int j = 0; j < 6; j++)
						wval_a[j] = _trackbar.wval[j];
					for (int i = 0; i < 15; i++)
					for (int j = 0; j < 3; j++)
						fval_a[i][j] = _trackbar.fval[i][j];

					//pose(b)
					_trackbar.load(-1);
					_trackbar.run();
					for (int j = 0; j < 6; j++)
						wval_b[j] = _trackbar.wval[j];
					for (int i = 0; i < 15; i++)
					for (int j = 0; j < 3; j++)
						fval_b[i][j] = _trackbar.fval[i][j];

				}				
			}

			//calculate difference between pose(a) and pose(b).
			if (classid == -1)
				updatePose("All", poseidx,_num_between);
			else{
				if (movingType[classid] == 0){
					if (halfSeq == true){
						if (poseidx < _num_between)
							updatePose("thumbFirst", poseidx, _num_between);
						else
							updatePose("thumbLast", poseidx, _num_between);
					}
					else{
						if (poseidx < _num_between)
							updatePose("thumbLast", poseidx, _num_between);
						else
							updatePose("thumbFirst", poseidx, _num_between);
					}
				}
				else if (movingType[classid] == 1){
					if (halfSeq == true){
						if (poseidx < _num_between)
							updatePose("thumbLast", poseidx, _num_between);
						else
							updatePose("thumbFirst", poseidx, _num_between);
					}
					else{
						if (poseidx < _num_between)
							updatePose("thumbFirst", poseidx, _num_between);
						else
							updatePose("thumbLast", poseidx, _num_between);
					}
				}
				else if (movingType[classid] == 2){
					updatePose("All", poseidx, _num_between);
				}
			}


			//
			poseidx++;
			if (poseidx == (int)num_seq){
				poseidx = 0;
				if (gotoOpenPalm == true) gotoOpenPalm = false;
				else gotoOpenPalm = true;

				if (halfSeq == true) halfSeq = false;
				else halfSeq = true;
			}

			if (classid == 25)
				return -1;

			return 0;
		}


		void calculateBetweenPostures(int opt){
			//openPalm->ESL
			if (opt == 0){
				//pose(a)
				_trackbar.load(-1);
				_trackbar.run();
				for (int j = 0; j < 6; j++)
					wval_a[j] = _trackbar.wval[j];
				for (int i = 0; i < 15; i++)
				for (int j = 0; j < 3; j++)
					fval_a[i][j] = _trackbar.fval[i][j];

				//pose(b)
				if (classid == 15){
					classid += 2;
					_trackbar.load(classid);
				}
				else{
					classid += 1;
					_trackbar.load(classid);
				}

				_trackbar.run();
				for (int j = 0; j < 6; j++)
					wval_b[j] = _trackbar.wval[j];
				for (int i = 0; i < 15; i++)
				for (int j = 0; j < 3; j++)
					fval_b[i][j] = _trackbar.fval[i][j];
			}
			//ESL -> openPalm
			else if (opt == 1){
				//pose(a)
				_trackbar.load(classid);
				_trackbar.run();
				for (int j = 0; j < 6; j++)
					wval_a[j] = _trackbar.wval[j];
				for (int i = 0; i < 15; i++)
				for (int j = 0; j < 3; j++)
					fval_a[i][j] = _trackbar.fval[i][j];

				//pose(b)
				_trackbar.load(-1);
				_trackbar.run();
				for (int j = 0; j < 6; j++)
					wval_b[j] = _trackbar.wval[j];
				for (int i = 0; i < 15; i++)
				for (int j = 0; j < 3; j++)
					fval_b[i][j] = _trackbar.fval[i][j];
			}
			//ESL->ESL
			else if (opt == 2){
				//pose(a)
				_trackbar.load(classid);
				_trackbar.run();
				for (int j = 0; j < 6; j++)
					wval_a[j] = _trackbar.wval[j];
				for (int i = 0; i < 15; i++)
				for (int j = 0; j < 3; j++)
					fval_a[i][j] = _trackbar.fval[i][j];

				//pose(b)
				if (classid == 15){
					classid += 2;
					_trackbar.load(classid);
				}
				else{
					classid += 1;
					_trackbar.load(classid);
				}

				_trackbar.run();
				for (int j = 0; j < 6; j++)
					wval_b[j] = _trackbar.wval[j];
				for (int i = 0; i < 15; i++)
				for (int j = 0; j < 3; j++)
					fval_b[i][j] = _trackbar.fval[i][j];
			}
		}

		
		int run_sequence_between(){
			static int animationIndex = 0;
			//moving posture (0:open->ESL, 1:ESL->open, 2:ESL->ESL ) 
			static int animationType[32] = { 0, 2, 2, 2, 2, 2,
									2, 2, 1, 0, 2,
									1, 0, 2, 1, 0, 1, 0, 1, 0,
									2, 2, 1, 0,
									1, 0, 2, 2, 2, 2,
									2, 2 };
		
			//0: thumb first,  1: thumb last, 2: all simultaneously.
			static int movingType[32] = { 2, 1, 2, 2, 1,1,
				2, 2, 2, 1,2,
				2, 2, 2, 2,0,1,0,1,2,
				2,2,2,1,
				0,0,2,2,2,2,
				0,2};


			//float num_between = 10; // 2,5,10,20
			float num_seq = 2 * _num_between;

			
			//set 2 key poses.
			if (poseidx == 0)
				calculateBetweenPostures(animationType[animationIndex]);
			
			//calculate difference between pose(a) and pose(b).
			if (classid == -1)
				updatePose("All", poseidx, _num_between);
			else{
				if (movingType[animationIndex] == 0){
					if (halfSeq == true){
						if (poseidx < _num_between)
							updatePose("thumbFirst", poseidx, _num_between);
						else
							updatePose("thumbLast", poseidx, _num_between);
					}
					else{
						if (poseidx < _num_between)
							updatePose("thumbLast", poseidx, _num_between);
						else
							updatePose("thumbFirst", poseidx, _num_between);
					}
				}
				else if (movingType[animationIndex] == 1){
					if (halfSeq == true){
						if (poseidx < _num_between)
							updatePose("thumbLast", poseidx, _num_between);
						else
							updatePose("thumbFirst", poseidx, _num_between);
					}
					else{
						if (poseidx < _num_between)
							updatePose("thumbFirst", poseidx, _num_between);
						else
							updatePose("thumbLast", poseidx, _num_between);
					}
				}
				else if (movingType[animationIndex] == 2){
					updatePose("All", poseidx, _num_between);
				}
			}


			//
			poseidx++;
			if (poseidx == (int)num_seq){
				poseidx = 0;
				animationIndex += 1;

				if (halfSeq == true) halfSeq = false;
				else halfSeq = true;
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
		for (int j = 0; j < 6; j++){
			float bu = boundary_max[1][j];
			float bl = boundary_max[0][j];
			float l = bu - bl;
			_trackbar.wval_tb[j] = (hsol[j] - bl)*(bar_range / l);

		}

		for (int i = 0; i < 5; i++)
		{
			float bu0 = boundary_max[1][6 + 4 * i + 0];
			float bl0 = boundary_max[0][6 + 4 * i + 0];
			float bu1 = boundary_max[1][6 + 4 * i + 1];
			float bl1 = boundary_max[0][6 + 4 * i + 1];
			float bu2 = boundary_max[1][6 + 4 * i + 2];
			float bl2 = boundary_max[0][6 + 4 * i + 2];
			float bu3 = boundary_max[1][6 + 4 * i + 3];
			float bl3 = boundary_max[0][6 + 4 * i + 3];

			float l0 = bu0 - bl0;
			float l1 = bu1 - bl1;
			float l2 = bu2 - bl2;
			float l3 = bu3 - bl3;

			_trackbar.fval_tb[3 * i + 0][0] = (hsol[6 + 4 * i] - bl0)*(bar_range / l0);
			_trackbar.fval_tb[3 * i + 0][2] = (hsol[7 + 4 * i] - bl1)*(bar_range / l1);
			_trackbar.fval_tb[3 * i + 1][0] = (hsol[8 + 4 * i] - bl2)*(bar_range / l2);
			_trackbar.fval_tb[3 * i + 2][0] = (hsol[9 + 4 * i] - bl3)*(bar_range / l3);
		}
		
		/*
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
		*/

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

		if (jid==12)
			hand.SetJoint(jid, positions_index, fx, -30, fz); //see setPose for '-30'
		else
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

	void showJoints(cv::Mat cam_color,cv::Mat cam_depth, cv::Mat cmat){

		//get all joints
		std::vector<float> jpos;
		hand.GetJointAllPosition(&jpos);

		//set joint index for fingertip.
		std::vector<int> jidx;
		jidx.push_back(4); jidx.push_back(9); jidx.push_back(14); jidx.push_back(19); jidx.push_back(24);//

		//visualize joint as circle 
		int color[5][3] = { { 255, 0, 0 }, { 0, 255, 0 }, { 0, 0, 255 }, { 255, 255, 0 }, { 255, 255, 255 } };
		for (int i = 0; i < jidx.size();i++)
		{
			int jid = jidx[i];

			float x = jpos[3 * jid + 0];
			float y = jpos[3 * jid + 1];
			float z = jpos[3 * jid + 2];

			float x_ = x * cmat.at<float>(0, 0) + y * cmat.at<float>(0, 1) + z * cmat.at<float>(0, 2);
			float y_ = x * cmat.at<float>(1, 0) + y * cmat.at<float>(1, 1) + z * cmat.at<float>(1, 2);
			float z_ = x * cmat.at<float>(2, 0) + y * cmat.at<float>(2, 1) + z * cmat.at<float>(2, 2);
			x_ /= z_;
			y_ /= z_;

			cv::circle(cam_color, cv::Point(x_, y_), 10, cv::Scalar(255, 0, 0), 1, 8, 0);
		}
		cv::imshow("groundtruth", cam_color);

		//color code visualization
		cv::Mat mask = cam_depth >0;
		double min;
		double max;
		cv::minMaxIdx(cam_depth, &min, &max,NULL,NULL,mask);

		cv::Mat adjMap;
		cam_depth.convertTo(adjMap, CV_8UC1, 255 / (max - min), -min);

		cv::Mat falseColorsMap;
		applyColorMap(adjMap, falseColorsMap, 2);// cv::COLORMAP_AUTUMN);
		cv::imshow("groundtruth1", falseColorsMap);
		cv::moveWindow("groundtruth1", 640 * 3, 0);

		//distance between joints	
		/*
		for (int i = 0; i < 19; i++){
			int jid0 = idx[i]; int jid1 = idx[i + 1];

			float x0 = jpos[3 * jid0 + 0];
			float y0 = jpos[3 * jid0 + 1];
			float z0 = jpos[3 * jid0 + 2];
			float x1 = jpos[3 * jid1 + 0];
			float y1 = jpos[3 * jid1 + 1];
			float z1 = jpos[3 * jid1 + 2];

			float dx = x0 - x1; float dy = y0 - y1; float dz = z0 - z1;
			float dist = sqrt(dx*dx + dy*dy + dz*dz);
			//printf("1dist[%d]=%f\n", i, dist);
		}
		*/


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
		
		//get all joints
		std::vector<float> jpos;
		hand.GetJointAllPosition(&jpos);

		//set joint index for fingertip.
		std::vector<int> jidx;
		jidx.push_back(4); jidx.push_back(9); jidx.push_back(14); jidx.push_back(19); jidx.push_back(24);//


		string fname = "save/sequence/groundtruth15D.csv";
		static ofstream file1(fname);

		for (int i = 0; i < jidx.size(); i++)
		{
			int jid = jidx[i];

			float x = jpos[3 * jid + 0];
			float y = jpos[3 * jid + 1];
			float z = jpos[3 * jid + 2];

			char str[100];
			if (i == (jidx.size() - 1))
				file1 << x << "," << y << "," << z << endl;
			else
				file1 << x << "," << y << "," << z << ",";
		}

	}

	void saveParameters26D(float* com_hand){
	
		string fname = "save/sequence/label26D.csv";
		static ofstream file1(fname);

		char str[100];
		float jpos[26];
		getHandPose(jpos);

		//relative position from com_hand
		for (int i = 0; i < 3; i++)
			jpos[i] -= com_hand[i];

		for (int i = 0; i < 26; i++)
		{
			if (i == 25)
				file1 << jpos[i] << endl;
			//sprintf(str, "%.2f\n", jpos[i]);
			else
				file1 << jpos[i] << ",";
				//sprintf(str, "%.2f,", jpos[i]);
		}

	}


private:
	
	HandParameters hp;

	int handParamNum;

	float* hand_param;

	
	void setPose(float* in){
			
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

		hand.SetJoint(12, 0, in[22], -30, in[23]); // see renderGui for '-30'
		//hand.SetJoint(12, 0, in[22], 0, in[23]);
		hand.SetJoint(13, 0, in[24], 0, 0);
		hand.SetJoint(14, 0, in[25], 0, 0);

	}
};