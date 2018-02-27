#pragma once

#include "GLRenderer.h"
#include "HandGenerator.h"
#include "CostFunction.h"
#include "HandParameter.h"
#include "MMF.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>

#include <random>

using namespace std;
//#define DEBUGGING

//orig: 다 지우기.
//fixed:  METHOD0,  METHOD3
//adaptive:  METHOD2,  METHOD3

//#define METHOD0 //fixed boundary
#define METHOD2 //adaptive boundary

#define METHOD3 //cnn term in cost function


//#define METHOD1 //floating boundary (skip)

#define CONSISTENT_SEED

#pragma comment(lib,"opencv_world310.lib")

float bound_stdev[26] = { 10, 10, 10, 10, 10, 10,
						10, 5, 10, 10,
						10, 5, 10, 10,
						10, 5, 10, 10,
						10, 5, 10, 10,
						10, 5, 10, 10 };


namespace {
#ifdef CONSISTENT_SEED
	unsigned int generate_seed() {
		return 0u;
	}
#else
	unsigned int generate_seed() {
		return std::random_device{}();
	}
#endif
}

class PSO{

public:
	cv::Mat gbest;
	

	PSO(int p_numX, int p_numY, int g_num, int dim,
		HandGenerator* renderer,GLRenderer& glrenderer,CostFunction& costFunct,MMF* mmf)
	    //: gen_gaussian{ generate_seed() }, gen_uniform{ generate_seed() }
		:gen_gaussian{ std::random_device{}() }, gen_uniform{ std::random_device{}() }
	{
		_renderer = renderer;
		_costFunct = costFunct;
		_glrenderer = glrenderer;
		_mmf = mmf;
		//_posespace = posespace;

		//size
		particle_numx = p_numX;
		particle_numy = p_numY;
		particle_num = particle_numx*particle_numy;
		max_generation = g_num;
		dimension = dim;

		fingerStartIdx = 6;


		//allocation

		position = cv::Mat(particle_num, dimension, CV_32FC1);
		pbest = cv::Mat(particle_num, dimension, CV_32FC1);

		gbest = cv::Mat(1, dimension, CV_32FC1);
		gbest_pre = cv::Mat(1, dimension, CV_32FC1);
		gbest_track = cv::Mat(1, dimension, CV_32FC1);
		gbest_cnn = cv::Mat(1, dimension, CV_32FC1);

		pvel = cv::Mat(particle_num, dimension, CV_32FC1);
		boundary_track = cv::Mat(2, dimension, CV_32FC1);
		boundary_cnn = cv::Mat(2, dimension, CV_32FC1);
		boundary_const = cv::Mat(2, dimension, CV_32FC1);
		cost = cv::Mat(1, particle_num, CV_32FC1);//
		cost_pre = cv::Mat(1, particle_num, CV_32FC1);
		cost_dif = cv::Mat(1, particle_num, CV_32FC1);
		cost_pbest = cv::Mat(1, particle_num, CV_32FC1);
		pose_cnn = cv::Mat(1, dimension, CV_32FC1);
		cov_track = cv::Mat(1, dimension, CV_32FC1);
		cov_cnn = cv::Mat(1, dimension, CV_32FC1);
		
		collisionNumber_track = cv::Mat(2, dimension, CV_32FC1);
		collisionNumber_cnn = cv::Mat(2, dimension, CV_32FC1);

		
		truepose = cv::Mat(1, dimension, CV_32FC1);
		for (int j = 0; j < dimension; j++){
			boundary_const.at<float>(0, j) = boundary_max[0][j];
			boundary_const.at<float>(1, j) = boundary_max[1][j];
		}

		bound_dvar_max=0.883;
		bound_dvar_min=0.00001;
		

		//debugging--
		debugmatrix = cv::Mat(11, dimension, CV_32FC1);
		//debugmatrix = cv::Mat(5, dimension, CV_32FC1);
		//debugging--


		weight[0] = 2.8; weight[1] = 1.3;
		float w = weight[0] + weight[1];
		weight[2] = 2 / (abs(2 - w - sqrt(w*w - 4 * w)));

		initialize();

	}


	~PSO(){
		//empty
	}

	cv::Mat getSolution(){
		cv::Mat out = cv::Mat(1, dimension, CV_32FC1);
		
		for (int j = 0; j < dimension; j++)
			out.at<float>(0, j) = position.at<float>(0, j);

		return out;
	}

	void setGbestPre(float* insol){
		for (int j = 0; j < dimension; j++)
			gbest_pre.at<float>(0, j) = insol[j];
	}

	void getFinalSolution(float* out){
		for (int j = 0; j < dimension; j++)
			out[j] = gbest_pre.at<float>(0, j);

	}


	void initialize(){

#ifdef GYMODEL_WRIST_DEF
		float p[hand_dof] = { 0, 112, 300,
			-90, 0,
			59, 7, 0,

			0, 23, 0, 0,
			0, 7, 0, 0,
			0, 6, 0, 0,
			0, -8, 0, 0,
			0, 4, 0, 0 };
#else

		std::vector<float> h_26d;
		//h_26d.push_back(-100); h_26d.push_back(-45); h_26d.push_back(700);
		h_26d.push_back(-28); h_26d.push_back(19); h_26d.push_back(350);
		h_26d.push_back(58); h_26d.push_back(-18); h_26d.push_back(-4);
		h_26d.push_back(-19); h_26d.push_back(10); h_26d.push_back(10); h_26d.push_back(10);
		h_26d.push_back(-12); h_26d.push_back(10); h_26d.push_back(10); h_26d.push_back(10);
		h_26d.push_back(10); h_26d.push_back(10); h_26d.push_back(10); h_26d.push_back(10);
		h_26d.push_back(10); h_26d.push_back(10); h_26d.push_back(10); h_26d.push_back(10);
		h_26d.push_back(10); h_26d.push_back(10); h_26d.push_back(10); h_26d.push_back(10);
		
#endif

		for (int j = 0; j < dimension; j++)
			gbest_pre.at<float>(0, j) = h_26d[j];
		
		cv::Mat gg = gbest_pre;

		limitPosition(gbest_pre, boundary_const);
		//showTextureFromCPU();
	}

	

	
	void run(cv::Mat cam_color,cv::Mat cam_depth,float* com_hand, std::string const& type){
		
		cv::Mat cam_depth_track;
		cv::resize(cam_depth, cam_depth_track, cv::Size(_glrenderer.width, _glrenderer.height), 0, 0, CV_INTER_NN);		
		_costFunct.transferObservation2GPU(cam_depth_track);

		collisionNumber_track.setTo(0);
		collisionNumber_cnn.setTo(0);

#pragma region set pose_cnn / boundary_cnn 
		if (type == "hybrid"){

			//from manual cnn
			/*
			for (int i = 0; i < dimension; i++)
			pose_cnn.at<float>(0, i) = truepose.at<float>(0, i);

			for (int i = 0; i < 6; i++)
			pose_cnn.at<float>(0, i) += frand_gaussian(0, 5, -15, 15);
			//pose_cnn.at<float>(0, i) += -15 +rand() / double(RAND_MAX) * 30;

			for (int i = 6; i < dimension; i++){
			pose_cnn.at<float>(0, i) += frand_gaussian(0, 20, -30, 30);
			//pose_cnn.at<float>(0, i) += -bs + rand() / double(RAND_MAX) * 2*bs;
			}
			*/

			//from trained cnn.

			_mmf->receiveData();
			_mmf->getLabel(&pose_cnn.at<float>(0, 0));

			pose_cnn.at<float>(0, 0) += com_hand[0];
			pose_cnn.at<float>(0, 1) -= com_hand[1];
			pose_cnn.at<float>(0, 2) += com_hand[2];

			//limit position
			limitPosition(pose_cnn, boundary_const);
			//for (int j = 0; j < dimension;j++)
			//	printf("[%d]=%.2f\n", j, pose_cnn.at<float>(0,j));

			//show model
			showObModel("cnn",cam_color,cam_depth, &pose_cnn.at<float>(0, 0),"nosave");

			//set boundary cnn
			setBoundary_cnn();
		}
#pragma endregion
		
#pragma region initialize 
		initializeVelocity();
		initializePcost(0,particle_num);
		setBoundary_track(); //check (around gbest_param_pre)
		

		if (type == "6D"){
			initializePalm(0, particle_num, gbest_pre, boundary_track);
		}
		else if (type=="26D"){
			//initializeFingers(0, 20, gbest_pre);
			//initializePalm(20, 26, gbest_pre, boundary);
			//initializeAll(26, particle_num, gbest_pre, boundary);
			initializeAll(0, particle_num, gbest_pre, boundary_track);
			calculateCost(0);
			calculatePbest();
			calculateGbest(0, particle_num, gbest);
		}
		else if (type=="hybrid"){
			initializeAll(0, particle_num / 2, gbest_pre, boundary_track);
			initializeAll(particle_num/2, particle_num, pose_cnn, boundary_cnn);
			calculateCost(0);
			calculatePbest();
			calculateGbest(0, particle_num / 2, gbest_track);
			calculateGbest(particle_num / 2, particle_num, gbest_cnn);
			calculateGbest(0, particle_num, gbest);
		}


	//check search space from true pose.
		/*
		printf("--------tracking boundary--------\n");
		for (int j = 0; j < dimension; j++){
			float t = truepose.at<float>(0, j);
			float b0 = boundary_track.at<float>(0, j);
			float b1 = boundary_track.at<float>(1, j);

			if (t>b0 && t < b1)
				printf("[%d]= 0 0\n", j);
			else
				printf("[%d]=%.2f %.2f\n", j, t - b0, t - b1);
		}

		printf("--------cnn boundary--------\n");
		for (int j = 0; j < dimension; j++){
			float t = truepose.at<float>(0, j);
			float b0 = boundary_cnn.at<float>(0, j);
			float b1 = boundary_cnn.at<float>(1, j);

			if (t>b0 && t<b1)
				printf("[%d]= 0 0\n", j);
			else
				printf("[%d]=%.2f %.2f\n", j, t - b0, t - b1);
		}
		*/

		//showObModelParticles("initial");
#pragma endregion

		for (int g = 0; g < max_generation; g++){
			
#pragma region calculate velocity and update particles.

			if (type=="6D"){
				calculateVelocity(0, particle_num, gbest, boundary_track);
				updatePalm(0, particle_num, gbest, boundary_track);
			}
			else if (type=="26D"){
				calculateVelocity(0, particle_num, gbest, boundary_track);
				updateAll(0, particle_num, boundary_track,collisionNumber_track,"damping");
			}
			else if (type=="hybrid"){
				
#ifdef METHOD0
				calculateVelocity(0, particle_num / 2, gbest_track, boundary_track);
				updateAll(0, particle_num / 2, boundary_track, collisionNumber_track, "damping");

				calculateVelocity(particle_num / 2, particle_num, gbest_cnn, boundary_cnn);
				updateAll(particle_num / 2, particle_num, boundary_cnn, collisionNumber_cnn, "damping");
#else
				
				if (g < 5){
					calculateVelocity(0, particle_num / 2, gbest_track, boundary_track);
					updateAll(0, particle_num / 2, boundary_track, collisionNumber_track,"damping");

					calculateVelocity(particle_num / 2, particle_num, gbest_cnn, boundary_cnn);
					updateAll(particle_num / 2, particle_num, boundary_cnn, collisionNumber_cnn,"damping");
				}
				else{

					calculateVelocity(0, particle_num / 2, gbest, boundary_track);
					updateAll(0, particle_num / 2, boundary_track, collisionNumber_track,"damping");

					calculateVelocity(particle_num / 2, particle_num, gbest, boundary_cnn);
					updateAll(particle_num / 2, particle_num, boundary_cnn, collisionNumber_cnn,"damping");
				}	
#endif
			}
#pragma endregion
			
				
#pragma region calculate cost / pbest / gbest	
			
			calculateCost(g);
			calculatePbest();

			if (type == "hybrid"){
				calculateGbest(0, particle_num / 2, gbest_track);
				calculateGbest(particle_num / 2, particle_num, gbest_cnn);
				calculateGbest(0, particle_num, gbest);
			}
			else{
				calculateGbest(0, particle_num, gbest);
			}
			
#pragma endregion
			
#pragma region change boundary
	
#ifdef METHOD1
			float b_alpha0 = 0.5;
			float b_alpha1 = 0.5;

			if (g >= 5 && g % 5 == 0)
			{
				for(int j=0;j<dimension;j++){

					updateBoundary2(j, collisionNumber_track, boundary_track, b_alpha0);
					updateBoundary2(j, collisionNumber_cnn, boundary_cnn, b_alpha1);

					limitBoundary(boundary_track);
					limitBoundary(boundary_cnn);
				}

			}
#endif

#ifdef METHOD2
			
			if (type == "hybrid"){
				if (g >= 5 && g % 5 == 0)
				{
					for (int j = 0; j < dimension; j++){
						//0~particle_num/2.
						{
							float avg = calculateAverage(position, j, 0, particle_num / 2);
							float stdev = calculateSTDEV(position, g, j, avg, 0, particle_num / 2);

							//if (checkUpperBoundary(j, g, avg, stdev) == 1)
							updateUpperBoundary(j, g, avg, stdev, boundary_track);

							//if (checkLowerBoundary(j, g, avg, stdev) == 1)
							updateLowerBoundary(j, g, avg, stdev, boundary_track);

							limitBoundary(boundary_track);
						}
						//0~particle_num/2.
						{
						float avg = calculateAverage(position, j, particle_num / 2, particle_num);
						float stdev = calculateSTDEV(position, g, j, avg, particle_num / 2, particle_num);

						updateUpperBoundary(j, g, avg, stdev, boundary_cnn);
						updateLowerBoundary(j, g, avg, stdev, boundary_cnn);

						limitBoundary(boundary_cnn);
					}
					}
				}
			}
			else{
				if (g >= 5 && g % 5 == 0)
				{
					for (int j = 0; j < dimension; j++){
						//0~particle_num/2.
						{
							float avg = calculateAverage(position, j, 0, particle_num );
							float stdev = calculateSTDEV(position, g, j, avg, 0, particle_num);

							//if (checkUpperBoundary(j, g, avg, stdev) == 1)
							updateUpperBoundary(j, g, avg, stdev, boundary_track);

							//if (checkLowerBoundary(j, g, avg, stdev) == 1)
							updateLowerBoundary(j, g, avg, stdev, boundary_track);

							limitBoundary(boundary_track);
						}
					}
				}

			}
#endif



#pragma endregion

#pragma region reinitialize finger
			
			if(g % 3 == 0){

				if (type == "hybrid"){
					reinitializeFingers(0, particle_num / 4, boundary_track);
					reinitializeFingers(3 * particle_num / 4, particle_num, boundary_cnn);
				}
				else{
					reinitializeFingers(0, particle_num / 2, boundary_track);
				}
			}
			
			
			

#pragma endregion

			//if (g==max_generation-1)
			//showObModelParticles("during", "nosave", g);
#ifdef DEBUGGING
			debugParticles(g);

			cv::Mat ddimg;
			ddimg = getObModelParticles();
			showObModelParticles("during", "nosave", g);
			cv::moveWindow("during", 2000, 0);

			cv::Mat bbt = boundary_track;
			cv::Mat bbc = boundary_cnn;
			cv::Mat vv = pvel;
			printf("g:%d\n", g);
			cv::waitKey(1);

			//
			
			if (g==1)
				cv::imshow("during_pre", debug_Particleimage);
			
			/*
			while (1){
				if (cv::waitKey(100) == '0')
					break;
				if (cv::waitKey(100) == 'q')
					exit(1);
			}
			*/

			if (g==0)
				ddimg.copyTo(debug_Particleimage);
			

#endif
		}
//generation finish.
	

		//select best solution.				
		for (int j = 0; j < dimension; j++){
			position.at<float>(0, j) = gbest.at<float>(0, j);
			gbest_pre.at<float>(0, j) = gbest.at<float>(0, j);
		}
		
		//printf("wt: %.2f %.2f %.2f\n", gbest.at<float>(0, 0), gbest.at<float>(0, 1), gbest.at<float>(0, 2));
		//printf("wr: %.2f %.2f %.2f\n", gbest.at<float>(0, 3), gbest.at<float>(0, 4), gbest.at<float>(0, 5));
		//for (int i = 0; i < 5; i++)
		//	printf("F[%d] %.2f %.2f %.2f %.2f\n", i, gbest.at<float>(0, 6 + 4 * i + 0), gbest.at<float>(0, 6 + 4 * i + 1), gbest.at<float>(0, 6 + 4 * i + 2), gbest.at<float>(0, 6 + 4 * i + 3));
		
		//visualize final solution
		//showDemo(cam_color,cam_depth);
		showObModel("final",cam_color, cam_depth, &position.at<float>(0, 0),"nosave");
		
		
		

		//line up
		//cv::moveWindow("cnn", 0, 0);
		//cv::moveWindow("cnn_dif", 640, 0);
		//cv::moveWindow("final", 0, 480);
		//cv::moveWindow("final_dif", 640, 480);
		//cv::moveWindow("initial", 640 * 3, 0);

	}

	void setBoundary_track(){

		for (int j = 0; j < dimension; j++){

			if (j < 3){
				boundary_track.at<float>(0, j) = gbest_pre.at<float>(0, j) - 10;  //palm
				boundary_track.at<float>(1, j) = gbest_pre.at<float>(0, j) + 10;
			}
			else if (j >= 3 && j < fingerStartIdx){
				boundary_track.at<float>(0, j) = gbest_pre.at<float>(0, j) - 10;  //palm
				boundary_track.at<float>(1, j) = gbest_pre.at<float>(0, j) + 10;
			}
			else if (j >= fingerStartIdx){
				if ((j - fingerStartIdx) % 4 == 1){
					//boundary_track.at<float>(0, j) = boundary_max[0][j];  
					//boundary_track.at<float>(1, j) = boundary_max[1][j]; 

					boundary_track.at<float>(0, j) = gbest_pre.at<float>(0, j) - 5;
					boundary_track.at<float>(1, j) = gbest_pre.at<float>(0, j) + 5;
				}
				else{
					//boundary_track.at<float>(0, j) = boundary_max[0][j];   
					//boundary_track.at<float>(1, j) = boundary_max[1][j];  

					boundary_track.at<float>(0, j) = gbest_pre.at<float>(0, j) - 20;
					boundary_track.at<float>(1, j) = gbest_pre.at<float>(0, j) + 20;
				}
			}

			

			//check limit of boundary
			if (boundary_track.at<float>(0, j) < boundary_max[0][j])
				boundary_track.at<float>(0, j) = boundary_max[0][j];

			if (boundary_track.at<float>(1, j) > boundary_max[1][j])
				boundary_track.at<float>(1, j) = boundary_max[1][j];

			//if (boundary_track.at<float>(1, j) > boundary_max[1][j])
			//	boundary_track.at<float>(1, j) = boundary_max[1][j];
			//if (boundary_track.at<float>(1, j) <= boundary_max[0][j])
			//	boundary_track.at<float>(1, j) = boundary_max[1][j];
			
			
		}

		//cv::Mat gg = gbest_pre;
		//cv::Mat bb = boundary_track;
		//cv::waitKey(1);

	}

	void setBoundary_cnn(){

		for (int j = 0; j < dimension; j++){
			if (j < fingerStartIdx){	
				boundary_cnn.at<float>(0, j) = pose_cnn.at<float>(0, j) - 10;  //palm
				boundary_cnn.at<float>(1, j) = pose_cnn.at<float>(0, j) + 10;
			}
			else if (j >= fingerStartIdx){
				if ((j - fingerStartIdx) % 4 == 1){
					//boundary_cnn.at<float>(0, j) = boundary_max[0][j];
					//boundary_cnn.at<float>(1, j) = boundary_max[1][j];

					boundary_cnn.at<float>(0, j) = pose_cnn.at<float>(0, j) - 5;
					boundary_cnn.at<float>(1, j) = pose_cnn.at<float>(0, j) + 5;

				}
				else{
					//boundary_cnn.at<float>(0, j) = boundary_max[0][j];
					//boundary_cnn.at<float>(1, j) = boundary_max[1][j];

					boundary_cnn.at<float>(0, j) = pose_cnn.at<float>(0, j) - 20;
					boundary_cnn.at<float>(1, j) = pose_cnn.at<float>(0, j) + 20;
				}
			}

			//check limit of boundary
			if (boundary_cnn.at<float>(0, j) < boundary_max[0][j])
				boundary_cnn.at<float>(0, j) = boundary_max[0][j];

			if (boundary_cnn.at<float>(1, j) > boundary_max[1][j])
				boundary_cnn.at<float>(1, j) = boundary_max[1][j];

		}
	}


	void initializeAll(int begin_p, int end_p, cv::Mat p, cv::Mat b){

		for (int i = begin_p; i < end_p; i++)
		for (int j = 0; j < dimension; j++){
			float b0 = b.at<float>(0, j);// boundary.at<float>(j, 0);
			float b1 = b.at<float>(1, j);// boundary.at<float>(j, 1);

			if (i == end_p - 1)
				position.at<float>(i, j) = p.at<float>(0, j);// gbest_pre.at<float>(j, 0);
			else
				position.at<float>(i, j) = frand_gaussian((b0 + b1) / 2, (b1 - b0)*0.3, b0, b1);
				//position.at<float>(i, j) = (b1 - b0)*(rand() / double(RAND_MAX)) + b0;
		}

		//cv::Mat pp = position;
		//cv::waitKey(1);
	}


	void initializePalm(int begin_p, int end_p, cv::Mat p, cv::Mat b){
		for (int i = begin_p; i < end_p; i++)
		for (int j = 0; j < dimension; j++)
		{
			float b0 = b.at<float>(0, j);//boundary.at<float>(j, 0);
			float b1 = b.at<float>(1, j);// boundary.at<float>(j, 1);
			//float s = co.at<float>(j, 0);

			if (j >= fingerStartIdx)
				position.at<float>(i, j) = p.at<float>(0, j);
			else
				position.at<float>(i, j) = frand_gaussian((b0 + b1) / 2, (b1 - b0)*0.3, b0, b1);
				//position.at<float>(i, j) = (b1 - b0)*(rand() / double(RAND_MAX)) + b0;
			//position.at<float>(j, i) = frand_gaussian(p.at<float>(j, 0), s, b0, b1);
		}
	}

	void reinitializeFingers(int begin_p, int end_p, cv::Mat b){
		for (int i = begin_p; i < end_p; i++)
		{
			int r = frand_uniform(0, 20);
			int dr = 6 + r;

			float b0 = b.at<float>(0, dr);
			float b1 = b.at<float>(1, dr);
			//position.at<float>(i, dr) = frand_gaussian((b0 + b1) / 2, (b1 - b0)*0.3, b0, b1);
			position.at<float>(i, dr) = frand_uniform(b0, b1);
			//position.at<float>(i, dr) = (b1 - b0)*(rand() / double(RAND_MAX)) + b0;
		}
	}

	void initializeFingers(int begin_p, int end_p, cv::Mat p,cv::Mat b){

		for (int i = begin_p; i < end_p; i++)
		for (int j = 0; j < dimension; j++)
		{
			float b0 = b.at<float>(0, j);
			float b1 = b.at<float>(1, j);

			if (j < fingerStartIdx)
				position.at<float>(i, j) = p.at<float>(0, j);
			else
				position.at<float>(i, j) = frand_gaussian((b0 + b1) / 2, (b1 - b0)*0.3, b0, b1);
				//position.at<float>(i, j) = (b1 - b0)*(rand() / double(RAND_MAX)) + b0;
		}
	}

	void initializePcost(int ps,int pe){

		for (int i = ps; i < pe; i++){
			cost_pbest.at<float>(0, i) = FLT_MAX;
		}
	}

	void initializeVelocity(){
		for (int i = 0; i < particle_num; i++)
		for (int j = 0; j < dimension; j++){
			pvel.at<float>(i, j) = 0;
		}

	}

	void calculatePbest(){
		for (int i = 0; i < particle_num; i++){
			if (cost.at<float>(0, i) < cost_pbest.at<float>(0, i))
			{
				cost_pbest.at<float>(0, i) = cost.at<float>(0, i);
				for (int j = 0; j < dimension; j++)
					pbest.at<float>(i, j) = position.at<float>(i, j);
			}
		}
	}
	void calculateGbest(int ps, int pe, cv::Mat& out){
		float min = FLT_MAX;
		for (int i = ps; i < pe; i++){
			if (cost_pbest.at<float>(0, i) < min)
			{
				min = cost_pbest.at<float>(0, i);
				cost_gbest = min;
				for (int j = 0; j < dimension; j++)
					out.at<float>(0, j) = pbest.at<float>(i, j);

				gbestIdx = i;
			}
		}

		

	}

	void calculateVelocity(int ps, int pe, cv::Mat gmat, cv::Mat b){
		
		/*
		cv::Mat v_before;
		pvel.copyTo(v_before);
		*/

		float r1, r2;
		for (int i = ps; i < pe; i++)
		for (int j = 0; j < dimension; j++){
			//r1 = rand() / double(RAND_MAX);
			//r2 = rand() / double(RAND_MAX);
			r1 = frand_uniform(0, 1);
			r2 = frand_uniform(0, 1);
			

			float v = pvel.at<float>(i, j);
			float pb = pbest.at<float>(i, j);
			float gb = gmat.at<float>(0, j);//gbest.at<float>(j, 0);
			float p = position.at<float>(i, j);

			pvel.at<float>(i, j) = weight[2] * (v + r1*weight[0] * (pb - p) + r2*weight[1] * (gb - p));

			
			float v_max = 0.5*(b.at<float>(1, j) - b.at<float>(0, j));
			if (pvel.at<float>(i, j) > v_max)
				pvel.at<float>(i, j) = v_max;
			else if (pvel.at<float>(i, j) < -v_max)
				pvel.at<float>(i, j) = -v_max;
			
		}

		//debugging
		/*
		printf("weight:%f %f %f\n", weight[0], weight[1], weight[2]);
		cv::Mat pb = pbest;
		cv::Mat p = position;
		cv::Mat g = gmat;
		cv::Mat btrack = boundary_track;
		cv::Mat bcnn = boundary_cnn;

		cv::Mat v_after;
		pvel.copyTo(v_after);
		cv::waitKey(1);
		*/
	}

	void calculateVelocity_backup(int ps, int pe, cv::Mat gmat, cv::Mat b){
		/*
		cv::Mat v_before;
		pvel.copyTo(v_before);
		*/

		float r1, r2;
		for (int i = ps; i < pe; i++)
		for (int j = 0; j < dimension; j++){
			//r1 = rand() / double(RAND_MAX);
			//r2 = rand() / double(RAND_MAX);
			r1 = frand_uniform(0, 1);
			r2 = frand_uniform(0, 1);

			float v = pvel.at<float>(i, j);
			float pb = pbest.at<float>(i, j);
			float gb = gmat.at<float>(0, j);//gbest.at<float>(j, 0);
			float p = position.at<float>(i, j);

			pvel.at<float>(i, j) = weight[2] * (v + r1*weight[0] * (pb - p) + r2*weight[1] * (gb - p));

			/*
			float v_max = 0.5*(b.at<float>(1, j) - b.at<float>(0, j));//0.5*(boundary.at<float>(j, 1) - boundary.at<float>(j, 0));
			if (pvel.at<float>(i, j) > v_max)
				pvel.at<float>(i, j) = v_max;
			else if (pvel.at<float>(i, j) < -v_max)
				pvel.at<float>(i, j) = -v_max;
			*/
		}

		//debugging
		/*
		printf("weight:%f %f %f\n", weight[0], weight[1], weight[2]);
		cv::Mat pb= pbest;
		cv::Mat p = position;
		cv::Mat g = gmat;
		cv::Mat btrack = boundary_track;
		cv::Mat bcnn = boundary_cnn;

		cv::Mat v_after;
		pvel.copyTo(v_after);
		cv::waitKey(1);
		*/
	}

	void updateFingers(int begin_p, int end_p, cv::Mat p, cv::Mat b){

		for (int i = begin_p; i < end_p; i++)
		for (int j = 0; j < dimension; j++)
		{
			if (j < fingerStartIdx)//palm
				position.at<float>(i, j) = p.at<float>(0, j);// gbest_pre.at<float>(j, 0);
			else{
				position.at<float>(i, j) += pvel.at<float>(i, j);

				if (position.at<float>(i, j) < b.at<float>(0, j)){
					position.at<float>(i, j) = b.at<float>(0, j);
					//pvel.at<float>(i, j) = -(rand() / double(RAND_MAX))*pvel.at<float>(i, j);
					float r = frand_uniform(0, 1);
					pvel.at<float>(i, j) = -r*pvel.at<float>(i, j);

					position.at<float>(i, j) += pvel.at<float>(i, j);
				}
				else if (position.at<float>(i, j) > b.at<float>(1, j)){
					position.at<float>(i, j) = b.at<float>(1, j);
					//pvel.at<float>(i, j) = -(rand() / double(RAND_MAX))*pvel.at<float>(i, j);
					float r = frand_uniform(0, 1);
					pvel.at<float>(i, j) = -r*pvel.at<float>(i, j);

					position.at<float>(i, j) += pvel.at<float>(i, j);			
				}
			}
		}
	}

	void updatePalm(int begin_p, int end_p, cv::Mat p, cv::Mat b){

		for (int i = begin_p; i < end_p; i++)
		for (int j = 0; j < dimension; j++)
		{
			if (j >= fingerStartIdx)//finger
				position.at<float>(i, j) = p.at<float>(0, j);// gbest_pre.at<float>(j, 0);
			else//palm
				position.at<float>(i, j) += pvel.at<float>(i, j);

			if (position.at<float>(i, j) < b.at<float>(0, j))
				position.at<float>(i, j) = b.at<float>(0, j);
			else if (position.at<float>(i, j) > b.at<float>(1, j))
				position.at<float>(i, j) = b.at<float>(1, j);
		}
	}


	void updateAll(int begin_p, int end_p, cv::Mat b, cv::Mat& cs,std::string opt){

		for (int i = begin_p; i < end_p; i++)
		{
			float w = 0;
			if (cost.at<float>(0, i)< cost_pre.at<float>(0,i))
				w = 1;

			for (int j = 0; j < dimension; j++)
			{
				position.at<float>(i, j) += pvel.at<float>(i, j);

				if (position.at<float>(i, j) < b.at<float>(0, j)){

					position.at<float>(i, j) = b.at<float>(0, j);
				
					if (opt.compare("damping") == 0){
						float r = frand_uniform(0, 1);
						pvel.at<float>(i, j) = -r*pvel.at<float>(i, j);
						position.at<float>(i, j) += pvel.at<float>(i, j);
					}
					
					cs.at<float>(0, j) += 1;
					//cs.at<float>(0, j) += w;
				}
				else if (position.at<float>(i, j) > b.at<float>(1, j)){

					position.at<float>(i, j) = b.at<float>(1, j);

					
					if (opt.compare("damping") == 0){
						float r = frand_uniform(0, 1);
						pvel.at<float>(i, j) = -r*pvel.at<float>(i, j);
						position.at<float>(i, j) += pvel.at<float>(i, j);
					}


					cs.at<float>(1, j) += 1;
					//cs.at<float>(1, j) += w;
				}
			}
		}

	}


	
	void updateAll_backup(int begin_p, int end_p,cv::Mat b){
		for (int i = begin_p; i < end_p; i++)
		for (int j = 0; j < dimension; j++)
		{
			position.at<float>(i, j) += pvel.at<float>(i, j);

			if (position.at<float>(i, j) < b.at<float>(0, j)){
				position.at<float>(i, j) = b.at<float>(0, j);
				
			}
			else if (position.at<float>(i, j) > b.at<float>(1, j)){
				position.at<float>(i, j) = b.at<float>(1, j);
				
			}
		}
	}

	void calculateCost(int g){
		
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		for (int px = 0; px < particle_numx; px++)
		for (int py = 0; py < particle_numy; py++)
		{

			float* solp = &position.at<float>(px + py*particle_numx,0 );
			_renderer->renderTile(px, py, solp, "depth");
		}
		glFinish();
		
		//data term
		_costFunct.calculateCost(cost);
		
		//cnn pose term
#ifdef METHOD3
		for (int i = 0; i < particle_num; i++)
		{
			float cost_pose = 0;
			for (int j = 6; j < dimension; j++)
				cost_pose += abs(pose_cnn.at<float>(0, j) - position.at<float>(i, j)) / (boundary_max[1][j] - boundary_max[0][j]);
				
			cost_pose /= (dimension - 6);
			cost.at<float>(0, i) += cost_pose;
		}
#endif
		
		
	}

	

	void showTextureFromCPU()
	{
		cv::Mat model_depth;
		_glrenderer.getDepthTexture(model_depth);
		cv::imshow("textureCPU", model_depth);
		cv::waitKey(1);
	}

	void showModel(char* wname,float* solp){
		_renderer->renderOrig(solp, "color");

		cv::Mat model_color;
		_glrenderer.getOrigImage(model_color, "color");

		cv::imshow(wname, model_color);
		cv::waitKey(1);
	}

	void showObModel(std::string wname,cv::Mat cam_color,cv::Mat cam_depth,float* solp,std::string opt)
	{

		//--color
		//float* solp = &position.at<float>(0, 0);
		/*
		_renderer->renderOrig(solp, "color");

		cv::Mat model_color;
		_glrenderer.getOrigImage(model_color, "color");

		cv::Mat fimg;
		cv::addWeighted(cam_color, 0.5, model_color, 1.0, 0, fimg);
		cv::imshow(wname, fimg);
		*/

		//-- depth
		_renderer->renderOrig(solp, "depth");
		cv::Mat model_depth;
		_glrenderer.getOrigImage(model_depth, "depth");

		//color code visualization
		cv::Mat mask = model_depth >0;
		double min;
		double max;
		cv::minMaxIdx(model_depth, &min, &max, NULL, NULL, mask);

		cv::Mat adjMap;
		model_depth.convertTo(adjMap, CV_8UC1, 255 / (max - min), -min);

		cv::Mat falseColorsMap;
		applyColorMap(adjMap, falseColorsMap, 2);
		cv::imshow("result", falseColorsMap);

		cv::Mat fimg;
		cv::addWeighted(cam_color, 0.5, falseColorsMap, 1.0, 0, fimg);
		cv::imshow(wname, fimg);


		//hard coding for debugging.
		/*
		cv::Mat difdepth = model_depth - cam_depth;
		cv::Mat difdepth8u = cv::Mat(480, 640, CV_8UC3);
		difdepth8u.setTo(0);
		for (int i = 0; i < 640;i++)
		for (int j = 0; j < 480; j++)
			difdepth.at<float>(j, i) = abs(difdepth.at<float>(j, i));

		cv::normalize(difdepth, difdepth8u, 0, 255, cv::NORM_MINMAX, CV_8UC3);
		cv::imshow(wname+"_dif", difdepth8u);
		cv::waitKey(1);
		*/

		if (opt.compare("save") == 0)
		{
			//result image
			/*
			char filename[200];
			sprintf(filename, "experiment/%d/result-%07u.png", experimentID,_frame);
			cv::imwrite(filename, fimg);
			*/

			//result cost
			/*
			FILE* fp;
			char filename1[200];
			sprintf(filename, "experiment/method0_difError%d.csv", experimentID);
			fp = fopen(filename, "a");
			
			char str[100];

			float c_dif = 0;
			int count = 0;
			for (int i = 0; i < 640;i++)
			for (int j = 0; j < 480; j++){
				float o = cam_depth.at<float>(j, i);
				
				if (o == 0)
					continue;

				float m = model_depth.at<float>(j, i);
				c_dif += abs(m - o);
				count++;
			}
		
			c_dif /= count;

			sprintf(str, "%.2f\n", c_dif);
			fputs(str, fp);
			fclose(fp);
			*/
		}

	}

	cv::Mat getObModelParticles()
	{
		//model
		cv::Mat model_depth;
		_glrenderer.getDepthTexture(model_depth);

		//observation
		cv::Mat omat = _costFunct.getTiledImages();

		int w = omat.size().width;
		int h = omat.size().height;

		//visualize 3color
		cv::Mat o_3c = cv::Mat(h, w, CV_8UC3);
		cv::Mat m_3c = cv::Mat(h, w, CV_8UC3);
		cv::Mat om_3c = cv::Mat(h, w, CV_8UC3);
		o_3c.setTo(0);
		m_3c.setTo(0);


		for (int i = 0; i < w; i++)
		for (int j = 0; j < h; j++){
			if (model_depth.at<float>(j, i) != 0)
			{
				m_3c.at <unsigned char>(j, 3 * i + 0) = 0;
				m_3c.at <unsigned char>(j, 3 * i + 1) = 0;
				m_3c.at <unsigned char>(j, 3 * i + 2) = 255;
			}
			if (omat.at<float>(j, i) != 0)
			{
				o_3c.at <unsigned char>(j, 3 * i + 0) = 255;
				o_3c.at <unsigned char>(j, 3 * i + 1) = 0;
				o_3c.at <unsigned char>(j, 3 * i + 2) = 0;
			}
		}
		cv::addWeighted(o_3c, 0.5, m_3c, 0.5, 0, om_3c);

		//write the likelihood as text
		int w_one = w / particle_numx;
		int h_one = h / particle_numy;
		CvFont font;
		cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX | CV_FONT_ITALIC, 0.4, 0.4, 0, 1);
		char text_c[100];
		for (int i = 0; i < particle_numx; i++)
		for (int j = 0; j < particle_numy; j++){

			sprintf(text_c, "%.3f", cost.at<float>(0, i + particle_numx*j));
			cv::putText(om_3c, text_c, cvPoint(w_one*i + 20, h_one*j + 30), 1, 2, cv::Scalar(255, 255, 0));

			//sprintf(text_c, "l:%.4f", weight[m].at<float>(0, i + particle_numx*j));
			//cv::putText(om_3c, text_c, cvPoint(w_one*i + 20, h_one*j + 30), 1, 1, cv::Scalar(255, 255, 0));
		}

		return om_3c;

	}

	cv::Mat showObModelParticles(const char* wname,std::string saveopt,int g)
	{
		//model
		cv::Mat model_depth;
		_glrenderer.getDepthTexture(model_depth);

		//observation
		cv::Mat omat = _costFunct.getTiledImages();

		int w = omat.size().width;
		int h = omat.size().height;

		//visualize 3color
		cv::Mat o_3c = cv::Mat(h, w, CV_8UC3);
		cv::Mat m_3c = cv::Mat(h, w, CV_8UC3);
		cv::Mat om_3c = cv::Mat(h, w, CV_8UC3);
		o_3c.setTo(0);
		m_3c.setTo(0);
		

		for (int i = 0; i < w; i++)
		for (int j = 0; j < h; j++){
			if (model_depth.at<float>(j, i) != 0)
			{
				m_3c.at <unsigned char>(j, 3 * i + 0) = 0;
				m_3c.at <unsigned char>(j, 3 * i + 1) = 0;
				m_3c.at <unsigned char>(j, 3 * i + 2) = 255;
			}
			if (omat.at<float>(j, i) != 0)
			{
				o_3c.at <unsigned char>(j, 3 * i + 0) = 255;
				o_3c.at <unsigned char>(j, 3 * i + 1) = 0;
				o_3c.at <unsigned char>(j, 3 * i + 2) = 0;
			}
		}
		cv::addWeighted(o_3c, 0.5, m_3c, 0.5, 0, om_3c);

		cv::resize(om_3c, om_3c,cv::Size(1024, 1024), 0, 0,1);

		//write the likelihood as text
		int w_one = om_3c.size().width / particle_numx;
		int h_one = om_3c.size().height / particle_numy;
		//int w_one = w / particle_numx;
		//int h_one = h / particle_numy;
		CvFont font;
		cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX | CV_FONT_ITALIC, 0.4, 0.4, 0, 1);
		char text_c[100];
		for (int i = 0; i < particle_numx; i++)
		for (int j = 0; j < particle_numy; j++){

	
			sprintf(text_c, "%.3f", cost.at<float>(0, i + particle_numx*j));
			cv::putText(om_3c, text_c, cvPoint(w_one*i + 20, h_one*j + 30), 1, 2, cv::Scalar(255, 255, 0));

			//sprintf(text_c, "l:%.4f", weight[m].at<float>(0, i + particle_numx*j));
			//cv::putText(om_3c, text_c, cvPoint(w_one*i + 20, h_one*j + 30), 1, 1, cv::Scalar(255, 255, 0));
		}

		//cv::Mat hoho = om_3c;
		//cv::waitKey(1);

		cv::imshow(wname, om_3c);
		//cv::Mat om_3c_resized;
		//cv::resize(om_3c, om_3c_resized, cv::Size(896, 896), 0, 0, 1);
		//cv::imshow(wname, om_3c_resized);




		//save//
		if (saveopt.compare("save") == 0){
			char filename[100];
			sprintf(filename, "savepso/%d.png", g);
			cv::imwrite(filename, om_3c);

		}
		

		

		return om_3c;
	}


	void updateResource(){

	}

	void getTruePose(float* out){
		for (int i = 0; i < dimension; i++)
			out[i] = truepose.at<float>(0, i);

	}
	void setTruePose(float* in){

		for (int i = 0; i < dimension; i++)
			truepose.at<float>(0, i) = in[i];

		cv::Mat tt = truepose;
		limitPosition(truepose, boundary_const);
		
	}

	void limitPosition(cv::Mat& in,cv::Mat b){
		for (int i = 0; i < dimension; i++)
		{
			if (in.at<float>(0, i) < b.at<float>(0, i))
				in.at<float>(0, i) = b.at<float>(0, i);

			if (in.at<float>(0, i) > b.at<float>(1, i))
				in.at<float>(0, i) = b.at<float>(1, i);
		}
	}


	float calculateAverage(cv::Mat in,int jid,int ps,int pe)
	{
		float avg = 0;
		for (int i = ps;i<pe;i++)
		{
			avg+=in.at<float>(i, jid);
		}
		avg /= (pe - ps);

		return avg;
	}

	float calculateSTDEV(cv::Mat in, int g,int jid, int avg, int ps, int pe){
		/*
		float s = 0;
		for (int i = ps; i < pe; i++){
			float p = in.at<float>(i, jid);
			s += p*p;
		}
		s /= (pe - ps);

		float stdev = sqrt(s - avg*avg);
		*/
		float best = gbest.at<float>(0, jid);
		float a = bound_dvar_min + g*(bound_dvar_max - bound_dvar_min) / max_generation;

		float stdev = sqrt(-(best - avg)*(best - avg) / (2 * log(a)));

		return stdev;
	}

	int checkUpperBoundary(int jid,int g,float avg,float stdev)
	{
		float best = gbest.at<float>(0, jid);
		//float a = exp(-(best - avg)*(best - avg) / (2*stdev*stdev));
		float a = bound_dvar_min + g*(bound_dvar_max - bound_dvar_min) / max_generation;

		if (avg + sqrt(-2 * stdev*stdev*log(a)) < best)
			return 1;

		return 0;
	}

	//modified updateBoundary (according to gbest)
	void updateUpperBoundary(int jid,int g,float avg,float stdev,cv::Mat& bmat)
	{	
		float best = gbest.at<float>(0, jid);

		float a = bound_dvar_min + g*(bound_dvar_max - bound_dvar_min) / max_generation;
		float newstdev = sqrt(-(best - avg)*(best - avg) / (2 * log(a)));
		//printf("(upper)newstdev:%f\n", newstdev);

		if (newstdev < bound_stdev[jid])
			newstdev = bound_stdev[jid];
		

		//new boundary
		bmat.at<float>(1,jid) = avg + sqrt(-2 * newstdev*newstdev*log(a));
	}

	int checkLowerBoundary(int jid, int g, float avg, float stdev)
	{
		float best = gbest.at<float>(0, jid);
		//float a = exp(-(best - avg)*(best - avg) / (2 * stdev*stdev));
		float a = bound_dvar_min + g*(bound_dvar_max - bound_dvar_min) / max_generation;

	
		if (best < avg - sqrt(-2 * stdev*stdev*log(a)))
			return 1;

		return 0;
	}

	void updateLowerBoundary(int jid,int g, float avg, float stdev, cv::Mat& bmat)
	{
		float best = gbest.at<float>(0, jid);

		float a = bound_dvar_min + g*(bound_dvar_max - bound_dvar_min) / max_generation;
		float newstdev = sqrt(-(best - avg)*(best - avg) / (2 * log(a)));
		//printf("(lower)newstdev:%f\n", newstdev);

		if (newstdev < bound_stdev[jid])
			newstdev = bound_stdev[jid];
		
		//new boundary
		bmat.at<float>(0, jid) = avg - sqrt(-2 * newstdev*newstdev*log(a));
	}

	void limitBoundary(cv::Mat& b){
		for (int j = 0; j < dimension; j++){
			if (b.at<float>(0, j) < boundary_max[0][j])
				b.at<float>(0, j) = boundary_max[0][j];

			if (b.at<float>(1, j) > boundary_max[1][j])
				b.at<float>(1, j) = boundary_max[1][j];
		}
	}

	// original updateBoundary (change specific dimension)
	void updateBoundary2(int j,cv::Mat& coll,cv::Mat& bmat,float alpha){

		float pitr = 10.0;// bound_pitr;// 5.0;
		int T1 = 1;// bound_T1;      // 1;
		int T2 = 2;// T1 * 2;        // 2;

		//--adapt boundary--//
		//float alpha = 0.5;

		float bl = bmat.at<float>(0, j);
		float bu = bmat.at<float>(1, j);

		float mlow = coll.at<float>(0, j);
		float mup = coll.at<float>(1, j);

		//lower boundary.
		if (mlow==0 || mlow / pitr < T1){
			bmat.at<float>(0, j) += alpha*(bu - bl);
			coll.at<float>(0, j) = 0;
		}
		else if (mlow / pitr > T2){
			bmat.at<float>(0, j) -= alpha*(bu - bl);
			coll.at<float>(0, j) = 0;
		}

		//upper boundary.
		if (mup==0 || mup / pitr < T1){
			bmat.at<float>(1, j) -= alpha*(bu - bl);
			coll.at<float>(1, j) = 0;
		}
		else if (mup / pitr > T2){
			bmat.at<float>(1, j) += alpha*(bu - bl);
			coll.at<float>(1, j) = 0;
		}
		

		// keep the boundary when the gap between lower and upper one is too narrowed.
		float bl2 = bmat.at<float>(0, j);
		float bu2 = bmat.at<float>(1, j);

		float bminconst = boundary_const.at<float>(0, j);
		float bmaxconst = boundary_const.at<float>(1, j);

		if ((bl2 >= bu2) || abs(bl2 - bu2)< 5)
		{
			bmat.at<float>(0, j) = bl;
			bmat.at<float>(1, j) = bu;
		}

		//check limit of boundary
		if (bmat.at<float>(0, j) < boundary_max[0][j])
			bmat.at<float>(0, j) = boundary_max[0][j];

		if (bmat.at<float>(1, j) > boundary_max[1][j])
			bmat.at<float>(1, j) = boundary_max[1][j];

	}

	//original updateBoundary (update all dimensions)
	void updateBoundary(int g, cv::Mat& coll, cv::Mat gmat, cv::Mat& bmat, int opt){

		float pitr = bound_pitr;// 5.0;
		int T1 = bound_T1;      // 1;
		int T2 = T1 * 2;        // 2;

		cv::Mat bt_before;
		bmat.copyTo(bt_before);

		//adapt boundary	
		float alpha = 0.5;

		for (int j = 0; j < dimension; j++){
			if (j >= fingerStartIdx && (j - fingerStartIdx) % 4 == 1)
				continue;


			float bl = bmat.at<float>(0, j);
			float bu = bmat.at<float>(1, j);

			float mlow = coll.at<float>(0, j);
			float mup = coll.at<float>(1, j);
			
			//lower boundary.
			if (mlow / pitr < T1){	
				bmat.at<float>(0, j) += alpha*(bu - bl);
	
			}
			if (mlow / pitr > T2){				
				bmat.at<float>(0, j) -= alpha*(bu - bl);
		
			}

			//upper boundary.
			if (mup / pitr > T2){	
				bmat.at<float>(1, j) += alpha*(bu - bl);

			}
			if (mup / pitr < T1){
				bmat.at<float>(1, j) -= alpha*(bu - bl);

			}

			// keep the boundary when the gap between lower and upper one is too narrowed.
			
			float bl2 = bmat.at<float>(0, j);
			float bu2 = bmat.at<float>(1, j);

		
			float bminconst = boundary_const.at<float>(0, j);
			float bmaxconst = boundary_const.at<float>(1, j);
			
			if ((bl2>=bu2) || abs(bl2-bu2)< 5)
			{
				bmat.at<float>(0, j) = bl;
				bmat.at<float>(1, j) = bu;
			}
			
			//check limit of boundary
			if (bmat.at<float>(0, j) < boundary_max[0][j])
				bmat.at<float>(0, j) = boundary_max[0][j];

			if (bmat.at<float>(1, j) > boundary_max[1][j])
				bmat.at<float>(1, j) = boundary_max[1][j];
			

		}


		
		/*
		cv::Mat pp;
		position.copyTo(pp);


		cv::Mat ppgbest = gbest;
		cv::Mat pptrue = truepose;

		cv::Mat bbtrack = boundary_track;
		cv::Mat bbcnn = boundary_cnn;
		cv::Mat colltrack = collisionNumber_track;
		cv::Mat collcnn = collisionNumber_cnn;

		cv::Mat vv = pvel;

			
		for (int j = 0; j < dimension; j++){
			debugmatrix.at<float>(0, j) = colltrack.at<float>(0, j);
			debugmatrix.at<float>(1, j) = bbtrack.at<float>(0, j) - truepose.at<float>(0, j);
			debugmatrix.at<float>(2, j) = gbest.at<float>(0, j) - truepose.at<float>(0, j);
			debugmatrix.at<float>(3, j) = bbtrack.at<float>(1, j) - truepose.at<float>(0, j);
			debugmatrix.at<float>(4, j) = colltrack.at<float>(1, j);
				
			debugmatrix.at<float>(5, j) = 0;

			debugmatrix.at<float>(6, j) = collcnn.at<float>(0, j);
			debugmatrix.at<float>(7, j) = bbcnn.at<float>(0, j) - truepose.at<float>(0, j);
			debugmatrix.at<float>(8, j) = gbest.at<float>(0, j) - truepose.at<float>(0, j);
			debugmatrix.at<float>(9, j) = bbcnn.at<float>(1, j) - truepose.at<float>(0, j);
			debugmatrix.at<float>(10, j) = collcnn.at<float>(1, j) ;
				
				
		}	
		if (opt == 1){
			cv::Mat dd = debugmatrix;

			cv::Mat img;
			img = getObModelParticles();
			cv::waitKey(1);
		}
		*/

		coll.setTo(0);	
	}

	

	void debugParticles(int g){
		
		

		cv::Mat sortid;
		cv::Mat cost0 = cv::Mat(1, particle_num / 2, CV_32FC1);
		cv::Mat cost1 = cv::Mat(1, particle_num / 2, CV_32FC1);
		for (int i = 0; i < particle_num/2; i++)
			cost0.at<float>(0, i) = cost.at<float>(0, i);
		for (int i = particle_num/2; i < particle_num; i++)
			cost1.at<float>(0, i-particle_num/2) = cost.at<float>(0, i);

		cv::sortIdx(cost0, sortid, CV_SORT_EVERY_ROW + CV_SORT_ASCENDING);
		int sortidgood0 = sortid.at<int>(0, 0);

		cv::sortIdx(cost1, sortid, CV_SORT_EVERY_ROW + CV_SORT_ASCENDING);
		int sortidgood1 = sortid.at<int>(0, 0);

		int imgwpre=0;
		int imgw = 0;
		int w = 0;
		int dw = 0;
		
		for (int j = 0; j < dimension; j++){
			/*
			imgw = 20 + boundary_max[1][j] - boundary_max[0][j]; //300;
			w = boundary_max[1][j] - boundary_max[0][j];//256;
			dw = 20;
			*/
			imgw = 300;
			w = 256;
			dw = 20;

			cv::Mat img = cv::Mat(particle_num, imgw, CV_8UC3);
			img.setTo(0);

			
			int jid = j;

			float xtrue = truepose.at<float>(0, jid);
			float b_track_min;
			float b_track_max;
			float b_cnn_min;
			float b_cnn_max;
			float bmin;
			float bmax;

			for (int i = 0; i < particle_num; i++)
			{
				
				float x = position.at<float>(i, jid);
				xtrue = truepose.at<float>(0, jid);
				b_track_min = boundary_track.at<float>(0, jid);
				b_track_max = boundary_track.at<float>(1, jid);
				b_cnn_min = boundary_cnn.at<float>(0, jid);
				b_cnn_max = boundary_cnn.at<float>(1, jid);

				bmin = boundary_max[0][jid];
				bmax = boundary_max[1][jid];

				xtrue = w * (xtrue - bmin) / (bmax - bmin);
				x =  w * (x - bmin) / (bmax - bmin);
				b_track_min =  w * (b_track_min - bmin) / (bmax - bmin);
				b_track_max =  w * (b_track_max - bmin) / (bmax - bmin);
				b_cnn_min =  w * (b_cnn_min - bmin) / (bmax - bmin);
				b_cnn_max =  w * (b_cnn_max - bmin) / (bmax - bmin);

				//true
				img.at<uchar>(i, 3 * int(xtrue) + 0) = 255;
				img.at<uchar>(i, 3 * int(xtrue) + 1) = 255;
				img.at<uchar>(i, 3 * int(xtrue) + 2) = 255;
				
				//boundary_track
				img.at<uchar>(i / 5, 3 * int(b_track_min) + 0) = 255;
				img.at<uchar>(i / 5, 3 * int(b_track_min) + 1) = 0;
				img.at<uchar>(i / 5, 3 * int(b_track_min) + 2) = 0;


				img.at<uchar>(i / 5, 3 * int(b_track_max) + 0) = 0;
				img.at<uchar>(i / 5, 3 * int(b_track_max) + 1) = 0;
				img.at<uchar>(i / 5, 3 * int(b_track_max) + 2) = 255;

				//boundary_cnn
				img.at<uchar>(particle_num / 2 + i / 5, 3 * int(b_cnn_min) + 0) = 255;
				img.at<uchar>(particle_num / 2 + i / 5, 3 * int(b_cnn_min) + 1) = 0;
				img.at<uchar>(particle_num / 2 + i / 5, 3 * int(b_cnn_min) + 2) = 0;

				img.at<uchar>(particle_num / 2 + i / 5, 3 * int(b_cnn_max) + 0) = 0;
				img.at<uchar>(particle_num / 2 + i / 5, 3 * int(b_cnn_max) + 1) = 0;
				img.at<uchar>(particle_num / 2 + i / 5, 3 * int(b_cnn_max) + 2) = 255;

				//particle
				
				if (i == sortidgood0 || i==(particle_num/2+sortidgood1)){
					
					if (i==gbestIdx)
						cv::circle(img, cv::Point(x, i), 3, cv::Scalar(255, 255, 255), 1, 8, 0);
					else
						cv::circle(img, cv::Point(x, i), 3, cv::Scalar(0, 255, 255), 1, 8, 0);
				}
				else{
					img.at<uchar>(i, 3 * int(x) + 0) = 0;
					img.at<uchar>(i, 3 * int(x) + 1) = 255;
					img.at<uchar>(i, 3 * int(x) + 2) = 0;
				}
					
				
			}

			//write the number of collision 
				//tracking particle's collision 
			{
				char text_c[100];
				sprintf(text_c, "%d", int(collisionNumber_track.at<float>(0, j)));
				cv::putText(img, text_c, cvPoint(0 ,10 ), 1, 1.0, cv::Scalar(255, 255, 0));
				
				sprintf(text_c, "%d", int(collisionNumber_track.at<float>(1, j)));
				cv::putText(img, text_c, cvPoint(w-5, 10), 1, 1.0, cv::Scalar(255, 255, 0));
			}
				//cnn particle's collision 
			{
				char text_c[100];
				sprintf(text_c, "%d", int(collisionNumber_cnn.at<float>(0, j)));
				cv::putText(img, text_c, cvPoint(0, 50), 1, 1.0, cv::Scalar(255, 255, 0));

				sprintf(text_c, "%d", int(collisionNumber_cnn.at<float>(1, j)));
				cv::putText(img, text_c, cvPoint(w-5, 50), 1, 1.0, cv::Scalar(255, 255, 0));
			}


			//show and arrange window
			char wname[100];
			sprintf(wname, "dim%d", j);
			cv::imshow(wname, img);
		
			
			if (j<fingerStartIdx)
				cv::moveWindow(wname, (imgwpre + dw), 0);
			else if (j>=6 && j < 10)
				cv::moveWindow(wname, (imgwpre + dw), 100);
			else if (j>=10 && j < 14)
				cv::moveWindow(wname, (imgwpre + dw), 200);
			else if (j>=14 && j < 18)
				cv::moveWindow(wname, (imgwpre + dw), 300);
			else if (j>=18 && j < 22)
				cv::moveWindow(wname, (imgwpre + dw), 400);
			else if (j>=22 && j < 26)
				cv::moveWindow(wname, (imgwpre + dw), 500);
			
			imgwpre += imgw;
			if (j == 5 || j == 9 || j == 13 || j == 17 || j == 21)
				imgwpre = 0;

			cv::waitKey(1);

			//result image 
			char filename[200];
			sprintf(filename, "experiment/result%d_%d.png",j,g);
			cv::imwrite(filename, img);
		}

	}

	void saveImage(cv::Mat cam_color, int frame){

		float* solp = &position.at<float>(0, 0);

		//-- depth
		_renderer->renderOrig(solp, "depth");
		cv::Mat model_depth;
		_glrenderer.getOrigImage(model_depth, "depth");

		//color code visualization
		cv::Mat mask = model_depth >0;
		double min;
		double max;
		cv::minMaxIdx(model_depth, &min, &max, NULL, NULL, mask);

		cv::Mat adjMap;
		model_depth.convertTo(adjMap, CV_8UC1, 255 / (max - min), -min);

		cv::Mat falseColorsMap;
		applyColorMap(adjMap, falseColorsMap, 2);
		
		cv::Mat fimg;
		cv::addWeighted(cam_color, 0.5, falseColorsMap, 1.0, 0, fimg);
		
		char filename[200];
		sprintf(filename, "save/sequence/uvrResult/result-%07u.png", frame);
		cv::imwrite(filename, fimg);
	}
	void saveJoints(cv::Mat cam_color,int  frame){

		//label (26D pose)
		/*
		{
			FILE* fp;
			char filenamel[200];
			sprintf(filenamel, "save/sequence/label/label26D_%d.txt",frame);
			fp = fopen(filenamel, "w");

			char str[100];
			float jpos[26];
			for (int j = 0; j < dimension; j++)
				jpos[j] = gbest.at<float>(0, j);

			for (int i = 0; i < 26; i++)
			{

				sprintf(str, "%.2f\n", jpos[i]);

				fputs(str, fp);
			}
			fclose(fp);
		}
		*/

		//fingertip (3*5 D)
		{
			
			//get all joints
			std::vector<float> jpos;
			_renderer->hand.GetJointAllPosition(&jpos);

			//set joint index for fingertip.
			std::vector<int> jidx;
			jidx.push_back(4); jidx.push_back(9); jidx.push_back(14); jidx.push_back(19); jidx.push_back(24);//

			string fname = "save/sequence/uvr.csv";
			static ofstream file1(fname);

			//save joints	
			for (int k = 0; k < jidx.size(); k++){
				int i = jidx[k];

				float x = jpos[3 * i + 0];
				float y = jpos[3 * i + 1];
				float z = jpos[3 * i + 2];

				char str[100];
				if (k == (jidx.size() - 1))
					file1 << x << "," << y << "," << z << endl;
				else
					file1 << x << "," << y << "," << z << ",";
			}
			

			//visualize joint as circle
			/*
			float cal[9] = { 477.9 , 0, 320 ,
			0, 477.9, 240,
			0, 0, 1 };

			int color[5][3] = { { 255, 0, 0 }, { 0, 255, 0 }, { 0, 0, 255 }, { 255, 255, 0 }, { 255, 255, 255 } };
			for (int i = 0; i < jidx.size(); i++)
			{
				int jid = jidx[i];

				float x = jpos[3 * jid + 0];
				float y = jpos[3 * jid + 1];
				float z = jpos[3 * jid + 2];

				float x_ = x * cal[0] + y * cal[1] + z * cal[2];
				float y_ = x * cal[3] + y * cal[4] + z * cal[5];
				float z_ = x * cal[6] + y * cal[7] + z * cal[8];
				x_ /= z_;
				y_ /= z_;

				cv::circle(cam_color, cv::Point(x_, y_), 10, cv::Scalar(255, 0, 0), 1, 8, 0);
			}
			cv::imshow("predicted", cam_color);
			*/
			
		}
	}

	//variable

	HandGenerator* _renderer;
	CostFunction _costFunct;
	GLRenderer _glrenderer;
	MMF* _mmf;
	//PoseSpace _posespace;

	int particle_numx;
	int particle_numy;
	int particle_num;
	int max_generation;
	int dimension;
	float weight[3];
	


	cv::Mat position;
	cv::Mat pbest;
	
	cv::Mat gbest_pre;
	cv::Mat gbest_track;
	cv::Mat gbest_cnn;
	
	cv::Mat pvel;
	cv::Mat boundary_track, boundary_cnn, boundary_const;
	cv::Mat cost;//
	cv::Mat cost_pre;
	cv::Mat cost_dif;//

	cv::Mat cost_pbest;
	cv::Mat pose_cnn;
	float cost_gbest;
	float cost_gbest_pre;
	cv::Mat cov_track;
	cv::Mat cov_cnn;

	
	double miliSec;
	//Watch watch;

	int fingerStartIdx;

	float com_hand[3];

	cv::Mat collisionNumber_track;
	cv::Mat collisionNumber_cnn;
	//cv::Mat position_avg;
	//cv::Mat position_stdev;

	cv::Mat truepose;

	cv::Mat debugmatrix;
	cv::Mat debug_Particleimage;

	//hyper parameter for floating boundary
	float bound_alpha0;
	float bound_alpha1;
	int experimentID;
	int _frame;

	float bound_pitr;
	int bound_T1;
	int bound_T2;

	float bound_dvar_max;
	float bound_dvar_min;
	

	int gbestIdx;

	//random
	std::mt19937 mutable gen_gaussian; //random geneartor (gaussian)
	std::mt19937 mutable gen_uniform;

	float frand_gaussian(float a, float std, float lo, float hi) const{
		std::normal_distribution<float> distribution(0, std);

		while (true) {
			float number = distribution(gen_gaussian);
			float result = a + number*(hi - lo);

			if (result >= lo && result <= hi)
				return result;
		}
	}

	float frand_uniform(float lo, float hi) const{
	
		std::uniform_real_distribution<float> distribution(lo, hi);

		
		float result=distribution(gen_uniform);

		return result;	
	}


	void stopWatch()
	{
		static DWORD start = 0.0;
		static DWORD end = 0.0;

		static bool bCheckTime = false;

		if (!bCheckTime) {
			start = GetTickCount();
		}
		if (bCheckTime) {
			end = GetTickCount();
			printf("       Time  : %lf \n", (end - start) / (double)1000);
		}

		bCheckTime = !bCheckTime;

	}

};

