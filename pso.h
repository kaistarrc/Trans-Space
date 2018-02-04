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


#define SYNTHETIC_TEST


#pragma comment(lib,"opencv_world310.lib")

class PSO{

public:
	cv::Mat gbest;
	

	PSO(int p_numX, int p_numY, int g_num, int dim,
		HandGenerator* renderer,GLRenderer& glrenderer,CostFunction& costFunct,MMF* mmf)
	{
		_renderer = renderer;
		_costFunct = costFunct;
		_glrenderer = glrenderer;
		_mmf = mmf;
		//_posespace = posespace;

		//random
		std::random_device rand_dev;
		gen_gaussian = new std::mt19937(rand_dev());

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
		gbest_part[0] = cv::Mat(1, dimension, CV_32FC1);
		gbest_part[1] = cv::Mat(1, dimension, CV_32FC1);
		gbest_pre = cv::Mat(1, dimension, CV_32FC1);
		
		pvel = cv::Mat(particle_num, dimension, CV_32FC1);
		boundary_track = cv::Mat(2, dimension, CV_32FC1);
		boundary_cnn = cv::Mat(2, dimension, CV_32FC1);
		boundary_const = cv::Mat(2, dimension, CV_32FC1);
		cost = cv::Mat(1, particle_num, CV_32FC1);//
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

		//debugging--
		debugmatrix = cv::Mat(5, dimension, CV_32FC1);
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
		h_26d.push_back(5); h_26d.push_back(-45); h_26d.push_back(250);
		h_26d.push_back(98); h_26d.push_back(6); h_26d.push_back(-4);
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

	

	
	void run(cv::Mat cam_color,cv::Mat cam_depth,float* com_hand, std::string type){

		cv::Mat cam_depth_track;
		cv::resize(cam_depth, cam_depth_track, cv::Size(128, 128), 0, 0, CV_INTER_NN);		
		_costFunct.transferObservation2GPU(cam_depth_track);

		collisionNumber_track.setTo(0);
		collisionNumber_cnn.setTo(0);

#pragma region set pose_cnn / boundary_cnn 
		if (type.compare("hybrid") == 0){
			//from trained cnn.
			
			_mmf->receiveData();
			_mmf->getLabel(&pose_cnn.at<float>(0, 0));
			pose_cnn.at<float>(0, 0) += com_hand[0];
			pose_cnn.at<float>(0, 1) -= com_hand[1];
			pose_cnn.at<float>(0, 2) += com_hand[2];

			
			//from input trackbar.
			/*
			for (int i = 0; i < 6; i++)
				pose_cnn.at<float>(0, i) = _renderer->_posesetgenerator._trackbar.wval[i];

			for (int i = 0; i < 5; i++)
			{
				pose_cnn.at<float>(0, 6 + 4 * i + 0) = _renderer->_posesetgenerator._trackbar.fval[3 * i + 0][0];
				pose_cnn.at<float>(0, 6 + 4 * i + 1) = _renderer->_posesetgenerator._trackbar.fval[3 * i + 0][2];
				pose_cnn.at<float>(0, 6 + 4 * i + 2) = _renderer->_posesetgenerator._trackbar.fval[3 * i + 1][0];
				pose_cnn.at<float>(0, 6 + 4 * i + 3) = _renderer->_posesetgenerator._trackbar.fval[3 * i + 2][0];
			}

			//move cnn pose beside true pose 
			for (int i = 0; i < 3;i++)
				pose_cnn.at<float>(0, i) += 20; 
			for (int i = 3; i < 6; i++)
				pose_cnn.at<float>(0, i) += 20;

			for (int i = 6; i < 26; i++)
				pose_cnn.at<float>(0, i) += 20;
			*/



			limitPosition(pose_cnn, boundary_const);
			//show model
			showObModel("cnn",cam_color,cam_depth, &pose_cnn.at<float>(0, 0));

			//set boundary cnn
			setBoundary_cnn();
		}
#pragma endregion
		
		initializeVelocity();
		initializeCost();

		setBoundary_track(); //check (around gbest_param_pre)
		

		if (type.compare("6D")==0){
			initializePalm(0, particle_num, gbest_pre, boundary_track);
		}
		else if (type.compare("26D")==0){
			//initializeFingers(0, 20, gbest_pre);
			//initializePalm(20, 26, gbest_pre, boundary);
			//initializeAll(26, particle_num, gbest_pre, boundary);
			initializeAll(0, particle_num, gbest_pre, boundary_track);
		}
		else if (type.compare("hybrid") == 0){
			initializeAll(0, particle_num / 2, gbest_pre, boundary_track);
			initializeAll(particle_num/2, particle_num, pose_cnn, boundary_cnn);
		}

		
		calculateCost();
		calculatePbest(); 
		calculateGbest(0, particle_num, gbest); 

		//showObModelParticles("initial");
		//cv::waitKey(1);
		//cv::waitKey(1);
		
		
		for (int g = 0; g < max_generation; g++){
			
			//showObModelParticles("during","save",g);
	
			if (type.compare("6D") == 0){
				calculateVelocity(0, particle_num, gbest, boundary_track);
				updatePalm(0, particle_num, gbest, boundary_track);
			}
			else if (type.compare("26D")==0){
				calculateVelocity(0, particle_num, gbest, boundary_track);
				updateAll(0, particle_num, boundary_track,collisionNumber_track);

				//calculateVelocity_backup(0, particle_num, gbest, boundary_track);
				//updateAll_backup(0, particle_num, boundary_track);
			}
			else if (type.compare("hybrid") == 0){
				
				calculateVelocity(0, particle_num/2, gbest, boundary_track);
				updateAll(0, particle_num/2, boundary_track,collisionNumber_track);

				calculateVelocity(particle_num/2, particle_num, gbest, boundary_cnn);
				updateAll(particle_num/2, particle_num, boundary_cnn,collisionNumber_cnn);
			}

			
			calculateCost();
			calculatePbest();//ok
			calculateGbest(0, particle_num, gbest);//ok
			
			//--change boundary--//
			updateBoundary(g, collisionNumber_track,boundary_track);
			updateBoundary(g, collisionNumber_cnn, boundary_cnn);

			//--show particles--//

			//showObModelParticles("during","save",g);
			//cv::moveWindow("during", 896, 0);
			//cv::waitKey(1);
			//printf("costgbest[%d]:%f\n", i, cost_gbest);
			//cv::waitKey(1);
		}
		

		//select best solution.		
		//printf("PSO\N");
		
		for (int j = 0; j < dimension; j++){
			position.at<float>(0, j) = gbest.at<float>(0, j);
			gbest_pre.at<float>(0, j) = gbest.at<float>(0, j);
			//cost_gbest_pre = cost_gbest;
			//printf("gbest[%d]=%f\n", j, gbest.at<float>(0, j));
		}
		
		//for (int i = 0; i < 5; i++)
		//	printf("F[%d] %.2f %.2f %.2f %.2f\n", i, gbest.at<float>(0, 6 + 4 * i + 0), gbest.at<float>(0, 6 + 4 * i + 1), gbest.at<float>(0, 6 + 4 * i + 2), gbest.at<float>(0, 6 + 4 * i + 3));
		

		//visualize final solution
		//showDemo(cam_color,cam_depth);
		showObModel("final",cam_color, cam_depth, &position.at<float>(0, 0));
		
		
		//line up
		cv::moveWindow("cnn", 0, 0);
		cv::moveWindow("cnn_dif", 640, 0);
		cv::moveWindow("final", 0, 480);
		cv::moveWindow("final_dif", 640, 480);
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

					boundary_track.at<float>(0, j) = gbest_pre.at<float>(0, j) - 30;
					boundary_track.at<float>(1, j) = gbest_pre.at<float>(0, j) + 30;
				}
			}

			

			//check limit of boundary
			if (boundary_track.at<float>(0, j) <= boundary_max[0][j])
				boundary_track.at<float>(0, j) = boundary_max[0][j];
			//else if (boundary_track.at<float>(0, j) >= boundary_max[1][j])
			//	boundary_track.at<float>(0, j) = boundary_max[1][j];

			if (boundary_track.at<float>(1, j) >= boundary_max[1][j])
				boundary_track.at<float>(1, j) = boundary_max[1][j];
			//else if (boundary_track.at<float>(1, j) <= boundary_max[0][j])
			//	boundary_track.at<float>(1, j) = boundary_max[1][j];
			
			
		}

		cv::Mat gg = gbest_pre;
		cv::Mat bb = boundary_track;
		cv::waitKey(1);

	}

	void setBoundary_cnn(){

		for (int j = 0; j < dimension; j++){
			if (j < fingerStartIdx){	
				boundary_cnn.at<float>(0, j) = pose_cnn.at<float>(0, j) - 20;  //palm
				boundary_cnn.at<float>(1, j) = pose_cnn.at<float>(0, j) + 20;
			}
			else if (j >= fingerStartIdx){
				if ((j - fingerStartIdx) % 4 == 1){
					boundary_cnn.at<float>(0, j) = pose_cnn.at<float>(0, j) - 5;
					boundary_cnn.at<float>(1, j) = pose_cnn.at<float>(0, j) + 5;
				}
				else{
					boundary_cnn.at<float>(0, j) = pose_cnn.at<float>(0, j) - 30;
					boundary_cnn.at<float>(1, j) = pose_cnn.at<float>(0, j) + 30;
				}
			}

			//check limit of boundary
			if (boundary_cnn.at<float>(0, j) < boundary_max[0][j])
				boundary_cnn.at<float>(0, j) = boundary_max[0][j];
			//else if (boundary_cnn.at<float>(0, j) > boundary_max[1][j])
			//	boundary_cnn.at<float>(0, j) = boundary_max[0][j];

			if (boundary_cnn.at<float>(1, j) > boundary_max[1][j])
				boundary_cnn.at<float>(1, j) = boundary_max[1][j];
			//else if (boundary_cnn.at<float>(1, j) < boundary_max[0][j])
			//	boundary_cnn.at<float>(1, j) = boundary_max[1][j];
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
				position.at<float>(i, j) = (b1 - b0)*(rand() / double(RAND_MAX)) + b0;
		}
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
				position.at<float>(i, j) = (b1 - b0)*(rand() / double(RAND_MAX)) + b0;
			//position.at<float>(j, i) = frand_gaussian(p.at<float>(j, 0), s, b0, b1);
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
				position.at<float>(i, j) = (b1 - b0)*(rand() / double(RAND_MAX)) + b0;
		}
	}

	void initializeCost(){

		for (int i = 0; i < particle_num; i++){
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
			r1 = rand() / double(RAND_MAX);
			r2 = rand() / double(RAND_MAX);

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
			r1 = rand() / double(RAND_MAX);
			r2 = rand() / double(RAND_MAX);

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
			else//finger
				position.at<float>(i, j) += pvel.at<float>(i, j);

			if (position.at<float>(i, j) < b.at<float>(0, j))
				position.at<float>(i, j) = b.at<float>(0, j);
			else if (position.at<float>(i, j) > b.at<float>(1, j))
				position.at<float>(i, j) = b.at<float>(1, j);
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

	void updateAll(int begin_p, int end_p, cv::Mat b,cv::Mat& cs){

		for (int i = begin_p; i < end_p; i++)
		for (int j = 0; j < dimension; j++)
		{
			position.at<float>(i, j) += pvel.at<float>(i, j);

			if (position.at<float>(i, j) < b.at<float>(0, j)){

				position.at<float>(i, j) = b.at<float>(0, j);
				pvel.at<float>(i, j) = -(rand() / double(RAND_MAX))*pvel.at<float>(i, j);
				position.at<float>(i, j) += pvel.at<float>(i, j);

				//collisionNumber.at<float>(0, j) += 1;
				cs.at<float>(0, j) += 1;
			}
			if (position.at<float>(i, j) > b.at<float>(1, j)){

				position.at<float>(i, j) = b.at<float>(1, j);

				pvel.at<float>(i, j) = -(rand() / double(RAND_MAX))*pvel.at<float>(i, j);
				position.at<float>(i, j) += pvel.at<float>(i, j);

				//collisionNumber.at<float>(1, j) += 1;
				cs.at<float>(1, j) += 1;
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

	void calculateCost(){

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		for (int px = 0; px < particle_numx; px++)
		for (int py = 0; py < particle_numy; py++)
		{

			float* solp = &position.at<float>(px + py*particle_numx,0 );
			_renderer->renderTile(px, py, solp, "depth");
		}
		glFinish();
		

		_costFunct.calculateCost(cost);
		//for (int px = 0; px < particle_numx; px++)
		//for (int py = 0; py < particle_numy; py++)
		//	printf("[%d][%d]=%f\n", py, px, cost.at<float>(0, px + py*particle_numx));

		//_costFunct.calculateCost(cost);

	}

	void reinitialize(int ps, int pe, cv::Mat g, cv::Mat b){
		for (int i = ps; i < pe; i++)
		for (int j = 0; j < dimension; j++){
			float b0 = b.at<float>(0, j);// boundary.at<float>(j, 0);
			float b1 = b.at<float>(1, j);// boundary.at<float>(j, 1);

			position.at<float>(i, j) = g.at<float>(0, j) + 0.5*(b1 - b0)*(rand() / double(RAND_MAX)) + b0;
		}

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

	void showObModel(std::string wname,cv::Mat cam_color,cv::Mat cam_depth,float* solp)
	{
		cv::Mat cam_depth_debug;
		cam_depth.copyTo(cam_depth_debug);

		//--color
		//float* solp = &position.at<float>(0, 0);
		_renderer->renderOrig(solp, "color");


		cv::Mat model_color;
		_glrenderer.getOrigImage(model_color, "color");

		cv::Mat fimg;
		cv::addWeighted(cam_color, 0.5, model_color, 1.0, 0, fimg);
		cv::imshow(wname, fimg);
		//cv::imshow("model", model_color);
		//cv::imshow("final", cam_img);
		cv::waitKey(1);

		//-- depth
		_renderer->renderOrig(solp, "depth");

		cv::Mat model_depth;
		_glrenderer.getOrigImage(model_depth, "depth");

		//hard coding for debugging.
		cv::Mat difdepth = model_depth - cam_depth;
		cv::Mat difdepth8u = cv::Mat(480, 640, CV_8UC3);
		difdepth8u.setTo(0);
		for (int i = 0; i < 640;i++)
		for (int j = 0; j < 480; j++)
			difdepth.at<float>(j, i) = abs(difdepth.at<float>(j, i));

		cv::normalize(difdepth, difdepth8u, 0, 255, cv::NORM_MINMAX, CV_8UC3);
		cv::imshow(wname+"_dif", difdepth8u);
		cv::waitKey(1);

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

	float frand_gaussian(float a, float std, float lo, float hi) const{
		std::normal_distribution<float> distribution(0, std);

		while (true) {
			float number = distribution(*gen_gaussian);
			float result = a + number*(hi - lo);

			if (result >= lo && result <= hi)
				return result;
		}
	}

	void updateResource(){

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


	void updateBoundary(int g,cv::Mat& coll,cv::Mat& bmat){

		float alpha = 0.5;
		float pitr = 5.0;
		int T1 = 1;
		int T2 = 2;

		cv::Mat bt_before;
		bmat.copyTo(bt_before);

		if ((g > 0) &  (g % 5) == 0){

			for (int j = 0; j < dimension; j++){
				float bl = bmat.at<float>(0, j);
				float bu = bmat.at<float>(1, j);

				float mlow = coll.at<float>(0, j);
				float mup = coll.at<float>(1, j);

				//printf("%f %f \n", mlow / pitr, mup / pitr);
				//lower boundary.
				if (mlow / pitr < T1){
					bmat.at<float>(0, j) += alpha*(bu - bl);
					//printf("++lower bound[%d]\n", j);
				}
				if (mlow / pitr > T2){
					bmat.at<float>(0, j) -= alpha*(bu - bl);
					//printf("--lower bound[%d]\n", j);
				}

				//upper boundary.
				if (mup / pitr > T2){
					bmat.at<float>(1, j) += alpha*(bu - bl);
					//printf("++upperr bound[%d]\n", j);
				}
				if (mup / pitr < T1){
					bmat.at<float>(1, j) -= alpha*(bu - bl);
					//printf("++upper bound[%d]\n", j);
				}

				// keep the boundary when the gap between lower and upper one is too narrowed.
				float bl2 = bmat.at<float>(0, j);
				float bu2 = bmat.at<float>(1, j);

				if ((bl2>bu2) || (bl2 == bu2))
				{
					bmat.at<float>(0, j) = bl;
					bmat.at<float>(1, j) = bu;
				}

			}

			cv::Mat pp = position;
			cv::Mat ppgbest = gbest;
			cv::Mat pptrue = truepose;

			cv::Mat bb = bmat;
			cv::Mat collision = coll;

			for (int j = 0; j < dimension; j++){
				debugmatrix.at<float>(0, j) = coll.at<float>(0, j);
				debugmatrix.at<float>(1, j) = bmat.at<float>(0, j) - truepose.at<float>(0, j);
				debugmatrix.at<float>(2, j) = gbest.at<float>(0, j) - truepose.at<float>(0, j);
				debugmatrix.at<float>(3, j) = bmat.at<float>(1, j) - truepose.at<float>(0, j);
				debugmatrix.at<float>(4, j) = coll.at<float>(1, j);

			}

			cv::Mat dd = debugmatrix;
			coll.setTo(0);

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
	
	cv::Mat gbest_part[2];
	cv::Mat gbest_pre;
	
	cv::Mat pvel;
	cv::Mat boundary_track, boundary_cnn, boundary_const;
	cv::Mat cost;//
	cv::Mat cost_pbest;
	cv::Mat pose_cnn;
	float cost_gbest;
	float cost_gbest_pre;
	cv::Mat cov_track;
	cv::Mat cov_cnn;

	std::mt19937* gen_gaussian;

	double miliSec;
	//Watch watch;

	int fingerStartIdx;

	float com_hand[3];

	cv::Mat collisionNumber_track;
	cv::Mat collisionNumber_cnn;

	cv::Mat truepose;

	cv::Mat debugmatrix;
};
