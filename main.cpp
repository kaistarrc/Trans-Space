#include <iostream>


#include "Camera.h"

#include "HandParameter.h"
#include "GLRenderer.h"
#include "Hand.h"
#include "HandGenerator.h"
#include "pso.h"
#include "CostFunction.h"
#include "Preprocessing.h"

#include <windows.h>

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

void main()
{
	
#pragma region user parameter setting
	int width = 640;
	int height = 480;
	int width_tile = 128;
	int height_tile = 128;
	int particle_numx = 8;
	int particle_numy = 4;
	int particle_num = particle_numx*particle_numy;
	int width_fb = width_tile * particle_numx;
	int height_fb = height_tile * particle_numy;
	int handParamNum = 26;

	int JOINT_COUNT = 19;
	int max_generation = 30;

	HandParameters hp = HandParameters::Default();
	hp.width = width; hp.height = height;
	hp.width_tile = width_tile; hp.height_tile = height_tile;
	hp.particle_numx = particle_numx; hp.particle_numy = particle_numy;
	hp.handParamNum = handParamNum;
#pragma endregion


#pragma region init class	
	
	GLRenderer glrenderer(width, height, width_tile, height_tile, width_fb, height_fb);
	HandGenerator handgenerator(hp);
	CostFunction costFunction(particle_numx, particle_numy, handParamNum,glrenderer);
	PSO pso(particle_numx, particle_numy, max_generation,handParamNum,&handgenerator,glrenderer,costFunction);
	
	//"realcamera", "playcamera", "glcamera",
	Camera camera(width, height, &handgenerator, &glrenderer, "playcamera");
	Preprocessing preproc(width, height,camera);

#pragma endregion



	//debug
	float in[26] = { 0, 0, 300,
		0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0 };
	pso.setGbestPre(in);
	//debug

	//key
	int recordbool = false;

	while (1){
		if (camera.queryFrames() == false)
			continue;

#pragma region camera input

		cv::Mat cam_depth;
		camera.getDepthBuffer(cam_depth);
		//cv::imshow("cam_depth", cam_depth);

		cv::Mat cam_color;
		camera.getMappedColorBuffer(cam_color);
		//cv::imshow("cam_color", cam_color);

#pragma endregion 

#pragma region record frames
		/*
		if (recordbool == true)
			camera.recordFrames();
		char key = cv::waitKey(10);
		if (key == 'r'){
			printf("record start\n");
			recordbool = true;
		}
		*/
#pragma endregion
		

#pragma region preprocessing
		//--preprocessing--//
		preproc.segmentHandFromBand(cam_color, cam_depth);
		cv::imshow("seg_depth", cam_depth);
		cvMoveWindow("seg_depth", 1000, 600);
		
#pragma endregion
	
#pragma region optimize 
		
		//--optimize--//
		//pso.run(cam_color, cam_depth, "6D"); 
		pso.run(cam_color, cam_depth, "26D");


#pragma endregion

#pragma region gui test
		//set track bar from pso result
		float* solp = &pso.gbest.at<float>(0, 0);
		handgenerator.run_setTbarFromResult(solp);

		//run gui
		handgenerator.run_gui();

		cv::Mat model_color;
		glrenderer.getOrigImage(model_color, "color");
		//cv::flip(model_color, model_color, 1);
		cv::imshow("manual", model_color);
		cvMoveWindow("manual", 1000, 0);

		//add	
		cv::Mat fimg;
		cv::addWeighted(cam_color, 0.3, model_color, 0.9, 0, fimg);
		cv::imshow("manual2", fimg);
		cvMoveWindow("manual2", 1000, 500);
		cv::waitKey(1);
		
		{
			//printf("Trackbar\n");
			float tx = handgenerator._trackbar.wval[0];
			float ty = handgenerator._trackbar.wval[1];
			float tz = handgenerator._trackbar.wval[2];
			float rx = handgenerator._trackbar.wval[3];
			float ry = handgenerator._trackbar.wval[4];
			float rz = handgenerator._trackbar.wval[5];

			int joint_index = handgenerator._trackbar.fid;
			float fx = handgenerator._trackbar.fval[0];
			float fy = handgenerator._trackbar.fval[1];
			float fz = handgenerator._trackbar.fval[2];

			//printf("wt:%f %f %f\n", tx, ty, tz);
			//printf("wr:%f %f %f\n", rx, ry, rz);
			//printf("jid:%d %f %f %f\n", joint_index, fx, fy, fz);
		}
		
#pragma endregion

#pragma region release
		camera.releaseFrames();
		char keyq=cv::waitKey(1);
		if (keyq == 'q')
			break;
#pragma endregion
		
		
	}


}