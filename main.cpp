#include <iostream>

#include "HandParameter.h"
#include "GLRenderer.h"
#include "Hand.h"
#include "HandGenerator.h"
#include "pso.h"
#include "CostFunction.h"

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
	//--user parameter--//
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
	int max_generation = 20;

	HandParameters hp = HandParameters::Default();
	hp.width = width; hp.height = height;
	hp.width_tile = width_tile; hp.height_tile = height_tile;
	hp.particle_numx = particle_numx; hp.particle_numy = particle_numy;
	hp.handParamNum = handParamNum;

	//--user parameter--//


	//input camera setting
	//model setting
	//optimize 26 parameters.

	GLRenderer glrenderer(width, height, width_tile, height_tile, width_fb, height_fb);
	//HandParameters hg(HandParameters::Default());
	HandParameters hg(hp);
	HandGenerator handgenerator(hg);
	

	CostFunction costFunction(particle_numx, particle_numy, handParamNum,glrenderer);
	PSO pso(particle_numx, particle_numy, max_generation,handParamNum,handgenerator,glrenderer,costFunction);
	

	float in[26] = { 0, 0, 10,
		0, 0, 10,
		-10, -20, -20, -20,
		-60, 15, -20, -20,
		-50, 4, -20, -20,
		-40, -3, -20, -20,
		-30, -10, -20, -20 };

	//debug
	pso.setGbestPre(in);
	//debug

	while (1){

		//camera input		
		handgenerator.run(in,"depth");
		cv::Mat cam_depth;
		glrenderer.getOrigImage(cam_depth, "depth");

		handgenerator.run(in, "color");
		cv::Mat cam_color;
		glrenderer.getOrigImage(cam_color, "color");
		
		//cv::imshow("cam_color", cam_color);
		//cv::imshow("cam_depth", cam_depth);

		//debugging
		
		//debugging

		
		//preprocessing
		//not implemented yet..

		//optimize by rendering a hand model.
		stopWatch();
		//pso.run(cam_color, cam_depth, "6D"); 
		pso.run(cam_color, cam_depth, "26D");
		stopWatch();
		
		

		cv::waitKey(1);

		//handgenerator.run_gui();
		/*
		{
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			float in[26] = { 0 };
			in[0] = 0; in[1] = 0; in[2] = 20;
			handgenerator.runTile(0, 0, in,"color");

			in[0] = 1; in[1] = 0; in[2] = 20;
			in[3] = 90;
			handgenerator.runTile(0, 1, in, "color");

			in[0] = 0; in[1] = 1; in[2] = 20;
			in[3] = 180;
			handgenerator.runTile(1, 0, in, "color");

			in[0] = 1; in[1] = 1; in[2] = 20;
			in[3] = 270;
			handgenerator.runTile(1, 1, in, "color");

			cv::Mat model_color;
			glrenderer.getColorTexture(model_color);
			cv::imshow("model_color", model_color);
			cv::Mat model_depth;
			glrenderer.getDepthTexture(model_depth);
			cv::imshow("model_depth", model_depth);

			cv::waitKey(1);
		}
		*/
		
		

	
		
	}


}