#include <iostream>

#include "HandParameter.h"
#include "GLRenderer.h"
#include "Hand.h"
#include "HandGenerator.h"
//#include "pso.h"
//#include "CostFunction.h"

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
	int particle_numW = 8;
	int particle_numH = 4;
	int particle_num = particle_numW*particle_numH;
	int width_fb = width_tile * particle_numW;
	int height_fb = height_tile * particle_numH;


	int JOINT_COUNT = 19;
	int max_generation = 20;

	//--user parameter--//


	//input camera setting
	//model setting
	//optimize 26 parameters.

	GLRenderer glrenderer(width, height, width_tile, height_tile, width_fb, height_fb);
	HandParameters hg(HandParameters::Default());
	HandGenerator handgenerator(hg);
	
	//PSO pso(particle_numW,particle_numH,max_generation,hg)

	
	while (1){

		//camera input
		{ 
			float in[26] = {0};
			in[2] = 20;
			handgenerator.run(in,"color");
		}

		cv::Mat cam_img;
		glrenderer.getOrigImage(cam_img, "color");
		cv::imshow("cam", cam_img);
		cv::waitKey(1);

		//preprocessing
		//not implemented yet..

		//optimize by rendering a hand model.
		//pso.run();

		//handgenerator.run_gui();
		
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
		
		

	
		
	}


}