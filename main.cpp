#include <iostream>


#include "Camera.h"

#include "HandParameter.h"
#include "GLRenderer.h"
#include "Hand.h"
#include "HandGenerator.h"
#include "pso.h"
#include "CostFunction.h"
#include "Preprocessing.h"
#include "MMF.h"
#include <windows.h>


#include <pthread.h>
#pragma comment(lib,"pthreadVC2.lib")

pthread_t thread_id;
int state;
MMF mmf(128,128);

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

void* sendImage_learning(void* data)
{
	mmf.send2CNN();

	//pthread_exit((void*)data);

	//return data;
	return((void*)1);
}

void main()
{
#pragma region user parameter setting
	std::string cameratype = "glcamera"; //"realcamera", "playcamera", "glcamera",
	int width = 640;
	int height = 480;
	int width_tile = 128;
	int height_tile = 128;
	int width_mmf = 128;
	int height_mmf = 128;
	int particle_numx = 8;
	int particle_numy = 4;
	int particle_num = particle_numx*particle_numy;
	int width_fb = width_tile * particle_numx;
	int height_fb = height_tile * particle_numy;
	int handParamNum = 26;

	int JOINT_COUNT = 19;
	int max_generation = 30;

	//option
	bool segmenthand_enable = true;  // in case of "realcamera", "playcamera".
	bool sendcamimg_enable = false;
	bool sendresultimg_enable = true;


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
	
	Camera camera(width, height, &handgenerator, &glrenderer, cameratype);
	Preprocessing preproc(width, height,camera);
#pragma endregion

	//initial pose before tracking.
	float in[26] = { 96, -27, 250,
		97, -18, 10,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0 };
	pso.setGbestPre(in);
	

	//key
	int recordbool = false;

	while (1){
		if (camera.queryFrames() == false)
			continue;

		

#pragma region camera input

		cv::Mat cam_depth;
		camera.getDepthBuffer(cam_depth);
		cv::imshow("cam_depth", cam_depth);
		
		cv::Mat cam_depth8u = cv::Mat(height, width, CV_8UC3);
		cv::normalize(cam_depth, cam_depth8u, 0, 255, cv::NORM_MINMAX, CV_8UC3);
		cv::imshow("cam_depth8u", cam_depth8u);

		cv::Mat cam_color;
		camera.getMappedColorBuffer(cam_color);
		cv::imshow("cam_color", cam_color);
		//glutSwapBuffers();
		
#pragma endregion 

#pragma region record frames
		
		if (recordbool == true)
			camera.recordFrames();
		char key = cv::waitKey(10);
		if (key == 'r'){
			printf("record start\n");
			recordbool = true;
		}
		
#pragma endregion
		

#pragma region preprocessing
		
		if (segmenthand_enable==true)
			preproc.segmentHandFromBand(cam_color, cam_depth);
	
		//cv::imshow("seg_depth", cam_depth);
		//cvMoveWindow("seg_depth", 1000, 600);
#pragma endregion

#pragma region make cnn image & transfer it to CNN.
		if (sendcamimg_enable == true){

			cv::Mat depth_cnn;
			preproc.makeCnnImage(cam_depth);
			preproc.getCnnImage(depth_cnn);

			if (mmf.getimg_bool == false)
			{
				mmf.getimg_bool = true;
				//mmf._cnnimg = depth_norm;
				//mmf._cnnimg= depth_cnn;
				depth_cnn.copyTo(mmf._cnnimg);
				//mmf._cnnimg = cam_colordepth;

				state = pthread_create(&thread_id, NULL, sendImage_learning, NULL);
				pthread_detach(thread_id);
			}
		}
#pragma endregion
	

#pragma region model fitting
		
		//pso.run(cam_color, cam_depth, "6D"); 
		pso.run(cam_color, cam_depth, "26D");

#pragma endregion

#pragma region gui test	
		{
			//set track bar from pso result
			
			char key = cv::waitKey(1);
			if (key == 'c'){
				float* solp = &pso.gbest.at<float>(0, 0);
				handgenerator.run_setTbarFromResult(solp);
			}
			if (key == 'v'){ //not implemented yet.
				for (int j = 0; j < 6;j++)
				pso.gbest.at<float>(0,j) = handgenerator._trackbar.wval[j];

				for (int i = 0; i < 5; i++)
				{
					pso.gbest.at<float>(0, 6 + 4 * i + 0) = handgenerator._trackbar.fval[1 + 3 * i+0][0];
					pso.gbest.at<float>(0, 6 + 4 * i + 1) = handgenerator._trackbar.fval[1 + 3 * i+0][1];
					pso.gbest.at<float>(0, 6 + 4 * i + 2) = handgenerator._trackbar.fval[1 + 3 * i+1][0];
					pso.gbest.at<float>(0, 6 + 4 * i + 3) = handgenerator._trackbar.fval[1 + 3 * i+2][0];
				}
			}
			
			//run gui
			if (sendresultimg_enable==true)
			{
				handgenerator.run_trackbar();
				handgenerator.run_gui("depth");

				cv::Mat model_depth;
				glrenderer.getOrigImage(model_depth, "depth");
				cv::imshow("manual", model_depth);
				cvMoveWindow("manual", 1000, 0);

				if (cv::waitKey(1) == 's')
					handgenerator.save_trackbar();

#pragma region make cnn image & transfer it to CNN.
				cv::Mat depth_cnn;
				preproc.makeCnnImage(model_depth);
				preproc.getCnnImage(depth_cnn);

				if (mmf.getimg_bool == false)
				{
					mmf.getimg_bool = true;
					//mmf._cnnimg = depth_norm;
					//mmf._cnnimg= depth_cnn;
					depth_cnn.copyTo(mmf._cnnimg);
					//mmf._cnnimg = cam_colordepth;

					state = pthread_create(&thread_id, NULL, sendImage_learning, NULL); 
					pthread_detach(thread_id);
				}
			}
#pragma endregion

			/*
			//add	
			cv::Mat fimg;
			cv::addWeighted(cam_color, 0.3, model_color, 0.9, 0, fimg);
			cv::imshow("manual2", fimg);
			cvMoveWindow("manual2", 1000, 500);
			cv::waitKey(1);

			//hard coding for debugging.
			handgenerator.run_gui("depth");
			cv::Mat model_depth;
			glrenderer.getOrigImage(model_depth, "depth");

			cv::Mat difdepth = model_depth - cam_depth;
			cv::Mat difdepth8u = cv::Mat(480, 640, CV_8UC3);
			difdepth8u.setTo(0);
			for (int i = 0; i < 640; i++)
			for (int j = 0; j < 480; j++)
			{
				difdepth.at<float>(j, i) = abs(difdepth.at<float>(j, i));

				//if (difdepth.at<float>(j, i) < 0)
				//	difdepth.at<float>(j, i) = 0;
			}
			cv::normalize(difdepth, difdepth8u, 0, 255, cv::NORM_MINMAX, CV_8UC3);
			cv::imshow("difmanual", difdepth8u);
			cv::waitKey(1);
			*/

		}
		
		
		
	
#pragma endregion

#pragma region release
		camera.releaseFrames();
		cv::waitKey(1);
		
#pragma endregion
		
		
	}


}