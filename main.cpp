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

//#define TWOINPUTTEST

#ifdef TWOINPUTTEST
void main()
{

#pragma region user parameter setting
	std::string cameratype = "realcamera"; //"realcamera", "playcamera", "glcamera",
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
	bool sendcamimg_enable = true;
	bool savegroundtruth_enable = false;
	bool sendresultimg_enable = false;

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
	Camera camera2(width, height, &handgenerator, &glrenderer, "glcamera");
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

	//common variable
	float com_hand[3];
	float com_hand2[3];
	cv::Mat cam_depth16= cv::Mat(height, width, CV_16UC1);

	while (1){
		if (camera.queryFrames() == false)
			break;

		if (camera2.queryFrames() == false)
			break;

#pragma region camera input

		cv::Mat cam_depth;
		camera.getDepthBuffer(cam_depth);
		//cv::imshow("cam_depth", cam_depth);

		//cv::Mat cam_depth8u = cv::Mat(height, width, CV_8UC3);
		//cv::normalize(cam_depth, cam_depth8u, 0, 255, cv::NORM_MINMAX, CV_8UC3);
		//cv::imshow("cam_depth8u", cam_depth8u);

		cv::Mat cam_color;
		camera.getMappedColorBuffer(cam_color);
		//cv::imshow("cam_color", cam_color);
		//glutSwapBuffers();

		//input2
		cv::Mat cam_color2;
		camera2.getMappedColorBuffer(cam_color2);
		//cv::imshow("cam_color2", cam_color2);

		cv::Mat cam_depth2;
		camera2.getDepthBuffer(cam_depth2);

		cv::Mat camcolor12;
		cv::addWeighted(cam_color, 0.5, cam_color2, 0.5, 0, camcolor12);
		cv::imshow("sum", camcolor12);

#pragma endregion 

#pragma region record frames

		if (cv::waitKey(10) == 'r')
			camera.recordFrames();
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

		if (segmenthand_enable==true)
			preproc.segmentHandFromBand(cam_color, cam_depth);

		preproc.getComHandxyz(cam_depth,com_hand);
		preproc.getComHandxyz(cam_depth2, com_hand2);
	
		printf("real_com: %f %f %f\n", com_hand[0], com_hand[1], com_hand[2]);
		printf("model_com: %f %f %f\n", com_hand2[0], com_hand2[1], com_hand2[2]);
		
		//cv::imshow("seg_depth", cam_depth);
		//cvMoveWindow("seg_depth", 1000, 600);
#pragma endregion

#pragma region make cnn image & transfer it to CNN.
		if (sendcamimg_enable == true){
			cv::Mat depth_cnn;
			preproc.makeCnnImage(cam_depth);
			preproc.getCnnImage(depth_cnn);
			cv::imshow("depth_cnn", depth_cnn);

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

		//input2
		if (sendcamimg_enable == true){
			cv::Mat depth_cnn2;
			preproc.makeCnnImage(cam_depth2);
			preproc.getCnnImage(depth_cnn2);
			cv::imshow("depth_cnn2", depth_cnn2);

		}
#pragma endregion

#pragma region make ground truth for cnn

		for (int i = 0; i < 5; i++){
			for (int j = 0; j < 3; j++){
				float jpos[3];
				handgenerator.hand.GetJointPosition(i, j, jpos);

				cv::Mat cmat;
				camera.getCalibrationMatrix(cmat);

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



		if (savegroundtruth_enable == true){

			//data
			for (int i = 0; i < width; i++)
			for (int j = 0; j < height; j++)
				cam_depth16.at<ushort>(j, i) = cam_depth.at<float>(j, i);

			char filenamed[200];
			sprintf(filenamed, "save/cnn/test/data/depth-%07u.png", camera._frame);
			cv::imwrite(filenamed, cam_depth16);

			//label
			FILE* fp;
			char filenamel[200];
			sprintf(filenamel, "save/cnn/test/label/label.csv");
			fp = fopen(filenamel, "a");

			char str[100];

			for (int i = 0; i < 5; i++)
			for (int j = 0; j < 3; j++){
				float jpos[3];
				handgenerator.hand.GetJointPosition(i, j, jpos);

				for (int k = 0; k < 3;k++)
					jpos[k] = com_hand[k] - jpos[k];

				if (i==4 & j==2)
					sprintf(str, "%.2f,%.2f,%.2f\n", jpos[0], jpos[1], jpos[2]);
				else
					sprintf(str, "%.2f,%.2f,%.2f,", jpos[0], jpos[1], jpos[2]);

				fputs(str, fp);
			}

			fclose(fp);

		}
#pragma endregion

#pragma region get cnn result

		float cnnresult[5 * 3 * 3];
		mmf.receiveData(); 
		mmf.getLabel(cnnresult);

		//visualize cnnresult
		cv::Mat cmat;
		camera.getCalibrationMatrix(cmat);

		//printf("com:%f %f %f\n", com_hand[0], com_hand[1], com_hand[2]);
		for (int i = 0; i < 15; i++)
		{
			float x = com_hand[0]-cnnresult[3 * i + 0];
			float y = com_hand[1]-cnnresult[3 * i + 1];
			float z = com_hand[2]-cnnresult[3 * i + 2];

			float x_ = x * cmat.at<float>(0, 0) + y * cmat.at<float>(0, 1) + z * cmat.at<float>(0, 2);
			float y_ = x * cmat.at<float>(1, 0) + y * cmat.at<float>(1, 1) + z * cmat.at<float>(1, 2);
			float z_ = x * cmat.at<float>(2, 0) + y * cmat.at<float>(2, 1) + z * cmat.at<float>(2, 2);
			x_ /= z_;
			y_ /= z_;

			//printf("p[%d]: x:%f y:%f z:%f\n", i, cnnresult[3 * i + 0], cnnresult[3 * i + 1], cnnresult[3 * i + 2]);
			//printf("[%d]: x_:%f y_:%f\n",i, x_, y_);
			cv::circle(cam_color, cv::Point(x_, y_), 10, cv::Scalar(255, 0, 0), 5, 8, 0);

		}
		cv::imshow("predicted", cam_color);



#pragma endregion


#pragma region model fitting

		//pso.run(cam_color, cam_depth, "6D"); 
		//pso.run(cam_color, cam_depth, "26D");

#pragma endregion


#pragma region gui test	
		/*
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
		//cv::imshow("manual", model_depth);
		//cvMoveWindow("manual", 1000, 0);

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


		////add	
		//cv::Mat fimg;
		//cv::addWeighted(cam_color, 0.3, model_color, 0.9, 0, fimg);
		//cv::imshow("manual2", fimg);
		//cvMoveWindow("manual2", 1000, 500);
		//cv::waitKey(1);

		////hard coding for debugging.
		//handgenerator.run_gui("depth");
		//cv::Mat model_depth;
		//glrenderer.getOrigImage(model_depth, "depth");

		//cv::Mat difdepth = model_depth - cam_depth;
		//cv::Mat difdepth8u = cv::Mat(480, 640, CV_8UC3);
		//difdepth8u.setTo(0);
		//for (int i = 0; i < 640; i++)
		//for (int j = 0; j < 480; j++)
		//{
		//	difdepth.at<float>(j, i) = abs(difdepth.at<float>(j, i));

		//	//if (difdepth.at<float>(j, i) < 0)
		//	//	difdepth.at<float>(j, i) = 0;
		//}
		//cv::normalize(difdepth, difdepth8u, 0, 255, cv::NORM_MINMAX, CV_8UC3);
		//cv::imshow("difmanual", difdepth8u);
		//cv::waitKey(1);
		}
		*/		



#pragma endregion


#pragma region release
		camera.releaseFrames();
		cv::waitKey(1);

#pragma endregion


	}


}
#endif

#ifndef TWOINPUTTEST

void main()
{
#pragma region user parameter setting
	std::string cameratype = "realcamera"; //"realcamera", "playcamera", "glcamera",
	int width = 640;
	int height = 480;
	int width_tile = 128;
	int height_tile = 128;
	int width_mmf = 128;
	int height_mmf = 128;
	int handParamNum = 26;
	int JOINT_COUNT = 19;
	int max_generation = 30;
	int particle_numx = 8;
	int particle_numy = 4;
	int particle_num = particle_numx*particle_numy;
	int width_fb = width_tile * particle_numx;
	int height_fb = height_tile * particle_numy;
	

	//option
	bool segmenthand_enable = true;  // use blue band

	bool sendcamimg_enable = true;//send
	bool sendresultimg_enable = false;//send
	
	bool savegroundtruth_enable = false;//save
	bool record_enable = false;//save

	bool showgroundtruth_enable = false;//show
	bool showcnnresult_enable = false;//show

	
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
	
	
	//common variable
	float com_hand[3];
	cv::Mat cam_depth16= cv::Mat(height, width, CV_16UC1);

	while (1){
		if (camera.queryFrames() == false)
			break;

		
#pragma region camera input

		cv::Mat cam_depth;
		camera.getDepthBuffer(cam_depth);
		//cv::imshow("cam_depth", cam_depth);
		
		//cv::Mat cam_depth8u = cv::Mat(height, width, CV_8UC3);
		//cv::normalize(cam_depth, cam_depth8u, 0, 255, cv::NORM_MINMAX, CV_8UC3);
		//cv::imshow("cam_depth8u", cam_depth8u);

		cv::Mat cam_color;
		camera.getMappedColorBuffer(cam_color);
		cv::imshow("cam_color", cam_color);
		//glutSwapBuffers();

#pragma endregion 

#pragma region record frames
		
		if (record_enable == true){
			if (cv::waitKey(1) == 'r'){
				printf("record start\n");
				camera.recordFrames();
			}
		}
		
		
#pragma endregion
		

#pragma region preprocessing
		
		if (segmenthand_enable==true)
			preproc.segmentHandFromBand(cam_color, cam_depth);
	
		preproc.getComHandxyz(cam_depth,com_hand);
		//cv::imshow("seg_depth", cam_depth);
		//cvMoveWindow("seg_depth", 1000, 600);
		//printf("com:%f %f %f\n", com_hand[0], com_hand[1], com_hand[2]);

#pragma endregion

#pragma region make cnn image & transfer it to CNN.
		if (sendcamimg_enable == true){
			cv::Mat depth_cnn;
			preproc.makeCnnImage(cam_depth);
			preproc.getCnnImage(depth_cnn);
			//cv::imshow("depth_cnn", depth_cnn);

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
	
#pragma region make ground truth for cnn
		if (showgroundtruth_enable == true)
		{
			for (int i = 0; i < 5; i++){
				for (int j = 0; j < 3; j++){
					float jpos[3];
					handgenerator.hand.GetJointPosition(i, j, jpos);

					cv::Mat cmat;
					camera.getCalibrationMatrix(cmat);

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
		}
		
		
		if (savegroundtruth_enable == true){

			//data
			for (int i = 0; i < width; i++)
			for (int j = 0; j < height; j++)
				cam_depth16.at<ushort>(j, i) = cam_depth.at<float>(j, i);

			char filenamed[200];
			sprintf(filenamed, "save/cnn/test/data/depth-%07u.png", camera._frame);
			cv::imwrite(filenamed, cam_depth16);

			//label
			FILE* fp;
			char filenamel[200];
			sprintf(filenamel, "save/cnn/test/label/label.csv");
			fp = fopen(filenamel, "a");

			char str[100];

			for (int i = 0; i < 5; i++)
			for (int j = 0; j < 3; j++){
				float jpos[3];
				handgenerator.hand.GetJointPosition(i, j, jpos);
			
				for (int k = 0; k < 3;k++)
					jpos[k] = com_hand[k] - jpos[k];

				if (i==4 & j==2)
					sprintf(str, "%.2f,%.2f,%.2f\n", jpos[0], jpos[1], jpos[2]);
				else
					sprintf(str, "%.2f,%.2f,%.2f,", jpos[0], jpos[1], jpos[2]);

				fputs(str, fp);
			}

			fclose(fp);

		}
#pragma endregion

#pragma region get cnn result
		if (showcnnresult_enable == true)
		{
			float cnnresult[5 * 3 * 3];
			mmf.receiveData();
			mmf.getLabel(cnnresult);

			//visualize cnnresult
			cv::Mat cmat;
			camera.getCalibrationMatrix(cmat);

			//printf("com:%f %f %f\n", com_hand[0], com_hand[1], com_hand[2]);
			for (int i = 0; i < 15; i++)
			{
				float x = com_hand[0] - cnnresult[3 * i + 0];
				float y = com_hand[1] - cnnresult[3 * i + 1];
				float z = com_hand[2] - cnnresult[3 * i + 2];

				float x_ = x * cmat.at<float>(0, 0) + y * cmat.at<float>(0, 1) + z * cmat.at<float>(0, 2);
				float y_ = x * cmat.at<float>(1, 0) + y * cmat.at<float>(1, 1) + z * cmat.at<float>(1, 2);
				float z_ = x * cmat.at<float>(2, 0) + y * cmat.at<float>(2, 1) + z * cmat.at<float>(2, 2);
				x_ /= z_;
				y_ /= z_;

				//printf("p[%d]: x:%f y:%f z:%f\n", i, cnnresult[3 * i + 0], cnnresult[3 * i + 1], cnnresult[3 * i + 2]);
				//printf("[%d]: x_:%f y_:%f\n",i, x_, y_);
				cv::circle(cam_color, cv::Point(x_, y_), 10, cv::Scalar(255, 0, 0), 5, 8, 0);

			}
			cv::imshow("predicted", cam_color);
		}


#pragma endregion


#pragma region model fitting
		
		//pso.run(cam_color, cam_depth, "6D"); 
		pso.run(cam_color, cam_depth, "26D");

#pragma endregion


#pragma region gui test	
/*
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
				//cv::imshow("manual", model_depth);
				//cvMoveWindow("manual", 1000, 0);

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

			
			////add	
			//cv::Mat fimg;
			//cv::addWeighted(cam_color, 0.3, model_color, 0.9, 0, fimg);
			//cv::imshow("manual2", fimg);
			//cvMoveWindow("manual2", 1000, 500);
			//cv::waitKey(1);

			////hard coding for debugging.
			//handgenerator.run_gui("depth");
			//cv::Mat model_depth;
			//glrenderer.getOrigImage(model_depth, "depth");

			//cv::Mat difdepth = model_depth - cam_depth;
			//cv::Mat difdepth8u = cv::Mat(480, 640, CV_8UC3);
			//difdepth8u.setTo(0);
			//for (int i = 0; i < 640; i++)
			//for (int j = 0; j < 480; j++)
			//{
			//	difdepth.at<float>(j, i) = abs(difdepth.at<float>(j, i));

			//	//if (difdepth.at<float>(j, i) < 0)
			//	//	difdepth.at<float>(j, i) = 0;
			//}
			//cv::normalize(difdepth, difdepth8u, 0, 255, cv::NORM_MINMAX, CV_8UC3);
			//cv::imshow("difmanual", difdepth8u);
			//cv::waitKey(1);
		}
*/		
		
		
	
#pragma endregion


#pragma region release
		camera.releaseFrames();
		cv::waitKey(1);
		
#pragma endregion
		
		
	}


}
#endif