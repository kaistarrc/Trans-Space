#pragma once

#include "PlayCamera.h"
#include "RealSenseCVWrapper.h"
#include "HandParameter.h"
#include "GLRenderer.h"
#include "Hand.h"
#include "HandGenerator.h"
class Camera{

public:
	Camera(){

	}
	Camera(int w, int h,HandGenerator* hg,GLRenderer* gr,std::string camtype){

		width = w;
		height = h;

		_camtype = camtype;

		if (camtype.compare("realcamera")==0)
			realcamera = new RealSenseCVWrapper(w, h);
		else if (camtype.compare("playcamera") == 0)
			playcamera = new PlayCamera(w, h);
		else if (camtype.compare("glcamera") == 0){
			handgenerator = hg;
			glcamera = gr;
		}

		cam_color = cv::Mat(h, w, CV_8UC3);
		cam_depth16 = cv::Mat(h, w, CV_16UC1);
		cam_depth = cv::Mat(h, w, CV_32FC1);
		calib_mat = cv::Mat(3, 3, CV_32FC1);

		calib_mat.at<float>(0, 0) = 477.9; calib_mat.at<float>(0, 1) = 0.0; calib_mat.at<float>(0, 2) = 326.6;
		calib_mat.at<float>(1, 0) = 0.0; calib_mat.at<float>(1, 1) = 477.9; calib_mat.at<float>(1, 2) = 245.9;
		calib_mat.at<float>(2, 0) = 0.0; calib_mat.at<float>(2, 1) = 0.0; calib_mat.at<float>(2, 2) = 1.0;

		_frame = 0;
	}

	~Camera(){

	}

	bool queryFrames(){
		

		if (_camtype.compare("realcamera") == 0){
			if (realcamera->queryFrames() == false)
				return false;
		}
		else if (_camtype.compare("playcamera") == 0){
			
			if (playcamera->queryFrames(_frame) == false)
				return false;

			char key = cv::waitKey(1);
			if (key == 'a'){
				_frame++;
				printf("frame:%d\n", _frame);
			}
			else if (key == 's'){
				_frame--;
				printf("frame:%d\n", _frame);
			}
		}
		//else if (_camtype.compare("glcamera") == 0){
		//	return true;
		//}

		

		return true;
	}

	void getDepthBuffer(cv::Mat& out){

		if (_camtype.compare("realcamera") == 0)
			realcamera->getDepthBuffer(cam_depth);

		else if (_camtype.compare("playcamera") == 0){
			playcamera->getDepthBuffer(cam_depth16);

			for (int i = 0; i < width;i++)
			for (int j = 0; j < height; j++)
				cam_depth.at<float>(j, i) = cam_depth16.at<ushort>(j, i);
		}

		else if (_camtype.compare("glcamera") == 0){
			float sol[26] = { 0, 0, 10,
				0, 0, 10,
				-10, -20, -20, -20,
				-60, 15, -20, -20,
				-50, 4, -20, -20,
				-40, -3, -20, -20,
				-30, -10, -20, -20 };
			handgenerator->run(sol, "depth");
			glcamera->getOrigImage(cam_depth, "depth");
		}

		//cv::flip(cam_depth,cam_depth, 1);
		cam_depth.copyTo(out);
		
	}

	void getMappedColorBuffer(cv::Mat& out){

		if (_camtype.compare("realcamera") == 0)
			realcamera->getMappedColorBuffer(cam_color);
		else if (_camtype.compare("playcamera") == 0)
			playcamera->getColorBuffer(cam_color);
		else if (_camtype.compare("glcamera") == 0){

			float sol[26] = { 0, 0, 10,
				0, 0, 10,
				-10, 0, 0, 0,
				-60, 0, 0, 0,
				-50, 0, 0, 0,
				-40, 0, 0, 0,
				-30, 0, 0, 0 };
			handgenerator->run(sol, "color");
			glcamera->getOrigImage(cam_color, "color");
		}

		//cv::flip(cam_color, cam_color, 1);
		cam_color.copyTo(out);
	}

	void recordFrames(){
		

		{
			for (int i = 0; i < width;i++)
			for (int j = 0; j < height; j++)
				cam_depth16.at<ushort>(j, i) = cam_depth.at<float>(j, i);

			char filename[200];
			sprintf(filename, "frames/depth-%07u.png", _frame);
			cv::imwrite(filename, cam_depth16);

		}

		{
			char filename[200];
			sprintf(filename, "frames/color-%07u.png", _frame);
			cv::imwrite(filename, cam_color);

		}
		_frame++;
	}

	void getCalibrationMatrix(cv::Mat& out){
		calib_mat.copyTo(out);

	}


	void releaseFrames(){

		if (_camtype.compare("realcamera") == 0)
			realcamera->releaseFrames();
		//else if (_camtype.compare("playcamera") == 0)
		//	printf("not implemented yet\n");

		
	}
	

private:

	RealSenseCVWrapper* realcamera;
	PlayCamera* playcamera;
	HandGenerator* handgenerator;
	GLRenderer* glcamera;

	std::string _camtype;

	cv::Mat cam_depth;
	cv::Mat cam_depth16;
	cv::Mat cam_color;

	int width;
	int height;

	int _frame;

	cv::Mat calib_mat;
};