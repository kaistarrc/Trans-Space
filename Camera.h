#pragma once

#include "PlayCamera.h"
#include "RealSenseCVWrapper.h"
#include "HandParameter.h"
#include "GLRenderer.h"
#include "Hand.h"
#include "HandGenerator.h"

#include "NYUCamera.h"

class Camera{

public:

	int _frame;

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
		else if (camtype.compare("nyucamera") == 0)
			nyucamera = new NYUCamera(w, h);
		else{
			handgenerator = hg;
			glcamera = gr;
		}
		

		cam_color = cv::Mat(h, w, CV_8UC3);
		cam_depth16 = cv::Mat(h, w, CV_16UC1);
		cam_depth = cv::Mat(h, w, CV_32FC1);
		calib_mat = cv::Mat(3, 3, CV_32FC1);

		calib_mat.at<float>(0, 0) = 477.9; calib_mat.at<float>(0, 1) = 0.0; calib_mat.at<float>(0, 2) = 320.0;
		calib_mat.at<float>(1, 0) = 0.0; calib_mat.at<float>(1, 1) = 477.9; calib_mat.at<float>(1, 2) = 240.0;
		calib_mat.at<float>(2, 0) = 0.0; calib_mat.at<float>(2, 1) = 0.0; calib_mat.at<float>(2, 2) = 1.0;

		_frame = 0;
		//_frame = 2;
		
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
			if (key == 'z'){
				_frame++;
				printf("frame:%d\n", _frame);
			}
			else if (key == 'x'){
				//_frame--;
				_frame += 4 * 4 * 4 - 10;
				printf("frame:%d\n", _frame);
			}

			
		}
		else if (_camtype == "nyucamera"){
			if (nyucamera->queryFrames(_frame) == false)
				return false;

		}

		else if (_camtype.compare("glcamera_gui") == 0){
			handgenerator->_trackbar.run();// run_trackbar();

			if (cv::waitKey(1) == 's')
				handgenerator->save_trackbar();
			if (cv::waitKey(1) == 'l')
				handgenerator->load_trackbar();
		}
		else if (_camtype.compare("glcamera_cnn_dataset") == 0){
			if (handgenerator->_posesetgenerator.run_cnndataset() == -1)
				return false;
		}
		else if (_camtype.compare("glcamera_sequence") == 0){
			//if (handgenerator->_posesetgenerator.run_sequence() == -1)
			//if (handgenerator->_posesetgenerator.run_sequence_between_FingerTest() == -1)
			//if (handgenerator->_posesetgenerator.run_sequence_between_includingOpenPalm() == -1)
			if (handgenerator->_posesetgenerator.run_sequence_between26() == -1)
			//if (handgenerator->_posesetgenerator.run_sequence_between52() == -1)
				return false;
		}
		else if (_camtype.compare("glcamera_test") == 0){
			if (handgenerator->_posesetgenerator.test() == -1)
				return false;
		}

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
		else if (_camtype == "nyucamera"){
			nyucamera->getDepthBuffer(cam_depth);

		}

		else if (_camtype.compare("glcamera_gui") == 0){
			handgenerator->run_gui2hand("depth");
			glFinish();
			glcamera->getOrigImage(cam_depth, "depth");
		}
		else if (_camtype.compare("glcamera_cnn_dataset") == 0){
			handgenerator->run_posegenerator2hand("depth");
			glFinish();
			glcamera->getOrigImage(cam_depth, "depth");
		}
		else if (_camtype.compare("glcamera_sequence") == 0){
			handgenerator->run_posegenerator2hand("depth");
			glFinish();
			glcamera->getOrigImage(cam_depth, "depth");
		}
		else if (_camtype.compare("glcamera_test") == 0){
			handgenerator->run_posegenerator2hand("depth");
			glFinish();
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
		else if (_camtype.compare("glcamera_gui") == 0){
			handgenerator->run_gui2hand("color");
			glFinish();
			glcamera->getOrigImage(cam_color, "color");
		}
		else if (_camtype == "nyucamera")
			nyucamera->getColorBuffer(cam_color);
		else if (_camtype.compare("glcamera_cnn_dataset") == 0){
			handgenerator->run_posegenerator2hand("color");
			glFinish();
			glcamera->getOrigImage(cam_color, "color");
		}
		else if (_camtype.compare("glcamera_sequence") == 0){
			handgenerator->run_posegenerator2hand("color");
			glFinish();
			glcamera->getOrigImage(cam_color, "color");
		}
		else if (_camtype.compare("glcamera_test") == 0){
			handgenerator->run_posegenerator2hand("color");
			glFinish();
			glcamera->getOrigImage(cam_color, "color");
		}
		//cv::flip(cam_color, cam_color, 1);


		//change color to white.
		/*
		if (_camtype != "realcamera" && _camtype !="nyucamera" ){
			for (int i = 0; i < width; i++)
			for (int j = 0; j < height; j++)
			{
				unsigned char b = cam_color.at<uchar>(j, 3 * i + 0);
				unsigned char g = cam_color.at<uchar>(j, 3 * i + 1);
				unsigned char r = cam_color.at<uchar>(j, 3 * i + 2);

				if (b>0 || g>0 || r > 0)
				{

					cam_color.at<uchar>(j, 3 * i + 0) = 125;
					cam_color.at<uchar>(j, 3 * i + 1) = 125;
					cam_color.at<uchar>(j, 3 * i + 2) = 125;
				}
			}
		}
		*/
		

		cam_color.copyTo(out);
	}

	void recordFramesTraining(int posname,cv::Mat img){
		printf("[%d]recording frame..:%d\n", posname, _frame);

		for (int i = 0; i < width; i++)
		for (int j = 0; j < height; j++)
			cam_depth16.at<ushort>(j, i) = img.at<float>(j, i);

		char filename[200];
		sprintf(filename, "save/sequence/train/%d/depth-%07u.png",posname, _frame);

		cv::imwrite(filename, cam_depth16);

	}


	void recordFrames(std::string opt){
		printf("Record frame: %d \n",_frame);

		//for my algorithm
		if (opt == "all" || opt == "uvr"){
			{
				for (int i = 0; i < width; i++)
				for (int j = 0; j < height; j++)
					cam_depth16.at<ushort>(j, i) = cam_depth.at<float>(j, i);

				char filename[200];
				sprintf(filename, "save/sequence/vga/depth-%07u.png", _frame);

				cv::imwrite(filename, cam_depth16);

			}

			{
			char filename[200];
			sprintf(filename, "save/sequence/vga/color-%07u.png", _frame);
			cv::imwrite(filename, cam_color);
			}
		}

		//for epfl algorithm
		if (opt == "all" || opt == "epfl"){
			{
				char filename[200];
				sprintf(filename, "save/sequence/qvga/color-%07u.png", _frame);

				cv::Mat color320t = cv::Mat(height / 2, width / 2, CV_8UC3);
				cv::Mat color320 = cv::Mat(height / 2, width / 2, CV_8UC3);
				cv::resize(cam_color, color320t, cv::Size(width / 2, height / 2), 0, 0, 1);
				for (int i = 0; i < width / 2; i++)
				for (int j = 0; j < height / 2; j++){
					color320.at<uchar>(j, 3 * i + 0) = color320t.at<uchar>(j, 3 * i + 2);
					color320.at<uchar>(j, 3 * i + 1) = color320t.at<uchar>(j, 3 * i + 1);
					color320.at<uchar>(j, 3 * i + 2) = color320t.at<uchar>(j, 3 * i + 0);
				}
				cv::imwrite(filename, color320);
			}

			{
			char filename[200];
			sprintf(filename, "save/sequence/qvga/depth-%07u.png", _frame);

			cv::Mat depth320 = cv::Mat(height / 2, width / 2, CV_16UC1);
			cv::resize(cam_depth16, depth320, cv::Size(width / 2, height / 2), 0, 0, 1);
			cv::imwrite(filename, depth320);

			}
		}
		
	}

	void getCalibrationMatrix(cv::Mat& out){
		calib_mat.copyTo(out);

	}


	void releaseFrames(){

		if (_camtype.compare("realcamera") == 0)
			realcamera->releaseFrames();
		
	}
	

	PlayCamera* playcamera;

private:

	RealSenseCVWrapper* realcamera;
	HandGenerator* handgenerator;
	GLRenderer* glcamera;
	NYUCamera* nyucamera;

	std::string _camtype;

	cv::Mat cam_depth;
	cv::Mat cam_depth16;
	cv::Mat cam_color;

	int width;
	int height;

	

	cv::Mat calib_mat;
};