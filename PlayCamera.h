#pragma once
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>

using namespace std;

class PlayCamera
{
public:

	PlayCamera(){

	}

	PlayCamera(int w, int h){
		width = w;
		height = h;

		cameraMatrix = cv::Mat(3, 3, CV_32FC1);

		//depth = cv::Mat(height, width, CV_16UC1);

		_frame = 0;//120;//200;

		//

	}

	void getCalibrationStatus(){
		cameraMatrix.at<float>(0, 0) = 477.9; cameraMatrix.at<float>(0, 1) = 0.0; cameraMatrix.at<float>(0, 2) = 326.6;
		cameraMatrix.at<float>(1, 0) = 0.0; cameraMatrix.at<float>(1, 1) = 477.9; cameraMatrix.at<float>(1, 2) = 245.9;
		cameraMatrix.at<float>(2, 0) = 0.0; cameraMatrix.at<float>(2, 1) = 0.0; cameraMatrix.at<float>(2, 2) = 1.0;

	}


	int queryFrames(int framein){

		{
			string data_path = "save/sequence/";
			ostringstream stringstream;
			stringstream << std::setw(7) << std::setfill('0') << framein;
			rgb = cv::imread(data_path + _testType + "/color-" + stringstream.str() + ".png", 1);
			if (rgb.empty())
				return false;
		}

		{
			string data_path = "save/sequence/";
			ostringstream stringstream;
			stringstream << std::setw(7) << std::setfill('0') << framein;
			depth = cv::imread(data_path + _testType + "/depth-" + stringstream.str() + ".png", 2);
			if (depth.empty())
				return false;
		}


		//color read
		//char filename[200];
		//sprintf(filename, "save/sequence/vga/color-%07u.png", framein);
		//rgb = cv::imread(filename, 1);
		//if (rgb.empty())
		//	return false;

		//depth read
		//sprintf(filename, "save/sequence/vga/depth-%07u.png", framein);
		//depth = cv::imread(filename, 2);
		//if (depth.empty())
		//	return false;

		return true;
	}

	int queryFrames_orig(int framein){
		//color read
		char filename[200];
		sprintf(filename, "save/sequence/vga/color-%07u.png", framein);
		rgb = cv::imread(filename, 1);
		if (rgb.empty())
			return false;

			

		//depth read
		sprintf(filename, "save/sequence/vga/depth-%07u.png",  framein);
		depth = cv::imread(filename, 2);	
		if (depth.empty())
			return false;
	
		//sprintf(filename, "save/cnn/train/data/depth-%07u.png", framein);
		//depth = cv::imread(filename, 2);
		

		cv::Mat rr = rgb;
		cv::Mat dd = depth;
		cv::waitKey(1);
		

		return true;
	}

	void getLearningImages(cv::Mat& out){
		char filename[200];
		sprintf(filename, "record/test_posture/data/data%d.png", _frame);
		out = cv::imread(filename, 0);


	}


	void getColorBuffer(cv::Mat& out){
		//out = rgb;
		rgb.copyTo(out);
	}

	void getDepthBuffer(cv::Mat& out){
		//out = depth;
		depth.copyTo(out);
	}

	



	cv::Mat cameraMatrix;
	int _frame;

	string _testType;

private:
	int width, height;

	cv::Mat rgb, depth_3code, depth;

};