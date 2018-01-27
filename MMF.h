#pragma once

#include <windows.h>
#pragma once
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#pragma comment(lib,"opencv_world310.lib")

class MMF
{
public:
	MMF(int, int);
	~MMF();

	int receiveData();
	//void getLabel(std::vector<float>&);
	void getLabel(float*);
	void send2CNN();
	cv::Mat _cnnimg;

	bool getimg_bool;
protected:

	int DATA_LEN;
	int DATA_LEN2;
	//learning
	float* lpMapping_send;
	HANDLE hMapWrite;
	HANDLE hEvent, hEvent2;

	int width, height;

	//receive
	float* lpMapping_receive;
	HANDLE hMapRead;
	//char memoryName_read[100];

	float* DL_result; 

};