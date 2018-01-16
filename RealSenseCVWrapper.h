#pragma once
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

#include <RealSense\SenseManager.h>
#include <RealSense\SampleReader.h>

#pragma comment(lib,"libpxc.lib")
#pragma comment(lib,"libpxcutils.lib")
#pragma comment(lib,"opencv_world310.lib")


class RealSenseCVWrapper
{
public:
	// constructor
	// @param
	//     width : width of picture size
	//     height: height of picture size
	RealSenseCVWrapper();
	RealSenseCVWrapper(int width, int height);
	~RealSenseCVWrapper();
	void safeRelease();

	// query RealSense frames
	// you should execute queryStream() function before get buffers
	// and releaseFrames(); function after all by every frame
	// @return
	//     true = frame is succcessfully queried
	bool queryFrames();
	void releaseFrames();

	// get camera buffers
	void getColorBuffer();
	void getDepthBuffer();
	void getMappedDepthBuffer();
	void getMappedColorBuffer();
	void getXYZBuffer();
	void getColorBuffer(cv::Mat &color);
	void getDepthBuffer(cv::Mat &depth);
	void getMappedDepthBuffer(cv::Mat &mappedDepth);
	void getMappedColorBuffer(cv::Mat &mappedColor);
	void getXYZBuffer(std::vector<cv::Point3f> xyz);
	void getDepth3cBuffer(cv::Mat &);

	void makeDepthFromColorcodeDepth(cv::Mat&);

	// adjust camera settings
	// @param
	//     use_aa: flag to use auto exposure and auto white balance
	void useAutoAdjust(bool use_aa);
	// set RGB camera exposure
	// @param
	//     value: exposure time = 2 ^ value [s]
	//            if you want 30fps you should set under 33 ~ 2 ^ -5 [ms]
	void setExposure(int32_t value);
	// set RGB camera white balance
	// @param
	//     value: white point = value [K]
	//            (ex: D65 standard light source = about 6500[K])
	void setWhiteBalance(int32_t value);

	// get calibration status
	//  - camera matrix
	//  - distortion parameters
	//  - transform matrix (RGB camera origin to depth camera origin)
	void getCalibrationStatus();

	// OpenCV image buffer
	cv::Size bufferSize;        // buffer size
	cv::Mat colorBuffer;        // RGB camera buffer (BGR, 8UC3)
	cv::Mat depthBuffer;        // depth camera buffer (Gray, 32FC1)
	cv::Mat colorBufferMapped;  // RGB camera buffer mapped to depth camera (BGR, 8UC3)
	cv::Mat depthBufferMapped;  // depth camera buffer mapped to RGB camera (Gray, 32FC1)
	std::vector<cv::Point3f> xyzBuffer;           // XYZ point cloud buffer from depth camera (XYZ)

	// RGB camera calibration data writtern by OpenCV camera model
	cv::Mat cameraMatrix;       // inclueds fx,fy,cx,cy
	cv::Mat distCoeffs;         // k1,k2,p1,p2,k3
	cv::Mat transform;          // 4x4 coordinate transformation from RGB camera origin to the world (=depth) system origin

protected:
	Intel::RealSense::SenseManager *rsm;
	Intel::RealSense::Sample *sample;


	int width, height;
	cv::Mat depth3c;//cv::Mat coloredDepth;

};