#pragma once

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

//for matlab
#include <stdlib.h>
#include "mat.h"

#pragma comment(lib, "libmx.lib") 
#pragma comment(lib, "libmat.lib")
#pragma comment(lib, "libeng.lib")


class NYUCamera
{
public:
	NYUCamera();
	NYUCamera(int, int);
	~NYUCamera();

	//int getFrame();
	bool queryFrames(int);
	void getCalibrationStatus();
	void getColorBuffer(cv::Mat&);
	void getDepthBuffer(cv::Mat&);
	void getSynthBuffer(cv::Mat&);

	void showJoint(int);
	//void setWristQuaternion(int, int);
	//void setFingerAngle(int fingerid);
	//void setFingerAngle_backup(int fingerid);//adhoc




	void getWristQuaternion(float* inout){
		for (int i = 0; i < 4; i++)
			inout[i] = quaternionW[i];

	}
	void getWristPosition(float* inout,int _frame){
		for (int i = 0; i < 3; i++)
			inout[i] = joint_xyz[_frame].at<float>(i, 35);

	}

	void getFingerAngle(float* inout){
		//for (int i = 0; i < 5; i++)
		//	inout[i] = angleF[i];
		for (int i = 0; i < 20; i++)
			inout[i] = angleF[i];
	}

	

	void getGroundtruthJoint2D(int f, int j, float* inout){
		inout[0] = joint_uvd[f].at<float>(0, j);
		inout[1] = joint_uvd[f].at<float>(1, j);

	}
	void getGroundtruthJoint(int f, int j, float* inout){

		for (int k = 0; k < 3; k++)
			inout[k] = joint_xyz[f].at<float>(k, j);

		/*
		float z_ = joint_uvd[f].at<float>(2, j);
		float x_ = joint_uvd[f].at<float>(0, j) * z_;
		float y_ = joint_uvd[f].at<float>(1, j) * z_;

		float x=cameraMatrix_inv.at<float>(0, 0)*x_ + cameraMatrix_inv.at<float>(0, 1)*y_ + cameraMatrix_inv.at<float>(0, 2)*z_;
		float y=cameraMatrix_inv.at<float>(1, 0)*x_ + cameraMatrix_inv.at<float>(1, 1)*y_ + cameraMatrix_inv.at<float>(1, 2)*z_;
		float z=cameraMatrix_inv.at<float>(2, 0)*x_ + cameraMatrix_inv.at<float>(2, 1)*y_ + cameraMatrix_inv.at<float>(2, 2)*z_;

		float xg = joint_xyz[f].at<float>(0, j);
		float yg = joint_xyz[f].at<float>(1, j);
		float zg = joint_xyz[f].at<float>(2, j);
		printf("dx:%f dy:%f dz:%f\n",x-dx,y-dy,z-dz);
		*/
	}
	void getPredictedJoint2D(int f, int j, float* inout){

		for (int k = 0; k < 2; k++)
			inout[k] = joint_uvc_pred[f].at<float>(k, j);
	}


	float augmentDepth(float z_, int f, int j, int msize){
		float out;

		if (z_ <= 0){
			float c = 0;
			float dsum = 0;

			for (int ii = -msize; ii < msize; ii++)
			for (int jj = -msize; jj < msize; jj++){

				int x_ = joint_uvc_pred[f].at<float>(0, j);
				int y_ = joint_uvc_pred[f].at<float>(1, j);

				float d = synth_data.at<float>(y_ + jj, x_ + ii);

				if (d>0 & d<1000){
					dsum += d;
					c += 1;
				}
			}
			if (c == 0)
				return -1;

			out = dsum / c;
			//printf("dsum:%f ,c:%f, out:%f\n", dsum, c,out);
		}

		return out;
	}

	void getPredictedJoint(int f, int j, float* inout){

		float z_ = depth_data.at<float>(joint_uvc_pred[f].at<float>(1, j), joint_uvc_pred[f].at<float>(0, j));

		//if (z_ == 0)
		//	z_ = synth_data.at<float>(joint_uvc_pred[f].at<float>(1, j), joint_uvc_pred[f].at<float>(0, j));
		//if (z_ == 0)
		//	z_ = joint_xyz[f].at<float>(2, j);

		//augment depth 
		if (z_ <= 0 | z_ > 1000){
			float msize[4] = { 3, 5, 7, 9 };

			for (int ii = 0; ii < 4; ii++){
				z_ = augmentDepth(z_, f, j, msize[ii]);

				if (z_>0 & z_ < 1000)
					break;
			}

			if (z_ == -1)
				z_ = joint_xyz[f].at<float>(2, 32);
		}

		//unproject 

		float x_ = joint_uvc_pred[f].at<float>(0, j) * z_;
		float y_ = joint_uvc_pred[f].at<float>(1, j) * z_;

		inout[0] = cameraMatrix_inv.at<float>(0, 0)*x_ + cameraMatrix_inv.at<float>(0, 1)*y_ + cameraMatrix_inv.at<float>(0, 2)*z_;
		inout[1] = cameraMatrix_inv.at<float>(1, 0)*x_ + cameraMatrix_inv.at<float>(1, 1)*y_ + cameraMatrix_inv.at<float>(1, 2)*z_;
		inout[2] = cameraMatrix_inv.at<float>(2, 0)*x_ + cameraMatrix_inv.at<float>(2, 1)*y_ + cameraMatrix_inv.at<float>(2, 2)*z_;


		//augment depth 
		/*
		if (inout[2] == 0){
		float c = 0;
		float dout = 0;

		for (int ii = -3; ii < 3; ii++)
		for (int jj = -3; jj < 3; jj++){

		float d = synth_data.at<float>(inout[1] + jj, inout[0] + ii);

		dout += d;
		if (d>0)
		c += 1;
		}
		inout[2] = dout / c;
		}
		*/
	}

	//Kinematics _kinematics;


	void releaseFrames();


	std::string savetype;
	cv::Mat depth_data;
	cv::Mat synth_data;
	cv::Mat rgb_data;

	

	//Preprocessing preprocessing;
	cv::Mat cameraMatrix;
	cv::Mat cameraMatrix_inv;

	//joint label
	void getJoint(cv::Mat& inout, int fidx){
		inout = joint_xyz[fidx];
	}

	int joint_max;
private:


	//int makeImage_NYU2014();

	float com[3];
	float cropbox[4];

	int width;
	int height;

	cv::Mat synth_img;

	//joint label
	void setJointLabel();
	//float calculateAnglefromVectors(float*, float*);
	int diagnose(const char*);
	int frame_max;

	//double* joint_xyz;
	//double* joint_uvd;
	//double* joint_uvc_pred;
	cv::Mat* joint_xyz;
	cv::Mat* joint_uvd;
	cv::Mat* joint_uvc_pred;
	double*  joint_mat;


	float rad2deg;



	//Quaternion with rigid rotation
	//RigidRotation rigidRotation;
	//InverseKinematics IK;
	float quaternionW[4];
	float angleF[20];

	float* hand_param;

	//pose reader
	//PoseReader palmReader;
};