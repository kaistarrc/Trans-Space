#pragma once

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

#include "Camera.h"

class Preprocessing
{
public:
	int width, height;
	cv::Mat calib_mat;
	cv::Mat calib_inv;

	float cropbox[4];

	cv::Mat cnnimg;
	cv::Mat mask1;
	cv::Mat mask2;
	cv::Mat maskwrist;
	cv::Mat vimg;

	cv::Mat vecimg;

	Preprocessing(){

	}

	Preprocessing(int w, int h, Camera camera){
		width = w;
		height = h;
		camera.getCalibrationMatrix(calib_mat);// c_mat;
		cv::invert(calib_mat, calib_inv);

		cnnimg = cv::Mat(128, 128, CV_32FC1);
		mask1 = cv::Mat(height, width, CV_8UC1);
		mask2 = cv::Mat(height, width, CV_8UC1);

		maskwrist = cv::Mat(height, width, CV_8UC1);
		vimg = cv::Mat(height, width, CV_8UC1);

		vecimg = cv::Mat(height, width, CV_32FC1);
	}

	~Preprocessing(){

	}

	void project3Dto2D(float* in, float* out)
	{
		float s = calib_mat.at<float>(2, 0) * in[0] + calib_mat.at<float>(2, 1) * in[1] + calib_mat.at<float>(2, 2) * in[2];
		out[0] = (calib_mat.at<float>(0, 0) * in[0] + calib_mat.at<float>(0, 1) * in[1] + calib_mat.at<float>(0, 2) * in[2]) / s;
		out[1] = (calib_mat.at<float>(1, 0) * in[0] + calib_mat.at<float>(1, 1) * in[1] + calib_mat.at<float>(1, 2) * in[2]) / s;

		if (out[0] <= 0)
			out[0] = 0;
		if (out[0]>width - 1)
			out[0] = width - 1;

		if (out[1] <= 0)
			out[1] = 0;
		if (out[1]>height - 1)
			out[1] = height - 1;

	}

	void project3Dcube2ImagePlane(float* com, float* cropbox)
	{
		//second cropbox
		/*
		float cube_p1[3] = { com[0] - 100, com[1] - 100, com[2] };
		float cube_p2[3] = { com[0] + 100, com[1] + 70, com[2] };
		*/
		float cube_p1[3] = { com[0] - 150, com[1] - 150, com[2] };
		float cube_p2[3] = { com[0] + 150, com[1] + 150, com[2] };

		project3Dto2D(cube_p1, &cropbox[0]);
		project3Dto2D(cube_p2, &cropbox[2]);
		//visualizeBoundary(img_in, cropbox, "crop_bound");

	}

	void showCropBoundary(cv::Mat vis_img)
	{
		//draw 2D line (boundary of crop)

		cv::line(vis_img, cvPoint(cropbox[0], cropbox[1]), cvPoint(cropbox[0], cropbox[3]), cvScalarAll(255), 3, 8, 0);
		cv::line(vis_img, cvPoint(cropbox[0], cropbox[1]), cvPoint(cropbox[2], cropbox[1]), cvScalarAll(255), 3, 8, 0);
		cv::line(vis_img, cvPoint(cropbox[2], cropbox[1]), cvPoint(cropbox[2], cropbox[3]), cvScalarAll(255), 3, 8, 0);
		cv::line(vis_img, cvPoint(cropbox[0], cropbox[3]), cvPoint(cropbox[2], cropbox[3]), cvScalarAll(255), 3, 8, 0);

		// segment crop image.
		/*
		for (int i = 0; i < width;i++)
		for (int j = 0; j < height; j++)
		{
		if (i >= cropbox[0]  & i < cropbox[2])
		if (j >= cropbox[1]  & j < cropbox[3])
		continue;

		for (int k = 0; k < 3; k++)
		vis_img.at<uchar>(j, 3 * i + k) = 0;
		}
		*/
		cv::imshow("crop", vis_img);


	}

	void cropImage(cv::Mat& img, float* cube)
	{
		//project3Dcube2ImagePlane(com, cropbox);

		//crop
		cv::Rect rect(cube[0], cube[2], cube[1] - cube[0], cube[3] - cube[2]);
		img = img(rect);
	}

	void makeTrackingImage(cv::Mat in, cv::Mat& out)
	{



		////////////
		//float com[3] = { 0, 0, 300 };
		float com[3] = { 0 };
		if (getComHand(in, com) == -1){
			com[0] = 0; com[1] = 0; com[2] = 300;
		}

		//zoom test
		cv::Mat aa = cv::Mat(height, width, CV_32FC1);
		cv::Mat aa_resized;
		aa.setTo(0);
		for (int i = 0; i < width; i++)
		for (int j = 0; j < height; j++){
			float x_ = i*in.at<float>(j, i);
			float y_ = j*in.at<float>(j, i);
			float z_ = in.at<float>(j, i);

			float x = calib_inv.at<float>(0, 0)*x_ + calib_inv.at<float>(0, 1)*y_ + calib_inv.at<float>(0, 2)*z_;
			float y = calib_inv.at<float>(1, 0)*x_ + calib_inv.at<float>(1, 1)*y_ + calib_inv.at<float>(1, 2)*z_;
			float z = calib_inv.at<float>(2, 0)*x_ + calib_inv.at<float>(2, 1)*y_ + calib_inv.at<float>(2, 2)*z_;

			x = x - com[0];
			y = y - com[1];
			z = z - com[2] + 300;

			x_ = calib_mat.at<float>(0, 0)*x + calib_mat.at<float>(0, 1)*y + calib_mat.at<float>(0, 2)*z;
			y_ = calib_mat.at<float>(1, 0)*x + calib_mat.at<float>(1, 1)*y + calib_mat.at<float>(1, 2)*z;
			z_ = calib_mat.at<float>(2, 0)*x + calib_mat.at<float>(2, 1)*y + calib_mat.at<float>(2, 2)*z;

			x_ = x_ / z_;
			y_ = y_ / z_;

			if (x_ >= 0 & x_<width & y_ >= 0 & y_<height)
				aa.at<float>(y_, x_) = in.at<float>(j, i);
		}
		//cv::medianBlur(aa, aa, 7);
		/*
		cv::resize(aa, aa_resized, cv::Size(128, 128), 0, 0, 3);//3
		cv::imshow("aa", aa);
		cv::imshow("aa_resized", aa_resized);

		out = aa_resized;
		*/
		//zoom test

		/*
		cv::Mat in_crop = in;
		cropImage(in_crop, com);
		//printf("com:%f %f %f\n", com[0], com[1], com[2]);
		//printf("box:%f %f %f %f\n", cropbox[0], cropbox[1], cropbox[2], cropbox[3]);

		cv::Mat in_resized;
		cv::resize(in_crop, in_resized, cv::Size(128, 128), 0, 0, CV_INTER_NN);
		out = in_resized;
		*/


		cv::Mat in_resized;
		cv::resize(in, in_resized, cv::Size(128, 128), 0, 0, CV_INTER_NN);
		out = in_resized;

		//out = in_crop;


		//cv::Mat in_temp = in;
		//cv::Mat in_temp;
		//in.copyTo(in_temp);
		//int bx0 = cropbox[0]; int by0 = cropbox[1];
		//int bx1 = cropbox[2]; int by1 = cropbox[3];
		//cv::imshow("boundary", in_temp);

		//cv::imshow("crop", in_crop);
		//cv::imshow("resized", in_crop);
	}

	void comToBounds(float* com, float* cubesize, float* cube){
		float fx = calib_mat.at<float>(0, 0);
		float fy = calib_mat.at<float>(1, 1);

		cube[4] = com[2] - cubesize[2] / 2.;//zstart
		cube[5] = com[2] + cubesize[2] / 2.;//zend
		cube[0] = int((com[0] * com[2] / fx - cubesize[0] / 2.) / com[2] * fx);//xstart
		cube[1] = int((com[0] * com[2] / fx + cubesize[0] / 2.) / com[2] * fx);//xend
		cube[2] = int((com[1] * com[2] / fy - cubesize[1] / 2.) / com[2] * fy);//ystart
		cube[3] = int((com[1] * com[2] / fy + cubesize[1] / 2.) / com[2] * fy);//yend

		if (cube[0] < 0)
			cube[0] = 0;
		if (cube[1] >= width)
			cube[1] = width;
		if (cube[2] < 0)
			cube[2] = 0;
		if (cube[3] >= height)
			cube[3] = height;
		if (cube[4] < 0)
			cube[4] = 0;

		
	}

	void normalizeImage(float* com, float* cubesize, cv::Mat& inout)
	{
		//normalize -1~1
		for (int i = 0; i < width; i++)
		for (int j = 0; j < height; j++){
			if (inout.at<float>(j, i) == 0)
				inout.at<float>(j, i) = com[2] + cubesize[2] / 2.0;
			//normZeroOne
			//in.at<float>(j, i) -= (com[2] - cube[2] / 2.0);
			inout.at<float>(j, i) -= com[2];

			inout.at<float>(j, i) /= (cubesize[2] / 2.0);
		}
		/*
		for (int i = 0; i < width; i++)
		for (int j = 0; j < height; j++){
		if (inout.at<float>(j, i) <= -1)
		inout.at<float>(j, i) = -1;
		if (inout.at<float>(j, i) >= 1)
		inout.at<float>(j, i) = 1;
		}
		*/

	}

	void segmentHandFromBand(cv::Mat color, cv::Mat& out)
	{
		//cv::Mat vecimg = cv::Mat(height, width, CV_32FC1);
		mask1.setTo(0);
		mask2.setTo(0);

		cv::Mat depth_t;
		out.copyTo(depth_t);

		//convert rgb to yuv
		cv::Mat yuv;
		cv::cvtColor(color, yuv, CV_BGR2YUV);

		//set mask based on v. (maskwrist)
		for (int i = 0; i < width; i++)
		for (int j = 0; j < height; j++)
		{
			vimg.at<uchar>(j, i) = yuv.at<uchar>(j, 3 * i + 2);

			maskwrist.at<uchar>(j, i) = 0;
			if (vimg.at<uchar>(j, i)>130)
				maskwrist.at<uchar>(j, i) = 255;
		}
		//cv::imshow("mask",maskwrist);
		
		//calculate depth of wrist band.
		float wristcom[3] = { 0, 0, 0 };
		float count = 0;
		for (int i = 0; i < width; i++)
		for (int j = 0; j < height; j++)
		{
			if (maskwrist.at<uchar>(j, i) == 255){
				wristcom[0] += i;
				wristcom[1] += j;
				wristcom[2] += depth_t.at<float>(j, i);
				count += 1.0;
			}
		}
		wristcom[0] /= count;
		wristcom[1] /= count;
		wristcom[2] /= count;

		//segment first based on z[mm] of wrist. segment in front of band.
		for (int i = 0; i < width; i++)
		for (int j = 0; j < height; j++){
			float d = abs(depth_t.at<float>(j, i) - wristcom[2]);

			//mask1.at<uchar>(j, i) = 255;
			if (depth_t.at<float>(j, i) < wristcom[2] | d < 20)
				//if (depth_t.at<float>(j, i) < wristcom[2] )
				mask1.at<uchar>(j, i) = 255;
			//depth_t.at<float>(j, i) = 0;
		}
		
		

		//test
		//calculate com of segmented hand first.

		{
			float com2d[2] = { 0, 0 };
			float count = 0;
			for (int i = 0; i < width; i++)
			for (int j = 0; j < height; j++){
				if (mask1.at<uchar>(j, i) == 0)
					continue;
				com2d[0] += i;
				com2d[1] += j;
				count += 1;
			}
			com2d[0] /= count;
			com2d[1] /= count;

			//segment finally based vector (wrist-com2d) / (wrist-hand)
			cv::Mat vec_ref = cv::Mat(2, 1, CV_32FC1);
			vec_ref.at<float>(0, 0) = com2d[0] - wristcom[0];
			vec_ref.at<float>(1, 0) = com2d[1] - wristcom[1];
			cv::Mat vec1 = cv::Mat(2, 1, CV_32FC1);

			
			for (int i = 0; i < width; i++)
			for (int j = 0; j < height; j++){
				if (depth_t.at<float>(j, i) == 0)
					continue;

				//vector
				vec1.at<float>(0, 0) = i - wristcom[0];
				vec1.at<float>(1, 0) = j - wristcom[1];

				//calculate theta
				//float a = vec1.dot(vec_ref) / (cv::norm(vec1)*cv::norm(vec_ref));
				float v1x = vec1.at<float>(0, 0); float v1y = vec1.at<float>(1, 0);
				float v2x = vec_ref.at<float>(0, 0); float v2y = vec_ref.at<float>(1, 0);
				float a = (v1x*v2x + v1y*v2y) / (sqrt(v1x*v1x + v1y*v1y)*sqrt(v2x*v2x + v2y*v2y));

		
				vecimg.at<float>(j, i) = a;

				//if (a < 0 | (a>20))
				if (a<0.5)
					mask2.at<uchar>(j, i) = 255;
			}
			

		}
		

		//test
		for (int i = 0; i < width; i++)
		for (int j = 0; j < height; j++){
			//if (mask2.at<uchar>(j, i) == 255 | maskwrist.at<uchar>(j,i)==255 )
			if (maskwrist.at<uchar>(j, i) == 255 | mask2.at<uchar>(j, i) == 255)
				out.at<float>(j, i) = 0;
		}

		
		//cv::imshow("maskwrist", maskwrist);
		//cv::imshow("mask1", mask1);
		//cv::imshow("mask2", mask2);
		//depth_t.copyTo(out);

	}

	void getCnnImage(cv::Mat& inout){
		cnnimg.copyTo(inout);
		//inout = cnnimg;
	}

	void makeCnnImage(cv::Mat in)
	{
		//calculate com

		//float cube[3] = { 300, 300, 300 };
		float cubesize[3] = { 200, 200, 200 };
		float cube[6];
		float com[3] = { 0 };
		if (getComHand(in, com) == -1){
			com[0] = width / 2; com[1] = height / 2; com[2] = 200;
		}

		
		//3D bound from 2.5d CoM.
		comToBounds(com, cubesize, cube);

		//printf("cube: %f %f %f %f %f %f\n", cube[0], cube[1], cube[2], cube[3], cube[4], cube[5]);

		//normalize -1~1
		cv::Mat in_norm;
		in.copyTo(in_norm);
		normalizeImage(com, cubesize, in_norm);

		//crop image
		cv::Mat in_crop = in_norm;
		cropImage(in_crop, cube);

		//resize image to 128*128 size.
		cv::Mat in_resized;
		cv::resize(in_crop, in_resized, cv::Size(128, 128), 0, 0, CV_INTER_NN);
		in_resized.copyTo(cnnimg);


		//cv::Mat in_temp = in;
		cv::Mat in_temp;
		in.copyTo(in_temp);
		int bx0 = cube[0]; int by0 = cube[2];
		int bx1 = cube[1]; int by1 = cube[3];

		cv::line(in_temp, cv::Point(bx0, by0), cv::Point(bx0, by1), cv::Scalar(255), 1, 8, 0);
		cv::line(in_temp, cv::Point(bx0, by0), cv::Point(bx1, by0), cv::Scalar(255), 1, 8, 0);
		cv::line(in_temp, cv::Point(bx0, by1), cv::Point(bx1, by1), cv::Scalar(255), 1, 8, 0);
		cv::line(in_temp, cv::Point(bx1, by0), cv::Point(bx1, by1), cv::Scalar(255), 1, 8, 0);
		//cv::imshow("boundary", in_temp);

	}


	void makeXyzImage(cv::Mat depth, cv::Mat& out)
	{
		float com[3] = { 0 };
		getComHand(depth, com);

		for (int i = 0; i < width; i++)
		for (int j = 0; j < height; j++){
			if (depth.at<float>(j, i) == 0)
			{
				out.at<float>(j, 3 * i + 0) = 0;// 1000;
				out.at<float>(j, 3 * i + 1) = 0;// 1000;
				out.at<float>(j, 3 * i + 2) = 0;// 1000;
				continue;
			}

			float xt = depth.at<float>(j, i)*i;
			float yt = depth.at<float>(j, i)*j;
			float zt = depth.at<float>(j, i);

			float x = calib_inv.at<float>(0, 0)*xt + calib_inv.at<float>(0, 1)*yt + calib_inv.at<float>(0, 2)*zt;
			float y = calib_inv.at<float>(1, 0)*xt + calib_inv.at<float>(1, 1)*yt + calib_inv.at<float>(1, 2)*zt;
			float z = calib_inv.at<float>(2, 0)*xt + calib_inv.at<float>(2, 1)*yt + calib_inv.at<float>(2, 2)*zt;

			//out.at<float>(j, 3 * i +0) = x;
			//out.at<float>(j, 3 * i +1) = y;
			//out.at<float>(j, 3 * i +2) = z;

			out.at<float>(j, 3 * i + 0) = x - com[0];
			out.at<float>(j, 3 * i + 1) = y - com[1];
			out.at<float>(j, 3 * i + 2) = z - com[2];
		}
	}

	int getComHandxyz(cv::Mat depth, float* com){
		float c = 0;
		com[0] = 0; com[1]=0; com[2]=0;
		for (int i = 0; i < width; i++)
		for (int j = 0; j < height; j++){

			if (depth.at<float>(j, i) == 0)
				continue;

			float xt = depth.at<float>(j, i)*i;
			float yt = depth.at<float>(j, i)*j;
			float zt = depth.at<float>(j, i);

			float x = calib_inv.at<float>(0, 0)*xt + calib_inv.at<float>(0, 1)*yt + calib_inv.at<float>(0, 2)*zt;
			float y = calib_inv.at<float>(1, 0)*xt + calib_inv.at<float>(1, 1)*yt + calib_inv.at<float>(1, 2)*zt;
			float z = calib_inv.at<float>(2, 0)*xt + calib_inv.at<float>(2, 1)*yt + calib_inv.at<float>(2, 2)*zt;

			com[0] += x;
			com[1] += y;
			com[2] += z;
			c += 1;
		}
		if (c == 0)
			return -1;

		com[0] /= c;
		com[1] /= c;
		com[2] /= c;

		return 0;

	}

	int getComHand(cv::Mat depth, float* com){

		float c = 0;
		for (int i = 0; i < width; i++)
		for (int j = 0; j < height; j++){

			if (depth.at<float>(j, i) == 0)
				continue;

			com[0] += i;
			com[1] += j;
			com[2] += depth.at<float>(j, i);
			c += 1;
		}
		if (c == 0)
			return -1;

		com[0] /= c;
		com[1] /= c;
		com[2] /= c;


		//visualization of com
		/*
		float com_2d[2];
		com_2d[0] = calib_mat.at<float>(0, 0)*com[0] + calib_mat.at<float>(0, 1)*com[1] + calib_mat.at<float>(0, 2)*com[2];
		com_2d[1] = calib_mat.at<float>(1, 0)*com[0] + calib_mat.at<float>(1, 1)*com[1] + calib_mat.at<float>(1, 2)*com[2];
		com_2d[2] = calib_mat.at<float>(2, 0)*com[0] + calib_mat.at<float>(2, 1)*com[1] + calib_mat.at<float>(2, 2)*com[2];
		com_2d[0] /= com_2d[2];
		com_2d[1] /= com_2d[2];

		cv::Mat depth_norm = cv::Mat(height,width, CV_8UC1);
		cv::normalize(depth,depth_norm, 0, 255, CV_MINMAX, CV_8UC1);
		cv::circle(depth_norm, cvPoint(com_2d[0], com_2d[1]), 5, cvScalar(255, 255, 255), 2, 8, 0);
		cv::imshow("circle", depth_norm);
		*/

		return 0;
	}

	void getWristCOM(float* inout){
		for (int i = 0; i < 3; i++)
			inout[i] = wristcomXYZ[i];
	}

	void calculateWristCOM(cv::Mat color, cv::Mat depth){
		//convert rgb to yuv
		cv::Mat yuv;
		cv::cvtColor(color, yuv, CV_BGR2YUV);

		//set mask based on v. (maskwrist)
		float c[3] = { 0 };
		float s = 0;

		for (int i = 0; i < width; i++)
		for (int j = 0; j < height; j++)
		{
			vimg.at<uchar>(j, i) = yuv.at<uchar>(j, 3 * i + 2);
			if (vimg.at<uchar>(j, i) < 150)
				continue;

			float x_ = i*depth.at<float>(j, i);
			float y_ = j*depth.at<float>(j, i);
			float z_ = depth.at<float>(j, i);

			float x = x_*calib_inv.at<float>(0, 0) + y_*calib_inv.at<float>(0, 1) + z_*calib_inv.at<float>(0, 2);
			float y = x_*calib_inv.at<float>(1, 0) + y_*calib_inv.at<float>(1, 1) + z_*calib_inv.at<float>(1, 2);
			float z = x_*calib_inv.at<float>(2, 0) + y_*calib_inv.at<float>(2, 1) + z_*calib_inv.at<float>(2, 2);

			c[0] += x;
			c[1] += y;
			c[2] += z;
			s += 1;
		}

		for (int i = 0; i < 3; i++){
			c[i] /= s;
			wristcomXYZ[i] = c[i];
		}

		//printf("x:%f y:%f z:%f\n", c[0], c[1], c[2]);
	}

	float wristcomXYZ[3];



};