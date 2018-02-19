#pragma once
//#include "PARAMETER.h"

#include "GLRenderer.h"
//#include "HandDataRenderer.h"


#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>

//#include "common.h"
#include "HandParameter.h"
#include "GLRenderer.h"

#pragma comment(lib,"opencv_world310.lib")

extern "C" void calculatecost_cu(float*, float);// , float*);// , float*);
extern "C" void initCudaMem(int, int, int, int, int, int);
extern "C" void releaseCudaMem();

extern "C" void getReduceResult(float*, float*, float*);
extern "C" void getCostFromGPU(cv::Mat&, cv::Mat&, cv::Mat&);
extern "C" void getDifferenceImageFromGPU(cv::Mat&, cv::Mat&, cv::Mat&);
extern "C" void get_depth_cu(float*);

class CostFunction{
public:

	CostFunction(){

	}

	CostFunction(int p_numX, int p_numY, int handParamNum,GLRenderer& glrenderer){

		_glrenderer = glrenderer;

		_glrenderer.getFBsize(width_fb, height_fb);
		_glrenderer.getImgSize(width, height);


		//size
		dimension = handParamNum;// hand_dof;
		particle_numx = p_numX;
		particle_numy = p_numY;
		particle_num = particle_numx*particle_numy;

		//allocation
		position_put = new float[dimension];

		cost = cv::Mat(1, particle_num, CV_32FC1);
		cost_dif = cv::Mat(1, particle_num, CV_32FC1);
		cost_and = cv::Mat(1, particle_num, CV_32FC1);
		cost_or = cv::Mat(1, particle_num, CV_32FC1);

		img_dif = cv::Mat(height_fb, width_fb, CV_32FC1);
		img_and = cv::Mat(height_fb, width_fb, CV_32FC1);
		img_or = cv::Mat(height_fb, width_fb, CV_32FC1);
		oimg = cv::Mat(height_fb, width_fb, CV_32FC1);
		cudaMalloc(&oimg_cu, sizeof(float)*width_fb*height_fb);

		dif_max = 40;
		block_numx = width / 32;
		block_numy = height / 32;
		//initCudaMem(width, height, particle_numx, particle_numy, 4, 4);
		initCudaMem(width, height, particle_numx, particle_numy, block_numx, block_numy);


		//for debugging
		debugImg = cv::Mat(height_fb, width_fb, CV_32FC1);
		debugImg_norm = cv::Mat(height_fb, width_fb, CV_8UC1);
		cudaMalloc(&debugImg_cu, sizeof(float)*width_fb*height_fb);

		cost_dif_reduce = new float[block_numx * particle_numx * block_numy * particle_numy];
		cost_and_reduce = new float[block_numx * particle_numx * block_numy * particle_numy];
		cost_or_reduce = new float[block_numx * particle_numx * block_numy * particle_numy];

	}


	~CostFunction(){
		releaseCudaMem();
	}

	void setPointsNumberObservation(cv::Mat in)
	{
		int c = 0;
		for (int i = 0; i < width; i++)
		for (int j = 0; j < height; j++){
			if (in.at<float>(j, i) == 0)
				continue;
			c++;
		}
		pointNum = c;
	}

	void transferObservation2GPU(cv::Mat oimg_in){
		cv::Mat ddd = oimg_in;

		for (int i = 0; i < width; i++)
		for (int j = 0; j < height; j++)
		for (int k = 0; k < particle_numx; k++)
		for (int m = 0; m < particle_numy; m++){
			oimg.at<float>(j + height*m, i + width*k) = oimg_in.at<float>(j, i);

		}
		cudaMemcpy(oimg_cu, oimg.data, sizeof(float)*width_fb*height_fb, cudaMemcpyHostToDevice);
	}

	void calculateCost(cv::Mat& out){

		calculatecost_cu(oimg_cu, dif_max);
		getCostFromGPU(cost_dif, cost_and, cost_or);

		
		//debugging
		/*
		float* ddcu;
		cudaMalloc(&ddcu, sizeof(float)* 8 * 4 * 128 * 128);
		get_depth_cu(ddcu);
		float* dd = new float[8 * 4 * 128 * 128];
		cudaMemcpy(dd, ddcu, sizeof(float)* 8 * 4 * 128 * 128, cudaMemcpyDeviceToHost);
		cv::Mat ddmat = cv::Mat(128 * 4, 128 * 8, CV_32FC1,dd);
		cv::imshow("ddmat", ddmat);
		*/
		//debugging
		
		
		for (int i = 0; i < particle_num; i++){

			//--data term--//
			float dif = cost_dif.at<float>(0, i);
			float and = cost_and.at<float>(0, i);
			float or = cost_or.at<float>(0, i);

			float r = and / or;
			float c = r*dif / (dif_max*and + 1) + (1 - r); //0: good, 1: bad

			out.at<float>(0, i) = c;

			//l = exp(-c*c / (2.0 * dev_track*dev_track));
			//out.at<float>(0, i) = l;

		}
		
	}

	void calculateLikelihood(cv::Mat& out){

		calculatecost_cu(oimg_cu, dif_max);
		getCostFromGPU(cost_dif, cost_and, cost_or);

		float l = 0;
		for (int i = 0; i < particle_num; i++){
			//--data term--//
			float dif = cost_dif.at<float>(0, i);
			float and = cost_and.at<float>(0, i);
			float or = cost_or.at<float>(0, i);

			float r = and / or;
			float c = r*dif / (dif_max*and + 1) + (1 - r); //0: good, 1: bad

			//out.at<float>(0, i) = c;

			l = exp(-c*c / (2.0 * 1.0*1.0));
			out.at<float>(0, i) = l;

		}
	}

	void calculateJointLimit(cv::Mat hp, float* out){


		for (int i = 0; i < particle_num; i++)
		{
			float pt = 0;
			for (int j = 0; j < dimension; j++)
			{
				float bmin = boundary_max[0][j];
				float bmax = boundary_max[1][j];

				float p = hp.at<float>(j, i);

				if (p < bmin)
					pt += bmin - p;
				if (p>bmax)
					pt += bmax - p;
			}

			out[i] = pt;
		}


	}

	//TO DO: we should make getcalculateCollisionTerm()
	void calculateCollision(cv::Mat* jmat, float* out){

		

	}

	cv::Mat getTiledImages()
	{
		return oimg;
	}


private:
	//HandDataRenderer _renderer;
	GLRenderer _glrenderer;
	//Watch watch;

	float* position_put;

	int width;
	int height;
	int dimension;
	int pointNum;
	int width_fb;
	int height_fb;
	int particle_numx;
	int particle_numy;
	int particle_num;
	int block_numx;
	int block_numy;

	//cpu data
	cv::Mat oimg;

	cv::Mat cost;
	cv::Mat cost_dif;
	cv::Mat cost_and;
	cv::Mat cost_or;

	cv::Mat img_dif;
	cv::Mat img_and;
	cv::Mat img_or;

	float* cost_dif_reduce;
	float* cost_and_reduce;
	float* cost_or_reduce;
	float dif_max;


	//gpu data
	float* oimg_cu;

	//debug data
	cv::Mat debugImg;
	cv::Mat debugImg_norm;
	float* debugImg_cu;

	void showDifferenceImage(float* img_cu){
		cudaMemcpy(debugImg.data, img_cu, sizeof(float)*width_fb*height_fb, cudaMemcpyDeviceToHost);

		cv::normalize(debugImg, debugImg_norm, 0, 255, CV_MINMAX, CV_8UC1);
		cv::imshow("difference", debugImg_norm);
		cv::waitKey(1);

	}
	void debugDifferenceImage()
	{
		//cudaMemcpy(debugImg.data, img_dif_cu, sizeof(float)*width_fb*height_fb, cudaMemcpyDeviceToHost);
		//getDifferenceImageFromGPU(debugImg);
		//cv::Mat difgpu = debugImg;

		cv::Mat difgpu = cv::Mat(height_fb, width_fb, CV_32FC1);
		cv::Mat andgpu = cv::Mat(height_fb, width_fb, CV_32FC1);
		cv::Mat orgpu = cv::Mat(height_fb, width_fb, CV_32FC1);
		getDifferenceImageFromGPU(difgpu, andgpu, orgpu);

		cv::Mat model_depth;
		_glrenderer.getDepthTexture(model_depth);

		for (int i = 0; i < width; i++)
		for (int j = 0; j < height; j++)
		for (int k = 0; k < particle_numx; k++)
		for (int m = 0; m < particle_numy; m++){

			float o = oimg.at<float>(j + height*m, i + width*k);
			float d = model_depth.at<float>(j + height*m, i + width*k);
			/*
			if (o == 0)
			img_dif.at<float>(j + height*m, i + width*k) = 0;
			else
			img_dif.at<float>(j + height*m, i + width*k) = abs(o - d);

			if (img_dif.at<float>(j + height*m, i + width*k) >100)
			img_dif.at<float>(j + height*m, i + width*k) = 100;
			*/

			img_and.at<float>(j + height*m, i + width*k) = 0;
			img_or.at<float>(j + height*m, i + width*k) = 0;
			img_dif.at<float>(j + height*m, i + width*k) = 0;

			if (o > 0 & d > 0){
				img_and.at<float>(j + height*m, i + width*k) = 1;
				img_dif.at<float>(j + height*m, i + width*k) = abs(o - d);

				if (img_dif.at<float>(j + height*m, i + width*k) > dif_max)
					img_dif.at<float>(j + height*m, i + width*k) = dif_max;
			}

			if (o > 0 | d > 0)
				img_or.at<float>(j + height*m, i + width*k) = 1;
		}
		cv::Mat difcpu = img_dif;
		cv::Mat and_cpu = img_and;
		cv::Mat or_cpu = img_or;


		cv::waitKey(1);
	}

	void debugReduction1(){

		//cudaMemcpy(cost_reduce, cost_reduce_cu, sizeof(float)* 4 * particle_numx * 4 * particle_numy, cudaMemcpyDeviceToHost);
		getReduceResult(cost_dif_reduce, cost_and_reduce, cost_or_reduce);

		cv::Mat cost_dif_reduce_gpu = cv::Mat(block_numy * particle_numy, block_numx * particle_numx, CV_32FC1);
		cv::Mat cost_and_reduce_gpu = cv::Mat(block_numy * particle_numy, block_numx * particle_numx, CV_32FC1);
		cv::Mat cost_or_reduce_gpu = cv::Mat(block_numy * particle_numy, block_numx * particle_numx, CV_32FC1);
		for (int i = 0; i < block_numx * particle_numx; i++)
		for (int j = 0; j < block_numy * particle_numy; j++){
			cost_dif_reduce_gpu.at<float>(j, i) = cost_dif_reduce[block_numx * particle_numx * j + i];
			cost_and_reduce_gpu.at<float>(j, i) = cost_and_reduce[block_numx * particle_numx * j + i];
			cost_or_reduce_gpu.at<float>(j, i) = cost_or_reduce[block_numx * particle_numx * j + i];
		}

		cv::Mat cost_dif_reduce_cpu = cv::Mat(block_numy * particle_numy, block_numx * particle_numx, CV_32FC1);
		cv::Mat cost_and_reduce_cpu = cv::Mat(block_numy * particle_numy, block_numx * particle_numx, CV_32FC1);
		cv::Mat cost_or_reduce_cpu = cv::Mat(block_numy * particle_numy, block_numx * particle_numx, CV_32FC1);
		for (int m = 0; m < particle_numy; m++)
		for (int k = 0; k < particle_numx; k++)
		{
			for (int m1 = 0; m1 < block_numy; m1++)
			for (int k1 = 0; k1 < block_numx; k1++)
			{
				float sumcpu = 0;
				float andcpu = 0;
				float orcpu = 0;

				for (int i = 0; i < 32; i++)
				for (int j = 0; j < 32; j++){
					sumcpu += img_dif.at<float>(j + 32 * m1 + height * m, i + 32 * k1 + width * k);
					andcpu += img_and.at<float>(j + 32 * m1 + height * m, i + 32 * k1 + width * k);
					orcpu += img_or.at<float>(j + 32 * m1 + height * m, i + 32 * k1 + width * k);
				}
				cost_dif_reduce_cpu.at<float>(m1 + block_numy * m, k1 + block_numx * k) = sumcpu;
				cost_and_reduce_cpu.at<float>(m1 + block_numy * m, k1 + block_numx * k) = andcpu;
				cost_or_reduce_cpu.at<float>(m1 + block_numy * m, k1 + block_numx * k) = orcpu;

			}
		}
		cv::waitKey(1);
	}

	void debugReduction2()
	{
		//gpu
		cv::Mat cost_dif_gpu = cv::Mat(particle_numy, particle_numx, CV_32FC1);
		cv::Mat cost_and_gpu = cv::Mat(particle_numy, particle_numx, CV_32FC1);
		cv::Mat cost_or_gpu = cv::Mat(particle_numy, particle_numx, CV_32FC1);

		for (int i = 0; i < particle_numx; i++)
		for (int j = 0; j < particle_numy; j++){
			cost_dif_gpu.at<float>(j, i) = cost_dif.at<float>(i + j*particle_numx);
			cost_and_gpu.at<float>(j, i) = cost_and.at<float>(i + j*particle_numx);
			cost_or_gpu.at<float>(j, i) = cost_or.at<float>(i + j*particle_numx);
		}

		//cpu
		cv::Mat cost_dif_cpu = cv::Mat(particle_numy, particle_numx, CV_32FC1);
		cv::Mat cost_and_cpu = cv::Mat(particle_numy, particle_numx, CV_32FC1);
		cv::Mat cost_or_cpu = cv::Mat(particle_numy, particle_numx, CV_32FC1);
		for (int m = 0; m < particle_numy; m++)
		for (int k = 0; k < particle_numx; k++)
		{
			float difcpu = 0;
			float andcpu = 0;
			float orcpu = 0;

			for (int i = 0; i < width; i++)
			for (int j = 0; j < height; j++){
				difcpu += img_dif.at<float>(j + height*m, i + width*k);
				andcpu += img_and.at<float>(j + height*m, i + width*k);
				orcpu += img_or.at<float>(j + height*m, i + width*k);
			}
			cost_dif_cpu.at<float>(m, k) = difcpu;
			cost_and_cpu.at<float>(m, k) = andcpu;
			cost_or_cpu.at<float>(m, k) = orcpu;
		}
		cv::waitKey(1);
	}
};