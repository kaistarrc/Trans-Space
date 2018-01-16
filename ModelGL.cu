#pragma once
//cv
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#pragma comment(lib,"opencv_world310.lib")

//gl
#include "GL\glew.h"
#include "GL\freeglut.h"

//cuda
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
#include "helper_cuda.h"
#include <cuda.h>
#include <device_launch_parameters.h>
#include <device_functions.h>
#include <malloc.h>
#include <cublas.h>
#include <npp.h>

//gl
#pragma comment(lib,"glew32.lib")
#pragma comment(lib,"glew32s.lib")

//cv
#pragma comment(lib,"opencv_world310.lib")

//cuda
#pragma comment(lib,"cuda.lib")
#pragma comment(lib,"cudart.lib")
#pragma comment(lib,"cublas.lib")

#pragma comment(lib,"nppi.lib")
#pragma comment(lib,"npps.lib")
#pragma comment(lib,"nppc.lib")

texture<uchar4, 2, cudaReadModeElementType> color_tex;
texture<float4, 2, cudaReadModeElementType> extra_tex;

static int width;// = 128;
static int height;// = 128;
static int block_numx;// = 4;
static int block_numy;// = 4;
static int particle_numx;// = 8;
static int particle_numy;// = 4;

static float* cost_dif_cu;
static float* cost_dif_reduce_cu;

static float* cost_and_cu;
static float* cost_and_reduce_cu;

static float* cost_or_cu;
static float* cost_or_reduce_cu;


static float* img_dif_cu;
static float* img_and_cu;
static float* img_or_cu;



extern "C" void kernel_bind_CUDA(cudaArray_t array1, cudaArray_t array2)
{
	cudaBindTextureToArray(color_tex, array1, color_tex.channelDesc);
	cudaBindTextureToArray(extra_tex, array2, extra_tex.channelDesc);
}

extern "C" void kernel_unbind_CUDA()
{
	cudaUnbindTexture(color_tex);
	cudaUnbindTexture(extra_tex);
}

extern "C" void initCudaMem(int w, int h, int p_numx, int p_numy, int b_numx, int b_numy)
{
	width = w;	height = h;
	particle_numx = p_numx;	particle_numy = p_numy;
	block_numx = b_numx;	block_numy = b_numy;


	cudaMalloc(&img_dif_cu, sizeof(float)*w*p_numx*h*p_numy);
	cudaMalloc(&img_and_cu, sizeof(float)*w*p_numx*h*p_numy);
	cudaMalloc(&img_or_cu, sizeof(float)*w*p_numx*h*p_numy);

	cudaMalloc(&cost_dif_reduce_cu, block_numx*block_numy*particle_numx*particle_numy * sizeof(float));
	cudaMalloc(&cost_and_reduce_cu, block_numx*block_numy*particle_numx*particle_numy * sizeof(float));
	cudaMalloc(&cost_or_reduce_cu, block_numx*block_numy*particle_numx*particle_numy * sizeof(float));

	cudaMalloc(&cost_dif_cu, sizeof(float)*particle_numx*particle_numy);
	cudaMalloc(&cost_and_cu, sizeof(float)*particle_numx*particle_numy);
	cudaMalloc(&cost_or_cu, sizeof(float)*particle_numx*particle_numy);

}

extern "C" void releaseCudaMem()
{
	cudaFree(cost_dif_reduce_cu);

}

extern "C" void getReduceResult(float* cost_dif_reduce, float* cost_and_reduce, float* cost_or_reduce)
{
	cudaMemcpy(cost_dif_reduce, cost_dif_reduce_cu, sizeof(float)* 4 * particle_numx * 4 * particle_numy, cudaMemcpyDeviceToHost);
	cudaMemcpy(cost_and_reduce, cost_and_reduce_cu, sizeof(float)* 4 * particle_numx * 4 * particle_numy, cudaMemcpyDeviceToHost);
	cudaMemcpy(cost_or_reduce, cost_or_reduce_cu, sizeof(float)* 4 * particle_numx * 4 * particle_numy, cudaMemcpyDeviceToHost);

}
extern "C" void getCostFromGPU(cv::Mat& cost_dif, cv::Mat& cost_and, cv::Mat& cost_or)
{
	cudaMemcpy(cost_dif.data, cost_dif_cu, sizeof(float)*particle_numx*particle_numy, cudaMemcpyDeviceToHost);
	cudaMemcpy(cost_and.data, cost_and_cu, sizeof(float)*particle_numx*particle_numy, cudaMemcpyDeviceToHost);
	cudaMemcpy(cost_or.data, cost_or_cu, sizeof(float)*particle_numx*particle_numy, cudaMemcpyDeviceToHost);

}
extern "C" void getDifferenceImageFromGPU(cv::Mat& dif, cv::Mat& and, cv::Mat& or)
{
	int s = width*particle_numx*height*particle_numy;

	cudaMemcpy(dif.data, img_dif_cu, sizeof(float)*s, cudaMemcpyDeviceToHost);
	cudaMemcpy(and.data, img_and_cu, sizeof(float)*s, cudaMemcpyDeviceToHost);
	cudaMemcpy(or.data, img_or_cu, sizeof(float)*s, cudaMemcpyDeviceToHost);

}

__global__ void differentiate(float* o, float* dif, float* and, float* or, int _w, int _h, int _pnumx, int _pnumy, float dif_max)
{
	unsigned int width_full = _w*_pnumx;  //width*particle_numx;
	unsigned int height_full = _h*_pnumy; //height*particle_numy;

	//index in an image
	unsigned int tx = threadIdx.x;
	unsigned int ty = threadIdx.y;
	unsigned int bw = blockDim.x;
	unsigned int bh = blockDim.y;

	//index of a particle
	unsigned int px = blockIdx.z % _pnumx; //particle_numx;
	unsigned int py = blockIdx.z / _pnumx; //particle_numx;

	//global index in whole images.
	unsigned int u = px*_w + (bw*blockIdx.x) + tx;
	unsigned int v = py*_h + (bh*blockIdx.y) + ty;

	//working
	/*
	if (o[v*width_full + u] == 0)
	dif[v*width_full + u] = 0;
	else
	{
	dif[v*width_full + u] = abs(o[v*width_full + u] - tex2D(extra_tex, u, height_full - v-1).z);
	if (dif[v*width_full + u] > 100)
	dif[v*width_full + u] = 100;
	}
	*/

	//modified ( working in PSO ) 
	/*
	and[v*width_full + u] = 0;	or[v*width_full + u] = 0;	dif[v*width_full + u] = 0;

	if (o[v*width_full + u] > 0){
	dif[v*width_full + u] = abs(o[v*width_full + u] - tex2D(extra_tex, u, height_full - v - 1).z);

	if (dif[v*width_full + u] > dif_max)
	dif[v*width_full + u] = dif_max;
	}

	if (o[v*width_full + u] > 0 && tex2D(extra_tex, u, height_full - v - 1).z > 0)
	{
	if (dif[v*width_full + u] < dif_max)
	and[v*width_full + u] = 1;
	}

	if (o[v*width_full + u] > 0 || tex2D(extra_tex, u, height_full - v - 1).z > 0)
	or[v*width_full + u] = 1;
	*/

	//modifying (in HMF)

	and[v*width_full + u] = 0;	or[v*width_full + u] = 0;	dif[v*width_full + u] = 0;

	if (o[v*width_full + u] > 0 && tex2D(extra_tex, u, height_full - v - 1).z > 0){


		dif[v*width_full + u] = abs(o[v*width_full + u] - tex2D(extra_tex, u, height_full - v - 1).z);
		if (dif[v*width_full + u] > dif_max)
			dif[v*width_full + u] = dif_max;

		//if (dif[v*width_full + u] <dif_max)  // cost may be bigger than 1 due to this.
		and[v*width_full + u] = 1;
	}

	if (o[v*width_full + u] > 0 || tex2D(extra_tex, u, height_full - v - 1).z > 0)
		or[v*width_full + u] = 1;


}

__global__ void sum_and_reduce(float* g_diff, float* g_odata, int _w, int _h, int _pnumx, int _pnumy)
{
	extern __shared__ float sData[];

	unsigned int width_full = _w*_pnumx;  //width*particle_numx;
	unsigned int height_full = _h*_pnumy; //height*particle_numy;

	//index in an image
	unsigned int tx = threadIdx.x;
	unsigned int ty = threadIdx.y;
	unsigned int bw = blockDim.x;//=32
	unsigned int bh = blockDim.y;//=32

	//index of a particle
	unsigned int px = blockIdx.z % _pnumx;
	unsigned int py = blockIdx.z / _pnumx;

	//global index in whole images.
	unsigned int u = px*_w + (bw*blockIdx.x) + tx;
	unsigned int v = py*_h + (bh*blockIdx.y) + ty;

	//thread ID / global block ID 
	unsigned int tid = tx + ty*bw;
	unsigned int bidx = blockIdx.x + px *  gridDim.x;
	unsigned int bidy = blockIdx.y + py *  gridDim.y;
	unsigned int bid = bidx + bidy *  gridDim.x*_pnumx;


	sData[tid] = g_diff[v*width_full + u];
	__syncthreads();

	for (int s = 1; s < blockDim.x * blockDim.y; s *= 2)
	{
		if (tid % (2 * s) == 0)
		{
			//sData[tid] = (sData[tid] & 0x0000ff) + (sData[tid + s] & 0x0000ff);
			sData[tid] += sData[tid + s];

		}
		__syncthreads();
	}
	if (tid == 0) g_odata[bid] = sData[0];
}

__global__ void sum_and_reduce2(float *g_idata, float *g_odata) {
	extern __shared__ float sData[];

	//index in an image
	unsigned int tx = threadIdx.x;
	unsigned int ty = threadIdx.y;
	unsigned int bw = blockDim.x;//=4
	unsigned int bh = blockDim.y;//=4

	//index of a particle
	unsigned int px = blockIdx.x;
	unsigned int py = blockIdx.y;

	//global index in whole images.
	unsigned int u = px*bw + tx;
	unsigned int v = py*bh + ty;

	unsigned int width_full = bw*gridDim.x;
	unsigned int height_full = bh*gridDim.y;

	//thread ID / global block ID 
	unsigned int tid = tx + ty*bw;
	unsigned int bid = px + py*gridDim.x;

	sData[tid] = g_idata[v*width_full + u];
	__syncthreads();

	// do reduction in shared mem
	for (int s = 1; s < blockDim.x*blockDim.y; s *= 2) {
		if (tid % (2 * s) == 0) {
			sData[tid] += sData[tid + s];
		}
		__syncthreads();
	}
	// write result for this block to global mem
	if (tid == 0) g_odata[bid] = sData[0];

}


extern "C" void calculatecost_cu(float* img_ob_cu, float dif_max)//, float* img_dif_cu)//,float* cost_cu)//,int g)
{
	dim3 block(width / block_numx, height / block_numy);  //=(32,32)
	dim3 grid(block_numx, block_numy, particle_numx*particle_numy);   //=(4,4,particle_num)

	// difference map	
	differentiate << <grid, block >> > (img_ob_cu, img_dif_cu, img_and_cu, img_or_cu, width, height, particle_numx, particle_numy, dif_max);
	cudaThreadSynchronize();

	// reduce
	int sbytes1 = block.x*block.y*sizeof(float);
	sum_and_reduce << <grid, block, sbytes1 >> > (img_dif_cu, cost_dif_reduce_cu, width, height, particle_numx, particle_numy);
	//cudaThreadSynchronize();

	sum_and_reduce << <grid, block, sbytes1 >> > (img_and_cu, cost_and_reduce_cu, width, height, particle_numx, particle_numy);
	//cudaThreadSynchronize();

	sum_and_reduce << <grid, block, sbytes1 >> > (img_or_cu, cost_or_reduce_cu, width, height, particle_numx, particle_numy);
	cudaThreadSynchronize();


	dim3 block2(block_numx, block_numy);  //=(4,4)
	dim3 grid2(particle_numx, particle_numy, 1);   //=(4,4,particle_num)

	int sbytes2 = block2.x*block2.y*sizeof(float);
	sum_and_reduce2 << <grid2, block2, sbytes2 >> > (cost_dif_reduce_cu, cost_dif_cu);
	//cudaThreadSynchronize();

	sum_and_reduce2 << <grid2, block2, sbytes2 >> > (cost_and_reduce_cu, cost_and_cu);
	//cudaThreadSynchronize();

	sum_and_reduce2 << <grid2, block2, sbytes2 >> > (cost_or_reduce_cu, cost_or_cu);
	cudaThreadSynchronize();

}

__global__ void get_texture_depth(float* dstBuffer, int _w, int _h, int _pnumx, int _pnumy)
{

	unsigned int width_full = _w*_pnumx;  //width*particle_numx;
	unsigned int height_full = _h*_pnumy; //height*particle_numy;

	unsigned int tx = threadIdx.x;
	unsigned int ty = threadIdx.y;
	unsigned int bw = blockDim.x;
	unsigned int bh = blockDim.y;

	unsigned int px = blockIdx.z % _pnumx; //particle_numx;
	unsigned int py = blockIdx.z / _pnumx; //particle_numx;

	unsigned int u = px*_w + (bw*blockIdx.x) + tx;
	unsigned int v = py*_h + (bh*blockIdx.y) + ty;

	dstBuffer[v*_w*_pnumx + u] = tex2D(extra_tex, u, height_full-v-1).z;

	//if (u >= 128 & v >= 128)
	//	dstBuffer[v*_w*_pnumx + u] = 0;
}


extern "C" void get_depth_cu(float* img_cu)
{
	dim3 block(width / block_numx, height / block_numy);  //=(32,32)
	dim3 grid(block_numx, block_numy, particle_numx*particle_numy);   //=(4,4,particle_num)

	get_texture_depth << <grid, block >> >(img_cu, width, height, particle_numx, particle_numy);
	cudaThreadSynchronize();
}


