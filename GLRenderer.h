#pragma once
//#include "Kinematics.h"
#include <time.h>
#include "CustomFrameBuffer.h"
#include <bitset>

//gl
#include "GL\glew.h"
#include "GL\freeglut.h"


//cv
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>


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

extern "C" void kernel_bind_CUDA(cudaArray_t, cudaArray_t);
extern "C" void kernel_unbind_CUDA();

class GLRenderer
{
public:
	GLRenderer();
	GLRenderer(int, int,int,int, cv::Mat);
	

	~GLRenderer();

	void getFBsize(int& out1, int& out2){
		out1 = width_fb;
		out2 = height_fb;
	}
	void getImgSize(int& out1, int& out2){
		out1 = width;
		out2 = height;
	}

	int queryFrames();

	//void renderHand(float*, int);

	void getRGBfromCPU(cv::Mat&);
	void getDEPTHfromCPU(cv::Mat&);
	void getLABELfromCPU(cv::Mat&);



	//void debugHandGL(float*);

	//need to another class(handrenderer,..)
	CustomFrameBuffer fb;
	float _proj[16];
	cudaArray* array1;
	cudaArray* array2;
	void mapCudaResource();

	void showObModel(const char*,const char*,cv::Mat,int,int,int);
	void showObModel(const char*,const char*,cv::Mat, int, int);
	void showModelDemo(const char*);
	void showObModelDemo(const char* wname,cv::Mat);
	void showObModelDemo(const char* wname, const char* text, cv::Mat oimg_in);
	void showDifferenceObModel(cv::Mat);
//private:
	
	int initGL();
	void initCUDA();
	

	//void onlineBind();
	//void onlineUnBind();
	void initialBind();
	void initialUnBind();



	//FRAME BUFFER
	void initFBO();

	
	struct cudaGraphicsResource* resouce1;
	struct cudaGraphicsResource* resouce2;
	GLuint color_id;
	GLuint extra_id;
	GLuint normal_id;
	GLuint depth_id;



	void Set_GL_PROJECTION(double fx, double fy, double cx, double cy, int xdim, int ydim, double zmin, double zmax, float * gl_vector);


	//
	int width;
	int height;
	int width_orig;
	int height_orig;

	cv::Mat calibMat;

	int width_fb;
	int height_fb;



	//debugging

	

};
