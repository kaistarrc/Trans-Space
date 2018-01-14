#include "GLRenderer.h"

GLRenderer::GLRenderer()
{


}
GLRenderer::GLRenderer(int worig, int horig,int wtile,int htile,int wfb,int hfb)
{
	/*
	width = w;
	height = h;
	calibMat = cMat;
	width_fb = width * pnumX;
	height_fb = height * pnumY;
	*/

	width_orig = worig;
	height_orig = horig;
	//calibMat = cMat;
	width = wtile;// 128;
	height = htile;// 128;
	width_fb = wfb;//width * pnumX;
	height_fb = hfb;// height * pnumY;

	initGL();
	initCUDA();
	initFBO();
	initialBind();
}


GLRenderer::~GLRenderer()
{
	initialUnBind();

}





void GLRenderer::initFBO()
{
	//FBO creation and registration with CUDA
	fb.init(width_fb, height_fb);//fb.init(DEPTH_WIDTH*tile_numx, DEPTH_HEIGHT*tile_numy);
	color_id = fb.color_tex_id();
	extra_id = fb.extra_tex_id();
	depth_id = fb.depth_tex_id();


	checkCudaErrors(cudaGraphicsGLRegisterImage(&resouce1, color_id, GL_TEXTURE_2D, cudaGraphicsMapFlagsWriteDiscard));
	checkCudaErrors(cudaGraphicsMapResources(1, &resouce1, 0));
	checkCudaErrors(cudaGraphicsGLRegisterImage(&resouce2, extra_id, GL_TEXTURE_2D, cudaGraphicsMapFlagsWriteDiscard));
	checkCudaErrors(cudaGraphicsMapResources(1, &resouce2, 0));

}


void GLRenderer::initCUDA(){
	/// As in de docs, I get the device after creating the context
	int devID = gpuGetMaxGflopsDeviceId();
	cudaError status = cudaGLSetGLDevice(devID);
	//cudaError status = cudaGLSetGLDevice(1);
	if (status != cudaSuccess){
		std::cout << "Could not get OpenGL compliant device... exiting" << std::endl;
		exit(0);
	}
}

int GLRenderer::initGL()
{
	char *myargv[1];
	int myargc = 1;
	myargv[0] = _strdup("Myappname");
	
	//init glut	
	glutInit(&myargc, myargv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(width, height);//(500, 500);
	glutCreateWindow("glut_test");
	//glutHideWindow();
	glEnable(GL_DEPTH_TEST); // ±íÀÌ È°¼ºÈ­

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glViewport(0, 0, width, height);// p_iWidth, p_iHeight);

	/*
	Set_GL_PROJECTION(calibMat.at<float>(0, 0), calibMat.at<float>(1, 1),
		calibMat.at<float>(0, 2), calibMat.at<float>(1, 2),
		width_orig, height_orig, 0.01, 10000, _proj);
	*/

	/*
	float aa = 320-50;
	float bb = 240-50;
	Set_GL_PROJECTION(calibMat.at<float>(0, 0), calibMat.at<float>(1, 1),
		aa, bb,
		aa*2, bb*2, 0.01, 10000, _proj);
	*/
	//glLoadMatrixf(_proj);


	
	
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//--- GLEW Initialization (must have a context)
	
	glewExperimental = true;
	if (glewInit() != GLEW_NO_ERROR){

		fprintf(stderr, "Failed to initialize GLEW\n");
		return EXIT_FAILURE;
	}
	

	return 0;
}



void GLRenderer::initialBind(){

	//fbo setting
	fb.bind();

	
	mapCudaResource();
	kernel_bind_CUDA(array1, array2);
	cudaThreadSynchronize();
}

void GLRenderer::initialUnBind()
{
	//unbind
	kernel_unbind_CUDA();
	fb.unbind();
}

void GLRenderer::mapCudaResource()
{
	cudaGraphicsMapResources(1, &resouce1, 0);
	cudaGraphicsSubResourceGetMappedArray(&array1, resouce1, 0, 0);
	cudaGraphicsUnmapResources(1, &resouce1, 0);

	cudaGraphicsMapResources(1, &resouce2, 0);
	cudaGraphicsSubResourceGetMappedArray(&array2, resouce2, 0, 0);
	cudaGraphicsUnmapResources(1, &resouce2, 0);
}


int GLRenderer::queryFrames()
{
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);//???
	cv::Mat img1 = cv::Mat(height, width, CV_8UC4, cv::Scalar(0));
	glBindTexture(GL_TEXTURE_2D, color_id);
	//glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_FLOAT, img1.data);
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_BYTE, img1.data);
	glBindTexture(GL_TEXTURE_2D, 0);

	return true;
}

void GLRenderer::getOrigImage(cv::Mat& out,std::string type)
{
	if (type.compare("color")==0){
		cv::Mat final_model2;
		getColorTexture(final_model2);

		cv::Mat final_model = cv::Mat(height_orig, width_orig, CV_32FC3);
		cv::Rect rect(0, height_fb - height_orig - 1, width_orig, height_orig);
		final_model = final_model2(rect);

	
		final_model.copyTo(out);
	}
	else if (type.compare("depth")==0){

		cv::Mat final_model2;
		getDepthTexture(final_model2);

		cv::Mat final_model = cv::Mat(height_orig, width_orig, CV_32FC1);
		cv::Rect rect(0, height_fb - height_orig - 1, width_orig, height_orig);
		final_model = final_model2(rect);

		//out = final_model;
		final_model.copyTo(out);
		
		//cv::Mat a = final_model2;
		//cv::Mat b = final_model;
		//cv::waitKey(1);

	}
}


void GLRenderer::getColorTexture(cv::Mat& out)
{
	cv::Mat img1 = cv::Mat(height_fb, width_fb, CV_32FC4, cv::Scalar(0));
	glBindTexture(GL_TEXTURE_2D, color_id);
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_FLOAT, img1.data);

	glBindTexture(GL_TEXTURE_2D, 0);
	cv::flip(img1, img1, 0);

	cv::Mat rgb = cv::Mat(height_fb, width_fb, CV_8UC3, cv::Scalar(0));
	for (int i = 0; i < width_fb; i++)
	for (int j = 0; j < height_fb; j++)
	{
		rgb.at<unsigned char>(j, 3 * i + 0) = 255 * img1.at<float>(j, 4 * i + 2);
		rgb.at<unsigned char>(j, 3 * i + 1) = 255 * img1.at<float>(j, 4 * i + 1);
		rgb.at<unsigned char>(j, 3 * i + 2) = 255 * img1.at<float>(j, 4 * i + 0);
	}

	out = rgb;
}

void GLRenderer::getDepthTexture(cv::Mat& out)
{
	cv::Mat img1 = cv::Mat(height_fb, width_fb, CV_32FC4, cv::Scalar(0));
	glBindTexture(GL_TEXTURE_2D, extra_id);
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_FLOAT, img1.data);
	glBindTexture(GL_TEXTURE_2D, 0);
	cv::flip(img1, img1, 0);

	cv::Mat depth = cv::Mat(height_fb, width_fb, CV_32FC1, cv::Scalar(0));
	for (int i = 0; i < width_fb; i++)
	for (int j = 0; j < height_fb; j++){
		depth.at<float>(j, i) = img1.at<float>(j, 4 * i + 2);
	}

	out = depth;
}



void GLRenderer::getLABELfromGL(cv::Mat& out)
{
	cv::Mat img1 = cv::Mat(height, width, CV_32FC4, cv::Scalar(0));
	glBindTexture(GL_TEXTURE_2D, extra_id);
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_FLOAT, img1.data);
	glBindTexture(GL_TEXTURE_2D, 0);
	cv::flip(img1, img1, 0);

	cv::Mat label = cv::Mat(height, width, CV_8UC1, cv::Scalar(0));
	for (int i = 0; i < width; i++)
	for (int j = 0; j < height; j++){
		label.at<unsigned char>(j, i) = img1.at<float>(j, 4 * i + 3);
	}

	out = label;
}




