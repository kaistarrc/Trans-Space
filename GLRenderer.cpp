#include "GLRenderer.h"

GLRenderer::GLRenderer()
{


}
GLRenderer::GLRenderer(int w, int h,int pnumX,int pnumY, cv::Mat cMat)
{
	/*
	width = w;
	height = h;
	calibMat = cMat;
	width_fb = width * pnumX;
	height_fb = height * pnumY;
	*/
	width_orig = w;
	height_orig = h;
	calibMat = cMat;
	width = w;// 128;
	height = h;// 128;
	width_fb = width * pnumX;
	height_fb = height * pnumY;




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

	
	Set_GL_PROJECTION(calibMat.at<float>(0, 0), calibMat.at<float>(1, 1),
		calibMat.at<float>(0, 2), calibMat.at<float>(1, 2),
		width_orig, height_orig, 0.01, 10000, _proj);
	
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


void GLRenderer::Set_GL_PROJECTION(double fx, double fy, double cx, double cy, int xdim, int ydim, double zmin, double zmax, float * gl_vector)
{

	/*
	for (int i = 0; i < 16; i++)
	gl_vector[i] = 0;

	gl_vector[0] = 2 * fx / xdim;
	gl_vector[2] = (xdim - 2 * cx) / xdim;

	//gl_vector[5] = -2 * fy / ydim;
	//gl_vector[6] = (ydim - 2 * cy) / ydim;
	gl_vector[5] = 2 * fy / ydim;
	gl_vector[6] = (-ydim + 2 * cy) / ydim;

	gl_vector[10] = (-zmax - zmin) / (zmax - zmin);
	gl_vector[11] = -2 * zmax*zmin / (zmax - zmin);

	gl_vector[14] = -1;
	*/


	double planes[4];
	planes[0] = 0;
	planes[1] = 0;
	planes[2] = 100;
	planes[3] = 0;

	double tdir[3][4], S[3][3], Stdir[3][4];
	float gl_mat[4][4];
	double zratio = 2.0 / (zmax - zmin);

	for (int i = 0; i < 3; i++)
	for (int j = 0; j < 4; j++)
		tdir[i][j] = 0.;

	tdir[0][0] = fx;
	tdir[1][1] = fy;
	tdir[0][2] = cx;
	tdir[1][2] = cy;
	tdir[2][2] = 1.;

	S[0][0] = 2. / xdim; S[0][1] = 0.;        S[0][2] = -1.;
	S[1][0] = 0.;        S[1][1] = 2. / ydim; S[1][2] = -1.;
	S[2][0] = 0.;        S[2][1] = 0.;        S[2][2] = 1.;

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			Stdir[i][j] = 0.;
			for (int k = 0; k < 3; k++)
				Stdir[i][j] += S[i][k] * tdir[k][j];
		}
	}

	for (int j = 0; j < 4; j++)
	{
		gl_mat[j][0] = float(Stdir[0][j]);
		gl_mat[j][1] = float(Stdir[1][j]);
		gl_mat[j][3] = float(Stdir[2][j]);
	}

	// Plane distance computation in 3rd column:
	for (int j = 0; j < 4; j++)
		gl_mat[j][2] = float(planes[j] * zratio);
	gl_mat[3][2] = float(-(1.0 + gl_mat[3][2] + zmin * zratio));

	for (int i = 0; i < 4; i++)
		gl_mat[i][1] = -gl_mat[i][1];

	for (int i = 0; i < 4; i++)
	for (int j = 0; j < 4; j++)
		gl_vector[j + 4 * i] = gl_mat[i][j];
}


void GLRenderer::getRGBfromCPU(cv::Mat& out)
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

	/*
	cv::Mat img1 = cv::Mat(height, width, CV_32FC4, cv::Scalar(0));
	glBindTexture(GL_TEXTURE_2D, color_id);
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_FLOAT, img1.data);
	
	glBindTexture(GL_TEXTURE_2D, 0);
	cv::flip(img1, img1, 0);

	
	cv::Mat rgb = cv::Mat(height, width, CV_8UC3, cv::Scalar(0));
	for (int i = 0; i < width; i++)
	for (int j = 0; j < height; j++)
	for (int k = 0; k < 3; k++)
		rgb.at<unsigned char>(j, 3 * i + k) = img1.at<float>(j, 4 * i + k);

	out = rgb;
	*/
}

void GLRenderer::getDEPTHfromCPU(cv::Mat& out)
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



void GLRenderer::getLABELfromCPU(cv::Mat& out)
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

void GLRenderer::showDifferenceObModel(cv::Mat oimg_in)
{
	//model
	cv::Mat final_model2;
	getDEPTHfromCPU(final_model2);

	cv::Mat final_model = cv::Mat(height_orig, width_orig, CV_32FC3);
	cv::Rect rect(0, height_fb - height_orig - 1, width_orig, height_orig);
	final_model = final_model2(rect);

	//observation
	cv::Mat final_o = oimg_in;

	//dif
	cv::Mat dif = final_model - final_o;

	cv::waitKey(1);



}

void GLRenderer::showModelDemo(const char* wname)
{
	cv::Mat final_model2;
	getRGBfromCPU(final_model2);

	cv::Mat final_model = cv::Mat(height_orig, width_orig, CV_32FC3);
	cv::Rect rect(0, height_fb - height_orig - 1, width_orig, height_orig);
	final_model = final_model2(rect);

	cv::imshow(wname, final_model);
}

void GLRenderer::showObModelDemo(const char* wname, const char* text, cv::Mat oimg_in)
{
	cv::Mat om_3c_origin = cv::Mat(480, 640, CV_8UC3);

	cv::Mat final_model2;
	getRGBfromCPU(final_model2);
	//cv::imshow("bbb", final_model2);

	cv::Mat final_model = cv::Mat(height_orig, width_orig, CV_32FC3);
	cv::Rect rect(0, height_fb - height_orig - 1, width_orig, height_orig);
	final_model = final_model2(rect);

	//cv::imshow("finalmodel", final_model);

	cv::Mat o_3c = cv::Mat(height_orig, width_orig, CV_8UC3);
	cv::Mat om_3c = cv::Mat(height_orig, width_orig, CV_8UC3);
	o_3c.setTo(0);
	for (int i = 0; i < width_orig; i++)
	for (int j = 0; j < height_orig; j++){
		if (oimg_in.at<float>(j, i) != 0)
		{
			o_3c.at <unsigned char>(j, 3 * i + 0) = 255;
			o_3c.at <unsigned char>(j, 3 * i + 1) = 255;
			o_3c.at <unsigned char>(j, 3 * i + 2) = 255;
		}
	}

	cv::addWeighted(o_3c, 0.3, final_model, 0.9, 0, om_3c);
	cv::resize(om_3c, om_3c_origin, cv::Size(640, 480), 0, 0, 1);

	cv::putText(om_3c_origin, text, cvPoint(50, 50), 1, 3, cv::Scalar(0, 255, 0));
	cv::imshow(wname, om_3c_origin);

}

void GLRenderer::showObModelDemo(const char* wname, cv::Mat oimg_in)
{
	cv::Mat om_3c_origin = cv::Mat(480, 640, CV_8UC3);

	cv::Mat final_model2;
	getRGBfromCPU(final_model2);
	//cv::imshow("bbb", final_model2);

	cv::Mat final_model = cv::Mat(height_orig, width_orig, CV_32FC3);
	cv::Rect rect(0, height_fb - height_orig - 1, width_orig, height_orig);
	final_model = final_model2(rect);

	//cv::imshow("finalmodel", final_model);

	cv::Mat o_3c = cv::Mat(height_orig, width_orig, CV_8UC3);
	cv::Mat om_3c = cv::Mat(height_orig, width_orig, CV_8UC3);
	o_3c.setTo(0);
	for (int i = 0; i < width_orig; i++)
	for (int j = 0; j < height_orig; j++){
		if (oimg_in.at<float>(j, i) != 0)
		{
			o_3c.at <unsigned char>(j, 3 * i + 0) = 255;
			o_3c.at <unsigned char>(j, 3 * i + 1) = 255;
			o_3c.at <unsigned char>(j, 3 * i + 2) = 255;
		}
	}

	cv::addWeighted(o_3c, 0.3, final_model, 0.9, 0, om_3c);
	cv::resize(om_3c, om_3c_origin, cv::Size(640, 480), 0, 0, 1);


	cv::imshow(wname, om_3c_origin);

}


void GLRenderer::showObModel(const char* wname,const char* text,cv::Mat oimg_in,int px,int py,int vismode)
{
	cv::Mat om_3c_origin = cv::Mat(480, 640, CV_8UC3);

	if (vismode == 0)//color
	{
		cv::Mat final_model2;
		getRGBfromCPU(final_model2);
		//cv::imshow("ObModel", final_model2);
		cv::Mat final_model = cv::Mat(height, width, CV_32FC1);
		cv::Rect rect(px*width, py*height, width, height);
		final_model = final_model2(rect);

		//cv::imshow("finalmodel", final_model);

		cv::Mat o_3c = cv::Mat(height, width, CV_8UC3);
		cv::Mat om_3c = cv::Mat(height, width, CV_8UC3);
		o_3c.setTo(0);
		for (int i = 0; i < width; i++)
		for (int j = 0; j < height; j++){
			if (oimg_in.at<float>(j, i) != 0)
			{
				o_3c.at <unsigned char>(j, 3 * i + 0) = 255;
				o_3c.at <unsigned char>(j, 3 * i + 1) = 255;
				o_3c.at <unsigned char>(j, 3 * i + 2) = 255;
			}
		}

		cv::addWeighted(o_3c, 0.3, final_model, 0.9, 0, om_3c);
		cv::resize(om_3c, om_3c_origin, cv::Size(640, 480), 0, 0, 1);
	}
	else if (vismode == 1)//depth
	{
		cv::Mat final_model2;
		getDEPTHfromCPU(final_model2);

		cv::Mat final_model = cv::Mat(height, width, CV_32FC1);
		//cv::Rect rect(0, 0, width, height);
		cv::Rect rect(px*width, py*height, width, height);
		final_model = final_model2(rect);
		//cv::imshow("finalmodel", final_model);

		//visualize 3color
		cv::Mat o_3c = cv::Mat(height, width, CV_8UC3);
		cv::Mat m_3c = cv::Mat(height, width, CV_8UC3);
		cv::Mat om_3c = cv::Mat(height, width, CV_8UC3);
		o_3c.setTo(0);
		m_3c.setTo(0);

		for (int i = 0; i < width; i++)
		for (int j = 0; j < height; j++){
			if (final_model.at<float>(j, i) != 0)
			{
				m_3c.at <unsigned char>(j, 3 * i + 0) = 0;
				m_3c.at <unsigned char>(j, 3 * i + 1) = 0;
				m_3c.at <unsigned char>(j, 3 * i + 2) = 255;
			}
			if (oimg_in.at<float>(j, i) != 0)
			{
				o_3c.at <unsigned char>(j, 3 * i + 0) = 255;
				o_3c.at <unsigned char>(j, 3 * i + 1) = 0;
				o_3c.at <unsigned char>(j, 3 * i + 2) = 0;
			}
		}
		cv::addWeighted(o_3c, 0.5, m_3c, 0.5, 0, om_3c);
		cv::resize(om_3c, om_3c_origin, cv::Size(640, 480), 0, 0, 0);
	}

	cv::putText(om_3c_origin,text, cvPoint(50, 50), 1, 3, cv::Scalar(0, 255, 0));
	cv::imshow(wname, om_3c_origin);

	cv::waitKey(1);
}

void GLRenderer::showObModel(const char* wname, const char* text,cv::Mat oimg_in, int px, int py)
{
	cv::Mat final_model2;
	//_renderer._glrenderer.getDEPTHfromCPU(final_model2);
	getDEPTHfromCPU(final_model2);


	cv::Mat final_model = cv::Mat(height, width, CV_32FC1);
	//cv::Rect rect(0, 0, width, height);
	cv::Rect rect(px*width, py*height, width, height);
	final_model = final_model2(rect);
	//cv::imshow("finalmodel", final_model);

	//visualize 3color
	cv::Mat o_3c = cv::Mat(height, width, CV_8UC3);
	cv::Mat m_3c = cv::Mat(height, width, CV_8UC3);
	cv::Mat om_3c = cv::Mat(height, width, CV_8UC3);
	o_3c.setTo(0);
	m_3c.setTo(0);

	for (int i = 0; i < width; i++)
	for (int j = 0; j < height; j++){
		if (final_model.at<float>(j, i) != 0)
		{
			m_3c.at <unsigned char>(j, 3 * i + 0) = 0;
			m_3c.at <unsigned char>(j, 3 * i + 1) = 0;
			m_3c.at <unsigned char>(j, 3 * i + 2) = 255;
		}
		if (oimg_in.at<float>(j, i) != 0)
		{
			o_3c.at <unsigned char>(j, 3 * i + 0) = 255;
			o_3c.at <unsigned char>(j, 3 * i + 1) = 0;
			o_3c.at <unsigned char>(j, 3 * i + 2) = 0;
		}
	}
	cv::addWeighted(o_3c, 0.5, m_3c, 0.5, 0, om_3c);

	//cv::imshow("o", o_3c);
	//cv::imshow("m", m_3c);
	//cv::imshow("om", om_3c);

	cv::Mat om_3c_origin = cv::Mat(480, 640, CV_8UC3);
	cv::resize(om_3c, om_3c_origin, cv::Size(640, 480), 0, 0, 1);
	//cv::imshow("om", om_3c_origin);

	cv::putText(om_3c_origin, text, cvPoint(50, 50), 1, 3, cv::Scalar(0, 255, 0));
	cv::imshow(wname, om_3c_origin);

	cv::Mat ddd = om_3c_origin;

	cv::waitKey(1);

}