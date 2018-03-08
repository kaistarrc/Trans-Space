#include "RealSenseCVWrapper.h"

using namespace Intel::RealSense;
using namespace cv;
using namespace std;


RealSenseCVWrapper::RealSenseCVWrapper()
{
}

RealSenseCVWrapper::RealSenseCVWrapper(int w = 640, int h = 480)
{
	width = w;
	height = h;

	rsm = SenseManager::CreateInstance();
	if (!rsm) {
		cout << "Unable to create SenseManager" << endl;
	}
	// RealSense settings
	rsm->EnableStream(Capture::STREAM_TYPE_COLOR, w, h);
	rsm->EnableStream(Capture::STREAM_TYPE_DEPTH, w, h);
	if (rsm->Init() < Status::STATUS_NO_ERROR) {
		cout << "Unable to initialize SenseManager" << endl;
	}
	bufferSize = Size(w, h);

	depth3c = cv::Mat(h, w, CV_8UC3);

}


RealSenseCVWrapper::~RealSenseCVWrapper()
{
	safeRelease();
}


void RealSenseCVWrapper::safeRelease()
{
	if (rsm) {
		//rsm->Release();
		rsm->Close();
	}
}

void RealSenseCVWrapper::getDepth3cBuffer(cv::Mat& out){

	depth3c.setTo(0);

	for (int i = 0; i < width; i++)
	for (int j = 0; j < height; j++){
		float d = depthBuffer.at<float>(j, i);

		depth3c.at<uchar>(j, i * 3 + 2) = (int)d / 65536;
		d -= 65536 * depth3c.at<uchar>(j, i * 3 + 2);

		depth3c.at<uchar>(j, i * 3 + 1) = (int)d / 256;
		d -= 256 * depth3c.at<uchar>(j, i * 3 + 1);

		depth3c.at<uchar>(j, i * 3 + 0) = (int)d;
	}

	out = depth3c;
}

void RealSenseCVWrapper::makeDepthFromColorcodeDepth(cv::Mat& out){

	for (int i = 0; i < width; i++)
	for (int j = 0; j < height; j++){
		float b = depth3c.at<uchar>(j, 3 * i + 0);
		float g = depth3c.at<uchar>(j, 3 * i + 1);
		float r = depth3c.at<uchar>(j, 3 * i + 2);

		out.at<float>(j, i) = 65536 * r + 256 * g + 1 * b;

	}


}


bool RealSenseCVWrapper::queryFrames()
{
	if (rsm->AcquireFrame(true) < Status::STATUS_NO_ERROR) {
		return false;
	}
	sample = rsm->QuerySample();

	return true;
}

void RealSenseCVWrapper::releaseFrames()
{
	rsm->ReleaseFrame();
}

void RealSenseCVWrapper::getColorBuffer()
{
	// Acquire access to image data
	Image *img_c = sample->color;
	Image::ImageData data_c;
	img_c->AcquireAccess(Image::ACCESS_READ_WRITE, Image::PIXEL_FORMAT_BGR, &data_c);
	// create OpenCV Mat from Image::ImageInfo
	Image::ImageInfo cinfo = img_c->QueryInfo();
	colorBuffer = Mat(cinfo.height, cinfo.width, CV_8UC3);
	// copy data
	colorBuffer.data = data_c.planes[0];
	colorBuffer = colorBuffer.clone();
	// release access
	img_c->ReleaseAccess(&data_c);
	//img_c->Release();
}

void RealSenseCVWrapper::getDepthBuffer()
{
	Image *img_d = sample->depth;
	Image::ImageData data_d;
	img_d->AcquireAccess(Image::ACCESS_READ_WRITE, Image::PIXEL_FORMAT_DEPTH_F32, &data_d);
	Image::ImageInfo dinfo = img_d->QueryInfo();
	depthBuffer = Mat(dinfo.height, dinfo.width, CV_32FC1);
	depthBuffer.data = data_d.planes[0];
	depthBuffer = depthBuffer.clone();
	img_d->ReleaseAccess(&data_d);
	//img_d->Release();

	//segmentation

	for (int i = 0; i < dinfo.width; i++)
	for (int j = 0; j < dinfo.height; j++)
	{
		if (depthBuffer.at<float>(j, i)>400)
			depthBuffer.at<float>(j, i) = 0;
	}

	//cv::imshow("segimg", depthBuffer);



}

void RealSenseCVWrapper::getMappedDepthBuffer()
{
	// create projection stream to acquire mapped depth image
	Projection *projection = rsm->QueryCaptureManager()->QueryDevice()->CreateProjection();
	Image *depth_mapped = projection->CreateDepthImageMappedToColor(sample->depth, sample->color);
	// acquire access to depth data
	Image::ImageData ddata_mapped;
	depth_mapped->AcquireAccess(Image::ACCESS_READ, Image::PIXEL_FORMAT_DEPTH_F32, &ddata_mapped);
	// copy to cv::Mat
	Image::ImageInfo dinfo = depth_mapped->QueryInfo();
	depthBufferMapped = Mat(dinfo.height, dinfo.width, CV_32FC1);
	depthBufferMapped.data = ddata_mapped.planes[0];
	depthBufferMapped = depthBufferMapped.clone();
	// release access
	depth_mapped->ReleaseAccess(&ddata_mapped);
	depth_mapped->Release();
	projection->Release();
}

void RealSenseCVWrapper::getMappedColorBuffer()
{
	// create projection stream to acquire mapped depth image
	Projection *projection = rsm->QueryCaptureManager()->QueryDevice()->CreateProjection();
	Image *color_mapped = projection->CreateColorImageMappedToDepth(sample->depth, sample->color);
	// acquire access to depth data
	Image::ImageData cdata_mapped;
	color_mapped->AcquireAccess(Image::ACCESS_READ, Image::PIXEL_FORMAT_BGR, &cdata_mapped);
	// copy to cv::Mat
	Image::ImageInfo cinfo = color_mapped->QueryInfo();
	colorBufferMapped = Mat(cinfo.height, cinfo.width, CV_8UC3);
	colorBufferMapped.data = cdata_mapped.planes[0];
	colorBufferMapped = colorBufferMapped.clone();
	// release access
	color_mapped->ReleaseAccess(&cdata_mapped);
	color_mapped->Release();
	projection->Release();
}



void RealSenseCVWrapper::getXYZBuffer()
{
	Projection *projection = rsm->QueryCaptureManager()->QueryDevice()->CreateProjection();
	std::vector<Point3DF32> vertices;
	vertices.resize(bufferSize.width * bufferSize.height);
	projection->QueryVertices(sample->depth, &vertices[0]);
	xyzBuffer.clear();
	for (int i = 0; i < bufferSize.width*bufferSize.height; i++) {
		Point3f p;
		p.x = vertices[i].x;
		p.y = vertices[i].y;
		p.z = vertices[i].z;
		xyzBuffer.push_back(p);
	}
	projection->Release();
}

void RealSenseCVWrapper::getColorBuffer(cv::Mat & color)
{
	getColorBuffer();
	//color = colorBuffer;
	color = colorBuffer.clone();
}

void RealSenseCVWrapper::getDepthBuffer(cv::Mat & depth)
{
	getDepthBuffer();
	//depth = depthBuffer;
	depthBuffer.copyTo(depth);


}

void RealSenseCVWrapper::getMappedDepthBuffer(cv::Mat & mappedDepth)
{
	getMappedDepthBuffer();
	mappedDepth = depthBufferMapped.clone();
}

void RealSenseCVWrapper::getMappedColorBuffer(cv::Mat & mappedColor)
{
	getMappedColorBuffer();
	mappedColor = colorBufferMapped.clone();
}

void RealSenseCVWrapper::getXYZBuffer(std::vector<cv::Point3f> xyz)
{
	getXYZBuffer();
	xyz = xyzBuffer;
}

void RealSenseCVWrapper::useAutoAdjust(bool use_aa)
{
	Device *device = rsm->QueryCaptureManager()->QueryDevice();
	// get current values if you use manual mode
	device->SetColorAutoExposure(use_aa);
	device->SetColorAutoWhiteBalance(use_aa);
}

void RealSenseCVWrapper::setExposure(int32_t value)
{
	rsm->QueryCaptureManager()->QueryDevice()->SetColorExposure(value);
}

void RealSenseCVWrapper::setWhiteBalance(int32_t value)
{
	rsm->QueryCaptureManager()->QueryDevice()->SetColorWhiteBalance(value);
}

void RealSenseCVWrapper::getCalibrationStatus()
{
	// aquire calibration data of depth camera stream
	Projection * projection = rsm->QueryCaptureManager()->QueryDevice()->CreateProjection();
	Calibration::StreamCalibration calib;
	Calibration::StreamTransform trans;
	projection->QueryCalibration()
		->QueryStreamProjectionParameters(StreamType::STREAM_TYPE_DEPTH, &calib, &trans);

	// copy depth camera calibration data to OpenCV
	cameraMatrix = Mat::eye(3, 3, CV_32FC1);
	cameraMatrix.at<float>(0, 0) = calib.focalLength.x;
	cameraMatrix.at<float>(1, 1) = calib.focalLength.y;
	cameraMatrix.at<float>(0, 2) = calib.principalPoint.x;
	cameraMatrix.at<float>(1, 2) = calib.principalPoint.y;

	
	distCoeffs = Mat::zeros(5, 1, CV_32FC1);
	distCoeffs.at<float>(0) = calib.radialDistortion[0];
	distCoeffs.at<float>(1) = calib.radialDistortion[1];
	distCoeffs.at<float>(2) = calib.tangentialDistortion[0];
	distCoeffs.at<float>(3) = calib.tangentialDistortion[1];
	distCoeffs.at<float>(4) = calib.radialDistortion[2];

	transform = Mat::eye(4, 4, CV_32FC1);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			transform.at<float>(i, j) = trans.rotation[i][j];        // todo: rotationªÎú¼ªÈÖªªÎâ÷ªòü¬ìã
		}
		transform.at<float>(i, 3) = trans.translation[i];
	}
	projection->Release();
}
