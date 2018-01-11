#include <iostream>

#include "HandParameter.h"
#include "GLRenderer.h"
#include "Hand.h"
#include "HandGenerator.h"

#define JOINT_COUNT 19
int width = 640;
int height = 480;

void stopWatch()
{
	static double time = 0.0;
	static bool bCheckTime = false;
	static int timerCount = 0;
	if (!bCheckTime) {
		time = (double)cv::getTickCount();
		timerCount++;
	}
	if (bCheckTime) {
		time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
		printf("                               Time:%d:%lfsec\n", timerCount, time);		//cout << "Time" << timerCount << ": " << time << " sec" << endl;
	}

	bCheckTime = !bCheckTime;

}

void main()
{
	cv::Mat glCalMat = cv::Mat(3, 3, CV_32FC1);
	glCalMat.at<float>(0, 0) = 477.9;
	glCalMat.at<float>(0, 1) = 0.0;
	glCalMat.at<float>(0, 2) = 326.6;
	glCalMat.at<float>(1, 0) = 0.0;
	glCalMat.at<float>(1, 1) = 477.9;
	glCalMat.at<float>(1, 2) = 245.9;
	glCalMat.at<float>(2, 0) = 0.0;
	glCalMat.at<float>(2, 1) = 0.0;
	glCalMat.at<float>(2, 2) = 1.0;

	HandParameters hg(HandParameters::Default());
	GLRenderer glrenderer(width, height, 1, 1, glCalMat);
	Hand hand;
	hand.Init(hg);
	HandGenerator handgenerator(hg);


	//for (int i = 0; i < JOINT_COUNT; i++) 
	//	hand.SetJoint(i,0, 90, 90, 90);
	//handgenerator.run();

	hand.Render(hg.render_fbo_width, hg.render_fbo_height, 0, 0, 0, false, 10);
	//glutSwapBuffers();

	
	while (1){
		cv::Mat model_img;
		//glrenderer.getRGBfromCPU(model_img);
		glrenderer.getDEPTHfromCPU(model_img);
		cv::imshow("cpu", model_img);
		cv::waitKey(1);
	}
	

	//std::cout << "Press enter to exit";
	//std::cin.ignore().get();
	
}