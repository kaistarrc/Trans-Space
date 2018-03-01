#include <iostream>


#include "Camera.h"

#include "HandParameter.h"
#include "GLRenderer.h"
#include "Hand.h"
#include "HandGenerator.h"
#include "pso.h"
#include "CostFunction.h"
#include "Preprocessing.h"
#include "MMF.h"
#include <windows.h>


#include <pthread.h>
#pragma comment(lib,"pthreadVC2.lib")

pthread_t thread_id;
int state;
static MMF mmf(128,128);

void stopWatch()
{
	static DWORD start = 0.0;
	static DWORD end = 0.0;
	
	static bool bCheckTime = false;
	
	if (!bCheckTime) {
		start = GetTickCount();
	}
	if (bCheckTime) {
		end = GetTickCount();
		printf("       Time  : %lf \n", (end - start) / (double)1000);
	}

	bCheckTime = !bCheckTime;

}

void* sendImage_learning(void* data)
{
	mmf.send2CNN();

	//pthread_exit((void*)data);

	//return data;
	return((void*)1);
}

/*/
*  for learning data
*
**/
//#define SAVE_TRACKBAR_POSTURE_SYNTHETIC    //o
//#define SAVE_TRAININGSET_SYNTHETIC          //o
//#define SAVE_TRAININGSET_REALHAND


/*/
*  test trained network
*
**/
//#define TEST_NETWORK_ON_TRAININGSET   //o
//#define TEST_NETWORK_ON_REALTIME      //o


/*/
*  save and test on sequence data
*
**/

//#define SAVE_SYNTHETIC_SEQUENCE_COMPARISON  //o
#define TEST_ON_SEQUENCE                   //o

/*/
*  real data test
*
**/

//#define TEST_REALTIME                      //o




void main(int argc, char** argv)
{
	
#pragma region user parameter setting

	//--model size---
	//good except for pinky.
	float sx_palm = 8; float sy_palm = 8; float sz_palm = 8; //x :가로  , y: 두께,   z: 세로
	float sx_finger=1, sy_finger=1, sz_finger=1;


	/**
	*   \camera type
	*   \"realcamera", "playcamera", "nyucamera"
	*   \"glcamera_gui","glcamera_cnn_dataset", "glcamera_sequence"
	*   \"glcamera_test"
	**/
	string cameratype = "playcamera";
	string testImageType = "vga2";

	/**
	*   \model type
	*   "hand with wrist", "hand without wrist"
	**/
	char* setup_model_path_low = "data/hand_20180226_1.dae";
	char* setup_modelTexture_path = "data/hand_2.bmp";

	/**
	*   \tracking type
	*    segmenthand_enable:  segmentation of blue band.
	**/
	string trackingtype = "26D";
	bool runPSO_enable = false;
	bool segmenthand_enable = false;
	bool sendCNN_enable = false;

	/**
	*   \ save 
	*  saveimage_enable : record sequence images for ours(VGA) and epfl(QVGA)
	*  savegroundturth_enable : save 26D(parameters), 15D(joints' position)
	*  savepsoresult_enable  : save tracking result (15D (joints' position))
	*  saveexperiment_enable : not used yet (we will use this for self evaluation)
	**/
	bool saveimage_enable = false;
	bool saveimage_training_enable = false; int postureName = 0;

	bool savegroundtruth_enable = false;
	bool savepsoresult_enable = false;
	bool saveexperiment_enable = false;
	std::string recordFrameOpt = "no";
	float num_between = 1;

	bool upframe_enable = false;
	/**
	*   \ visualize
	*  showgroundtruth_enable:  joint visualization as circle.
	*  showcnnresult_enable:   not used in present.
	**/
	bool showgroundtruth_enable = false;//show
	bool showcnnresult_enable = false;//show
	bool showtrackerresult_enable = false;//show

	/**
	*  \ setting for test
	*  groundtruth_cnn.
	*  testsequence for comparison
	*  run model fitting on testsequence. 
	**/

#ifdef SAVE_TRAININGSET_SYNTHETIC
	cameratype = "glcamera_cnn_dataset";
	saveimage_enable = true;
	recordFrameOpt = "uvr";
	savegroundtruth_enable = true;
	//showgroundtruth_enable = true;
	upframe_enable = true;

	setup_model_path_low = "data/hand_20180226_1.dae";
	setup_modelTexture_path = "data/hand_2.bmp";
#endif

#ifdef SAVE_TRAININGSET_REALHAND
	cameratype = "realcamera";

	segmenthand_enable = true;
	saveimage_training_enable = true;

#endif

#ifdef TEST_NETWORK_ON_TRAININGSET
	cameratype = "playcamera";
	sendCNN_enable = true;
	showcnnresult_enable = true;
#endif

#ifdef TEST_NETWORK_ON_REALTIME
	cameratype = "realcamera";
	sendCNN_enable = true;
	showcnnresult_enable = true;
	segmenthand_enable = true;
#endif


#ifdef TEST_REALTIME

	cameratype = "realcamera";

	trackingtype = "adaptive";
	if (trackingtype == "adaptive")
		sendCNN_enable=true;
	
	runPSO_enable = true;
	segmenthand_enable = true;



	setup_model_path_low = "data/hand_20180226_1.dae";
	setup_modelTexture_path = "data/hand_2.bmp";
#endif

#ifdef SAVE_SYNTHETIC_SEQUENCE_COMPARISON
	cameratype = "glcamera_sequence";
	saveimage_enable = true;
	recordFrameOpt = "all";
	savegroundtruth_enable = true;
	upframe_enable = true;
	showgroundtruth_enable = true;

	setup_model_path_low = "data/wristHand_20180226_2.dae";
	setup_modelTexture_path = "data/wristHand_1.bmp";

	num_between = 2;//2,5,10
	
#endif

#ifdef TEST_ON_SEQUENCE
	cameratype = "playcamera";
	//testImageType = "vga2"; //"vga2","vga5","vga10"
	//trackingtype = "fixed"; //"26D", "fixed", "adaptive"
	testImageType = argv[2];
	trackingtype = argv[3];
	
	//char* aa = "ho";
	//string ho = aa;
	//std::cout << ho;

	if (trackingtype != "26D")
		sendCNN_enable=true;
	
	runPSO_enable = true;
	segmenthand_enable = true;
	savepsoresult_enable = true;
	upframe_enable = true;

	setup_model_path_low = "data/hand_20180226_1.dae";
	setup_modelTexture_path = "data/hand_2.bmp";

	//sx_palm =9 ; sy_palm = 9; sz_palm = 9; //x :가로  , y: 두께,   z: 세로
	


	
#endif

#ifdef SAVE_TRACKBAR_POSTURE_SYNTHETIC
	cameratype = "glcamera_gui";

	setup_model_path_low = "data/wristHand_20180226_2.dae";
	setup_modelTexture_path = "data/wristHand_1.bmp";

	showgroundtruth_enable = true;
#endif

#ifdef SAVE_TRACKBAR_POSTURE_REALDATA
	cameratype = "realcamera";
	segmenthand_enable = true;


#endif


	/**
	* default setting.. 
	*
	**/
	//
	int width = 640;
	int height = 480;
	int width_tile = 64;// 128;
	int height_tile = 64;// 128;
	int width_mmf = 128;
	int height_mmf = 128;
	int handParamNum = 26;
	int JOINT_COUNT = 19;
	int max_generation = 30;
	int particle_numx = 8;
	int particle_numy = 8;
	int particle_num = particle_numx*particle_numy;
	int width_fb = width_tile * particle_numx;
	int height_fb = height_tile * particle_numy;

	float fx, fy, cx, cy;

	fx = 477.9; fy = 477.9;
	cx = 320.0; cy = 240.0;
	




	
#pragma endregion

#pragma region init class	

	HandParameters hp = HandParameters::Default();
	hp.width = width; hp.height = height;
	hp.width_tile = width_tile; hp.height_tile = height_tile;
	hp.particle_numx = particle_numx; hp.particle_numy = particle_numy;
	hp.handParamNum = handParamNum;
	hp.sx_palm = sx_palm; hp.sy_palm = sy_palm; hp.sz_palm = sz_palm;
	hp.sx_finger = sx_finger; hp.sy_finger = sy_finger; hp.sz_finger = sz_finger;
	hp.changeCameraProperty(fx, fy, cx, cy);

	hp.setup_model_path_low = setup_model_path_low;
	hp.setup_modelTexture_path = setup_modelTexture_path;
	

	GLRenderer glrenderer(width, height, width_tile, height_tile, width_fb, height_fb);
	HandGenerator handgenerator(hp);
	CostFunction costFunction(particle_numx, particle_numy, handParamNum,glrenderer);
	PSO pso(particle_numx, particle_numy, max_generation,handParamNum,
		&handgenerator,glrenderer,costFunction,&mmf);
	
	Camera camera(width, height, &handgenerator, &glrenderer, cameratype);
	Preprocessing preproc(width, height,camera);

	handgenerator._posesetgenerator._num_between = num_between;
#pragma endregion

	//temporary for test(experiment)
	if (cameratype == "playcamera")	
		camera.playcamera->_testType = testImageType;

	//common variable
	float com_hand[3];
	
	while (1){
		
		if (camera.queryFrames() == false)
			break;
		
	
#pragma region camera input

		cv::Mat cam_depth;
		camera.getDepthBuffer(cam_depth);
		//cv::imshow("cam_depth", cam_depth);
		
		//cv::Mat cam_depth8u = cv::Mat(height, width, CV_8UC3);
		//cv::normalize(cam_depth, cam_depth8u, 0, 255, cv::NORM_MINMAX, CV_8UC3);
		//cv::imshow("cam_depth8u", cam_depth8u);

		cv::Mat cam_color;
		camera.getMappedColorBuffer(cam_color);
		cv::imshow("cam_color", cam_color);
		//cv::moveWindow("cam_color", 640 * 2, 0);
		//glutSwapBuffers();

#pragma endregion 

		
		
		
#pragma region preprocessing
		if (segmenthand_enable==true)
			preproc.segmentHandFromBand(cam_color, cam_depth);
		preproc.getComHandxyz(cam_depth,com_hand);

#pragma endregion
		

#pragma region make cnn image & transfer it to CNN.
		if (sendCNN_enable==true){
			cv::Mat depth_cnn;
			preproc.makeCnnImage(cam_depth);
			preproc.getCnnImage(depth_cnn);
			//cv::imshow("depth_cnn", depth_cnn);

			if (mmf.getimg_bool == false)
			{
				mmf.getimg_bool = true;
				//mmf._cnnimg = depth_norm;
				//mmf._cnnimg= depth_cnn;
				depth_cnn.copyTo(mmf._cnnimg);
				//mmf._cnnimg = cam_colordepth;

				//state = pthread_create(&thread_id, NULL, sendImage_learning, NULL);
				//pthread_detach(thread_id);
				mmf.send2CNN();
			}
		}
#pragma endregion
	
#pragma region show ground truth 
		if (showgroundtruth_enable == true)
		{
			cv::Mat cmat;
			camera.getCalibrationMatrix(cmat);
			handgenerator.showJoints(cam_color,cam_depth, cmat);
		}
#pragma endregion
		


#pragma region get cnn result
		if (showcnnresult_enable == true)
		{
			//45D
			/*
			float cnnresult[5 * 3 * 3];
			mmf.receiveData();
			mmf.getLabel(cnnresult);

			//visualize cnnresult
			cv::Mat cmat;
			camera.getCalibrationMatrix(cmat);

			int color[5][3] = { { 255, 0, 0 }, { 0, 255, 0 }, { 0, 0, 255 }, { 255, 255, 0 }, { 0, 255, 255 } };
			for (int i = 0; i < 15; i++)
			{
				float x = com_hand[0] - cnnresult[3 * i + 0];
				float y = com_hand[1] - cnnresult[3 * i + 1];
				float z = com_hand[2] - cnnresult[3 * i + 2];

				float x_ = x * cmat.at<float>(0, 0) + y * cmat.at<float>(0, 1) + z * cmat.at<float>(0, 2);
				float y_ = x * cmat.at<float>(1, 0) + y * cmat.at<float>(1, 1) + z * cmat.at<float>(1, 2);
				float z_ = x * cmat.at<float>(2, 0) + y * cmat.at<float>(2, 1) + z * cmat.at<float>(2, 2);
				x_ /= z_;
				y_ /= z_;

				//printf("p[%d]: x:%f y:%f z:%f\n", i, cnnresult[3 * i + 0], cnnresult[3 * i + 1], cnnresult[3 * i + 2]);
				//printf("[%d]: x_:%f y_:%f\n",i, x_, y_);
	
				cv::circle(cam_color, cv::Point(x_, y_), 10, cv::Scalar(color[i / 3][0], color[i / 3][1], color[i / 3][2]), 5, 8, 0);

			}
			cv::imshow("predicted", cam_color);
			*/

			//26D
			float cnnresult[26];
			mmf.receiveData();
			mmf.getLabel(cnnresult);

			//visualize cnnresult
			float solp[26];
			for (int i = 0; i < 26; i++)
				solp[i] = cnnresult[i];
			solp[0] += com_hand[0]; solp[1] += com_hand[1];	solp[2] += com_hand[2];

			//printf("t: %.2f %.2f %.2f\n", solp[0], solp[1], solp[2]);
			//printf("r: %.2f %.2f %.2f\n", solp[3], solp[4], solp[5]);
			//for (int i = 0; i < 5; i++)
			//	printf("f[%d]: %.2f %.2f %.2f %.2f\n",i,solp[6 + 4 * i], solp[7 + 4 * i], solp[8 + 4 * i], solp[9 + 4 * i]);

			handgenerator.renderOrig(solp, "color");

			cv::Mat model_color;
			glrenderer.getOrigImage(model_color, "color");
			cv::imshow("cnnresult", model_color);

			cv::Mat fimg;
			cv::addWeighted(cam_color, 0.3, model_color, 0.9, 0, fimg);
			cv::imshow("cnnresult2rgb", fimg);
		}


#pragma endregion

#pragma region model fitting
		if (runPSO_enable == true){

			if (cameratype=="glcamera_sequence" || cameratype=="glcamera_test")
			{
				float jpos[26];
				handgenerator.getHandPose(jpos);
				pso.setTruePose(jpos);
			}
			

			//pso.bound_alpha0 =  std::atof(argv[1]);  //0.5 or 0.3
			//pso.bound_alpha1 = std::atof(argv[2]); //0.5
			//pso.experimentID = std::atoi(argv[3]);
			//pso.experimentID = std::atoi(argv[1]);

			pso.experimentID = 2;

			pso.run(cam_color, cam_depth, com_hand, trackingtype);
			
		}
#pragma endregion

		//if(showtrackerresult_enable==true)
		//	pso.showObModel

#pragma region save ground truth
		if (saveimage_enable == true)
			camera.recordFrames(recordFrameOpt);

		if (saveimage_training_enable == true){
			camera.recordFramesTraining(postureName, cam_depth);
			
			if (cv::waitKey(1) == 'a')
				upframe_enable = true;

			if (camera._frame == 300){
				camera._frame = 0;
				upframe_enable = false;
				postureName += 1;

			}
		}

		if (savegroundtruth_enable == true){
			handgenerator.saveJoints();
			handgenerator.saveParameters26D(com_hand);
		}
			
		
		if (savepsoresult_enable == true){
			pso.saveJoints(cam_color,testImageType,trackingtype, camera._frame);
			//pso.saveImage(cam_color,camera._frame);
			pso.saveImage(cam_color, testImageType, trackingtype, camera._frame);
		}

		
#pragma endregion

#pragma region accuracy experiment
		if (saveexperiment_enable == true){
			
			float groundtruth[26];
			if (cameratype == "glcamera_sequence" || cameratype == "glcamera_test")
				handgenerator.getHandPose(groundtruth);
			else
				pso.getTruePose(groundtruth);

			float estimated[26];
			pso.getFinalSolution(estimated);

			FILE* fp;
			char filename[200];
			//sprintf(filename, "experiment/method%d.csv",std::atoi(argv[4]));
			sprintf(filename, "experiment/method0_poseError%d.csv", pso.experimentID);
			fp = fopen(filename, "a");

			char str[100];
			for (int i = 0; i < 26; i++){
				float err = estimated[i]- groundtruth[i];

				if (i == 25)
					sprintf(str, "%.2f\n", err);
				else
					sprintf(str, "%.2f,", err);

				fputs(str, fp);
			}
			fclose(fp);
		}

#pragma endregion

	
#pragma region gui test	
		/*
		{
			//set track bar from pso result

			char key = cv::waitKey(1);
			if (key == 'c'){
				float* solp = &pso.gbest.at<float>(0, 0);
				handgenerator.run_setTbarFromResult(solp);
				handgenerator.save_trackbar();
			}
			if (key == 'v'){ //not implemented yet.
				for (int j = 0; j < 6; j++)
					pso.gbest.at<float>(0, j) = handgenerator._trackbar.wval[j];

				for (int i = 0; i < 5; i++)
				{
					pso.gbest.at<float>(0, 6 + 4 * i + 0) = handgenerator._trackbar.fval[1 + 3 * i + 0][0];
					pso.gbest.at<float>(0, 6 + 4 * i + 1) = handgenerator._trackbar.fval[1 + 3 * i + 0][1];
					pso.gbest.at<float>(0, 6 + 4 * i + 2) = handgenerator._trackbar.fval[1 + 3 * i + 1][0];
					pso.gbest.at<float>(0, 6 + 4 * i + 3) = handgenerator._trackbar.fval[1 + 3 * i + 2][0];
				}
			}

			//run gui

			handgenerator._trackbar.run();
			handgenerator.run_gui2hand("color");

			cv::Mat model_depth;
			glrenderer.getOrigImage(model_depth, "color");
			cv::imshow("manual", model_depth);
			//cvMoveWindow("manual", 1000, 0);

		// make cnn image & transfer it to CNN.
			//cv::Mat depth_cnn;
			//preproc.makeCnnImage(model_depth);
			//preproc.getCnnImage(depth_cnn);

			//if (mmf.getimg_bool == false)
			//{
			//	mmf.getimg_bool = true;
			//	//mmf._cnnimg = depth_norm;
			//	//mmf._cnnimg= depth_cnn;
			//	depth_cnn.copyTo(mmf._cnnimg);
			//	//mmf._cnnimg = cam_colordepth;

			//	state = pthread_create(&thread_id, NULL, sendImage_learning, NULL); 
			//	pthread_detach(thread_id);
			//}
		}
*/
#pragma endregion
	

#pragma region release
		
		
		//debugging with synthetic data
		
		//for (int j = 0; j < 26; j++)
		//	printf("err[%d]=%f\n", j, pso.gbest.at<float>(0, j) - pso.truepose.at<float>(0, j));
		
		//if (pso.gbestIdx < particle_num / 2)
		//	printf("[%d]got from tracking\n",camera._frame);
		//else
		//	printf("[%d]got from cnn\n",camera._frame);
		
		
		/*
		while (1){
			if (cv::waitKey(1) == '0')
				break;

			if (cv::waitKey(1) == 'q')
				exit(1);
		}
		*/

		if (cv::waitKey(1) == 'q')
			break;



	
		
		
		//if (cv::waitKey(1) == 'a'){
		//	int cid;
		//	printf("frame id:\n");
		//	std::cin >> cid;
		//	camera._frame = cid;
		//}


		if (upframe_enable == true)
			camera._frame++;

		camera.releaseFrames();
		cv::waitKey(1);
		
		
		
		
		
#pragma endregion
		
	}
}
