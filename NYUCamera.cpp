#include "NYUCamera.h"

NYUCamera::NYUCamera(){

}

NYUCamera::NYUCamera(int w,int h)
{
	//savetype = "train";
	savetype = "test";

	width = w;// 640;
	height = h;// 480;
	joint_max = 36;

	if (savetype.compare("train") == 0)
		frame_max = 72757;
	else if (savetype.compare("test") == 0)
		frame_max = 8252;//8252;


	cameraMatrix = cv::Mat(3, 3, CV_32FC1);
	getCalibrationStatus();
	cv::invert(cameraMatrix, cameraMatrix_inv);

	//preprocessing = Preprocessing(width, height, cameraMatrix);

	depth_data = cv::Mat(height, width, CV_32FC1);
	synth_data = cv::Mat(height, width, CV_32FC1);
	rgb_data = cv::Mat(height, width, CV_8UC3);


	//_frame = 2;//520

	//joint label
	joint_xyz = new cv::Mat[frame_max];
	joint_uvd = new cv::Mat[frame_max];
	joint_uvc_pred = new cv::Mat[frame_max];

	for (int i = 0; i < frame_max; i++){
		joint_xyz[i] = cv::Mat(3, joint_max, CV_32FC1);
		joint_uvd[i] = cv::Mat(3, joint_max, CV_32FC1);
		joint_uvc_pred[i] = cv::Mat(3, joint_max, CV_32FC1);
	}

	joint_mat = new double[3 * frame_max * joint_max];

	setJointLabel();

	rad2deg = 3.14 / 180.0;


	
	//hand_param = new float[handParamNum];



}


NYUCamera::~NYUCamera()
{


}

void NYUCamera::releaseFrames()
{

}


void NYUCamera::getCalibrationStatus()
{
	cameraMatrix.at<float>(0, 0) = 588.036865;  cameraMatrix.at<float>(0, 1) = 0.0;			cameraMatrix.at<float>(0, 2) = 320;
	cameraMatrix.at<float>(1, 0) = 0.0;			cameraMatrix.at<float>(1, 1) = 587.075073;  cameraMatrix.at<float>(1, 2) = 240;
	cameraMatrix.at<float>(2, 0) = 0.0;			cameraMatrix.at<float>(2, 1) = 0.0;			cameraMatrix.at<float>(2, 2) = 1.0;
}



bool NYUCamera::queryFrames(int _frame)
{
	//get depth image
	char filename[100];
	if (savetype.compare("train") == 0)
		sprintf(filename, "D:\\research\\handdataset\\nyu_hand_dataset_v2\\dataset\\train\\depth_1_%07u.png", _frame);
	else
		sprintf(filename, "D:\\research\\handdataset\\nyu_hand_dataset_v2\\dataset\\test\\depth_1_%07u.png", _frame);
	cv::Mat depth_img_3c = cv::imread(filename, 1);

	//get synthetic image
	if (savetype.compare("train") == 0)
		sprintf(filename, "D:\\research\\handdataset\\nyu_hand_dataset_v2\\dataset\\train\\synthdepth_1_%07u.png", _frame);
	else
		sprintf(filename, "D:\\research\\handdataset\\nyu_hand_dataset_v2\\dataset\\test\\synthdepth_1_%07u.png", _frame);
	synth_img = cv::imread(filename, 1);
	//cv::imshow("synth_img", synth_img);

	//get color image
	if (savetype.compare("train") == 0)
		sprintf(filename, "D:\\research\\handdataset\\nyu_hand_dataset_v2\\dataset\\train\\rgb_1_%07u.png", _frame);
	else
		sprintf(filename, "D:\\research\\handdataset\\nyu_hand_dataset_v2\\dataset\\test\\rgb_1_%07u.png", _frame);
	rgb_data = cv::imread(filename, 1);

	//decode data to real depth(mm) ,and segment based on synthetic model.
	unsigned char b1, b2;
	unsigned char g1, g2;
	unsigned char r1, r2;
	unsigned short d1, d2;
	for (int i = 0; i<width; i++)
	for (int j = 0; j<height; j++)
	{

		b1 = synth_img.at<uchar>(j, 3 * i + 0);
		g1 = synth_img.at<uchar>(j, 3 * i + 1);
		r1 = synth_img.at<uchar>(j, 3 * i + 2);
		d1 = 65536 * r1 + 256 * g1 + 1 * b1;
		synth_data.at<float>(j, i) = d1;

		//if (synth_img.at<uchar>(j,i)==0)
		if (d1 == 0){
			depth_data.at<float>(j, i) = 0;
		}
		else
		{
			b2 = depth_img_3c.at<uchar>(j, 3 * i + 0);
			g2 = depth_img_3c.at<uchar>(j, 3 * i + 1);
			r2 = depth_img_3c.at<uchar>(j, 3 * i + 2);
			d2 = 65536 * r2 + 256 * g2 + 1 * b2;

			if (d2 > 1000)
				depth_data.at<float>(j, i) = 0;
			else{
				depth_data.at<float>(j, i) = d2;

			}
		}
	}


	//erase wrist region.
	
	{
		cv::Mat vec1 = cv::Mat(3, 1, CV_32FC1);
		cv::Mat vec2 = cv::Mat(3, 1, CV_32FC1);
		vec1.at<float>(0, 0) = joint_xyz[_frame].at<float>(0, 17) - joint_xyz[_frame].at<float>(0, 35);
		vec1.at<float>(1, 0) = joint_xyz[_frame].at<float>(1, 17) - joint_xyz[_frame].at<float>(1, 35);
		vec1.at<float>(2, 0) = joint_xyz[_frame].at<float>(2, 17) - joint_xyz[_frame].at<float>(2, 35);

		for (int i = 0; i < width; i++)
		for (int j = 0; j < height; j++)
		{
			float x_ = i*depth_data.at<float>(j, i);
			float y_ = j*depth_data.at<float>(j, i);
			float z_ = depth_data.at<float>(j, i);

			float x = x_*cameraMatrix_inv.at<float>(0, 0) + y_*cameraMatrix_inv.at<float>(0, 1) + z_*cameraMatrix_inv.at<float>(0, 2);
			float y = x_*cameraMatrix_inv.at<float>(1, 0) + y_*cameraMatrix_inv.at<float>(1, 1) + z_*cameraMatrix_inv.at<float>(1, 2);
			float z = x_*cameraMatrix_inv.at<float>(2, 0) + y_*cameraMatrix_inv.at<float>(2, 1) + z_*cameraMatrix_inv.at<float>(2, 2);

			vec2.at<float>(0, 0) = x - joint_xyz[_frame].at<float>(0, 35);
			vec2.at<float>(1, 0) = y - joint_xyz[_frame].at<float>(1, 35);
			vec2.at<float>(2, 0) = z - joint_xyz[_frame].at<float>(2, 35);

			float a = vec1.dot(vec2) / (cv::norm(vec1)*cv::norm(vec2));
			if (a < 0)
				depth_data.at<float>(j, i) = 0;
		}
	}
	
	//if (makeImage_NYU2014() == -1)
	//	return -1;


	return true;
}

void NYUCamera::getColorBuffer(cv::Mat& inout){
	//inout = rgb_data;
	rgb_data.copyTo(inout);
}

void NYUCamera::getDepthBuffer(cv::Mat& inout){
	//inout = depth_data;
	depth_data.copyTo(inout);
}
void NYUCamera::getSynthBuffer(cv::Mat& inout){
	//inout = depth_data;
	synth_img.copyTo(inout);
}


void NYUCamera::setJointLabel(){


	joint_max = 36;
	//ground truth (xyz)
	{
		char filename[100];
		sprintf(filename, "D:\\research\\handdataset\\nyu_hand_dataset_v2\\dataset\\%s\\joint_data.mat", savetype);

		diagnose(filename);

		MATFile *matfin;
		matfin = matOpen(filename, "r");

		mxArray *buffer;
		buffer = matGetVariable(matfin, "joint_xyz");
		joint_mat = (double*)mxGetData(buffer);

		//joint_xyz = new double[frame_max * joint_max * 3];
		for (int k = 0; k < 3; k++)
		for (int i = 0; i < frame_max; i++)
		for (int j = 0; j < joint_max; j++)
		{
			//if (k == 1) // inverse of y axis.
			//	joint_xyz[j + joint_max * i + joint_max * frame_max * k] = -data_mat[3 * i + 3 * frame_max * j + 3 * frame_max * joint_max * k];
			//else
			//	joint_xyz[j + joint_max * i + joint_max * frame_max * k] = data_mat[3 * i + 3 * frame_max * j + 3 * frame_max * joint_max * k];
			if (k == 1)
				joint_xyz[i].at<float>(k, j) = -joint_mat[3 * i + 3 * frame_max * j + 3 * frame_max * joint_max * k];
			else
				joint_xyz[i].at<float>(k, j) = joint_mat[3 * i + 3 * frame_max * j + 3 * frame_max * joint_max * k];

			//printf("%d %d %d\n", k, i, j);
		}

		mxDestroyArray(buffer);
		matClose(matfin);
	}

	//ground truth(uvd)
	{
		char filename[100];
		sprintf(filename, "D:\\research\\handdataset\\nyu_hand_dataset_v2\\dataset\\%s\\joint_data.mat", savetype);

		diagnose(filename);

		MATFile *matfin;
		matfin = matOpen(filename, "r");

		mxArray *buffer;
		buffer = matGetVariable(matfin, "joint_uvd");
		joint_mat = (double*)mxGetData(buffer);

		//joint_uvd = new double[frame_max * joint_max * 3];
		for (int k = 0; k < 3; k++)
		for (int i = 0; i < frame_max; i++)
		for (int j = 0; j < joint_max; j++)
		{
			joint_uvd[i].at<float>(k, j) = joint_mat[3 * i + 3 * frame_max * j + 3 * frame_max * joint_max * k];
			//joint_uvd[j + joint_max * i + joint_max * frame_max * k] = data_mat[3 * i + 3 * frame_max * j + 3 * frame_max * joint_max * k];

		}
		mxDestroyArray(buffer);
		matClose(matfin);
	}
	//predicted only on test.
	if (savetype.compare("test") == 0){
		char filename[100];
		sprintf(filename, "D:\\research\\handdataset\\nyu_hand_dataset_v2\\dataset\\%s\\test_predictions.mat", savetype);
		diagnose(filename);

		MATFile *matfin;
		matfin = matOpen(filename, "r");

		mxArray *buffer;
		buffer = matGetVariable(matfin, "pred_joint_uvconf");
		joint_mat = (double*)mxGetData(buffer);

		//joint_uvc_pred = new double[frame_max * joint_max * 3];
		//joint_xyz_pred = new double[frame_max * joint_max * 3];

		for (int i = 0; i < frame_max; i++)
		for (int j = 0; j < joint_max; j++){
			for (int k = 0; k < 3; k++)
				joint_uvc_pred[i].at<float>(k, j) = joint_mat[i + frame_max * j + frame_max * joint_max * k]; //original

			//joint_uvc_pred[i].at<float>(k, j) = joint_mat[3 * i + 3 * frame_max * j + 3 * frame_max * joint_max * k];
			//joint_uvc_pred[j + joint_max * i + joint_max * frame_max * k] = data_mat[i + frame_max * j + frame_max * joint_max * k];

		}

		mxDestroyArray(buffer);
		matClose(matfin);
	}


	//check length
	int jj0 = 30;
	int jj1 = 31;
	float x1 = joint_xyz[100].at<float>(0, jj0);
	float y1 = joint_xyz[100].at<float>(1, jj0);
	float z1 = joint_xyz[100].at<float>(2, jj0);

	float x2 = joint_xyz[100].at<float>(0, jj1);
	float y2 = joint_xyz[100].at<float>(1, jj1);
	float z2 = joint_xyz[100].at<float>(2, jj1);
	printf("length:%f\n", sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2)));
	cv::waitKey(1);
}

int NYUCamera::diagnose(const char *file) {
	MATFile *pmat;
	const char **dir;
	const char *name;
	int	  ndir;
	int	  i;
	mxArray *pa;

	printf("Reading file %s...\n\n", file);

	/*
	* Open file to get directory
	*/
	pmat = matOpen(file, "r");
	if (pmat == NULL) {
		printf("Error opening file %s\n", file);
		return(1);
	}

	/*
	* get directory of MAT-file
	*/
	dir = (const char **)matGetDir(pmat, &ndir);
	if (dir == NULL) {
		printf("Error reading directory of file %s\n", file);
		return(1);
	}
	else {
		printf("Directory of %s:\n", file);
		for (i = 0; i < ndir; i++)
			printf("%s\n", dir[i]);
	}
	mxFree(dir);

	/* In order to use matGetNextXXX correctly, reopen file to read in headers. */
	if (matClose(pmat) != 0) {
		printf("Error closing file %s\n", file);
		return(1);
	}
	pmat = matOpen(file, "r");
	if (pmat == NULL) {
		printf("Error reopening file %s\n", file);
		return(1);
	}

	/* Get headers of all variables */
	printf("\nExamining the header for each variable:\n");
	for (i = 0; i < ndir; i++) {
		pa = matGetNextVariableInfo(pmat, &name);
		if (pa == NULL) {
			printf("Error reading in file %s\n", file);
			return(1);
		}
		/* Diagnose header pa */
		printf("According to its header, array %s has %d dimensions\n",
			name, mxGetNumberOfDimensions(pa));
		if (mxIsFromGlobalWS(pa))
			printf("  and was a global variable when saved\n");
		else
			printf("  and was a local variable when saved\n");
		mxDestroyArray(pa);
	}

	/* Reopen file to read in actual arrays. */
	if (matClose(pmat) != 0) {
		printf("Error closing file %s\n", file);
		return(1);
	}
	pmat = matOpen(file, "r");
	if (pmat == NULL) {
		printf("Error reopening file %s\n", file);
		return(1);
	}

	/* Read in each array. */
	printf("\nReading in the actual array contents:\n");
	for (i = 0; i<ndir; i++) {
		pa = matGetNextVariable(pmat, &name);
		if (pa == NULL) {
			printf("Error reading in file %s\n", file);
			return(1);
		}
		/*
		* Diagnose array pa
		*/
		printf("According to its contents, array %s has %d dimensions\n",
			name, mxGetNumberOfDimensions(pa));
		if (mxIsFromGlobalWS(pa))
			printf("  and was a global variable when saved\n");
		else
			printf("  and was a local variable when saved\n");
		mxDestroyArray(pa);
	}

	if (matClose(pmat) != 0) {
		printf("Error closing file %s\n", file);
		return(1);
	}
	printf("Done\n");
	return(0);
}

void NYUCamera::showJoint(int _frame)
{
	/*
	char key = cv::waitKey(1);
	if (key == 'a')
	jid++;
	if (key == 's')
	jid--;
	printf("jid:%d\n", jid);
	*/

	cv::Mat nyu_joint;
	getJoint(nyu_joint, _frame);


	int visidx[6] = { 5, 11, 17, 23, 29, 35 };
	for (int i = 0; i < 6; i++)
		//for (int i = 0; i < joint_max;i++)
	{
		int jid = visidx[i];
		//int jid = i;

		float xt = nyu_joint.at<float>(0, jid);
		float yt = nyu_joint.at<float>(1, jid);
		float zt = nyu_joint.at<float>(2, jid);

		float x = xt*cameraMatrix.at<float>(0, 0) + yt*cameraMatrix.at<float>(0, 1) + zt*cameraMatrix.at<float>(0, 2);
		float y = xt*cameraMatrix.at<float>(1, 0) + yt*cameraMatrix.at<float>(1, 1) + zt*cameraMatrix.at<float>(1, 2);

		x /= zt;
		y /= zt;

		cv::circle(synth_img, cv::Point(x, y), 1, cv::Scalar(0, 0, 255), 1, 8, 0);

	}
	cv::imshow("joint", synth_img);
	cv::waitKey(1);

}


