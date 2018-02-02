#include "MMF.h"


MMF::MMF(int w, int h){
	width = w;
	height = h;

	DATA_LEN = width*height*sizeof(float); //how to do this automatically?
	DATA_LEN2 = 26;// 5 * 3 * 3;
	_cnnimg = cv::Mat(height, width, CV_32FC1);

	//char memoryname[1024];
	//sprintf(memoryname, "cnnCamera");

	///
	hMapWrite = NULL;

	// 매핑 파일 만들기	
	hEvent2 = CreateEventW(0, 0, TRUE, (LPCWSTR)"e2");
	hEvent = CreateEventW(0, 0, 0, (LPCWSTR)"e1");

	hMapWrite = CreateFileMappingW(INVALID_HANDLE_VALUE,
		NULL,
		PAGE_READWRITE,
		0,
		DATA_LEN, (LPCWSTR)
		"cnnCamera");


	if (hMapWrite == NULL)
		printf("hmapwrite fail\n");

	if (GetLastError() == ERROR_ALREADY_EXISTS)
		printf("이미 매핑 오브젝트가 있음.\n");

	// 파일에 매핑하기
	lpMapping_send = (float*)MapViewOfFile(hMapWrite,
		FILE_MAP_ALL_ACCESS,
		0,
		0,
		0);

	if (lpMapping_send == NULL)
		printf("lpmapping fail\n");


	//memory
	DL_result = new float[DATA_LEN2];

	//??
	getimg_bool = false;
}

MMF::~MMF(){

	//for send begin
	if (CloseHandle(hMapWrite) == 0)
		printf("메모리맵 핸들 종료 실패\n");
	else
		printf("메모리맵 핸들 종료 성공\n");

	UnmapViewOfFile(lpMapping_send);
	//for send end

}


void MMF::send2CNN(){

	cvWaitKey(1); // to 1?

	WaitForSingleObject(hEvent2, INFINITE);

	for (int i = 0; i<width; i++)
	for (int j = 0; j<height; j++)
	{
		lpMapping_send[i + j * width] = _cnnimg.at<float>(j, i);
	}
	SetEvent(hEvent); // 상대를 깨운다

	getimg_bool = false;
	//printf("mmf.data has been sended\n");

}

int MMF::receiveData()
{
	if (lpMapping_receive == NULL)
	{
		// 매핑 파일 열기
		hMapRead = OpenFileMappingA(FILE_MAP_READ,
			FALSE,
			"LearningResult");

		// 파일에 매핑하기
		lpMapping_receive = (float*)MapViewOfFile(hMapRead,
			FILE_MAP_READ,
			0,
			0,
			0);

		return -1;
	}


	//DL_result = lpMapping_receive[0];
	for (int i = 0; i < DATA_LEN2; i++)
		DL_result[i] = lpMapping_receive[i];

	return 0;
}

void MMF::getLabel(float* out)
{
	for (int i = 0; i < DATA_LEN2; i++)
		out[i] = DL_result[i];

	
		//out.push_back(i);
		//out.push_back(DL_result[i]);

	//out = DL_result;
}
