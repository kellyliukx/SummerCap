#include "process.h"

//���Գ�������ʱ��Ŀ�ܣ�����ֱ�ӵ���
void testTime()
{
	double time_ = static_cast<double>(cv::getTickCount());	

	//�ڴ˴���������ĳ���

	time_ = 1000*static_cast<double>(cv::getTickCount()-time_)/cv::getTickFrequency();
	std::cout<<"time = "<<time_<<"ms"<<std::endl;
}
int main()
{
	VideoCapture capture(0);
	if (!capture.isOpened())
		return -1;
	Mat frame;
	Detect code;
	vector<vector<Point>> cube;
	vector<vector<Point>>blankDetect;
	vector<int> results;
	vector<Point2f>toyCenters;
	while (capture.isOpened())
	{
		capture>>frame;
		Mat img = frame; //��ȡͼƬ
		if (!frame.data)
			break;		
		vector <Point> blankPoint;
		code.ledCubeDetect(img,cube);
//		imshow("cube",imgforshow);
		if(waitKey(30)>=0) break;
	}
	frame.release();
	capture.release();
	destroyAllWindows();
	return 0;
}

/*
int main()
{
	Detect code;
	vector<Point> blankPoint;
	const string inputDIR = "E:\\RoboMaster2016\\code\\data\\img\\img1\\14.png";
	Mat img = imread(inputDIR);
	code.ToyDetect(img,blankPoint);
	waitKey(0);
	return 0;
}
*/