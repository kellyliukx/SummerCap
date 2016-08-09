#ifndef PROCESS_H
#define PROCESS_H
#include "cv.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string.h>
#include <stdlib.h>
//�����ռ�
using namespace cv;
using namespace std;

const int thresh_r[][2] = {{120,150},{95,180}};
const int thresh_b[][2] = {{5,15},{125,160}};
const int thresh_y[][2] = {{70,115},{50,95}};

class Detect{
private:
	Mat imgforshow;
	Mat imgGray;
	Mat imgHSV;
	std::vector<cv::Point2f> m_markerCorners2d;
	cv::Size markerSize;
	//�ҵ����ʴ�С����ͨ��
	void findContours(const cv::Mat& thresholdImg, std::vector<std::vector<cv::Point> >& contours,int type,int minContour);
	//�ж��Ƿ�Ϊ������
	void iscube(std::vector<cv::Point> points,int &flag);
	//�����ı��Σ���Ҫ�ĸ���˳��һ��
	void plotLine(Mat &img,vector<Point>&points);
	//�ҵ���ѡ�۾�����
	void Detect::selectContour(vector<vector<Point>> &contours,vector<Rect>&selectRoi,Mat &imgGray);
	//���ƾ���
	void plotRect(vector<Rect>&rects,Mat &img);
	int getImgcolor(Mat&img);//��ȡ�Ź������ɫ
public:
	Detect();
	//*****************************ץ����****************************//
	//����⣬���޼��
	void BlankDetect(Mat&img,vector<Point>&blankPoint);
	void ToyDetect(Mat&img,vector<Point>&toyPoint);
	//*****************************�Ź���****************************//
	//�Ź�����
	void cube9Detect(Mat&img,int color,vector<vector<Point>> &cube);
	//******************************LED******************************//
	//LED��ʶ��������
	void ledCubeDetect(Mat&img,vector<vector<Point>>&cubePoint);
	void ledDetect(Mat&img,vector<int> &results,vector<Point2f>&toyCenters);
};

#endif