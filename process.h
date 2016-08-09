#ifndef PROCESS_H
#define PROCESS_H
#include "cv.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string.h>
#include <stdlib.h>
//命名空间
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
	//找到合适大小的连通域
	void findContours(const cv::Mat& thresholdImg, std::vector<std::vector<cv::Point> >& contours,int type,int minContour);
	//判断是否为正方形
	void iscube(std::vector<cv::Point> points,int &flag);
	//绘制四边形，需要四个点顺序一定
	void plotLine(Mat &img,vector<Point>&points);
	//找到待选眼睛区域
	void Detect::selectContour(vector<vector<Point>> &contours,vector<Rect>&selectRoi,Mat &imgGray);
	//绘制矩形
	void plotRect(vector<Rect>&rects,Mat &img);
	int getImgcolor(Mat&img);//获取九宫格的颜色
public:
	Detect();
	//*****************************抓球区****************************//
	//球框检测，娃娃检测
	void BlankDetect(Mat&img,vector<Point>&blankPoint);
	void ToyDetect(Mat&img,vector<Point>&toyPoint);
	//*****************************九宫格****************************//
	//九宫格检测
	void cube9Detect(Mat&img,int color,vector<vector<Point>> &cube);
	//******************************LED******************************//
	//LED屏识别，篮筐检测
	void ledCubeDetect(Mat&img,vector<vector<Point>>&cubePoint);
	void ledDetect(Mat&img,vector<int> &results,vector<Point2f>&toyCenters);
};

#endif