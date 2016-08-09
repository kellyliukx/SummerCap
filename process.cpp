# include"process.h"

Detect::Detect():markerSize(50,50)
{
	m_markerCorners2d.push_back(cv::Point2f(0,0));
	m_markerCorners2d.push_back(cv::Point2f(markerSize.width-1,0));
	m_markerCorners2d.push_back(cv::Point2f(markerSize.width-1,markerSize.height-1));
	m_markerCorners2d.push_back(cv::Point2f(0,markerSize.height-1));
}

void Detect::iscube(std::vector<cv::Point> points,int &flag)
{
	Point2f point0,point1,point2;
	float ratio;
	float large=0,small=0;
	point0 = points[0];
	point1 = points[1];
	point2 = points[2];
	int size1 = pow(abs(point1.y - point0.y),2)+pow(abs(point1.x - point0.x),2);
	int size2 = pow(abs(point2.y - point1.y),2)+pow(abs(point2.x - point1.x),2);
	if(size1 > size2)
	{
		large = size1;
		small = size2;
	}
	else
		large = size2;
	small = size1;
	ratio = large/small;
	if (ratio < 1.5)
	{
		flag = 1;
	}
}
void Detect::findContours(const cv::Mat& thresholdImg, std::vector<std::vector<cv::Point> >& contours,int type ,int minContour)
{
	cv::Mat thresholdImgCopy;
	thresholdImg.copyTo(thresholdImgCopy);

	std::vector<std::vector<cv::Point> > allContours;
	cv::findContours(thresholdImgCopy, allContours, type, CV_CHAIN_APPROX_NONE);

	contours.clear();
	for (size_t i=0;i<allContours.size();i++)
	{
		int contourSize = allContours[i].size();
		if (contourSize > minContour)
		{
			contours.push_back(allContours[i]);
		}
	}
	//画出所有连通域
	drawContours(imgforshow,contours,-1,Scalar(0,0,255),3);
	imshow("contour",imgforshow);

}
void Detect::plotLine(Mat &img,vector<Point>&points )
{

	for (int j =0;j<points.size()-1;j++)
	{
		line(img,points[j],points[j+1],Scalar(255,0,0),2,8);
	}
	line(img,points[0],points[points.size()-1],Scalar(255,0,0),2,8);
	//imshow("imgline",img);
}
void Detect::BlankDetect(Mat&img,vector<Point>&blankPoint)
{
	img.copyTo(imgforshow);
	Mat imgGray;
	Mat m_thresholdImg,s_threshold;
	vector<vector<Point>> m_contours;
	Mat imgHSV ;
	cvtColor(img,imgGray,CV_RGB2GRAY);
	cvtColor(img,imgHSV,CV_RGB2HSV);
	Mat equalImg;
	adaptiveThreshold(imgGray,s_threshold,255,CV_ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY_INV,7,7);
	//连通域
	findContours(s_threshold, m_contours,CV_RETR_EXTERNAL ,imgGray.cols/5);
	vector<Point>approxCurve;
	vector<vector<Point>>choosePoint;
	int cubeflag=0;
	//vector<vector<Point>> possibleMarkers;
	for (size_t i=0;i<m_contours.size();i++)
	{
		// Approximate to a poligon
		cv::approxPolyDP(Mat(m_contours[i]), approxCurve, double(m_contours[i].size())*0.07 , true);
		// We interested only in polygons that contains only four vertices
		int counSize = contourArea(m_contours[i]);
		if (counSize<3000)
		{
			continue;
		}
		if (approxCurve.size() !=4)//必须是四个顶点
			continue;
		// And they have to be convex必须是凸的
		if (!cv::isContourConvex(approxCurve))
			continue;
		cv::Point v1 = approxCurve[1] - approxCurve[0];
		cv::Point v2 = approxCurve[2] - approxCurve[0];
		double o = (v1.x * v2.y) - (v1.y * v2.x);
		if (o  < 0.0)         //if the third point is in the left side, then sort in anti-clockwise order
		{
			std::swap(approxCurve[1],approxCurve[3]);
		}
		iscube(approxCurve,cubeflag);
		if(!cubeflag)
			continue;
		cubeflag = 0;
		//看看能不能变换哈
		/*
		vector<Point2f>possiblecube;
		for (size_t j=0;j<4;j++)
		{
			possiblecube.push_back(Point2f(approxCurve[j].x,approxCurve[j].y));
		}
		Mat canonicalMarkerImage;
		cv::Mat markerTransform = cv::getPerspectiveTransform(possiblecube, m_markerCorners2d); 
		cv::warpPerspective(imgHSV, canonicalMarkerImage,  markerTransform, markerSize);  
		//imshow("canonicalMarkerImage",canonicalMarkerImage);
	 	Mat img_threshold_white;
		//检测mask内的像素分布
		Mat white_low(Scalar(0,0,180));
		Mat white_higher(Scalar(70,70,255));
		uchar pixel =0;
		int pixels=0;
        inRange(canonicalMarkerImage,white_low,white_higher,img_threshold_white);
		for (size_t a=0;a<img_threshold_white.rows;a++)
		{
			for (size_t b=0;b<img_threshold_white.cols;b++)
			{
				pixel = img_threshold_white.at<uchar>(a,b);
				if (pixel==1)
				{
					pixels++;
				}		
			}

		}
		float ratioROI = pixels/(img_threshold_white.rows*img_threshold_white.cols);
		if (ratioROI>0.15)
			continue;
			*/
		choosePoint.push_back(approxCurve);
		//plotLine(imgforshow,approxCurve);
	}
	if (choosePoint.data())
	{
		int maxConture=0;
		int contoutIndex =0;
		blankPoint.clear();
		for (size_t i =0;i<choosePoint.size();i++)
		{
			int contoutAreaa = contourArea(choosePoint[i]);
			if(maxConture<contoutAreaa)
			{
				maxConture = contoutAreaa;
				contoutIndex = i;
			}
		}
		blankPoint = choosePoint[contoutIndex];
		plotLine(imgforshow,blankPoint);
	}
	//plotLine(imgforshow,blankPoint);
	imshow("cube",imgforshow);
	//imshow("threshold2",s_threshold);
}
void Detect::ToyDetect(Mat&img,vector<Point>&toyPoint)
{
	
	vector<Point>blankPoint;
	//先检测篮筐，在篮筐的区域内检测娃娃
	BlankDetect(img,blankPoint);
	if (blankPoint.size()==1)
	{
		// Set ROI
		Mat mask(img.rows,img.cols,CV_8UC3,Scalar::all(0));
		drawContours(mask,blankPoint,0,Scalar::all(255),-1);
		Mat imgROI;
		img.copyTo(imgROI,mask);
		//imshow("imgroi",imgROI);
		//在感兴趣区域内部检测
		//通过颜色检测确定娃娃区域
		// convert to HSV
		Mat img_hsv_blue, img_hsv_red1,img_hsv_red2,img_hsv_yellow;
		cvtColor(imgROI, img_hsv_blue, CV_BGR2HSV);
		Mat img_threshold_blue, img_threshold_red, img_threshold_red1, img_threshold_red2, img_threshold_yellow;

		Mat element = getStructuringElement(MORPH_RECT,Size(5,5));
		morphologyEx(img_hsv_blue,img_hsv_blue ,MORPH_OPEN,element,Point(-1,-1),3);
		morphologyEx(img_hsv_blue,img_hsv_blue ,MORPH_CLOSE,element,Point(-1,-1),3);

		img_hsv_red1   = img_hsv_blue.clone();
		img_hsv_red2   = img_hsv_blue.clone();
		img_hsv_yellow = img_hsv_blue.clone();

		Mat blue_low(Scalar(60,43,46));
		Mat blue_higher(Scalar(140,255,255));

		Mat red1_low(Scalar(0,43,46));
		Mat red1_higher(Scalar(3,255,255));

		Mat red2_low(Scalar(170,43,46));
		Mat red2_higher(Scalar(180,255,255));

		Mat yellow_low(Scalar(20,43,46));
		Mat yellow_higher(Scalar(34,255,255));

		inRange(img_hsv_blue,   blue_low,   blue_higher,   img_threshold_blue);
		inRange(img_hsv_red1,   red1_low,   red1_higher,   img_threshold_red1);
		inRange(img_hsv_red2,   red2_low,   red2_higher,   img_threshold_red2);
		inRange(img_hsv_yellow, yellow_low, yellow_higher, img_threshold_yellow);
		img_threshold_red = img_threshold_red1 | img_threshold_red2;

		//imshow("123",img_threshold_red1);

		vector<vector<Point>> contours_blue, contours_red, contours_yellow;
		vector<Vec4i> hierarchy_blue, hierarchy_red, hierarchy_yellow;
		cv::findContours(img_threshold_blue,   contours_blue,hierarchy_blue, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
		cv::findContours(img_threshold_red,    contours_red,hierarchy_red,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
		cv::findContours(img_threshold_yellow, contours_yellow,hierarchy_yellow, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
		// Find the max area of contours
		for (int i=0;i<contours_blue.size();i++){
			int contourSize = contourArea(contours_blue[i]);
			if(contourSize>6000){
				Rect box = boundingRect(contours_blue[i]);
				rectangle(imgforshow,box,Scalar(255,0,0),5);
			}
		}

		for (int i=0;i<contours_red.size();i++){
			int contourSize = contourArea(contours_red[i]);
			if(contourSize>10000){
				Rect box = boundingRect(contours_red[i]);
				rectangle(imgforshow,box,Scalar(0,0,255),5);
			}
		}

		for (int i=0;i<contours_yellow.size();i++){
			int contourSize = contourArea(contours_yellow[i]);
			if(contourSize>10000){
				Rect box = boundingRect(contours_yellow[i]);
				rectangle(imgforshow,box,Scalar(0,255,255),5);
			}
		}
		imshow("imgforshow",imgforshow);
	}

}
void Detect::selectContour(vector<vector<Point>> &contours,vector<Rect>&selectRoi,Mat &imgGray)
{
	Rect boundRect;
	Mat roiImg;
	for (int i = 0;i<contours.size();i++)
	{
		boundRect = boundingRect(Mat(contours[i]));
		//	boundRect = boundRect+Size(10,10);
		//提取外接矩形图像
		roiImg = imgGray(boundRect);
		//计算外接矩形的像素分布
		int countnum = 0;
		//cout<<roiImg<<endl;
		for (int i= 0;i< roiImg.rows;i++)
		{
			for (int j=0;j<roiImg.cols;j++)
			{
				if (roiImg.at<uchar>(i,j)>180 )//像素值满足条件
				{
					countnum ++;
				}

			}
		}
		float minlen,maxlen,bili;
		maxlen=max(boundRect.height,boundRect.width);
		minlen=min(boundRect.height,boundRect.width);
		bili = maxlen/minlen;
		if(countnum > roiImg.rows*roiImg.cols/5)//满足条件
		{
			if (bili<1.5)
			{
				selectRoi.push_back(boundRect);
			}
		}
	}
}
void Detect::plotRect(vector<Rect>&rects,Mat &img)
{
	for (int i=0;i<rects.size();i++)
	{
		rectangle(img,rects[i].tl(),rects[i].br(),(0,0,255),2,8,0);
	}
	imshow("rects",img);
}
void Detect::cube9Detect(Mat&img,int color,vector<vector<Point>> &cube)
{
	Mat imgHSV3D[3],img3D[3];
	Mat imgresult,imgGauss;
	img.copyTo(imgforshow);
	cvtColor(img,imgGray,CV_RGB2GRAY);
	cvtColor(img,imgHSV,CV_RGB2HSV);
	split(imgHSV,imgHSV3D);
	Mat threshold,medianImg;
	vector<vector<Point>> m_contours;
	cv::threshold(imgGray,threshold,240,255,THRESH_BINARY);
	//getCube(imgHSV,m_thresholdImg,3, cube);

		//vector<Marker> 
	//findContours(threshold, m_contours, threshold.cols/5);
	vector<Vec4i> lines;//定义一个矢量结构lines用于存放得到的线段矢量集合
	HoughLinesP(threshold, lines, 1, CV_PI/90, 80, 10, 50 );
	Mat imgLines = Mat::zeros(threshold.rows,threshold.cols,threshold.type());
	for( size_t i = 0; i < lines.size(); i++ )
	{
		Vec4i l = lines[i];
		//依次在图中绘制出每条线段
		line( imgLines, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 2,
			CV_AA);
	}
	//腐蚀一下
	Mat element = getStructuringElement(MORPH_ELLIPSE,Size(25,25));
	Mat out;
	dilate(imgLines,out,element);

	imshow("imgLines",imgLines);
	//imshow("element",out);
	findContours(out, m_contours,CV_RETR_LIST, threshold.cols/5);
	vector <cv::Point>  approxCurve;
	vector<vector<Point>> possibleMarkers;
	int maxsize=0,minsize=0;
	int  cubeflag = 0;
	// For each contour, analyze if it is a paralelepiped likely to be the marker
	for (size_t i=0; i<m_contours.size(); i++)
	{
		int contourSize = contourArea(m_contours[i]);
		if (contourSize > imgGray.rows*imgGray.cols/3)
		{
			continue;
		}
		// Approximate to a poligon
		cv::approxPolyDP(Mat(m_contours[i]), approxCurve, double(m_contours[i].size())*0.07 , true);
		// We interested only in polygons that contains only four vertices
		if (approxCurve.size() !=4)//必须是四个顶点
			continue;
		// And they have to be convex必须是凸的
		if (!cv::isContourConvex(approxCurve))
			continue;
		iscube(approxCurve,cubeflag);
		//if(!cubeflag)
		//	continue;
		cubeflag = 0;	
		possibleMarkers.push_back(approxCurve);
	}
	
	//保证四个点事逆时针顺序
	vector<Point>colorPoints;
	//vector<Mat>cubeImg;//截取的图片部分
	for (size_t i=0; i<possibleMarkers.size(); i++)
	{
		//trace a line between the first and second point.
		//if the thrid point is at the right side, then the points are anti-clockwise
		cv::Point v1 = possibleMarkers[i][1] - possibleMarkers[i][0];
		cv::Point v2 = possibleMarkers[i][2] - possibleMarkers[i][0];

		double o = (v1.x * v2.y) - (v1.y * v2.x);
		if (o  < 0.0)         //if the third point is in the left side, then sort in anti-clockwise order
		{
			std::swap(possibleMarkers[i][1],possibleMarkers[i][3]);
		}
		//看看能不能变换哈
		vector<Point2f>possiblecube;
		for (size_t j=0;j<4;j++)
		{
			possiblecube.push_back(Point2f(possibleMarkers[i][j].x,possibleMarkers[i][j].y));
		}
		 Mat canonicalMarkerImage;
		 cv::Mat markerTransform = cv::getPerspectiveTransform(possiblecube, m_markerCorners2d); 
		 cv::warpPerspective(imgHSV, canonicalMarkerImage,  markerTransform, markerSize);  
		 //获取颜色值红色：1，黄色：2，蓝色：3
		 int nowColor = getImgcolor(canonicalMarkerImage);
		 if (nowColor==color)
		 {
			 int roi_x = (possibleMarkers[i][0].x+possibleMarkers[i][1].x+possibleMarkers[i][2].x+possibleMarkers[i][3].x)/4;
			 int roi_y = (possibleMarkers[i][0].y+possibleMarkers[i][1].y+possibleMarkers[i][2].y+possibleMarkers[i][3].y)/4;
			 cube.push_back(possibleMarkers[i]);
			 colorPoints.push_back(Point(roi_x,roi_y));
		 }
	} 
	for (int i=0;i<colorPoints.size();i++)
	{
		circle(imgforshow,colorPoints[i],3,Scalar(0,0,255),2,8,0);
	}
	// drawContours(imgforshow,colorPoints,3,Scalar(0,0,255),3);
	 drawContours(imgforshow,possibleMarkers,-1,Scalar(0,0,255),3);
	//提取出contour区域
	 
	//标记颜色
	imshow("contour",imgforshow);
	imshow("threshold",threshold);
}
int  Detect::getImgcolor(Mat&img)
{
	int count_r=0,count_b=0,count_y=0;
	int pixel_h=0,pixel_s=0;
	int color = 0;
	for (int i=0;i<img.rows;i++)
	{
		for (int j=0;j<img.cols;j++)
		{
			pixel_h = img.at<cv::Vec3b>(i,j)[0];
			pixel_s = img.at<cv::Vec3b>(i,j)[1];
			if (pixel_h>=thresh_r[0][0]&&pixel_h<=thresh_r[0][1])
			{
				if (pixel_s>=thresh_r[1][0]&&pixel_s<=thresh_r[1][1]){
					count_r++;
				}
			}
			if (pixel_h>=thresh_b[0][0]&&pixel_h<=thresh_b[0][1])
			{
				//if (pixel_s>=thresh_b[1][0]&&pixel_s<=thresh_b[1][1]){
				count_b++;
				//}
			}
			if (pixel_h>=thresh_y[0][0]&&pixel_h<=thresh_y[0][1])
			{
				if (pixel_s>=thresh_y[1][0]&&pixel_s<=thresh_y[1][1]){
					count_y++;
				}
			}
		}
	}
	int max_count = max(max(count_r,count_y),count_b);
	if (max_count ==count_r)
	{
		return 1;
	}
	else if (max_count ==count_b)
	{
		return 2;
	}
	else if (max_count ==count_y)
	{
		return 3;
	}
	else
		return 0;
}
void Detect::ledCubeDetect(Mat&img,vector<vector<Point>>&cubePoint)
{
	img.copyTo(imgforshow);
	Mat imgGray;
	Mat m_thresholdImg,s_threshold;
	vector<vector<Point>> m_contours;
	Mat imgHSV ;
	cvtColor(img,imgGray,CV_RGB2GRAY);
	Mat equalImg;
	adaptiveThreshold(imgGray,s_threshold,255,CV_ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY_INV,7,7);
	//连通域
	findContours(s_threshold, m_contours,CV_RETR_LIST ,imgGray.cols/5);
	vector<Point>approxCurve;
	int cubeflag=0;
	//vector<vector<Point>> possibleMarkers;
	for (size_t i=0;i<m_contours.size();i++)
	{
		// Approximate to a poligon
		cv::approxPolyDP(Mat(m_contours[i]), approxCurve, double(m_contours[i].size())*0.07 , true);
		// We interested only in polygons that contains only four vertices
		int counSize = contourArea(m_contours[i]);
		if (counSize<3000)
		{
			continue;
		}
		if (approxCurve.size() !=4)//必须是四个顶点
			continue;
		// And they have to be convex必须是凸的
		if (!cv::isContourConvex(approxCurve))
			continue;
		cv::Point v1 = approxCurve[1] - approxCurve[0];
		cv::Point v2 = approxCurve[2] - approxCurve[0];
		double o = (v1.x * v2.y) - (v1.y * v2.x);
		if (o  < 0.0)         //if the third point is in the left side, then sort in anti-clockwise order
		{
			std::swap(approxCurve[1],approxCurve[3]);
		}
		iscube(approxCurve,cubeflag);
		if(!cubeflag)
			continue;
		cubeflag = 0;
		cubePoint.push_back(approxCurve);
		plotLine(imgforshow,approxCurve);
	}
	imshow("cube",imgforshow);
	//imshow("threshold2",s_threshold);
}
void Detect::ledDetect(Mat&img,vector<int> &results,vector<Point2f>&toyCenters)
{
	Mat imgHSV3D[3],img3D[3];
	Mat imgresult,imgGauss,imgMedian;
	Mat threshold_y,threshold_r,threshold_b;
	img.copyTo(imgforshow);
	cvtColor(img,imgGray,CV_RGB2GRAY);
	cvtColor(img,imgHSV,CV_RGB2HSV);
	split(imgHSV,imgHSV3D);
	Mat m_thresholdImg;
	vector<vector<Point>> m_contours;
	Point cube;
	//processImgH(imgGray,imgresult);
	//提取感兴趣区域
	//
	threshold(imgGray,m_thresholdImg,230,255,THRESH_BINARY);
    Mat MimgHMasked = Mat::zeros(img.rows,img.cols,img.type());
	img.copyTo(MimgHMasked,m_thresholdImg);
	//***中值滤波相当有用
	medianBlur(MimgHMasked,imgMedian,5);
	Mat imgGrayS ,s_threshold;
	//对提取的图像二值化
	cvtColor(imgMedian,imgGrayS,CV_RGB2GRAY);
	imshow("MimgHMasked",imgMedian);
	adaptiveThreshold(imgGrayS,s_threshold,255,CV_ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY_INV,17,7);
	findContours(s_threshold, m_contours,CV_RETR_EXTERNAL ,imgGray.cols/4);
	vector<Rect>toyRects; 
	//vector<Point2f>toyCenters;
	for (size_t i=0;i<m_contours.size();i++)
	{
		//计算面积
		int contourSize = contourArea(m_contours[i]);
		if (contourSize<3000)
		{
			continue;
		}
		 //****************************************************************//
		 //寻找最小外接圆
		Point2f toycenter;
		float toyradius =0;
		minEnclosingCircle(Mat(m_contours[i]),toycenter,toyradius);
		//计算连通域面积占外接圆面积的比例
		float ratioToy = (float)contourSize/(3.14*pow(toyradius,2));
		int result =0;
		if (ratioToy<0.6&&ratioToy>0)
		{
			result = 1;
			circle(imgforshow,toycenter,3,(0,0,255),3,8,0);
		}
		if(ratioToy>=0.6)
		{
			result = 2;
			circle(imgforshow,toycenter,3,(255,0,0),3,8,0);
		}
		toyCenters.push_back(toycenter);
		results.push_back(result);
		result = 0;
	}
	imshow("s_threshold",s_threshold);
	imshow("img",imgforshow);
	//plotline(img,cube9);
}