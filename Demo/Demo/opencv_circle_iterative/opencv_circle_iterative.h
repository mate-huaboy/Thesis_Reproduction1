#pragma once
#ifndef OPENCV_CIRCLE_ITERATIVE_INCLUDED
#define OPENCV_CIRCLE_ITERATIVE_INCLUDED

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <fstream>
#include <vector>

using namespace cv;
using namespace std;


struct SingleCalData {
	SingleCalData() {}

	SingleCalData(std::string img_cata) : _img_cata(img_cata) {}

	SingleCalData(std::string img_cata, int board_flag, int cornerRows, int cornerCols, double real_interval_size) :
		_boardSize(cv::Size(cornerCols, cornerRows)),
		_img_cata(img_cata),
		_real_interval_size(real_interval_size),
		_board_flag(board_flag) {
	}

	//标定前	
	int _board_flag;              //标定类型，0--圆标定板，1--棋盘格标定板
	cv::Size _boardSize;
	std::string _img_cata;		//保存标定结果的图像文件=====by-wenhua
	double _real_interval_size;   //角点实际间隔距离 ,单位mm 

	//标定后
	cv::Size _imageSize;        //图片大小，标定时设置
	cv::Mat _cameraMatrix;      //内参
	cv::Mat _distortMatrix;     //畸变参数向量 

	//畸变矫正后
	cv::Mat mapx, mapy;
};

class opencv_circle_iterative {
private:
	Size image_size, board_size;
	Mat src[30], gray_src[30];//最大三十张图片
	vector<vector <Point2f> > image_points_seq;
	int src_count = 0;
	vector <Mat> tvecsMat, rvecsMat;
	Mat cameraMatrix, distCoeffs;
	Mat unproject_src[30];
	Mat H[30];
	double total_err = 0, err = 0;
	vector<Point2f> srcTri[30], disTri[30];
public:
	void set_mat(Size bs);
	void _FindCorner(bool flag);
	void calibration();
	void init();
	void iterative(int num);
	void unproject();
	void reproject();
	vector<Point2f> cornersort(vector<Point2f> tmp, vector<Point2f> res);
	void unproject_origin();
	void set_mat(Size bs, string file_name);
	bool writeCalibFile(SingleCalData data,string foldername);
};

#endif // !OPENCV_ITERATIVE_INCLUDED
