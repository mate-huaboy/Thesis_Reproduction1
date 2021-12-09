#include "opencv_circle_iterative.h"
#include"read_file.h"
void opencv_circle_iterative::set_mat(Size bs,string file_name) {
	this->src_count = 0;
	this->board_size = bs;
	this->cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
	this->distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));

	//ifstream fin("./calibdata2.txt");
	std::vector<string> files; vector<string> ownname;
	getFiles(file_name, files, ownname);
	for (int i = 0; i < files.size(); i++)
	{
		this->src[++src_count] = imread(files[i]);
		if (src_count == 1) {
			this->image_size.width = this->src[src_count].cols;
			this->image_size.height = this->src[src_count].rows;
		}
	}
	/*while (getline(fin, filename)) {
		this->src[++src_count] = imread(filename);
		if (src_count == 1) {
			this->image_size.width = this->src[src_count].cols;
			this->image_size.height = this->src[src_count].rows;
		}
	}*/
	cout << "src_count " << " " << src_count << endl;
}

void opencv_circle_iterative::init() {//将圆图片转成gray图片
	int ln = this->src_count;
	for (int i = 1; i <= ln; i++) {
		this->src[i].copyTo(this->gray_src[i]);
		cvtColor(this->src[i], this->gray_src[i], COLOR_BGR2GRAY);
	}
}

void opencv_circle_iterative::_FindCorner(bool flag = true) {
	this->image_points_seq.clear();
	int ln = this->src_count;
	cout << "ln: " << ln << endl;
	for (int i = 1; i <= ln; i++) {
		cout << "i: " << i << endl;
		cout << this->board_size << endl;
		vector <Point2f> image_points_buf;
		//cout << board_size.width << " " << board_size.height << endl;
		//imshow("Camera Calibration", this->gray_src[i]);
		//waitKey(0);

		cv::SimpleBlobDetector::Params params;//参数设置可能需要更改
		//params.maxArea = 10000;
		//params.minArea = 70;
		//params.minConvexity = 0.85;
		//params.minThreshold = 80;
		//params.maxThreshold = 230;
		//params.thresholdStep = 20;
		//params.minInertiaRatio = 0.05;


		//=============
		params.filterByArea = true;
		params.minArea = 25;
		params.maxArea = 12000;
		params.thresholdStep = 2;
		params.minThreshold = 5;
		params.maxThreshold = 220;
		Ptr<FeatureDetector> blobDetector = SimpleBlobDetector::create(params);

		auto detector = cv::SimpleBlobDetector::create(params);

		//image_points_buf.clear();
		if (!flag) {
			if (findCirclesGrid(this->gray_src[i], this->board_size, image_points_buf, CALIB_CB_SYMMETRIC_GRID, blobDetector) == 0) {
				cout << "Can't find circleboard corners!\n";
				/*this->image_points_seq.push_back(image_points_buf);
				cout << "true" << image_points_buf.size() << endl;*/

			}else {
				Mat view_gray = this->gray_src[i];
				find4QuadCornerSubpix(view_gray, image_points_buf, this->board_size);
				this->image_points_seq.push_back(image_points_buf);

				drawChessboardCorners(view_gray, this->board_size, image_points_buf, true);
				/*cout << "false" << image_points_buf.size() << endl;*/
				/*imshow("Camera Calibration", view_gray);
				waitKey(0);*/
			}
		}
		else {
			if (findCirclesGrid(this->gray_src[i], this->board_size, image_points_buf, cv::CALIB_CB_SYMMETRIC_GRID, detector) == 0) {
				cout << "Can't find circleboard corners!\n";
			}else {
				Mat view_gray = this->gray_src[i];
				find4QuadCornerSubpix(view_gray, image_points_buf, this->board_size);
				this->image_points_seq.push_back(image_points_buf);

				drawChessboardCorners(view_gray, this->board_size, image_points_buf, true);

				//imshow("Camera Calibration", view_gray);
				//waitKey(0);
			}
		}
	}
}

void opencv_circle_iterative::calibration() {
	Size square_size = Size(7, 7);//中心距
	vector <vector <Point3f> >object_points;
	vector <int> point_counts;
	//for (int t = 1; t <= this->src_count; t++) {//当有图片检测不出时这里有bug
	for (int t = 1; t <= this->image_points_seq.size(); t++) {
		vector <Point3f> tmpPointSet;
		for (int i = 0; i < this->board_size.height; i++) {
			for (int j = 0; j < this->board_size.width; j++) {
				Point3f realpoint;
				realpoint.x = i * square_size.width;
				realpoint.y = j * square_size.height;
				realpoint.z = 0;
				tmpPointSet.push_back(realpoint);
			}
		}
		object_points.push_back(tmpPointSet);
	}

	//计算单应性矩阵
	//for (int i = 1; i <= this->src_count; i++) {
	//	this->H[i] = findHomography(object_points[i-1], this->image_points_seq[i-1]);
	//	cout << format(H[i], Formatter::FMT_NUMPY) << endl;
	//}

	for (int t = 1; t <= src_count; t++) {
		int width = board_size.width, height = board_size.height;
		cout << "width: " << width << "  " << " height: " << height << endl;
		point_counts.push_back(width * height);
	}
	this->rvecsMat.clear(); this->rvecsMat.clear();
	calibrateCamera(object_points, this->image_points_seq, this->image_size, this->cameraMatrix, this->distCoeffs, this->rvecsMat, this->tvecsMat, 0);
	cout << "camera calibration finished !" << endl;

	double total_err = 0, err = 0;
	vector <Point2f> image_points2; // 保存重新计算得到的投影点
	for (int i = 1; i <= this->src_count; i++) {
		vector<Point3f> tmpPointSet = object_points[i - 1];

		projectPoints(tmpPointSet, this->rvecsMat[i - 1], this->tvecsMat[i - 1], this->cameraMatrix, this->distCoeffs, image_points2);

		vector <Point2f> tmpImagePoint = image_points_seq[i - 1];
		Mat tmpImagePointMat = Mat(1, tmpImagePoint.size(), CV_32FC2);
		Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
		int ln = (int)tmpImagePoint.size();
		for (int j = 0; j < ln; j++) {
			image_points2Mat.at<Vec2f>(0, j) = Vec2f(image_points2[j].x, image_points2[j].y);
			tmpImagePointMat.at<Vec2f>(0, j) = Vec2f(tmpImagePoint[j].x, tmpImagePoint[j].y);
		}
		err = norm(image_points2Mat, tmpImagePointMat, NORM_L2);
		err /= point_counts[i - 1];
		total_err += err;
		cout << "No." << i << " picture's average error: " << err << " pixel!" << endl;
		//fout << "No." << i << " picture's average error: " << err << " pixel!";
	}
	cout << "Total average error: " << total_err / this->src_count << endl;
}

void opencv_circle_iterative::iterative(int num) {
	int tmp_num = num;
	init();
	_FindCorner(false);
	calibration();
	for (int t = 1; t <= num; t++) {//矫正
		init();
		cout << "This is the " << t << " time(s) !" << endl;
		Mat mapx = Mat(this->image_size, CV_32FC1);
		Mat mapy = Mat(this->image_size, CV_32FC1);
		Mat R = Mat::eye(3, 3, CV_32F);
		for (int i = 1; i <= this->src_count; i++) {
			initUndistortRectifyMap(this->cameraMatrix, this->distCoeffs, R, this->cameraMatrix, this->image_size, CV_32FC1, mapx, mapy);
			Mat imageSrc = this->src[i];

			//if (i == 1) {
			//	imshow("newsrc", this->src[i]);
			//	imshow("gray", this->gray_src[i]);
			//	waitKey(0);
			//}

			Mat newSrc = imageSrc.clone();
			remap(imageSrc, newSrc, mapx, mapy, INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0));
			this->src[i] = newSrc;
			cvtColor(this->src[i], this->gray_src[i], COLOR_BGR2GRAY);

			if (i == 1) {
				/*imshow("newsrc", this->src[i]);
				imshow("gray", this->gray_src[i]);
				waitKey(0);*/
			}

			//imshow("imagestc", imageSrc);
		}
		unproject();//得正视图
		_FindCorner();//在正视图中找角点
		reproject();//角点重投影回原始视图
		calibration();//标定
	}
}


void opencv_circle_iterative::unproject_origin() {//没改之前的

	//总体思路：得到标定板的4个角，然后在投影得到正视图
	int ln = (int)this->src_count;
	for (int i = 1; i <= ln; i++) {
		cout << "i: " << i << "   ";

		srcTri[i - 1].clear();
		disTri[i - 1].clear();

		// 后续换底板时需要修改！！！！
		// 此处应该将srcTri四个点换成背景板中白色的四个角点
		Mat graypic, binpic, cannypic;
		this->gray_src[i].copyTo(graypic);
		this->gray_src[i].copyTo(binpic);
		this->gray_src[i].copyTo(cannypic);
		medianBlur(graypic, graypic, 7);
		threshold(graypic, binpic, 80, 255, THRESH_BINARY);
		Canny(binpic, cannypic, 200, 2.5);

		/*	imshow("unproject", binpic);
			waitKey(0);*/

		vector <vector<Point> > contours; // 用于存储轮廓
		vector <Vec4i> hierarchy;

		findContours(cannypic, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);

		Mat linepic = Mat::zeros(cannypic.rows, cannypic.cols, CV_8UC3);
		for (int j = 0; j < contours.size(); j++)
			drawContours(linepic, contours, j, Scalar(rand() & 255, rand() & 255, rand() & 255), 1, 8);

		/*	imshow("unproject", linepic);
			waitKey(0);*/

		vector <vector<Point> > polyContours(contours.size());
		int pmax = 0;
		for (int j = 0; j < contours.size(); j++) {
			if (contourArea(contours[j]) > contourArea(contours[pmax])) pmax = j;
			approxPolyDP(contours[j], polyContours[j], 10, true);
		}
		Mat polypic = Mat::zeros(this->gray_src[i].size(), CV_8UC3);
		drawContours(polypic, polyContours, pmax, Scalar(0, 0, 255), 2);


		/*imshow("unproject", polypic);
		waitKey(0);*/

		vector <int> hull;
		convexHull(polyContours[pmax], hull, false);//计算凸包

		vector <Point2f> quadangle, tmp_quadangle;

		for (int j = 0; j < hull.size(); ++j) {
			circle(polypic, polyContours[pmax][j], 10, Scalar(rand() & 255, rand() & 255, rand() & 255), 3);
			//cout << polyContours[pmax][i].x << " " << polyContours[pmax][i].y << endl;
			tmp_quadangle.push_back(polyContours[pmax][j]);
		}
		Mat showpic;
		this->src[i].copyTo(showpic);
		addWeighted(polypic, 0.5, showpic, 0.5, 0, showpic);




		cout << "11111" << endl;
		cout << "ln: " << image_points_seq.size() << endl;
		cout << this->board_size.width << " " << this->board_size.height << endl;
		cout << "polyC ln: " << hull.size() << endl;
		quadangle.push_back(image_points_seq[i - 1][0]);
		quadangle.push_back(image_points_seq[i - 1][this->board_size.width - 1]);
		quadangle.push_back(image_points_seq[i - 1][this->board_size.width * (this->board_size.height - 1)]);
		quadangle.push_back(image_points_seq[i - 1][this->board_size.height * this->board_size.width - 1]);

		cout << "2222" << endl;
		srcTri[i - 1] = cornersort(tmp_quadangle, quadangle);
		cout << "3333" << endl;

		circle(showpic, image_points_seq[i - 1][0], 10, Scalar(255, 0, 0), 3);
		circle(showpic, image_points_seq[i - 1][this->board_size.width - 1], 10, Scalar(0, 0, 255), 3);
		circle(showpic, image_points_seq[i - 1][this->board_size.width * (this->board_size.height - 1)], 10, Scalar(0, 255, 0), 3);
		circle(showpic, image_points_seq[i - 1][this->board_size.height * this->board_size.width - 1], 10, Scalar(255, 0, 255), 3);

		/*	imshow("showPic", showpic);
			waitKey(0);*/

		cout << image_points_seq[i - 1][0] << endl;
		cout << image_points_seq[i - 1][this->board_size.width * (this->board_size.height - 1)] << endl;
		cout << image_points_seq[i - 1][this->board_size.width - 1] << endl;
		cout << image_points_seq[i - 1][this->board_size.height * this->board_size.width - 1] << endl;

		//cout << this->src[i].cols << " " << this->src[i].rows << endl;

		//srcTri[0] = Point2f(this->image_points_seq[i - 1][28].x, this->image_points_seq[i - 1][28].y);
		//srcTri[1] = Point2f(this->image_points_seq[i - 1][34].x, this->image_points_seq[i - 1][34].y);
		//srcTri[2] = Point2f(this->image_points_seq[i - 1][0].x, this->image_points_seq[i - 1][0].y);
		//srcTri[3] = Point2f(this->image_points_seq[i - 1][6].x, this->image_points_seq[i - 1][6].y);

		disTri[i - 1].push_back(Point2f(0, 0));
		disTri[i - 1].push_back(Point2f(this->src[i].cols, 0));
		disTri[i - 1].push_back(Point2f(0, this->src[i].rows));
		disTri[i - 1].push_back(Point2f(this->src[i].cols, this->src[i].rows));//正视图的点？==》有点草率

		cout << "??????" << endl;
		//自己改========================》这里直接利用四个中心点进行变换===》利用外框的方法很不稳定，加之网上很多就是用这种方法，并且原方法本身也是没有比该方法更有说服力

		Mat warpPerspective_mat = getPerspectiveTransform(srcTri[i - 1], disTri[i - 1]);
		warpPerspective(this->src[i], this->unproject_src[i], warpPerspective_mat, this->image_size);
		this->src[i] = this->unproject_src[i];
		cvtColor(this->src[i], this->gray_src[i], COLOR_BGR2GRAY);
		/*imshow("unproject", this->unproject_src[i]);
		waitKey(0);*/
	}
	cout << endl;
}


void opencv_circle_iterative::unproject() {

	//总体思路：得到标定板的4个角，然后在投影得到正视图
	int ln = (int)this->src_count;
	for (int i = 1; i <= ln; i++) {
		cout << "i: " << i << "   ";

		srcTri[i - 1].clear();
		disTri[i - 1].clear();

		// 后续换底板时需要修改！！！！
		// 此处应该将srcTri四个点换成背景板中白色的四个角点
		Mat graypic, binpic, cannypic;
		this->gray_src[i].copyTo(graypic);
		this->gray_src[i].copyTo(binpic);
		this->gray_src[i].copyTo(cannypic);
		medianBlur(graypic, graypic, 7);
		threshold(graypic, binpic, 80, 255, THRESH_BINARY);
		Canny(binpic, cannypic, 200, 2.5);

	/*	imshow("unproject", binpic);
		waitKey(0);*/

	//	vector <vector<Point> > contours; // 用于存储轮廓
	//	vector <Vec4i> hierarchy;

	//	findContours(cannypic, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);

	//	Mat linepic = Mat::zeros(cannypic.rows, cannypic.cols, CV_8UC3);
	//	for (int j = 0; j < contours.size(); j++)
	//		drawContours(linepic, contours, j, Scalar(rand() & 255, rand() & 255, rand() & 255), 1, 8);

	///*	imshow("unproject", linepic);
	//	waitKey(0);*/

	//	vector <vector<Point> > polyContours(contours.size());
	//	int pmax = 0;
	//	for (int j = 0; j < contours.size(); j++) {
	//		if (contourArea(contours[j]) > contourArea(contours[pmax])) pmax = j;
	//		approxPolyDP(contours[j], polyContours[j], 10, true);
	//	}
	//	Mat polypic = Mat::zeros(this->gray_src[i].size(), CV_8UC3);
	//	drawContours(polypic, polyContours, pmax, Scalar(0, 0, 255), 2);


	//	imshow("unproject", polypic);
	//	waitKey(0);

	//	vector <int> hull;
	//	convexHull(polyContours[pmax], hull, false);//计算凸包

	//	vector <Point2f> quadangle, tmp_quadangle;

	//	for (int j = 0; j < hull.size(); ++j) {
	//		circle(polypic, polyContours[pmax][j], 10, Scalar(rand() & 255, rand() & 255, rand() & 255), 3);
	//		//cout << polyContours[pmax][i].x << " " << polyContours[pmax][i].y << endl;
	//		tmp_quadangle.push_back(polyContours[pmax][j]);
	//	}
	//	Mat showpic;
	//	this->src[i].copyTo(showpic);
	//	addWeighted(polypic, 0.5, showpic, 0.5, 0, showpic);




	//	cout << "11111" << endl;
	//	cout << "ln: " << image_points_seq.size() << endl;
	//	cout << this->board_size.width << " " << this->board_size.height << endl;
	//	cout << "polyC ln: " << hull.size() << endl;
		vector <Point2f> quadangle, tmp_quadangle;
		//外扩个像素
		quadangle.push_back(Point2f(image_points_seq[i - 1][0].x, image_points_seq[i - 1][0].y ));
		quadangle.push_back(Point2f(image_points_seq[i - 1][this->board_size.width - 1].x, image_points_seq[i - 1][this->board_size.width - 1].y));
		quadangle.push_back(Point2f(image_points_seq[i - 1][this->board_size.width * (this->board_size.height - 1)].x, image_points_seq[i - 1][this->board_size.width * (this->board_size.height - 1)].y));
		quadangle.push_back(Point2f(image_points_seq[i - 1][this->board_size.height * this->board_size.width - 1].x, image_points_seq[i - 1][this->board_size.height * this->board_size.width - 1].y));

		cout << "2222" << endl;

		srcTri[i - 1] = quadangle;
		cout << "3333" << endl;

		/*circle(showpic, image_points_seq[i - 1][0], 10, Scalar(255, 0, 0), 3);
		circle(showpic, image_points_seq[i - 1][this->board_size.width - 1], 10, Scalar(0, 0, 255), 3);
		circle(showpic, image_points_seq[i - 1][this->board_size.width * (this->board_size.height - 1)], 10, Scalar(0, 255, 0), 3);
		circle(showpic, image_points_seq[i - 1][this->board_size.height * this->board_size.width - 1], 10, Scalar(255, 0, 255), 3);*/

	/*	imshow("showPic", showpic);
		waitKey(0);*/

		cout << image_points_seq[i - 1][0] << endl;
		cout << image_points_seq[i - 1][this->board_size.width * (this->board_size.height - 1)] << endl;
		cout << image_points_seq[i - 1][this->board_size.width - 1] << endl;
		cout << image_points_seq[i - 1][this->board_size.height * this->board_size.width - 1] << endl;

		//cout << this->src[i].cols << " " << this->src[i].rows << endl;

		//srcTri[0] = Point2f(this->image_points_seq[i - 1][28].x, this->image_points_seq[i - 1][28].y);
		//srcTri[1] = Point2f(this->image_points_seq[i - 1][34].x, this->image_points_seq[i - 1][34].y);
		//srcTri[2] = Point2f(this->image_points_seq[i - 1][0].x, this->image_points_seq[i - 1][0].y);
		//srcTri[3] = Point2f(this->image_points_seq[i - 1][6].x, this->image_points_seq[i - 1][6].y);

		disTri[i - 1].push_back(Point2f(this->src[i].cols / 4, this->src[i].rows / 4));
		disTri[i - 1].push_back(Point2f(this->src[i].cols*3/4, this->src[i].rows / 4));
		disTri[i - 1].push_back(Point2f(this->src[i].cols / 4, this->src[i].rows*3/4));
		disTri[i - 1].push_back(Point2f(this->src[i].cols*3/4, this->src[i].rows*3/4));//正视图的点？==》有点草率

		cout << "??????" << endl;
		//自己改========================》这里直接利用四个中心点进行变换===》利用外框的方法很不稳定，加之网上很多就是用这种方法，并且原方法本身也是没有比该方法更有说服力

		Mat warpPerspective_mat = getPerspectiveTransform(srcTri[i - 1], disTri[i - 1]);
		warpPerspective(this->src[i], this->unproject_src[i], warpPerspective_mat, this->image_size);
		this->src[i] = this->unproject_src[i];
		cvtColor(this->src[i], this->gray_src[i], COLOR_BGR2GRAY);
	/*	imshow("unproject", this->unproject_src[i]);
		waitKey(0);*/
	}
	cout << endl;
}

void opencv_circle_iterative::reproject() {
	int ln = (int)this->src_count;
	for (int i = 1; i <= ln; i++) {
		cout << "i: " << i << "   ";
		// 后续换底板时需要修改！！！！

		Mat warpPerspective_mat = getPerspectiveTransform(disTri[i - 1], srcTri[i - 1]);
		warpPerspective(this->src[i], this->unproject_src[i], warpPerspective_mat, this->image_size);

		//imshow("unproject", this->unproject_src[i]);
		//waitKey(0);

		int l = (int)image_points_seq[i - 1].size();
		for (int j = 0; j < l; j++) {
			Mat a = (Mat_<double>(3, 1) << image_points_seq[i - 1][j].x, image_points_seq[i - 1][j].y, 1);
			Mat y = warpPerspective_mat * a;
			image_points_seq[i - 1][j] = Point2f(y.at<double>(0, 0) / y.at<double>(2, 0), y.at<double>(1, 0) / y.at<double>(2, 0));//中心点投回圆图片视图
		}

		//Mat view_gray = this->unproject_src[i];
		//drawChessboardCorners(view_gray, this->board_size, image_points_seq[i-1], true);

		//imshow("view_gray", view_gray);
		//waitKey(0);

	}
	cout << endl;
}

vector<Point2f> opencv_circle_iterative::cornersort(vector<Point2f> tmp, vector<Point2f> res) {//将tmp排序
	vector <Point2f> ans;
	int ln = (int)res.size();
	for (int i = 0; i < ln; i++) {
		int min = 0x3f3f3f3f, p = 0;
		for (int j = 0; j < ln; j++) {
			int dis = (tmp[j].x - res[i].x) * (tmp[j].x - res[i].x) + (tmp[j].y - res[i].y) * (tmp[j].y - res[i].y);//以res[i]为标准，每次选择离res【i】最近的点
			if (dis < min) {
				min = dis; p = j;
			}
		}
		ans.push_back(tmp[p]);
	}
	return ans;
}