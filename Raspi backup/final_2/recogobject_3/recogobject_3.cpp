/* g++ recogobject_3.cpp -o recogobject_3 `pkg-config --libs opencv` -lwiringPi */
#include <iostream>
#include <stdio.h>
#include "opencv2/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/core/ocl.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/xfeatures2d.hpp"

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

const int LOOP_NUM = 5;// 10;
const int GOOD_PTS_MAX = 50;
const float GOOD_PORTION = 0.15f;

int64 work_begin = 0;
int64 work_end = 0;

static void workBegin()
{
	work_begin = getTickCount();
}

static void workEnd()
{
	work_end = getTickCount() - work_begin;
}

static double getTime()
{
	return work_end / ((double)getTickFrequency()) * 1000.;
}

struct SURFDetector
{
	Ptr<Feature2D> surf;
	SURFDetector(double hessian = 800.0)
	{
		surf = SURF::create(hessian);
	}
	template<class T>
	void operator()(const T& in, const T& mask, std::vector<cv::KeyPoint>& pts, T& descriptors, bool useProvided = false)
	{
		surf->detectAndCompute(in, mask, pts, descriptors, useProvided);
	}
};

template<class KPMatcher>
struct SURFMatcher
{
	KPMatcher matcher;
	template<class T>
	void match(const T& in1, const T& in2, std::vector<cv::DMatch>& matches)
	{
		matcher.match(in1, in2, matches);
	}
};

static Mat drawGoodMatches(
	const Mat& img1,
	const Mat& img2,
	const std::vector<KeyPoint>& keypoints1,
	const std::vector<KeyPoint>& keypoints2,
	std::vector<DMatch>& matches,
	std::vector<Point2f>& scene_corners_
)
{
	//-- Sort matches and preserve top 10% matches
	std::sort(matches.begin(), matches.end());
	std::vector< DMatch > good_matches;
	double minDist = matches.front().distance;
	double maxDist = matches.back().distance;

	const int ptsPairs = std::min(GOOD_PTS_MAX, (int)(matches.size() * GOOD_PORTION));
	for (int i = 0; i < ptsPairs; i++)
	{
		good_matches.push_back(matches[i]);
	}
	std::cout << "\nMax distance: " << maxDist << std::endl;
	std::cout << "Min distance: " << minDist << std::endl;

	std::cout << "Calculating homography using " << ptsPairs << " point pairs." << std::endl;

	// drawing the results
	Mat img_matches;

	drawMatches(img1, keypoints1, img2, keypoints2,
		good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
		std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	if (ptsPairs == 0) return img_matches;


	//-- Localize the object
	std::vector<Point2f> obj;
	std::vector<Point2f> scene;

	for (size_t i = 0; i < good_matches.size(); i++)
	{
		//-- Get the keypoints from the good matches
		obj.push_back(keypoints1[good_matches[i].queryIdx].pt);
		scene.push_back(keypoints2[good_matches[i].trainIdx].pt);
	}
	//-- Get the corners from the image_1 ( the object to be "detected" )
	std::vector<Point2f> obj_corners(4);
	obj_corners[0] = Point(0, 0);
	obj_corners[1] = Point(img1.cols, 0);
	obj_corners[2] = Point(img1.cols, img1.rows);
	obj_corners[3] = Point(0, img1.rows);
	std::vector<Point2f> scene_corners(4);

	Mat H = findHomography(obj, scene, RANSAC);

	if (H.empty()) return img_matches;

	perspectiveTransform(obj_corners, scene_corners, H);

	//if (scene_corners.empty()) return img_matches;

	scene_corners_ = scene_corners;

	//-- Draw lines between the corners (the mapped object in the scene - image_2 )
	line(img_matches,
		scene_corners[0] + Point2f((float)img1.cols, 0), scene_corners[1] + Point2f((float)img1.cols, 0),
		Scalar(0, 255, 0), 2, LINE_AA);
	line(img_matches,
		scene_corners[1] + Point2f((float)img1.cols, 0), scene_corners[2] + Point2f((float)img1.cols, 0),
		Scalar(0, 255, 0), 2, LINE_AA);
	line(img_matches,
		scene_corners[2] + Point2f((float)img1.cols, 0), scene_corners[3] + Point2f((float)img1.cols, 0),
		Scalar(0, 255, 0), 2, LINE_AA);
	line(img_matches,
		scene_corners[3] + Point2f((float)img1.cols, 0), scene_corners[0] + Point2f((float)img1.cols, 0),
		Scalar(0, 255, 0), 2, LINE_AA);
	return img_matches;
}

////////////////////////////////////////////////////
// This program demonstrates the usage of SURF_OCL.
// use cpu findHomography interface to calculate the transformation matrix
int main(int argc, char* argv[])
{
	Mat img_input, img_target;
	UMat img_input_gray, img_target_gray;


	double surf_time = 0.;

	//declare input/output
	std::vector<KeyPoint> keypoints1, keypoints2;
	std::vector<DMatch> matches;

	UMat _descriptors1, _descriptors2;
	Mat descriptors1 = _descriptors1.getMat(ACCESS_RW),
		descriptors2 = _descriptors2.getMat(ACCESS_RW);

	//instantiate detectors/matchers
	SURFDetector surf;
	SURFMatcher<BFMatcher> matcher;

	// OCL 활성화
	ocl::setUseOpenCL(true);

	//비디오 캡쳐 초기화
	VideoCapture cap(0);
	if (!cap.isOpened()) {
		cerr << "에러 - 카메라를 열 수 없습니다.\n";
		return -1;
	}

	cap.read(img_input);
	if (img_input.empty()) {
		cerr << "빈 영상이 캡쳐되었습니다.\n";
		return -1;
	}

	int width = img_input.cols;
	int height = img_input.rows;

	int step = 0;


	while (1)
	{
		// 카메라로부터 캡쳐한 영상을 frame에 저장합니다.
		cap.read(img_input);
		if (img_input.empty()) {
			cerr << "빈 영상이 캡쳐되었습니다.\n";
			break;
		}

		if (step == 1) {
			cvtColor(img_input, img_input_gray, COLOR_BGR2GRAY);

			//-- start of timing section

			for (int i = 0; i <= LOOP_NUM; i++)
			{
				if (i == 1) workBegin();
				surf(img_target_gray.getMat(ACCESS_READ), Mat(), keypoints1, descriptors1);
				surf(img_input_gray.getMat(ACCESS_READ), Mat(), keypoints2, descriptors2);
				matcher.match(descriptors1, descriptors2, matches);
			}
			workEnd();
			std::cout << "FOUND " << keypoints1.size() << " keypoints on first image" << std::endl;
			std::cout << "FOUND " << keypoints2.size() << " keypoints on second image" << std::endl;

			surf_time = getTime();
			std::cout << "SURF run time: " << surf_time / LOOP_NUM << " ms" << std::endl << "\n";


			std::vector<Point2f> corner;
			
			Mat img_matches = drawGoodMatches(img_target_gray.getMat(ACCESS_READ), img_input_gray.getMat(ACCESS_READ), keypoints1, keypoints2, matches, corner);

			//-- Show detected matches
			namedWindow("surf matches", 0);
			imshow("surf matches", img_matches);
		}



		// ESC 키를 입력하면 루프가 종료됩니다. 
		int key = waitKey(25);
		if (key == 27)
			break;
		else if (key == 32) {
			if (step == 0) {
				img_input(Rect(width / 2 - 150, height / 2 - 150, 300, 300)).copyTo(img_target);
				cvtColor(img_target, img_target_gray, COLOR_BGR2GRAY);

				imshow("target", img_target);
				step++;
			}
		}

		if (step == 0)
		{
			rectangle(img_input, Rect(width / 2 - 150, height / 2 - 150, 300, 300), Scalar(0, 255, 0), 3);
		}


		// 영상을 화면에 보여줍니다. 
		imshow("Scene", img_input);
	}


	return 0;
}