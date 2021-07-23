/* g++ real_final.cpp -o real_final `pkg-config --libs opencv` -lwiringPi */
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <math.h>
#include <iostream>

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>
void uart_ch(char ch)
{
	int fd ;

	if ((fd = serialOpen ("/dev/ttyAMA0", 9600)) < 0)
	{
		fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno));
		return;
	}
	serialPutchar(fd,ch);
	serialClose(fd);
}
void uart_str(char *str)
{
	while(*str) uart_ch(*str++);
}

/**/

using namespace cv;
using namespace std;

void detectHScolor(const Mat &image, double minHue, double maxHue, double minSat, double maxSat, Mat& mask)
{
	Mat hsv;
	cvtColor(image, hsv, CV_BGR2HSV);

	vector<Mat> channels;
	split(hsv, channels);	// split HSV channel

	Mat mask1;	// Hue mask (0~255)
	threshold(channels[0], mask1, maxHue, 255, THRESH_BINARY_INV);
	Mat mask2;
	threshold(channels[0], mask2, minHue, 255, THRESH_BINARY);
	Mat hueMask;
	if (minHue < maxHue)	hueMask = mask1 & mask2;
	else					hueMask = mask1 | mask2;

	threshold(channels[1], mask1, maxSat, 255, THRESH_BINARY_INV);
	threshold(channels[1], mask2, minSat, 255, THRESH_BINARY);

	Mat satMask;
	satMask = mask1 & mask2;
	mask = hueMask & satMask;
}

int main()
{
	VideoCapture capture(0);
	namedWindow("Video");
	
	/*	변수 선언	*/
	Mat Video;
	Mat mask;
	Mat gray;
	
	/*				*/
	char flag;
	int temp;
	int vector_real = 0;
	double vector = 0.0;
	double degree;
	char buffer;
	int times = 0;
	int vetor_real_of_real = 3000;
		
	while(true)	{
		if(waitKey(1) > 0)	break;
		capture >> Video;
		if (Video.empty()) break;

		/* 변수 초기화 */
		flag = 0;
		degree = 0;
		buffer = 0;
		
		Mat Yellow_detected(Video.size(), CV_8UC3, Scalar(0, 0, 0));
		
		/* 노란색 검출 */
		detectHScolor(Video, 35, 60, 30, 255, mask);	// Hue range (20~70), Saturation (15, 230)
		Video.copyTo(Yellow_detected, mask);			// filtering Video by mask
		
		/* Yellow_detected를 gray scale로 변환 */
		cvtColor(Yellow_detected, gray, CV_BGR2GRAY);
		blur(gray, gray, Size(3, 3));
		
		/* 차선 검출하여 좌표 생성 */

		Point2f R_H;
		temp = 0;
		for (int i = 0; i < 320; i++)
		{
			if (gray.at<uchar>(350, 310 + i) > 150)
			{
				for (int j = 0; j < 10; j++)	temp += gray.at<uchar>(350, 310 + i + j);
				if (temp > 600)
				{
					temp = 0;
					flag += 1;
					R_H = Point(320 + i, 350);
					break;
				}
				temp = 0;
			}
		}

		Point2f R_L;
		for (int i = 0; i < 320; i++)
		{
			if (gray.at<uchar>(400, 310 + i) > 150)
			{
				for (int j = 0; j < 10; j++)	temp += gray.at<uchar>(400, 310 + i + j);
				if (temp > 600)
				{
					temp = 0;
					flag += 2;
					R_L = Point(320 + i, 400);
					break;
				}
				temp = 0;
			}

		}

		Point2f L_H;
		for (int i = 0; i < 320; i++)
		{
			if (gray.at<uchar>(350, 330 - i) > 150)
			{
				for (int j = 0; j < 10; j++)	temp += gray.at<uchar>(350, 330 - i - j);
				if (temp > 600)
				{
					temp = 0;
					flag += 4;
					L_H = Point(320 - i, 350);
					break;
				}
				temp = 0;
			}
		}

		Point2f L_L;
		for (int i = 0; i < 320; i++)
		{
			if (gray.at<uchar>(400, 330 - i) > 150)
			{
				for (int j = 0; j < 10; j++)	temp += gray.at<uchar>(400, 330 - i - j);
				if (temp > 600)
				{
					temp = 0;
					flag += 8;
					L_L = Point(320 - i, 400);
					break;
				}
				temp = 0;
			}
		}
		
		if (flag == 15) {
			line(gray, L_H, L_L, Scalar(255, 0, 0), 1, 8, 0);
			line(gray, R_H, R_L, Scalar(255, 0, 0), 1, 8, 0);
			line(gray, (R_H + L_H) / 2, (R_L + L_L) / 2, Scalar(255, 0, 0), 1, 8, 0);
			vector = (float)(L_H.x - L_L.x + R_H.x - R_L.x) / (L_L.y - L_H.y + R_L.y - R_H.y);
			cout << "mode 1" << endl;
		}
		else if ((flag & 8) && (flag & 4)) {
			line(gray, L_H, L_L, Scalar(255, 0, 0), 1, 8, 0);
			//vector = (double)((L_H.x - L_L.x) / (L_L.y - L_H.y));
			vector = (double)((L_H.x - L_L.x) / (L_L.y - L_H.y)) * (1 - (320 - (float)L_L.x) / 480);
			cout << "mode 2" << endl;
		}
		else if ((flag & 2) && (flag & 1)) {
			line(gray, R_H, R_L, Scalar(255, 0, 0), 1, 8, 0);
			//vector = (double)((R_H.x - R_L.x) / (R_L.y - R_H.y));
			vector = (double)((R_H.x - R_L.x) / (R_L.y - R_H.y)) * (1 - ( (float)R_L.x - 320)/ 480);
			cout << "mode 3" << endl;
		}
		else {
			line(gray, Point(320, 480), Point(320, 400), Scalar(255, 0, 0), 1, 8, 0);
			cout << "mode 4" << endl;
		}

		//line(gray, Point(320, 480), Point(390, 450), Scalar(255, 0, 0), 1, 8, 0);
		//line(gray, Point(250, 480), Point(250, 450), Scalar(255, 0, 0), 1, 8, 0);

		line(gray, Point(320, 480), Point(320, 450), Scalar(255, 0, 0), 1, 8, 0);
		line(gray, Point(0, 350), Point(640, 350), Scalar(255, 0, 0), 1, 8, 0);
		line(gray, Point(0, 400), Point(640, 400), Scalar(255, 0, 0), 1, 8, 0);
		imshow("Video", gray);

		//int c = waitKey(delay);
		//if (c == 27) break;   // Esc 키의 ASCII 값은 27

		if(vector != 0)	vetor_real_of_real = int(3000 - vector * 680);

		if(vetor_real_of_real > 3900)		vector_real = 3900;
		else if(vetor_real_of_real < 2100)	vector_real = 2100;
		else								vector_real = vetor_real_of_real;

		//imshow("Video", gray);
		buffer += vector_real / 100;
		if(times == 3)
		{
			times = 0;
			cout << (int)buffer << endl;
			uart_ch(buffer);
		}
		else times++;
	}
	destroyAllWindows();
	return 0;
}

