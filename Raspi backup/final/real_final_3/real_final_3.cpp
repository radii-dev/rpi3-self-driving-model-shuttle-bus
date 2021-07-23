/* g++ -Wall -w -o2 real_final_3.cpp cv.cpp system_management.cpp image_processing.cpp car_control.cpp car_lib.c -o real_final_3 `pkg-config --libs opencv` -lwiringPi */
/*
g++ -c system_management.cpp
g++ -c cv.cpp
g++ -c image_processing.cpp
g++ -c car_control.cpp
g++ -c car_lib.c
g++ -Wall -w -o2 -o real_final_3 real_final_3.cpp system_management.o cv.o image_processing.o car_control.o car_lib.o 'pkg-config --libs opencv' -lwiringPi

// remove object files
rm *.o
*/
#include <iostream>
#include <string>
#include "opencv2/core/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include <fstream>
#include <math.h>
#include <ctime>
#include <bitset>

#include "system_management.h"
#include "image_processing.h"
#include "cv.h"
#include "car_control.h"
#include "car_lib.h"

#include <errno.h>
#include <string.h>
#include <wiringPi.h>
#include <wiringSerial.h>

//add OpenCV namespaces
using namespace cv;
using namespace std;

//	Functions for UART
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

void Mattouint8(Mat src, uint8_t des[VPE_OUTPUT_H][VPE_OUTPUT_W][3]) {
    memcpy(des, src.data, VPE_OUTPUT_IMG_SIZE);
}

void uint8toMat(uint8_t src[VPE_OUTPUT_H][VPE_OUTPUT_W][3], Mat des) {
    memcpy(des.data, src, VPE_OUTPUT_IMG_SIZE);
}
/* // bgrpick
void onMouse(int event, int x, int y, int flags, void* param)
{
	Mat *im = reinterpret_cast<Mat*>(param);
    float fR = 0, fG = 0, fB = 0;

	switch (event) {// 이벤트 전달
	case CV_EVENT_LBUTTONDOWN: // 마우스 좌클릭시 이벤트
        fB = static_cast<int>(im->at<Vec3b>(Point(x, y))[0]);
        fG = static_cast<int>(im->at<Vec3b>(Point(x, y))[1]);
        fR = static_cast<int>(im->at<Vec3b>(Point(x, y))[2]);
		cout << "(" << x << ", " << y << "): " << "BGR : " << fB << "\t\t" << fG << "\t\t" << fR << endl; // (x, y)에 있는 화소값 출력
        cout << endl;
		break;
	}
}
*/
int main()
{
    // to calculate fps
    clock_t begin, end;

    // read from video file on disk
    // to read from webcam initialize as: cap = VideoCapture(int device_id);
    VideoCapture cap(0);
    Mat source;
    uint8_t buffer;
    
    // Class declaration
    Navigator navigator;
    BGR24_to_HSV hsvConverter;
    Draw draw;
    
    colorFilter yellow(YELLOW);

    CVinfo cvResult;

    Driver driver;

    // Matrix for vision
    uint8_t display_buf[VPE_OUTPUT_H][VPE_OUTPUT_W][3];
    uint8_t image_buf[VPE_OUTPUT_H][VPE_OUTPUT_W][3];
    uint8_t yellowImage[VPE_OUTPUT_H][VPE_OUTPUT_W][3];
    Mat result(VPE_OUTPUT_H, VPE_OUTPUT_W, CV_8UC1);
    // Mat result = Mat::ones(Size(VPE_OUTPUT_W, VPE_OUTPUT_H), CV_8UC1);
    // Mat yellowImageresult = Mat::ones(Size(VPE_OUTPUT_W, VPE_OUTPUT_H), CV_8UC1);

    while(cap.isOpened()) { // check if camera/ video stream is available
        cap >> source;
        resize(source, source, Size(VPE_OUTPUT_W, VPE_OUTPUT_H));
        result = source.clone();
        if(!cap.grab())
            continue;

        // to calculate fps
        begin = clock();



        buffer = 75;

        Mattouint8(source, display_buf);
        memcpy(display_buf, display_buf, VPE_OUTPUT_IMG_SIZE);
        hsvConverter.bgr24_to_hsv(display_buf, image_buf);
        yellow.detectColor(image_buf, yellowImage);
        cvResult = navigator.getInfo(display_buf, yellowImage);
        cout << cvResult.direction << endl;
        
        navigator.drawPath(yellowImage, yellowImage);

        draw.vertical_line(yellowImage, 159, (179 - ISROADCLOSE_DISTANCE), 179 );
        //draw.vertical_line(yellowImage, 159, (179 - FOWARD_PATH_EXIST_DISTANCE), 179 );
        draw.horizontal_line(yellowImage, FRONT_UP, 0, 320);
        draw.horizontal_line(yellowImage, FRONT_DOWN, 0, 320);
        draw.horizontal_line(yellowImage, SIDE_UP, 0, 320);
        draw.horizontal_line(yellowImage, SIDE_DOWN, 0, 320);
        draw.horizontal_line(yellowImage, DOWNHILL_CHECK_UP, 20, 300);
        draw.horizontal_line(yellowImage, DOWNHILL_CHECK_DOWN, 20, 300);

        uint8toMat(yellowImage, result);
        // Large Size Display
        // resize( result, result, Size( CAPTURE_IMG_W, CAPTURE_IMG_H ), 0, 0, CV_INTER_NN );
        imshow("lane detection", result);

        // bgrpick
        // imshow("source", source);
        // setMouseCallback("lane detection", onMouse, reinterpret_cast<void*>(&source)); // 특정 창에 마우스 핸들러 설정

        driver.drive(cvResult, buffer);
        //buffer += 0b10000000;
        //buffer += (cvResult.direction / 20);

        cout << bitset<8>(buffer & 0b10000000) << endl;
        cout << (int)(buffer & 0b01111111) << endl;
        uart_ch(buffer);



        // to calculate fps
        end = clock();
        cout << "fps: " << 1 / (double(end - begin) / CLOCKS_PER_SEC) << endl;

        // hit 'esc' to exit program
        int k = cv::waitKey(1);
        if (k == 27)
            break;
    }
    cv::destroyAllWindows();
    return 0;
}
