/* g++ hsvpick.cpp -o hsvpick `pkg-config --libs opencv` */

#include <opencv2/core.hpp>

#include <opencv2/highgui.hpp>

#include <iostream>

using namespace std;
using namespace cv;

void onMouse(int event, int x, int y, int flags, void* param); 


int main()

{
    VideoCapture cap(0);

	Mat image;

    while(cap.isOpened()) {

        cap >> image;


        imshow("hsvpick", image); //Jecheon이라는 창에 이미지 출력


        setMouseCallback("hsvpick", onMouse, reinterpret_cast<void*>(&image)); // 특정 창에 마우스 핸들러 설정
        
        int k = cv::waitKey(1);
        if (k == 27)
            break;

    }


    return 0;

}


void onMouse(int event, int x, int y, int flags, void* param)

{

	Mat *im = reinterpret_cast<Mat*>(param);

    float fR = 0, fG = 0, fB = 0;

	switch (event) // 이벤트 전달

	{

	case CV_EVENT_LBUTTONDOWN: // 마우스 좌클릭시 이벤트

        fB = static_cast<int>(im->at<Vec3b>(Point(x, y))[0]);

        fG = static_cast<int>(im->at<Vec3b>(Point(x, y))[1]);

        fR = static_cast<int>(im->at<Vec3b>(Point(x, y))[2]);

		cout << "(" << x << ", " << y << "): " << "BGR : " << fB << "\t\t" << fG << "\t\t" << fR << endl; // (x, y)에 있는 화소값 출력

        cout << endl;

		break;

	}

}