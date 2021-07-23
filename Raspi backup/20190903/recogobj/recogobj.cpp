/* g++ recogobj.cpp -o recogobj `pkg-config --libs opencv` */
#include "opencv2/objdetect.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
 
#include <iostream>
#include <stdio.h>
 
using namespace std;
using namespace cv;
 
#define CAM_WIDTH 640
#define CAM_HEIGHT 480
 
/** Function Headers */
void detectAndDisplay(Mat frame);
 
/** Global variables */
String Bus_cascade_name;
CascadeClassifier Bus_cascade;
String window_name = "Bus detection";
 
/** @function main */
int main(int argc, const char** argv)
{
    Bus_cascade_name = "/home/pi/dev/Capstone2019/recogobject/Bus.xml";
    if (!Bus_cascade.load(Bus_cascade_name)) { printf("--(!)Error loading Bus cascade\n"); return -1; };
 
    VideoCapture cam(0); 
    Mat frame;
 
    cam.set(CV_CAP_PROP_FRAME_WIDTH, CAM_WIDTH);
    cam.set(CV_CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT);
 
    if (!cam.isOpened()) { printf("--(!)Error opening video cam\n"); return -1; }
 
    while (cam.read(frame))
    {
        if (frame.empty())
        {
            printf(" --(!) No camd frame -- Break!");
            break;
        }
 
        detectAndDisplay(frame);
        char c = (char)waitKey(10);
        if (c == 27) { break; } 
    }
    return 0;
}
 
/** @function detectAndDisplay */
void detectAndDisplay(Mat frame)
{
    std::vector<Rect> Bus;
    Mat frame_gray;
 
    cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
    equalizeHist(frame_gray, frame_gray);
 
    Bus_cascade.detectMultiScale(frame_gray, Bus, 1.1, 2, 0, Size(30, 30));
 
    for (size_t i = 0; i < Bus.size(); i++)
    {
        rectangle(frame, Point(Bus[i].x, Bus[i].y), Point(Bus[i].x + Bus[i].width, Bus[i].y + Bus[i].height), Scalar(0, 0, 255), 2);
    }
 
    imshow(window_name, frame);
}
