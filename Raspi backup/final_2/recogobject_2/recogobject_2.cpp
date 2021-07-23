/* g++ recogobject_2.cpp -o recogobject_2 `pkg-config --libs opencv` -lwiringPi */
#include "opencv2/objdetect.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
 
#include <iostream>
#include <stdio.h>
 
using namespace std;
using namespace cv;
 
#define CAM_WIDTH 480
#define CAM_HEIGHT 320
 
/** Function Headers */
void detectAndDisplay(Mat frame);
 
/** Global variables */
String busstop_cascade_name;
CascadeClassifier busstop_cascade;
String window_name = "Busstop detection";
 
/** @function main */
int main(int argc, const char** argv)
{
    //busstop_cascade_name = "/home/pi/dev/Capstone2019/recogobject_2/Bus5(35-3019).xml";
    busstop_cascade_name = "/home/pi/dev/Capstone2019/recogobject_2/Rain2(35-3019).xml";
    if (!busstop_cascade.load(busstop_cascade_name)) { printf("--(!)Error loading cascade\n"); return -1; };
 
    VideoCapture cam(0); 
    Mat frame;
 
    cam.set(CV_CAP_PROP_FRAME_WIDTH, CAM_WIDTH);
    cam.set(CV_CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT);
 
    if (!cam.isOpened()) { printf("--(!)Error opening video cam\n"); return -1; }
    
    double fps = 0;
    
    // Number of frames to capture
    int num_frames = 60;
    int started_frames = 0;
    
    // Start and end times
    time_t start, end;
    
    cout << "Capturing " << num_frames << " frames" << endl ;
    
    // Start time
    time(&start);
 
    while (cam.read(frame))
    {
        started_frames++;
        
        if (started_frames == num_frames){
        // End Time
        time(&end);
            
        
        // Time elapsed
        double seconds = difftime (end, start);
        cout << "Time taken : " << seconds << " seconds" << endl;
        
        // Calculate frames per second
        fps  = num_frames / seconds;
        
        cout << "fps : " << fps << endl;
        started_frames = 0;
        time(&start);
        }

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
    std::vector<Rect> busstop;
    Mat frame_gray;
 
    cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
    equalizeHist(frame_gray, frame_gray);
 
    busstop_cascade.detectMultiScale(frame_gray, busstop, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));
 
    for (size_t i = 0; i < busstop.size(); i++)
    {
        rectangle(frame, Point(busstop[i].x, busstop[i].y), Point(busstop[i].x + busstop[i].width, busstop[i].y + busstop[i].height), Scalar(0, 0, 255), 2);
    }
 
    imshow(window_name, frame);
}