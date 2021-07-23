/* g++ recogobject.cpp -o recogobject `pkg-config --libs opencv` -lwiringPi */
#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"
#include <iostream>
#include <vector>
#include <string>
#include "opencv2/videoio.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/opencv_modules.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <math.h>
#include <iomanip>
using namespace cv;
using namespace std;

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

void draw_locations(Mat & img, vector< Rect > & locations, const Scalar & color, string text);

#define CASCADE_FILE_BUS "/home/pi/dev/Capstone2019/recogobject/Bus4.xml"
#define CASCADE_FILE_RAIN "/home/pi/dev/Capstone2019/recogobject/Rain2(35-3019).xml"

#define WINDOW_NAME "MATCHING"

int main()
{
    cout << CV_VERSION <<endl;
    VideoCapture cap(0);
    Mat mFrame, mGray, imageROI, mGray1, mGray2;
    CascadeClassifier bus, rain;
    vector<Rect> bus_found, rain_found;

    bus.load(CASCADE_FILE_BUS);
    rain.load(CASCADE_FILE_RAIN);
    
    double fps = 0;
    
    // Number of frames to capture
    int num_frames = 60;
    int started_frames = 0;
    
    // Start and end times
    time_t start, end;
    
    cout << "Capturing " << num_frames << " frames" << endl ;
    
    // Start time
    time(&start);


   while (cap.read(mFrame))
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
        
        // Apply the classifier to the frame
        imageROI = mFrame(Rect(0, mFrame.rows/2, mFrame.cols, mFrame.rows/2));

        cvtColor(imageROI, mGray, COLOR_BGR2GRAY);
        cvtColor(mFrame, mGray2, COLOR_BGR2GRAY);
        
        //imshow("before", mGray);
        mGray.copyTo(mGray1);

        //sign cascade
        bus.detectMultiScale(mGray2, bus_found, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));
        draw_locations(mFrame, bus_found, Scalar(0, 0, 255),"BusStop");
        
        rain.detectMultiScale(mGray2, rain_found, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));
        draw_locations(mFrame, rain_found, Scalar(255, 0, 0),"Rain?");
        
        imshow(WINDOW_NAME, mFrame);

        // hit 'esc' to exit program
        int k = cv::waitKey(1);
        if (k == 27)
            break;
   }

   return 0;
}

void draw_locations(Mat& img, vector<Rect>& locations, const Scalar& color, string text)
{
    Mat img1, roi1, BusStop, BusMask, Rain, RainMask;

    img.copyTo(img1);
    string dis;

   if (!locations.empty())
   {

        double distance= 0;
        
        for(int i = 0; i < locations.size(); ++i){
            if((text == "BusStop")){
                distance = (0.0397 * 0.4) / ((locations[i].width) * 0.00007);//0.35 is avg. width of the   Chevron Arrow sign
            }
            if((text == "Rain?")){
                distance = (0.0397 * 0.4) / ((locations[i].width) * 0.00007);//0.35 is avg. width of the   Chevron Arrow sign
            }
            stringstream stream;
            stream << fixed << setprecision(2) << distance;
            dis = stream.str() + "m"; 
        }
        
        for( int i = 0 ; i < locations.size() ; ++i){
        
            rectangle(img, locations[i], color, 1.8);
            
            putText(img, text, Point(locations[i].x+1, locations[i].y+8), FONT_HERSHEY_DUPLEX, 0.3, color, 1);
            putText(img, dis, Point(locations[i].x, locations[i].y+locations[i].height-5), FONT_HERSHEY_DUPLEX, 0.3, Scalar(255, 255, 255), 1);
        }
        
   }
}