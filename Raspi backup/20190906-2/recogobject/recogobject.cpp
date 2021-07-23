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

#define BUS_SIGN_IMAGE "/home/pi/dev/Capstone2019/recogobject/Bus.png"

#define WINDOW_NAME "MATCHING"

int main()
{
    cout << CV_VERSION <<endl;
   VideoCapture cap(0);
   Mat mFrame, mGray, imageROI,mGray1, mGray2;
   CascadeClassifier left, right, bus;
   vector<Rect> left_found, right_found, bus_found;

//    left.load(CASCADE_FILE_LEFT);
//    right.load(CASCADE_FILE_RIGHT);
    bus.load(CASCADE_FILE_BUS);
    
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
        
        if (started_frames==num_frames){
        // End Time
        time(&end);
            
        
        // Time elapsed
        double seconds = difftime (end, start);
        cout << "Time taken : " << seconds << " seconds" << endl;
        
        // Calculate frames per second
        fps  = num_frames / seconds;
        
        cout <<"fps : "<<fps<<endl;
        started_frames=0;
        time(&start);
        }
        
      // Apply the classifier to the frame
        imageROI = mFrame(Rect(0,mFrame.rows/2,mFrame.cols,mFrame.rows/2));

      cvtColor(imageROI, mGray, COLOR_BGR2GRAY);
        cvtColor(mFrame, mGray2, COLOR_BGR2GRAY);
        
        //imshow("before", mGray);
        mGray.copyTo(mGray1);

        //sign cascade
//        left.detectMultiScale(mGray2, left_found, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));
//        draw_locations(mFrame, left_found, Scalar(255, 0, 0),"Left Arrow");
        
//        right.detectMultiScale(mGray2, right_found, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));
//        draw_locations(mFrame, right_found, Scalar(0, 255, 0),"Right Arrow");

        bus.detectMultiScale(mGray2, bus_found, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));
        draw_locations(mFrame, bus_found, Scalar(0, 0, 255),"Bus Stop");
        
      imshow(WINDOW_NAME, mFrame);

      // hit 'esc' to exit program
        int k = cv::waitKey(1);
        if (k == 27)
            break;
   }

   return 0;
}

void draw_locations(Mat & img, vector < Rect > &locations, const Scalar & color, string text)
{
    Mat img1, roi1, LeftArrow , LeftMask, RightArrow, RightMask, BusStop, BusMask;

    img.copyTo(img1);
    string dis;

   if (!locations.empty())
   {

        double distance= 0;
        
        for( int i = 0 ; i < locations.size() ; ++i){
            if((text=="Bus Stop")){
                BusStop = imread(BUS_SIGN_IMAGE);
                BusMask = BusStop.clone();
                cvtColor(BusMask, BusMask, CV_BGR2GRAY);
                //locations[i].y = locations[i].y + img.rows/2; // shift the bounding box
                Size size(locations[i].width/2, locations[i].height/1.5);
                resize(BusStop,BusStop,size, INTER_NEAREST);
                resize(BusMask,BusMask,size, INTER_NEAREST);
                distance = (0.0397*0.4)/((locations[i].width)*0.00007);//0.35 is avg. width of the   Chevron Arrow sign

                if (locations[i].y-size.height>0){
                    
                    Mat roi1 = img.rowRange(locations[i].y-size.height,(locations[i].y+locations[i].height/1.5)-size.height).colRange(locations[i].x+5, (locations[i].x+5+locations[i].width/2));
                    bitwise_and(BusStop, roi1, BusStop);
                    BusStop.setTo(color, BusMask);
                    add(roi1,BusStop,BusStop);
                    BusStop.copyTo(img1.rowRange(locations[i].y-size.height,(locations[i].y+locations[i].height/1.5)-size.height).colRange(locations[i].x+5 ,(locations[i].x +5+locations[i].width/2 )));
                }
                
            }
            stringstream stream;
            stream << fixed << setprecision(2) << distance;
            dis = stream.str() + "m";
            rectangle(img, locations[i], color, -1);
        }
        addWeighted(img1, 0.8, img, 0.2, 0, img);
        
        for( int i = 0 ; i < locations.size() ; ++i){
        
            rectangle(img, locations[i], color, 1.8);
            
            putText(img, text, Point(locations[i].x+1, locations[i].y+8), FONT_HERSHEY_DUPLEX, 0.3, color, 1);
            putText(img, dis, Point(locations[i].x, locations[i].y+locations[i].height-5), FONT_HERSHEY_DUPLEX, 0.3, Scalar(255, 255, 255), 1);
        }
        
   }
}