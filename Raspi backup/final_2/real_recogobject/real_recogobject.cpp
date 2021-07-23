/* g++ real_recogobject.cpp -o real_recogobject `pkg-config --libs opencv` -lwiringPi */
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

Scalar red(48, 48, 192);
Scalar green(48, 104, 40); //(144, 64, 24)
Scalar white(255, 255, 255);

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

class detectColor{
public:
	Mat source;
	int range_count = 0;
//	Scalar rgb = Scalar(0, 0, 0);
	Mat rgb_color;
	Mat hsv_color;
	int hue;
	int saturation;
	int value;
	int low_hue;
	int high_hue;
	int low_hue1 = 0, low_hue2 = 0;
	int high_hue1 = 0, high_hue2 = 0;

	Mat img_hsv;
	Mat img_mask1, img_mask2;
	Mat img_labels, stats, centroids;

	int numOfLables;

	string name;
	string dis;
	double distance = 0;

	int left, top, width, height;

	detectColor() {}

	void rgbtohsv(Scalar rgb) {
		rgb_color = Mat(1, 1, CV_8UC3, rgb);
		cvtColor(rgb_color, hsv_color, COLOR_BGR2HSV);
	}

	void setScale() {
		if (low_hue < 10 ) {
			range_count = 2;
			high_hue1 = 180;
			low_hue1 = low_hue + 180;
			high_hue2 = high_hue;
			low_hue2 = 0;
		}
		else if (high_hue > 170) {
			range_count = 2;
			high_hue1 = low_hue;
			low_hue1 = 180;
			high_hue2 = high_hue - 180;
			low_hue2 = 0;
		}
		else {
			range_count = 1;
			low_hue1 = low_hue;
			high_hue1 = high_hue;
		}
		cout << low_hue1 << "  " << high_hue1 << endl;
		cout << low_hue2 << "  " << high_hue2 << endl;
	}

	void binarization() {
		//HSV로 변환
		cvtColor(source, img_hsv, COLOR_BGR2HSV);
		
		//지정한 HSV 범위를 이용하여 영상을 이진화
		inRange(img_hsv, Scalar(low_hue1, 50, 50), Scalar(high_hue1, 255, 255), img_mask1);
		if (range_count == 2) {
			inRange(img_hsv, Scalar(low_hue2, 50, 50), Scalar(high_hue2, 255, 255), img_mask2);
			img_mask1 |= img_mask2;
		}
	}

	void calib() {
		//morphological opening 작은 점들을 제거 
		erode(img_mask1, img_mask1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(img_mask1, img_mask1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		//morphological closing 영역의 구멍 메우기 
		dilate(img_mask1, img_mask1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(img_mask1, img_mask1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	}

	void labeling() {
		//라벨링 
		numOfLables = connectedComponentsWithStats(img_mask1, img_labels, stats, centroids, 8, CV_32S);

		//영역박스 그리기
		int max = -1, idx = 0;
		for (int j = 1; j < numOfLables; j++) {
			int area = stats.at<int>(j, CC_STAT_AREA);
			if (max < area)
			{
				max = area;
				idx = j;
			}
		}

		left = stats.at<int>(idx, CC_STAT_LEFT);
		top = stats.at<int>(idx, CC_STAT_TOP);
		width = stats.at<int>(idx, CC_STAT_WIDTH);
		height = stats.at<int>(idx, CC_STAT_HEIGHT);
		
		if((width * height > 30000) || (width * height < 1000)) {
			left = 0;
			top = 0;
			width = 0;
			height = 0;
		}

        cout << name << "\t" << left << "\t" << top << "\t" << width << "\t" << height << "\t" << width * height << endl;

        distance = (0.0397 * 0.4) / (width * 0.00007);//0.35 is avg. width of the   Chevron Arrow sign
        stringstream stream;
        stream << fixed << setprecision(2) << distance;
        dis = stream.str() + "m"; 
        cout << distance << endl;
	}
};

void drawlocation(Mat img_frame, detectColor Busstop, detectColor Rain);

int main()
{
	detectColor Busstop;
	detectColor Rain;

	// Busstop set
	Busstop.name = "Busstop";
	Busstop.rgbtohsv(red);
	Busstop.hue = (int)Busstop.hsv_color.at<Vec3b>(0, 0)[0];
	Busstop.saturation = (int)Busstop.hsv_color.at<Vec3b>(0, 0)[1];
	Busstop.value = (int)Busstop.hsv_color.at<Vec3b>(0, 0)[2];
	Busstop.low_hue = Busstop.hue - 10;
	Busstop.high_hue = Busstop.hue + 10;
	cout << "Busstop.hue = " << Busstop.hue << endl;
	cout << "Busstop.saturation = " << Busstop.saturation << endl;
	cout << "Busstop.value = " << Busstop.value << endl;
	Busstop.setScale();

	// Rain set
	Rain.name = "Rain";
	Rain.rgbtohsv(green);
	Rain.hue = (int)Rain.hsv_color.at<Vec3b>(0, 0)[0];
	Rain.saturation = (int)Rain.hsv_color.at<Vec3b>(0, 0)[1]; 
	Rain.value = (int)Rain.hsv_color.at<Vec3b>(0, 0)[2];
	Rain.low_hue = Rain.hue - 15;
	Rain.high_hue = Rain.hue + 15;
	cout << "Rain.hue = " << Rain.hue << endl;
	cout << "Rain.saturation = " << Rain.saturation << endl;
	cout << "Rain.value = " << Rain.value << endl;
	Rain.setScale();

	VideoCapture cap(0);
	Mat img_frame;
	char buffer;

	if (!cap.isOpened()) {
		cerr << "ERROR! Unable to open camera\n";
		return -1;
	}
	
	double fps = 0;
    
    // Number of frames to capture
    int num_frames = 60;
    int started_frames = 0;
    
    // Start and end times
    time_t start, end;
    
    cout << "Capturing " << num_frames << " frames" << endl ;
    
    // Start time
    time(&start);

	while (1)
	{
		started_frames++;
        
        if (started_frames == num_frames) {
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

		// initialize
		buffer = 0b00000000;

		// wait for a new frame from camera and store it into 'frame'
		cap.read(img_frame);
		Busstop.source = img_frame.clone();
		Rain.source = img_frame.clone();

		// check if we succeeded
		if (img_frame.empty()) {
			cerr << "ERROR! blank frame grabbed\n";
			break;
		}

		// Busstop draw location
		Busstop.binarization();
		Busstop.calib();
		// Busstop.drawlocation(red);
		Busstop.labeling();

		// Rain draw location
		Rain.binarization();
		Rain.calib();
		// Rain.drawlocation(green);
		Rain.labeling();
/*
		addWeighted(img_frame, 0.5, Busstop.source, 0.5, 0, img_frame);
		addWeighted(img_frame, 0.5, Rain.source, 0.5, 0, img_frame);
		imshow("detect", img_frame);
*/

		drawlocation(img_frame, Busstop, Rain);

		if(Busstop.width * Busstop.height > 7800) {
			buffer += 0b01100000;
			cout << "Busstop!" << endl;
		}		
		if(Rain.width * Rain.height > 3300) {
			buffer += 0b00000110;
			cout << "Rain?" << endl;
		}
		uart_ch(buffer);

		if (waitKey(5) >= 0)
			break;
	}
	return 0;
}

void drawlocation(Mat img_frame, detectColor Busstop, detectColor Rain) {
	rectangle(img_frame, Point(Busstop.left - 5, Busstop.top - 5), Point(Busstop.left + Busstop.width + 5, Busstop.top + Busstop.height + 5), red, 2);
	putText(img_frame, Busstop.name, Point(Busstop.left + 1, Busstop.top + 3), FONT_HERSHEY_DUPLEX, 0.3, white, 1);
	putText(img_frame, Busstop.dis, Point(Busstop.left, Busstop.top + Busstop.height + 2), FONT_HERSHEY_DUPLEX, 0.3, white, 1);

	rectangle(img_frame, Point(Rain.left - 5, Rain.top - 5), Point(Rain.left + Rain.width + 5, Rain.top + Rain.height + 5), green, 2);
	putText(img_frame, Rain.name, Point(Rain.left + 1, Rain.top + 3), FONT_HERSHEY_DUPLEX, 0.3, white, 1);
	putText(img_frame, Rain.dis, Point(Rain.left, Rain.top + Rain.height + 2), FONT_HERSHEY_DUPLEX, 0.3, white, 1);
	
	imshow("detect", img_frame);
	// imshow("source", source);
}