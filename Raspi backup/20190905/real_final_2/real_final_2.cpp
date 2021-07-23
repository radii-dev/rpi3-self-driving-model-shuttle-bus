/* g++ real_final_2.cpp -o real_final_2 `pkg-config --libs opencv` -lwiringPi */
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

class LineFinder {
private:
    // original image
    cv::Mat img;

    // vector containing the end points
    // of the detected lines
    vector<Vec4i> lines;

    // accumulator resolution parameters
    double deltaRho;
    double deltaTheta;

    // minimum number of votes that a line
    // 선으로 간주되기 전에 받아야 하는 투표 최소 개수 (threshold)
    int minVote;

    // 선의 최소 길이
    double minLength;

    // 선을 따라가는 최대 허용 간격(gap)
    double maxGap;
public:
    // Default accumulator resolution is 1 pixel by 1 degree
    // no gap, no mimimum length
    LineFinder() : deltaRho(1), deltaTheta(CV_PI/180), minVote(10), minLength(0.), maxGap(0.) {}

    // Set the minimum number of votes
    void setMinVote(int minv) {
        minVote = minv;
    }

    // Set line length and gap
    void setLineLengthAndGap(double length, double gap) {
        minLength = length;
        maxGap = gap;
    }

    // Apply probabilistic Hough Transform
    vector<Vec4i> findLines(cv::Mat& binary) {
        lines.clear();
        HoughLinesP(binary, lines, deltaRho, deltaTheta, minVote, minLength, maxGap);
        return lines;
    }

    // Draw the detected lines on an image
    void drawDetectedLines(cv::Mat &image, Scalar color = Scalar(0, 0, 255)) {
        // Draw the lines
        vector<Vec4i>::const_iterator it2 = lines.begin();
        while (it2 != lines.end()) {
            if((*it2)[0] != (*it2)[2]) {
                Point pt1((*it2)[0], (*it2)[1]);
                Point pt2((*it2)[2], (*it2)[3]);
                line(image, pt1, pt2, color, 2);
            }
            ++it2;
        }
    }
    
    //일관성 없는 방향 라인 제거
    vector<Vec4i> removeLinesOfInconsistentOrientations(const cv::Mat &orientations, double percentage, double delta) {
        vector<Vec4i>::iterator it = lines.begin();
        // check all lines
        while (it != lines.end()) {
            // end points
            int x1= (*it)[0];
            int y1= (*it)[1];
            int x2= (*it)[2];
            int y2= (*it)[3];
    
            // line orientation + 90o to get the parallel line
            double ori1 = atan2(static_cast<double>(y1 - y2), static_cast<double>(x1 - x2)) + CV_PI / 2; // double 형으로 변환하는 함수를 호출!
            if (ori1 > CV_PI) ori1 = ori1 - 2 * CV_PI;

            double ori2 = atan2(static_cast<double>(y2 - y1), static_cast<double>(x2 - x1)) + CV_PI / 2;
            if (ori2 > CV_PI) ori2 = ori2 - 2 * CV_PI;

            // for all points on the line
            LineIterator lit(orientations, Point(x1, y1), Point(x2, y2));
            int i, count = 0;
            for(i = 0, count = 0; i < lit.count; i++, ++lit) {
                float ori = *(reinterpret_cast<float *>(*lit)); // lineiterate의 값들을 float형으로 인식해라!

                // is line orientation similar to gradient orientation ?
                if (min(fabs(ori - ori1), fabs(ori - ori2)) < delta) // fabs 부동소수점 숫자의 절대값 반환
                    count++;
            }   
            double consistency = count / static_cast<double>(i); // 일관성

            // set to zero lines of inconsistent orientation
            if (consistency < percentage) {
                (*it)[0] = (*it)[1] = (*it)[2] = (*it)[3] = 0;
            }
            ++it;
        }
        return lines;
    }
};
    
class EdgeDetector {
private:
    // original image
    cv::Mat img;

    // 16-bit signed int image
    cv::Mat sobel;

    // Aperture size of the Sobel kernel
    int aperture; // 조리개??

    // Sobel magnitude
    cv::Mat sobelMagnitude;

    // Sobel orientation
    cv::Mat sobelOrientation;
public:
    EdgeDetector() : aperture(3) {}

    // Compute the Sobel
    void computeSobel(const cv::Mat& image) {
        cv::Mat sobelX;
        cv::Mat sobelY;

        // Compute Sobel
        Sobel(image, sobelX, CV_32F, 1, 0, aperture);
        Sobel(image, sobelY, CV_32F, 0, 1, aperture);

        // Compute magnitude and orientation
        cartToPolar(sobelX, sobelY, sobelMagnitude, sobelOrientation);
    }

    // Get Sobel orientation
    cv::Mat getOrientation() {
        return sobelOrientation;
    }
};

int main()
{
    // to calculate fps
    clock_t begin, end;

    // read from video file on disk
    // to read from webcam initialize as: cap = VideoCapture(int device_id);
    VideoCapture cap(0);
    cv::Mat source;

	int vector_real = 0;
	double vectorm = 0.0;
	char buffer;
	int times = 0;
	int vetor_real_of_real = 3000;
    
    while(cap.isOpened()) { // check if camera/ video stream is available
        cap >> source;
        cv::resize(source, source, cv::Size(480, 320));
        if(!cap.grab())
            continue;

        // to calculate fps
        begin = clock();

        buffer = 0;

        cv::Mat ROI(source, Rect(0, 240, 480, 80));
        //cv::Mat ROI(source, Rect(0, 160, 480, 160));
        cv::Mat tmp = ROI;
        medianBlur(tmp, tmp, 5);
        
        // Compute Sobel
        EdgeDetector ed;
        ed.computeSobel(tmp);

        // Apply Canny algorithm
        cv::Mat contours;
        Canny(tmp, contours, 150, 200); // 125, 200

        cv::namedWindow("contours");
        imshow("contours", contours);

        // Create LineFinder instance
        LineFinder ld;
        ld.setLineLengthAndGap(30, 20); // 50, 20
        //ld.setLineLengthAndGap(80, 20); // 50, 20
        ld.setMinVote(60); // 60
        
        // Detect lines
        vector<Vec4i> li = ld.findLines(contours);

        // remove inconsistent lines
        // ld.removeLinesOfInconsistentOrientations(ed.getOrientation(), 0.4, 0.1); // 0.4  0.1
                
         ld.drawDetectedLines(tmp);
        
        vector<double> temp_2;
        vector< vector<double> > lpoints;
        vector< vector<double> > rpoints;
        for (int i = 0; i < li.size(); i++) {
            for (int j = 0; j < 4; j++) {
                if(j == 1 || j == 3) li[i][j] += 240;
				temp_2.push_back(li[i][j]);
			}
            if (li[i][1] > li[i][3] && li[i][0] != li[i][2]) lpoints.push_back(temp_2);
            if (li[i][1] < li[i][3] && li[i][0] != li[i][2]) rpoints.push_back(temp_2);
			temp_2.clear();
        }
        for (int i = 0; i < lpoints.size(); i++) {
            cout << "lpoints: ";
            for (int j = 0; j < 4; j++) cout << lpoints[i][j] << " ";
            cout << endl;
        }
        for (int i = 0; i < rpoints.size(); i++) {
            cout << "rpoints: ";
            for (int j = 0; j < 4; j++) cout << rpoints[i][j] << " ";
            cout << endl;
        }
        
        if (lpoints.empty() && rpoints.size() > 0) {
			line(tmp, Point(rpoints[0][0], rpoints[0][1]), Point(rpoints[0][2], rpoints[0][3]), Scalar(255, 0, 0), 1, 8, 0);
			vectorm = (double)((rpoints[0][0] - rpoints[0][2]) / (rpoints[0][3] - rpoints[0][1]));
            cout << vectorm << endl;
            cout << "mode 1" << endl;
        }
        else if (rpoints.empty() && lpoints.size() > 0) {
            if(lpoints.size() == 1) {                
                line(tmp, Point(lpoints[0][0], lpoints[0][1]), Point(lpoints[0][2], lpoints[0][3]), Scalar(255, 0, 0), 1, 8, 0);
                vectorm = (double)((lpoints[0][2] - lpoints[0][0]) / (lpoints[0][1] - lpoints[0][3]));
            }
            else {
                line(tmp, Point(lpoints[1][0], lpoints[1][1]), Point(lpoints[1][2], lpoints[1][3]), Scalar(255, 0, 0), 1, 8, 0);
                vectorm = (double)((lpoints[1][2] - lpoints[1][0]) / (lpoints[1][1] - lpoints[1][3]));
            }
            cout << "mode 2" << endl;
        }
        else if (lpoints.empty() && rpoints.empty()) {
			line(tmp, Point(160, 240), Point(160, 200), Scalar(255, 0, 0), 1, 8, 0);
            cout << "mode 3" << endl;
        }
        else if (rpoints.size() > 0 && lpoints.size() > 0) {
            if(lpoints.size() == 1) {                
                line(tmp, Point(lpoints[0][0], lpoints[0][1]), Point(lpoints[0][2], lpoints[0][3]), Scalar(255, 0, 0), 1, 8, 0);
			    line(tmp, Point(rpoints[0][0], rpoints[0][1]), Point(rpoints[0][2], rpoints[0][3]), Scalar(255, 0, 0), 1, 8, 0);
			    line(tmp, Point(rpoints[0][0], rpoints[0][1]) + Point(lpoints[0][2], lpoints[0][3]) / 2, Point(rpoints[0][2], rpoints[0][3]) + Point(lpoints[0][0], lpoints[0][1]) / 2, Scalar(255, 0, 0), 1, 8, 0);
			    vectorm = (float)(lpoints[0][2] - lpoints[0][0] + rpoints[0][0] - rpoints[0][2]) / (lpoints[0][1] - lpoints[0][3] + rpoints[0][3] - rpoints[0][1]);
            }
            else {
                line(tmp, Point(lpoints[1][0], lpoints[1][1]), Point(lpoints[1][2], lpoints[1][3]), Scalar(255, 0, 0), 1, 8, 0);
			    line(tmp, Point(rpoints[0][0], rpoints[0][1]), Point(rpoints[0][2], rpoints[0][3]), Scalar(255, 0, 0), 1, 8, 0);
			    line(tmp, Point(rpoints[0][0], rpoints[0][1]) + Point(lpoints[1][2], lpoints[1][3]) / 2, Point(rpoints[0][2], rpoints[0][3]) + Point(lpoints[1][0], lpoints[1][1]) / 2, Scalar(255, 0, 0), 1, 8, 0);                
			    vectorm = (float)(lpoints[1][2] - lpoints[1][0] + rpoints[0][0] - rpoints[0][2]) / (lpoints[1][1] - lpoints[1][3] + rpoints[0][3] - rpoints[0][1]);
            }
        }
		line(tmp, Point(160, 240), Point(160, 225), Scalar(255, 0, 0), 1, 8, 0);
		line(tmp, Point(0, 175), Point(320, 175), Scalar(255, 0, 0), 1, 8, 0);
		line(tmp, Point(0, 200), Point(320, 200), Scalar(255, 0, 0), 1, 8, 0);
        circle(tmp, Point(240, 240), 15, cv::Scalar(0, 0, 255), 2);

		if(vectorm != 0)	vetor_real_of_real = int(3000 - vectorm * 680);

		if(vetor_real_of_real > 3900)		vector_real = 3900;
		else if(vetor_real_of_real < 2100)	vector_real = 2100;
		else								vector_real = vetor_real_of_real;

		//imshow("Video", gray);
		buffer += vector_real / 100;
		//buffer += 39;
		if(times == 3)
		{
			times = 0;
			cout << (int)buffer << endl;
			uart_ch(buffer);
		}
		else times++;
        source.copyTo(tmp);

        imshow("edge", tmp);

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
