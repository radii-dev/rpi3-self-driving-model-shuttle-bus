/* g++ final_6.cpp -o final_6 `pkg-config --libs opencv` -larmadillo -lwiringPi */
#include <iostream>
#include <armadillo>
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

//add OpenCV and Armadillo namespaces
using namespace cv;
using namespace std;
using namespace arma;

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

// int times = 0;

class vanishingPt {
	public:
	cv::Mat image;
	cv::Mat frame;
	vector< vector<int> > points;
	mat A, b, prevRes;
	mat Atemp, btemp, res, aug, error, soln;
	// store minimum length for lines to be considered while estimating vanishing point
    // define minimum length requirement for any line
    int minlength = image.cols * image.cols * 0.001;

	// temporary vector for intermediate storage
	vector<int> temp;

	// store (x1, y1) and (x2, y2) endpoints for each line segment
	vector<cv::Vec4i> lines_std;

	// to store intermediate errors
	double temperr;

    
	// constructor to set video/webcam and find vanishing point
	vanishingPt() {}
	void init() {
		for(int i = 0; i < lines_std.size(); i++) {
			// ignore if almost vertical
			if (abs(lines_std[i][0] - lines_std[i][2]) < 10 || abs(lines_std[i][1] - lines_std[i][3]) < 10) // check if almost vertical
				continue;
			// ignore shorter lines (x1-x2)^2 + (y2-y1)^2 < minlength
			if (((lines_std[i][0] - lines_std[i][2]) * (lines_std[i][0] - lines_std[i][2]) + (lines_std[i][1] - lines_std[i][3]) * (lines_std[i][1] - lines_std[i][3])) < minlength)
				continue;
            /*    
            // ignore (0, x)
            double slope = abs(lines_std[i][1] - lines_std[i][3]) / abs(lines_std[i][0] - lines_std[i][2]);
            if(slope < 0.001) continue;
            */

            // store valid lines' endpoints for calculations
			for(int j = 0; j < 4; j++) {
				temp.push_back(lines_std[i][j]);
			}
			points.push_back(temp);
			temp.clear();
		}
		cout << "Detected:" << lines_std.size() << endl;
		cout << "Filtered:" << points.size() << endl;
	}

	void makeLines() {
		// to solve Ax = b for x
	    A = zeros<mat>(points.size(), 2);
	    b = zeros<mat>(points.size(), 1);

	    // convert given end-points of line segment into a*x + b*y = c format for calculations
	    // do for each line segment detected
	    for(int i = 0; i < points.size(); i++) {
			A(i, 0) = -(points[i][3] - points[i][1]);			// -(y2-y1)
			A(i, 1) = (points[i][2] - points[i][0]);				// x2-x1
			b(i, 0) = A(i, 0) * points[i][0] + A(i, 1) * points[i][1];	// -(y2-y1)*x1 + (x2-x1)*y1 = x2*y1 - x1*y2
	    }
	}

	// estimate the vanishing point
	void eval() {
		// stores the estimated co-ordinates of the vanishing point with respect to the image
		soln = zeros<mat>(2, 1);

        // theta
        int theta = 0;
        double delta = 0.0;

        // buffer for UART
        char buffer;

		// initialize error
		double err = 9999999999;

		// calculate point of intersection of every pair of lines and
		// find the sum of distance from all other lines
		// select the point which has the minimum sum of distance
		for(int i = 0; i < points.size(); i++) {
			for(int j = 0; j < points.size(); j++) {
				if(i >= j)
				continue;

				// armadillo vector
				uvec indices;

				// store indices of lines to be used for calculation
				indices << i << j;

				// extract the rows with indices specified in uvec indices
				// stores the ith and jth row of matrices A and b into Atemp and btemp respectively
				// hence creating a 2x2 matrix for calculating point of intersection
				Atemp = A.rows(indices);
				btemp = b.rows(indices);

				// if lines are parallel then skip
				if(arma::rank(Atemp) != 2)
					continue;

				// solves for 'x' in A*x = b
				res = calc(Atemp, btemp);

				if(res.n_rows == 0 || res.n_cols == 0)
					continue;

				// calculate error assuming perfect intersection is 
				error = A * res - b;

				// reduce size of error
				error = error / 1000;

				// to store intermediate error values
				temperr = 0;

				// summation of errors
				for(int k = 0; k < error.n_rows; k++)
                    temperr += (error(k, 0) * error(k, 0)) / 1000;

                // scale errors to prevent any overflows
				temperr /= 1000000;

				// if current error is smaller than previous min error then update the solution (point)
				if(err > temperr) {
					soln = res;
					err = temperr;
				}
			}
		}

		cout<<"\n\nResult:\n"<<soln(0,0)<<","<<soln(1,0)<<"\nError:"<<err<<"\n\n";
/*
		// draw a circle to visualize the approximate vanishing point
		if(soln(0, 0) > 0 && soln(0, 0) < image.cols && soln(1, 0) > 0 && soln(1, 0) < image.rows) {
			cv::circle(frame, Point(soln(0, 0), soln(1, 0)), 15, cv::Scalar(0, 0, 255), 2);
            Point start(240, 320);
            Point end(soln(0, 0), soln(1, 0));
            line(frame, start, end, cv::Scalar(0, 0, 255), 2);
            theta = atan2(320 - soln(1, 0), soln(0, 0) - 240) * (180 / CV_PI);
            cout << theta << endl;
            // toDo: use previous frame's result to reduce calculations and stabilize the region of vanishing point
            prevRes = soln;
        }
        else {
            cv::circle(frame, Point(prevRes(0, 0), prevRes(1, 0)), 15, cv::Scalar(0, 0, 255), 2);
            Point start(240, 320);
            Point end(prevRes(0, 0), prevRes(1, 0));
            line(frame, start, end, cv::Scalar(0, 0, 255), 2);
            theta = atan2(320 - prevRes(1, 0), prevRes(0, 0) - 240) * (180 / CV_PI);
            cout << theta << endl;
        }
        if(theta < 0) {
            theta += 360;
        }
		if(theta > 120)		theta = 120;
		else if(theta < 90)	theta = 90;
        buffer = theta;
        uart_ch(buffer);
        
        cv::namedWindow("Line");
		cv::imshow("Line", frame);

		// flush the vector
		points.clear();
*/      
        // line for detect position of vanishing point
        line(frame, Point(240, 0), Point(240, 320), cv::Scalar(0, 0, 255), 1);
        line(frame, Point(0, 150), Point(480, 150), cv::Scalar(0, 0, 255), 1);

        // draw a circle to visualize the approximate vanishing point
		if(soln(0, 0) > 0 && soln(0, 0) < image.cols && soln(1, 0) > 0 && soln(1, 0) < image.rows) {
			cv::circle(frame, Point(soln(0, 0), soln(1, 0)), 15, cv::Scalar(0, 0, 255), 2);
            // toDo: use previous frame's result to reduce calculations and stabilize the region of vanishing point
            prevRes = soln;
        }
        else {
            cv::circle(frame, Point(prevRes(0, 0), prevRes(1, 0)), 15, cv::Scalar(0, 0, 255), 2);
        }

        //float angle = 0, a = 0;
        vector<int> temp_2;
        vector< vector<int> > lpoints;
        vector< vector<int> > rpoints;
/*
        for (int i = 0; i < points.size(); i++){
            float t = 0, p = 0;
            if((points[i][0] - points[i][2]) == 0) {
                a = -CV_PI / 2;
                angle = -90;
            }
            else {
                t = (points[i][1] - points[i][3]) / (points[i][0] - points[i][2]);
                a = atan(t);
                angle = a * 180 / CV_PI;
            }
            for (int j = 0; j < 4; j++) {
				temp_2.push_back(points[i][j]);
			}
            if(angle > 0 || angle < 0){
                p = (points[i][0] + points[i][2]) / 2;
                if(p < 240) { // 왼쪽차선이면
                    lpoints.push_back(temp_2);
                }
                if(p > 240) { // 오른쪽차선이면
                    rpoints.push_back(temp_2);
                }
            }
			temp_2.clear();
        }
*/
       for (int i = 0; i < points.size(); i++){
            for (int j = 0; j < 4; j++) {
				temp_2.push_back(points[i][j]);
			}
            if (points[i][1] > points[i][3]) lpoints.push_back(temp_2);
            if (points[i][1] < points[i][3]) rpoints.push_back(temp_2);
			temp_2.clear();
        }
        for (int i = 0; i < lpoints.size(); i++){
            cout << "lpoints: ";
            for (int j = 0; j < 4; j++) cout << lpoints[i][j] << " ";
            cout << endl;
        }
        for (int i = 0; i < rpoints.size(); i++){
            cout << "rpoints: ";
            for (int j = 0; j < 4; j++) cout << rpoints[i][j] << " ";
            cout << endl;
        }
/*        
        if (lpoints.empty() && rpoints.size() > 0) {
            if (times == 40) delta = 120;
            else {
                delta = 0;
                times++;
            }
            cout << "mode 1" << endl;
        }
        else if (rpoints.empty() && lpoints.size() > 0) {
            if (times == 40) delta = -120;
            else {
                delta = 0;
                times++;
            }
            cout << "mode 2" << endl;
        }
        else if (lpoints.empty() && rpoints.empty()) {
            delta = ((240 - prevRes(0, 0)) / 240) * 120 * ((300 - prevRes(1, 0)) / 150);
            times = 0;
            cout << "mode 3" << endl;
        }
        else if (rpoints.size() > 0 && lpoints.size() > 0) {
            delta = ((240 - soln(0, 0)) / 240) * 120 * ((300 - soln(1, 0)) / 150);
            times = 0;
            cout << "mode 4" << endl;
        }
*/        
        if (rpoints.empty() && lpoints.size() > 0) {
            soln(0, 0) = lpoints[0][2];
            soln(1, 0) = lpoints[0][3];
        }
        if (lpoints.empty() && rpoints.size() > 0) {
            soln(0, 0) = rpoints[0][2];
            soln(1, 0) = rpoints[0][3];
        }
        delta = ((240 - soln(0, 0)) / 240) * 120 * ((300 - soln(1, 0)) / 150);

        buffer = (int)delta + 120;
        //buffer = 240;
        cout << (int)buffer - 60 << endl;
        uart_ch(buffer);
        
        cv::namedWindow("Line");
		cv::imshow("Line", frame);

		// flush the vector
		points.clear();
	}

	//function to calculate and return the intersection point
	mat calc(mat A, mat b) {
	    mat x = zeros<mat>(2, 1);
		solve(x, A, b);
	    return x;
	}
};

int main()
{
	// make object
    vanishingPt vp;

    // to calculate fps
    clock_t begin, end;

    // read from video file on disk
    // to read from webcam initialize as: cap = VideoCapture(int device_id);
    VideoCapture cap(0);
    cv::Mat source;
    
    if(cap.isOpened()) { // check if camera/ video stream is available
        // get first frame to intialize the values
        cap.read(vp.frame);
        vp.image = cv::Mat(cv::Size(vp.frame.rows, vp.frame.cols), CV_8UC1, 0.0);
    }

    while(cap.isOpened()) { // check if camera/ video stream is available
        cap >> source;
        cv::resize(source, source, cv::Size(480, 320));
        if(!cap.grab())
            continue;

        // to calculate fps
        begin = clock();
        
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
        ld.setLineLengthAndGap(50, 20); // 50, 20
        //ld.setLineLengthAndGap(80, 20); // 50, 20
        ld.setMinVote(60); // 60
        
        // Detect lines
        vector<Vec4i> li = ld.findLines(contours);

        // remove inconsistent lines
        // ld.removeLinesOfInconsistentOrientations(ed.getOrientation(), 0.4, 0.1); // 0.4  0.1
                
        ld.drawDetectedLines(tmp);
        
        for(int i = 0; i < li.size(); i++){
            for(int j = 0; j < 4; j++){
                if(j == 1 || j == 3) li[i][j] += 240;
                //if(j == 1 || j == 3) li[i][j] += 160;
                cout << li[i][j] << " ";
            }
            cout << endl;
        }
        
        source.copyTo(tmp);

        vp.lines_std = li;

        // it's advised not to modify image stored in the buffer structure of the opencv.
        vp.frame = tmp.clone();

        cv::cvtColor(vp.frame, vp.image, cv::COLOR_BGR2GRAY);

        // resize frame to 480x320
        cv::resize(vp.image, vp.image, cv::Size(480, 320));
        cv::resize(vp.frame, vp.frame, cv::Size(480, 320));

        // equalize histogram
        cv::equalizeHist(vp.image, vp.image);

        // initialize the line segment matrix in format y = m*x + c	
        vp.init();

        // draw lines on image and display
        vp.makeLines();
        
        // approximate vanishing point
        vp.eval();

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
