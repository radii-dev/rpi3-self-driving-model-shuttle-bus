/* g++ final_4.cpp -o final_4 `pkg-config --libs opencv` -lwiringPi */
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

Point GetWrappedPoint(Mat M, const Point& p);

#define WINDOW_NAME_1 "WINDOW1"
#define WINDOW_NAME_2 "WINDOW2"

int main()
{
    cout << CV_VERSION <<endl;
	VideoCapture cap(0);
	Mat mFrame, mGray, mCanny, imageROI,mGray1, mGray2, carTrack , mask, IPM_ROI, IPM, IPM_Gray, IPM1, IPM2 ,IPM_Gray2, mFrame2;
    
    double fps = 0;
    int level=0,a=mFrame.rows;
    
    // Number of frames to capture
    int num_frames = 60;
    int started_frames = 0;
    
    // Start and end times
    time_t start, end;
    
    // Variable for storing video frames
    Mat frame;
    
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
        
        mFrame2 = mFrame.clone();
        imageROI = mFrame(Rect(0,mFrame.rows/2,mFrame.cols,mFrame.rows/2));
        IPM_ROI = imageROI(Rect(0,65,imageROI.cols,(imageROI.rows-65)));
        IPM_ROI = IPM_ROI.clone();


		cvtColor(imageROI, mGray, COLOR_BGR2GRAY);
        cvtColor(mFrame, mGray2, COLOR_BGR2GRAY);
        
        //imshow("before", mGray);
        mGray.copyTo(mGray1);
		//equalizeHist(mGray, mGray);
        //imshow("after", mGray);
        
        
        Point2f inputQuad[4];
        Point2f outputQuad[4];
        
        imshow("3333", IPM_ROI);
        
        Mat IPM_Matrix( 2, 4, CV_32FC1 );
        Mat IPM_Matrix_inverse;
        
        // Set the IPM_Matrix matrix the same type and size as input
        IPM_Matrix = Mat::zeros( mFrame.rows, mFrame.cols, mFrame.type() );
        
        // The 4 points that select quadilateral on the input , from top-left in clockwise order
        // These four pts are the sides of the rect box used as input
        inputQuad[0] = Point2f( 0,0);
        inputQuad[1] = Point2f( IPM_ROI.cols,0);
        inputQuad[2] = Point2f( IPM_ROI.cols,IPM_ROI.rows);
        inputQuad[3] = Point2f( 0,IPM_ROI.rows);           //
        // The 4 points where the mapping is to be done , from top-left in clockwise order
        outputQuad[0] = Point2f( 0,0 );
        outputQuad[1] = Point2f( mFrame.cols,0);
        outputQuad[2] = Point2f( mFrame.cols-250,mFrame.rows);
        outputQuad[3] = Point2f( 250,mFrame.rows);
        
        // Get the Perspective Transform Matrix i.e. IPM_Matrix
        IPM_Matrix = getPerspectiveTransform( inputQuad, outputQuad );
        invert(IPM_Matrix,IPM_Matrix_inverse);

        // Apply the Perspective Transform just found to the src image
 
        warpPerspective(IPM_ROI,IPM,IPM_Matrix,mFrame.size() );


        imshow("IPM", IPM);
        cvtColor(IPM, IPM_Gray, COLOR_BGR2GRAY);
        GaussianBlur(IPM_Gray, IPM_Gray, Size(7,7), 1.5, 1.5);
        //imshow("IPM BEFORE CANNY", IPM_Gray);
        Canny(IPM_Gray, IPM_Gray, 5, 40, 3);
        //imshow("IPM AFTER CANNY", IPM_Gray);
        IPM.copyTo(IPM1);
        IPM.copyTo(IPM2);
        

        //nested loops to eliminate the angled "lines" edges and trim the IPM
        
        for (int i=0; i<IPM_Gray.rows; i++){
            uchar* data= IPM_Gray.ptr<uchar>(i);
            for (int j=0; j<IPM_Gray.cols; j++)
            {
                if(i<0 || i>480)
                {
                    // process each pixel
                    data[j]= data[j]>level?level:0;
                }else{
                    if(data[j]<=255 && data[j]>240 ){
                        for(int m=j;m<j+20;m++){
                            a=m;
                            data[m]=0;
                        }
                        j=a;
                        break;
                    }
                }
            }
        }
        
        for (int i=0; i<IPM_Gray.rows; i++){
            uchar* data= IPM_Gray.ptr<uchar>(i);
            for(int j=IPM_Gray.cols;j>0;j--){
                if(data[j]<=255 && data[j]>240){
                    for(int m=j;m>j-20;m--){
                        data[m]=0;
                    }
                    j=j-20;
                    break;
                }	
            }   
        }

        GaussianBlur( IPM_Gray,IPM_Gray, Size( 5, 5 ), 1.5, 1.5 );
        imshow("IPM BINRARY AFTER FILTERING", IPM_Gray);
      
        
        
        vector<Vec4i> lines;
        HoughLinesP(IPM_Gray,lines,1, 0.01, 120 ,10,600 );
       // HoughLinesP(IPM_Gray,lines,1, 0.01, 120  );

 

        vector<Point> laneShade,laneShade1,laneShade2;
       	float d=0.00,d1=0.00;
        int s=0;
        int n=mFrame.cols;
        Point e,f,g,h,A,B,C,D;
        float angle;float a;
        for( size_t i = 0; i < lines.size(); i++ ){
            float p=0,t=0;
            Vec4i l = lines[i];
            if((l[0]-l[2])==0){
                a=-CV_PI/2;
                angle=-90;
            }else{
                t=(l[1]-l[3])/(l[0]-l[2]);
                a=atan(t);
                angle=a*180/CV_PI;
            }

            if(angle>50 ||  angle<(-50)){
                
                p=(l[0]+l[2])/2;
                line(IPM1,Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,0,0), 3,CV_AA);
                if(p<320){
                    if(p>s){
                        s=p;
                        d=320-(s);
                        e=Point(l[0],l[1]);
                        f=Point(l[2],l[3]);
                        A= GetWrappedPoint(IPM_Matrix_inverse,e);
                        B =GetWrappedPoint(IPM_Matrix_inverse,f);
                        A.y += 245;
                        B.y += 245;

                        double lengthAB = sqrt((A.x - B.x)*(A.x - B.x) + (A.y - B.y)*(A.y - B.y));
                        if(A.y > B.y){
                        A.x = B.x + (B.x - A.x) / lengthAB * -350;
                        A.y = B.y + (B.y - A.y) / lengthAB * -350;
                            
                        }else{
                            B.x = B.x + (B.x - A.x) / lengthAB * 350;
                            B.y = B.y + (B.y - A.y) / lengthAB * 350;
                        
                        }

                    }
                    
                }
                if(p>320){
                    if(p<n){
                        n=p;
                        d1=(n)-320;
                        g=Point(l[0],l[1]);
                        h=Point(l[2],l[3]);
                        C= GetWrappedPoint(IPM_Matrix_inverse,g);
                        C.y +=245;
                        D =GetWrappedPoint(IPM_Matrix_inverse,h);
                        D.y +=245;
                        double lengthCD = sqrt((C.x - D.x)*(C.x - D.x) + (C.y - D.y)*(C.y - D.y));
                        if(C.x > D.x){
                        C.x = D.x + (D.x - C.x) / lengthCD * -350;
                        C.y = D.y + (D.y - C.y) / lengthCD * -350;
                        }else{
                            D.x = D.x + (D.x - C.x) / lengthCD * +350;
                            D.y = D.y + (D.y - C.y) / lengthCD * +350;
                        
                        }
                    }
                    
                }
                
            }
        }


        line(IPM2,e, f, Scalar(0,255,255), 3,CV_AA);
        line(IPM2,g, h, Scalar(0,145,255), 3,CV_AA);
        
        if(A.x < B.x){
            laneShade.push_back(B);
            laneShade.push_back(A);
        }else{
            laneShade.push_back(A);
            laneShade.push_back(B);
        }
        
        if(C.x > D.x){
            laneShade.push_back(C);
            laneShade.push_back(D);
        }else{
            laneShade.push_back(D);
            laneShade.push_back(C);
        }
        
        laneShade1.push_back(Point((laneShade[0].x+laneShade[3].x)/2,laneShade[0].y+20));
        laneShade1.push_back(Point((laneShade[0].x+laneShade[3].x)/2 +45,laneShade[1].y));
        laneShade1.push_back(Point((laneShade[0].x+laneShade[3].x)/2 -45,laneShade[2].y));
        laneShade1.push_back(Point((laneShade[0].x+laneShade[3].x)/2,laneShade[3].y+20));
        
        laneShade2.push_back(Point((laneShade[0].x+laneShade[3].x)/2,laneShade[0].y+20));
        laneShade2.push_back(Point((laneShade[0].x+laneShade[3].x)/2 +25,laneShade[2].y));
        laneShade2.push_back(Point((laneShade[0].x+laneShade[3].x)/2 -25,laneShade[2].y));
        laneShade2.push_back(Point((laneShade[0].x+laneShade[3].x)/2,laneShade[3].y+20));
        

        Point zero  = Point(0,0);
        if(laneShade[0]!=zero && laneShade[1]!=zero && laneShade[2]!=zero && laneShade[3]!=zero && laneShade[2].y>0){
        Mat laneMask= mFrame.clone();
        fillConvexPoly(laneMask, laneShade, Scalar(0,200,0));  //(255,144,30)
        fillConvexPoly(mFrame, laneShade1, Scalar(0,200,0));
        fillConvexPoly(mFrame, laneShade2, Scalar(255,255,255));
        addWeighted(mFrame, 0.6, laneMask, 0.4, 3, mFrame);
        }

        
        imshow("HOUGH BEFORE FILTERING",IPM1);
        imshow("HOUGH AFTER FILTERING",IPM2);

		imshow(WINDOW_NAME_1, mFrame);
        imshow(WINDOW_NAME_2, mFrame2);
        

		waitKey(10);
	}

	return 0;
}

Point GetWrappedPoint(Mat M, const Point& p)
{
    cv::Mat_<double> src(3/*rows*/,1 /* cols */);
 
    src(0,0)=p.x;
    src(1,0)=p.y;
    src(2,0)=1.0;
 
    cv::Mat_<double> dst = M*src;
    dst(0,0) /= dst(2,0);
        dst(1,0) /= dst(2,0);
    return Point(dst(0,0),dst(1,0));
}