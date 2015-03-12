// GAMBIT.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"


// GAMBIT.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"


#include "opencv/cv.h"

#include "opencv/cxcore.h"

#include "opencv/highgui.h"

#include "math.h"

#include <iostream>

#include <stdio.h>

#include <string.h>

#include <conio.h>

#include <sstream>

#include <windows.h>
//Definitions
#define h 300
#define w 400


using namespace std;

/*

--------------------------------------------*/
IplImage* FC_FindBiggestContours(IplImage* src);
//CvRect R;

int main()

{
    //Structure to get feed from CAM
CvCapture* capture=cvCreateCameraCapture(-1);
 
//Windows
cvNamedWindow("Live",CV_WINDOW_AUTOSIZE);
cvNamedWindow("Threshs",CV_WINDOW_AUTOSIZE);
cvNamedWindow("Threshy",CV_WINDOW_AUTOSIZE);
cvNamedWindow("Threshg",CV_WINDOW_AUTOSIZE);
cvNamedWindow("Threshb",CV_WINDOW_AUTOSIZE);
cvNamedWindow("cnt",CV_WINDOW_AUTOSIZE);
 
//Image Variables
IplImage *frame=cvCreateImage(cvSize(w,h),8,3);   //Original Image
IplImage *hsv1=cvCreateImage(cvSize(w,h),8,3);
//IplImage *hsv2=cvCreateImage(cvSize(w,h),8,3);
//IplImage *hsv3=cvCreateImage(cvSize(w,h),8,3);//Image in HSV color space
IplImage *threshy=cvCreateImage(cvSize(w,h),8,1); //Threshold image of yellow color-Tracker 1
IplImage *threshg=cvCreateImage(cvSize(w,h),8,1);//Threshold image of green color-Tracker 2
IplImage *threshb=cvCreateImage(cvSize(w,h),8,1);//Threshold image of blue color-Tracker 3
IplImage *threshs=cvCreateImage(cvSize(w,h),8,1);//Threshold image of skin color-Hand



//Variables for trackbars
int h1=23;int s1=41;int v1=133;
int h2=40;int s2=150;int v2=255;
 
//Creating the trackbars
cvCreateTrackbar("H1","cnt",&h1,255,0);
cvCreateTrackbar("H2","cnt",&h2,255,0);
cvCreateTrackbar("S1","cnt",&s1,255,0);
cvCreateTrackbar("S2","cnt",&s2,255,0);
cvCreateTrackbar("V1","cnt",&v1,255,0);
cvCreateTrackbar("V2","cnt",&v2,255,0);
 
//Infinite Loop
while(1)
{
 
//Getting the current frame
IplImage *fram=cvQueryFrame(capture);
//If failed to get break the loop
if(!fram)
break;
 
//1.PREPROCESSING OF FRAME
//Resizing the capture
cvResize(fram,frame,CV_INTER_LINEAR );
//Flipping the frame
cvFlip(frame,frame,1);
//Changing the color space
cvCvtColor(frame,hsv1,CV_BGR2HSV);
//Thresholding the frame for yellow
cvInRangeS(hsv1,cvScalar(20,100,100),cvScalar(30,255,255),threshy);
//Thresholding frame for Green
cvInRangeS(hsv1,cvScalar(78,106,43),cvScalar(105,255,85),threshg);
//cvInRangeS(hsv1,cvScalar(142,62,38),cvScalar(180,255,255),threshg);
//Thresholding frame for blue
cvInRangeS(hsv1,cvScalar(113,52,0),cvScalar(131,255,71),threshb);
//Thresholding frame for Skin
cvInRangeS(hsv1,cvScalar(0,30,80),cvScalar(20,155,255),threshs);
//Filtering the frame
cvSmooth(threshy,threshy,CV_MEDIAN,7,7);
cvSmooth(threshs,threshs,CV_MEDIAN,7,7);
cvSmooth(threshg,threshg,CV_MEDIAN,7,7);
cvSmooth(threshb,threshb,CV_MEDIAN,7,7);
threshs= FC_FindBiggestContours(threshs);
//Getting the screen information
int screenx = GetSystemMetrics(SM_CXSCREEN);
int screeny = GetSystemMetrics(SM_CYSCREEN);
//Calculating the moments
CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));//Moments for skin
CvMoments *momenty = (CvMoments*)malloc(sizeof(CvMoments));//Moments for yellow
CvMoments *momentg = (CvMoments*)malloc(sizeof(CvMoments));//Moments for green
CvMoments *momentb = (CvMoments*)malloc(sizeof(CvMoments));//Moments for blue

cvMoments(threshy, momenty, 1);
cvMoments(threshg, momentg, 1);
cvMoments(threshb, momentb, 1);
cvMoments(threshs, moments, 1);
// The actual moment values of hand
double moment10 = cvGetSpatialMoment(moments, 1, 0);
double moment01 = cvGetSpatialMoment(moments, 0, 1);
double area = cvGetCentralMoment(moments, 0, 0);
//moment values of green
double momentg0 = cvGetSpatialMoment(momentg, 1, 0);
double moment0g = cvGetSpatialMoment(momentg, 0, 1);
double areag = cvGetCentralMoment(momentg, 0, 0);
//moment values of yellow
double momenty0 = cvGetSpatialMoment(momenty, 1, 0);
double moment0y = cvGetSpatialMoment(momenty, 0, 1);
double areay = cvGetCentralMoment(momenty, 0, 0);
//moment values of blue
double momentb0 = cvGetSpatialMoment(momentb, 1, 0);
double moment0b = cvGetSpatialMoment(momentb, 0, 1);
double areab = cvGetCentralMoment(momentb, 0, 0);


//Position Variables
int x1,xy,xb,xg;
int y1,yy,yb,yg;
//Calculating the current position of hand
x1 = moment10/area;
y1 = moment01/area;
//Calculating current position of green
xg = momentg0/areag;
yg = moment0g/areag;
//Calculating current position of blue
xb = momentb0/areab;
yb = moment0b/areab;
//Calculating current position of yellow
xy = momenty0/areay;
yy = moment0y/areay;
//Fitting to the screen
int x=(int)(x1*screenx/w);
int y=(int)(y1*screeny/h);

if(x>>0 && y>>0 )
{
cvLine(threshs, cvPoint(x1,y1), cvPoint(x1,y1), cvScalar(0,25,255),5);
cout<<"X:"<<x<<"\tY:"<<y<<endl;
SetCursorPos(x,y);
}



if(areay!=0)
{
if(abs(xy-xb)<50 && abs(yy-yb)<50){

    mouse_event(MOUSEEVENTF_LEFTDOWN,x1,y1,0,0);
    mouse_event(MOUSEEVENTF_LEFTUP,x1,y1,0,0);
    cout<<"LEFT CLICK";
	cvWaitKey(250);


}

if(abs(xy-xg)<50 && abs(yy-yg)<50){

    mouse_event(MOUSEEVENTF_RIGHTDOWN,x1,y1,0,0);
    mouse_event(MOUSEEVENTF_RIGHTUP,x1,y1,0,0);
    cout<<"RIGHT";
    cvWaitKey(1000);
}}

//Showing the images
cvShowImage("Live",frame);
cvShowImage("Threshs",threshs);
cvShowImage("Threshb",threshb);
cvShowImage("Threshy",threshy);
cvShowImage("Threshg",threshg);
//Escape Sequence
char c=cvWaitKey(50);
if(c==27)
break;
 
}
//Cleanup
cvReleaseCapture(&capture);
cvDestroyAllWindows();
    



}

IplImage* FC_FindBiggestContours(IplImage* src)
  {
    IplImage temp= *src;
    IplImage *src_img=cvCreateImage(cvSize(temp.width,temp.height),IPL_DEPTH_32S,1);
    IplImage *dest=cvCreateImage(cvSize(temp.width,temp.height),IPL_DEPTH_8U,1);
    CvArr* _mask=&temp;
    int poly1Hull0=1;
    CvPoint offset;
    offset.x=0;
    offset.y=0;
   CvMat mstub, *mask = cvGetMat( _mask, &mstub );
   CvMemStorage* tempStorage = cvCreateMemStorage();
   CvSeq *contours, *c;
   int nContours = 0;
   double largest_length = 0,len = 0;
   CvContourScanner scanner;
   // clean up of mask
   cvMorphologyEx( mask, mask, 0, 0, CV_MOP_OPEN, 1 );
   cvMorphologyEx( mask, mask, 0, 0, CV_MOP_CLOSE, 1 );
   // find contours around only bigger regions
   scanner = cvStartFindContours( mask, tempStorage, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, offset );
   while( (c = cvFindNextContour( scanner )) != 0 )
   {
     len = cvContourPerimeter( c );
     if(len > largest_length)
     {
       largest_length = len;
     }
   }
   contours=cvEndFindContours( &scanner );
   scanner = cvStartFindContours( mask, tempStorage,
                   sizeof(CvContour), CV_RETR_EXTERNAL, 

CV_CHAIN_APPROX_SIMPLE, offset );
   while( (c = cvFindNextContour( scanner )) != 0 )
   {
     len = cvContourPerimeter( c );
     double q = largest_length ;
     if( len < q ) //Get rid of blob if it's perimeter is too small
       cvSubstituteContour( scanner, 0 );
     else  //Smooth it's edges if it's large enough
     {
       CvSeq* newC;
       if( poly1Hull0 ) //Polygonal approximation of the segmentation
         newC = cvApproxPoly( c, sizeof(CvContour), tempStorage, 

CV_POLY_APPROX_DP, 2, 0 );
       else //Convex Hull of the segmentation
         newC = cvConvexHull2( c, tempStorage, CV_CLOCKWISE, 1 );
       cvSubstituteContour( scanner, newC );
       nContours++;
       R=cvBoundingRect(c,0);
     }
   }
   contours = cvEndFindContours( &scanner );
   // paint the found regions back into the image
   cvZero( src_img );
   cvZero( _mask );
   for( c=contours; c != 0; c = c->h_next )
   {
     cvDrawContours( src_img, c, cvScalarAll(1), cvScalarAll(1), -1, -1, 8, cvPoint(-offset.x,-offset.y));
   }
     cvReleaseMemStorage( &tempStorage );
 // convert to 8 bit IplImage
  for( int i = 0; i < src_img->height; i++ )
    for( int j = 0; j < src_img->width; j++ )
    {
     int idx = CV_IMAGE_ELEM( src_img, int, i, j );  //get reference to pixel at (col,row),
     uchar* dst = &CV_IMAGE_ELEM( dest, uchar, i, j );                    

      //for multi-channel images (col) should be multiplied by number of channels
     if( idx == -1 || idx == 1 )
      *dst = (uchar)255;
     else if( idx <= 0 || idx > 1 )
      *dst = (uchar)0; // should not get here
     else {
      *dst = (uchar)0;
        }
      }
  cvReleaseImage(&src_img);
 
  return dest; 
 }




