#include "stdafx.h"

//OpenCV Headers
#include<opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
//Input-Output
#include<stdio.h>
#include <iostream>
#include <windows.h>
//Definitions
#define h 240
#define w 320
//NameSpaces
using namespace std;

//Global variables
int fcount=0;//Counts the number of frames
IplImage* FC_FindBiggestContours(IplImage* src);
CvRect R;
//Main Function
int main()
{


//Structure to get feed from CAM
CvCapture* capture=cvCreateCameraCapture(-1);

//Windows
cvNamedWindow("Live",CV_WINDOW_AUTOSIZE);
cvNamedWindow("Threshy",CV_WINDOW_AUTOSIZE);
cvNamedWindow("cnt",CV_WINDOW_AUTOSIZE);

//Image Variables
IplImage *frame=cvCreateImage(cvSize(w,h),8,3); //Original Image
IplImage *hsvframe=cvCreateImage(cvSize(w,h),8,3);//Image in HSV color space
IplImage *threshy=cvCreateImage(cvSize(w,h),8,1); //Threshold image of yellow color

//Variables for trackbars
int h1=0;int s1=30;int v1=80;
int h2=20;int s2=150;int v2=255;

//Creating the trackbars
cvCreateTrackbar("H1","cnt",&h1,255,0);
cvCreateTrackbar("H2","cnt",&h2,255,0);
cvCreateTrackbar("S1","cnt",&s1,255,0);
cvCreateTrackbar("S2","cnt",&s2,255,0);
cvCreateTrackbar("V1","cnt",&v1,255,0);
cvCreateTrackbar("V2","cnt",&v2,255,0);

//Infinate Loop
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
cvCvtColor(frame,hsvframe,CV_BGR2HSV);
//Thresholding the frame for yellow
cvInRangeS(hsvframe,cvScalar(h1,s1,v1),cvScalar(h2,s2,v2),threshy);
//Filtering the frame
cvSmooth(threshy,threshy,CV_MEDIAN,7,7);
//Finding largest contour
threshy=FC_FindBiggestContours(threshy);
//Getting the screen information
int screenx = GetSystemMetrics(SM_CXSCREEN);
int screeny = GetSystemMetrics(SM_CYSCREEN);
//Calculating the moments
CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
cvMoments(threshy, moments, 1);
// The actual moment values
double moment10 = cvGetSpatialMoment(moments, 1, 0);
double moment01 = cvGetSpatialMoment(moments, 0, 1);
double area = cvGetCentralMoment(moments, 0, 0);


//Getting the current frame
IplImage *fram2=cvQueryFrame(capture);
//If failed to get break the loop
if(!fram)
break;

//1.PREPROCESSING OF FRAME
//Resizing the capture
cvResize(fram2,frame,CV_INTER_LINEAR );
//Flipping the frame
cvFlip(frame,frame,1);
//Changing the color space
cvCvtColor(frame,hsvframe,CV_BGR2HSV);
//Thresholding the frame for yellow
cvInRangeS(hsvframe,cvScalar(h1,s1,v1),cvScalar(h2,s2,v2),threshy);
//Filtering the frame
cvSmooth(threshy,threshy,CV_MEDIAN,7,7);
//Biggest Contour
threshy=FC_FindBiggestContours(threshy);
//Getting the screen information
int screenx2 = GetSystemMetrics(SM_CXSCREEN);
int screeny2 = GetSystemMetrics(SM_CYSCREEN);
//Calculating the moments
CvMoments *moments2 = (CvMoments*)malloc(sizeof(CvMoments));
cvMoments(threshy, moments2, 1);
// The actual moment values
double moment20 = cvGetSpatialMoment(moments2, 1, 0);
double moment02 = cvGetSpatialMoment(moments2, 0, 1);
double area2 = cvGetCentralMoment(moments2, 0, 0);
//Position Variables
int x1;
int y1;
int x2;
int y2;
//Calculating the current position
x1 = moment10/area;
y1 = moment01/area;
x2 = moment20/area2;
y2 = moment20/area2;
//Fitting to the screen
int x=(int)(x1*screenx/w);
int y=(int)(y1*screeny/h);

/*if(x>>0 && y>>0 )
{
cvLine(frame, cvPoint(x1,y1), cvPoint(x1,y1), cvScalar(0,25,255),5);
cout<<"X:"<<x<<"\tY:"<<y<<endl;
}*/

//Moving the mouse pointer
//SetCursorPos(x,y);
if(x2-x1>80)
{

    keybd_event(VK_RIGHT,0,KEYEVENTF_EXTENDEDKEY|0,0);
keybd_event(VK_RIGHT,0,KEYEVENTF_EXTENDEDKEY|KEYEVENTF_KEYUP,0);
keybd_event(VK_PRIOR,0,KEYEVENTF_EXTENDEDKEY|0,0);
keybd_event(VK_PRIOR,0,KEYEVENTF_EXTENDEDKEY|KEYEVENTF_KEYUP,0);
cout<<"RIGHT"<<endl;
}

if(x1-x2>80)
{
    keybd_event(VK_LEFT,0,KEYEVENTF_EXTENDEDKEY|0,0);
keybd_event(VK_LEFT,0,KEYEVENTF_EXTENDEDKEY|KEYEVENTF_KEYUP,0);
keybd_event(VK_NEXT,0,KEYEVENTF_EXTENDEDKEY|0,0);
keybd_event(VK_NEXT,0,KEYEVENTF_EXTENDEDKEY|KEYEVENTF_KEYUP,0);
cout<<"LEFT"<<endl;
}



//Showing the images
cvShowImage("Live",frame);
cvShowImage("Threshy",threshy);
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
   // clean up raw mask
   cvMorphologyEx( mask, mask, 0, 0, CV_MOP_OPEN, 1 );
   cvMorphologyEx( mask, mask, 0, 0, CV_MOP_CLOSE, 1 );
   // find contours around only bigger regions
   scanner = cvStartFindContours( mask, tempStorage,
                   sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, offset );
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
                   sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, offset );
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
         newC = cvApproxPoly( c, sizeof(CvContour), tempStorage, CV_POLY_APPROX_DP, 2, 0 );
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
     cvDrawContours( src_img, c, cvScalarAll(1), cvScalarAll(1), -1, -1, 8,
             cvPoint(-offset.x,-offset.y));
   }
     cvReleaseMemStorage( &tempStorage );
 // convert to 8 bit IplImage
  for( int i = 0; i < src_img->height; i++ )
    for( int j = 0; j < src_img->width; j++ )
    {
     int idx = CV_IMAGE_ELEM( src_img, int, i, j );  //get reference to pixel at (col,row),
     uchar* dst = &CV_IMAGE_ELEM( dest, uchar, i, j );                          //for multi-channel images (col) should be multiplied by number of channels
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
