#include <cv.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <math.h>

int lowerH=0;
int lowerS=0;
int lowerV=0;

int upperH=180;
int upperS=256;
int upperV=256;

int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
const char* window_name = "Edge Map";

//This function threshold the HSV image and create a binary image
IplImage* GetThresholdedImage(IplImage* imgHSV){

IplImage* imgThresh=cvCreateImage(cvGetSize(imgHSV),IPL_DEPTH_8U, 1);
cvInRangeS(imgHSV, cvScalar(lowerH,lowerS,lowerV), cvScalar(upperH,upperS,upperV), imgThresh);

return imgThresh;

}

//This function create two windows and 6 trackbars for the "Ball" window
void setwindowSettings(){
	cvNamedWindow("Video");
	cvNamedWindow("Ball");

	cvCreateTrackbar("LowerH", "Ball", &lowerH, 180, NULL);
			cvCreateTrackbar("UpperH", "Ball", &upperH, 180, NULL);

	cvCreateTrackbar("LowerS", "Ball", &lowerS, 256, NULL);
			cvCreateTrackbar("UpperS", "Ball", &upperS, 256, NULL);

	cvCreateTrackbar("LowerV", "Ball", &lowerV, 256, NULL);
			cvCreateTrackbar("UpperV", "Ball", &upperV, 256, NULL);
}

int main(){
CvCapture* capture =0;

capture = cvCaptureFromCAM(0);
if(!capture){
printf("Capture failure\n");
return -1;
}

IplImage* frame=0;

setwindowSettings();

//iterate through each frames of the video
while(true){

	frame = cvQueryFrame(capture);
	if(!frame) break;
	frame=cvCloneImage(frame);

	//smooth the web cam's image
	cvSmooth(frame, frame, CV_GAUSSIAN,3,3);

	frame = cvQueryFrame(capture);
	if(!frame)  break;
	frame=cvCloneImage(frame);

	IplImage* imgHSV = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
	cvCvtColor(frame, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV

	IplImage* imgThresh = GetThresholdedImage(imgHSV);

	cvShowImage("Ball", imgThresh);
	cvShowImage("Video", frame);


	cvReleaseImage(&imgHSV);
	cvReleaseImage(&imgThresh);
	cvReleaseImage(&frame);

	//Wait 80mS
	int c = cvWaitKey(80);
	//If 'ESC' is pressed, break the loop
	if((char)c==27 ) break;

}

cvDestroyAllWindows();
cvReleaseCapture(&capture);

       return 0;
}
