#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <math.h>
#include <iostream>
#include <time.h>
#include <string>

#define LEFT						0
#define RIGHT						1
#define TOP							2
#define BOT							3


#define GOAL_HEIGHT					90.0
#define GOAL_WIDTH					167.0
#define GOAL_DEPTH					44.0

#define UPPER_LEFT_CORNER			0
#define UPPER_RIGHT_CORNER			1
#define LOWER_LEFT_FRONT_CORNER		2
#define LOWER_RIGHT_FRONT_CORNER	3
#define LOWER_LEFT_BACK_CORNER		4
#define LOWER_RIGHT_BACK_CORNER		5

#define PI							3.141592
#define INT_FLAG					-100
#define DOUBLE_FLAG					-0.12345

#define BALL_DIAMETER				19.735

using namespace cv;

int history_size = 8;

int lowerH=15;//11;//20;//10;//18;
int lowerS=202;//165;//100;//227;//158;
int lowerV=255;//145;//100;//141;//80;
int thresh1 = 5;
int thresh2 = 5;
int thresh3 = 5;

int upperH=47;//30;//30;//31;//23;
int upperS=256;//250;//255;//256;//220;
int upperV=256;//255;//255;//256;//178;

int kernel_size_erode = 4;

int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
const char* window_name = "Edge Map";

double distance[8];
double distance_confidence[8];
double angle[8];
double angle_confidence[8];
int distance_index = 0;
int angle_index = 0;

void setwindowSettings();
double toRadians(double degrees) {
	double result = (degrees * PI / 180.0);
	return result;
}

double toDegrees(double radians) {
	double result = (radians * 180.0 / PI);
	return result;
}

std::vector<double> getLine(Point p1, Point p2) {
	std::vector<double> arr;

	double m = ((double)p2.y-p1.y)/((double)p2.x-p1.x);
	if (p1.x == p2.x) {
		m = DOUBLE_FLAG;
	}
	double b = p1.y - m*p1.x;
	arr.push_back(m);
	arr.push_back(b);

	std::cout << "M: " << arr[0] << "\tb: " << arr[1] << std::endl;
	return arr;
}
double getDistance(Point p1, Point p2) {
	double result;
	int x1 = (int)p1.x/640.0*1600.0;
	int y1 = (int)p1.y/480.0*1200.0;

	int x2 = (int)p2.x/640.0*1600.0;
	int y2 = (int)p2.y/480.0*1200.0;

	long int dx = (x2 - x1);
	long int dy = (y2 - y1);
	result = sqrt(dx*dx + dy*dy);
	return result;
}

VideoCapture setup() {
	/// Declare variables
	Mat src;

	// To open live webcam
	VideoCapture cap(0);

	// To open file
	//VideoCapture cap("/home/unicorn/Videos/Webcam_view_goal.webm");

	/// Load an image
	src = imread( "/home/unicorn/Pictures/goal_1.jpg" );

	if( !src.data ) {
		//std::cout << "Failed to open image" << std::endl;
		return -1;
	}

	/// Create window
	setwindowSettings();

	std::cin.clear();

	return cap;
}

Mat GetThresholdedImage(Mat imgHSV) {
	Mat imgThresh;
	Scalar lower(lowerH, lowerS, lowerV);
	Scalar upper(upperH, upperS, upperV);
	//	Scalar lower(18, 177, 103);
	//	Scalar upper(26, 242, 183);
	inRange(imgHSV, lower, upper, imgThresh);

	return imgThresh;
}

Mat func_dilate(Mat image) {
	Mat result;
	Mat element = getStructuringElement(MORPH_RECT, Size(2*kernel_size_erode+1, 2*kernel_size_erode+1), Point(kernel_size_erode, kernel_size_erode));
	dilate(image, result, element);
	return result;
}

Mat func_erode(Mat image) {
	Mat result;
	Mat element = getStructuringElement(MORPH_RECT, Size(2*kernel_size_erode+1, 2*kernel_size_erode+1), Point(kernel_size_erode, kernel_size_erode));
	erode(image, result, element);
	return result;
}

// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
//static double angle( Point pt1, Point pt2, Point pt0 )
//{
//	double dx1 = pt1.x - pt0.x;
//	double dy1 = pt1.y - pt0.y;
//	double dx2 = pt2.x - pt0.x;
//	double dy2 = pt2.y - pt0.y;
//	return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
//}

//This function create two windows and 6 trackbars for the "Ball" window
void setwindowSettings(){
	cvNamedWindow("Ball", WINDOW_NORMAL);
	cvNamedWindow("Circle Detect", WINDOW_NORMAL);
//
//	cvCreateTrackbar("LowerH", "Ball", &lowerH, 256, NULL);
//	cvCreateTrackbar("UpperH", "Ball", &upperH, 256, NULL);
//	//		cvCreateTrackbar("Kernel", "Ball", &kernel_size_erode, 100, NULL);
//
//	cvCreateTrackbar("LowerS", "Ball", &lowerS, 256, NULL);
//	cvCreateTrackbar("UpperS", "Ball", &upperS, 256, NULL);
//
//	cvCreateTrackbar("LowerV", "Ball", &lowerV, 256, NULL);
//	cvCreateTrackbar("UpperV", "Ball", &upperV, 256, NULL);
	//	cvCreateTrackbar("Threshold 1", "Ball", &thresh1, 1000, NULL);
	//	cvCreateTrackbar("Threshold 2", "Ball", &thresh2, 1000, NULL);
	//	cvCreateTrackbar("Threshold 3", "Ball", &thresh3, 10.0, NULL);

}

Mat preprocess(Mat original) {
	Mat Rimg, Gimg, Bimg;
	std::vector<Mat> RGB;
	RGB.resize(3);
	RGB[0] = Rimg;
	RGB[1] = Gimg;
	RGB[2] = Bimg;
	split(original, RGB);
	//		threshold(RGB[0], RGB[0], 0, 125, CV_THRESH_BINARY | CV_THRESH_OTSU);
	//		threshold(RGB[1], RGB[1], 0, 125, CV_THRESH_BINARY | CV_THRESH_OTSU);
	threshold(RGB[2], RGB[2], 0, 255, CV_THRESH_BINARY);
	Mat NoBlue;

	merge(RGB, NoBlue);

	Mat temp;

	// Reduce
	medianBlur(NoBlue, temp, kernel_size);

	Mat imgHSV;
	cvtColor(temp, imgHSV, CV_BGR2HSV);

	Mat imgThresh = GetThresholdedImage(imgHSV);


	Mat canny;
	Canny(imgThresh, canny, 900, 1000);
	//
	//
	////			goodFeaturesToTrack(imgHSV, canny, 4, 0.1, 50 );
	//
	//

	canny = func_dilate(canny);
	canny = func_erode(canny);

	return canny;
}

std::vector<Point> determineCorners(std::vector<Point> refined_poi) {
	std::vector<Point> corners;
	corners.resize(8);

	std::cout << "Refined Positions Vector Size: " << refined_poi.size() << std::endl;

	if (refined_poi.size() >= 4  ) {

		std::vector<int> yMinIndex;
		std::vector<int> yMaxIndex;
		std::vector<int> xMaxIndex;
		std::vector<int> xMinIndex;
		int lower_right_index = -1;
		int lower_left_index = -1;
		int upper_right_index = -1;
		int upper_left_index = -1;
		int ymin = 100000;		// Some large number that is greater than the resolution size
		int ymax = 0;
		int xmax = 0;
		int xmin = 100000;

		/* Find the bottom most points */
		for (unsigned i = 0; i < refined_poi.size(); i++) {
			if (refined_poi[i].y > ymax) {
				ymax = refined_poi[i].y;
				yMaxIndex.clear();
				yMaxIndex.push_back(i);
			}
			else if (refined_poi[i].y == ymax) {
				yMaxIndex.push_back(i);
			}

			if (refined_poi[i].x > xmax) {
				xmax = refined_poi[i].x;
				xMaxIndex.clear();
				xMaxIndex.push_back(i);
			}
			else if (refined_poi[i].x == xmax) {
				xMaxIndex.push_back(i);
			}

			if (refined_poi[i].y < ymin) {
				ymin = refined_poi[i].y;
				yMinIndex.clear();
				yMinIndex.push_back(i);
			}
			else if (refined_poi[i].y == ymin) {
				yMinIndex.push_back(i);
			}

			if (refined_poi[i].x < xmin) {
				xmin = refined_poi[i].x;
				xMinIndex.clear();
				xMinIndex.push_back(i);
			}
			else if (refined_poi[i].x == xmin) {
				xMinIndex.push_back(i);
			}

		}

		ymax = 0;
		ymin = 100000;
		xmin = 100000;
		xmax = 0;

		if (yMaxIndex.size() < 2) {
			/* We only have one lowest point */
			lower_right_index = yMaxIndex[0];
			lower_left_index = yMaxIndex[0];
		}

		else {

			for (unsigned i = 0; i < yMaxIndex.size(); i++) {
				if (refined_poi[yMaxIndex[i]].x > xmax) {
					//	int lower_left_index = -1;
					xmax = refined_poi[yMaxIndex[i]].x;
					lower_right_index = yMaxIndex[i];
				}
				if (refined_poi[yMaxIndex[i]].x < xmin) {
					xmin = refined_poi[yMaxIndex[i]].x;
					lower_left_index = yMaxIndex[i];
				}
			}
		}



		/* lower_right_index now contains the bottom-right most index 	*/
		/* lower_left_index now contains the bottom-left most index 	*/


		/* Find the top most points */

		ymin = 10000;
		ymax = 0;
		xmin = 10000;
		xmax = 0;

		if (yMinIndex.size() == 1) {
			upper_right_index = yMinIndex[0];
			upper_left_index = yMinIndex[0];
		}
		else {
			for (unsigned i = 0; i < yMinIndex.size(); i++) {
				if (refined_poi[yMinIndex[i]].x < xmin) {
					xmin = refined_poi[yMinIndex[i]].x;
					upper_right_index = yMinIndex[i];
				}
				if (refined_poi[yMinIndex[i]].x > xmax) {
					xmax = refined_poi[yMinIndex[i]].x;
					upper_left_index = yMinIndex[i];
				}
			}
		}

		/* upper_right_index now contains the top-right most index		*/
		/* upper_left_index now contains the top-left most index		*/


		int left_top_index = 0;
		int left_bot_index = 0;

		ymin = 10000;
		ymax = 0;

		/* Find the left most points */
		if (xMinIndex.size() == 1) {
			left_top_index = xMinIndex[0];
			left_bot_index = xMinIndex[0];
		}
		else {
			for (unsigned i = 0; i < xMinIndex.size(); i++) {
				if (refined_poi[xMinIndex[i]].y < ymin) {
					ymin = refined_poi[xMinIndex[i]].y;
					left_top_index = xMinIndex[i];
				}
				if (refined_poi[xMinIndex[i]].y > ymax) {
					ymax = refined_poi[xMinIndex[i]].y;
					left_bot_index = xMinIndex[i];
				}
			}
		}

		/* left_top_index now contains the left-top most index		*/
		/* left_bot_index now contains the left-bottom most index	*/

		int right_top_index = 0;
		int right_bot_index = 0;

		ymin = 10000;
		ymax = 0;

		/* Find the right most points */
		if (xMaxIndex.size() == 1) {
			right_top_index = xMaxIndex[0];
			right_bot_index = xMaxIndex[0];
		}
		else {
			for (unsigned i = 0; i < xMaxIndex.size(); i++) {
				if (refined_poi[xMaxIndex[i]].y < ymin) {
					ymin = refined_poi[xMaxIndex[i]].y;
					right_top_index = xMaxIndex[i];
				}
				if (refined_poi[xMaxIndex[i]].y > ymax) {
					ymax = refined_poi[xMaxIndex[i]].y;
					right_bot_index = xMaxIndex[i];
				}
			}
		}



		/* right_top_index now contains the right-top most index	*/
		/* right_bot_index now contains the right-bottom most index	*/

		/*
		 * So now we have up to eight different corners.
		 * We'll see if any of them are the same to reduce the number of elements we're dealing with.
		 */

		Point flag(-1, -1);


		if (upper_left_index != upper_right_index) {
			// We are working with two points along the top, they must be the two top corners
			if (getDistance(refined_poi[upper_left_index], refined_poi[upper_left_index]) >= GOAL_WIDTH) {
				corners[UPPER_LEFT_CORNER] = refined_poi[upper_left_index];
				corners[UPPER_RIGHT_CORNER] = refined_poi[upper_left_index];
			}
			else if ((refined_poi[upper_left_index].x - refined_poi[left_top_index].x) <
					((refined_poi[left_top_index].x - refined_poi[right_top_index].x) / 2)) {
				// We are closer to the left, so we'll assume the leftmost point is the corner
				corners[UPPER_LEFT_CORNER] = refined_poi[upper_left_index];
				corners[UPPER_RIGHT_CORNER] = flag;
			}
			else {
				corners[UPPER_RIGHT_CORNER] = refined_poi[upper_right_index];
				corners[UPPER_LEFT_CORNER] = flag;
			}
		}
		else if ((refined_poi[upper_left_index].x - refined_poi[left_top_index].x) <
				((refined_poi[left_top_index].x - refined_poi[right_top_index].x) / 2)) {
			// The top most point is on the left half, so we can assume it's the top left corner
			corners[UPPER_LEFT_CORNER] = refined_poi[upper_left_index];
			corners[UPPER_RIGHT_CORNER] = flag;
		}
		else {
			// Guess the topmost point is the top right corner
			corners[UPPER_RIGHT_CORNER] = refined_poi[upper_right_index];
			corners[UPPER_LEFT_CORNER] = flag;
		}

		//  If these two are different, and the top left most index doesn't equal the left topmost index
		if ((corners[UPPER_LEFT_CORNER] == flag)) {

			if ((left_top_index != left_bot_index)) {
				// The topmost index is the upper left corner
				corners[UPPER_LEFT_CORNER] = refined_poi[left_top_index];
			}
			// We know where the top right corner is
			if (getDistance(refined_poi[lower_right_index], refined_poi[right_bot_index]) < GOAL_WIDTH) {
				corners[LOWER_RIGHT_FRONT_CORNER] = refined_poi[lower_right_index];
				corners[LOWER_LEFT_BACK_CORNER] = refined_poi[right_bot_index];
			}

			else {

				// Unless MU-L8 is on some crazy tilt, the bottommost index would be the left front corner
				corners[LOWER_LEFT_FRONT_CORNER] = refined_poi[left_bot_index];
				corners[LOWER_LEFT_BACK_CORNER] = flag;
			}

		}
		else {
			if (getDistance(refined_poi[lower_left_index], refined_poi[left_bot_index]) < GOAL_WIDTH) {
				corners[LOWER_LEFT_BACK_CORNER] = refined_poi[left_bot_index];
				corners[LOWER_LEFT_FRONT_CORNER] = refined_poi[lower_left_index];
			}
			else {
				// We know where the upper left corner is, so if we don't know where the right top corner is,
				// we're probably looking at the back corner
				corners[LOWER_LEFT_BACK_CORNER] = refined_poi[left_bot_index];
				corners[LOWER_LEFT_FRONT_CORNER] = flag;
			}
		}

		if (corners[UPPER_RIGHT_CORNER] == flag) {
			if (right_top_index != right_bot_index) {
				corners[UPPER_RIGHT_CORNER] = refined_poi[right_top_index];
			}
			corners[LOWER_RIGHT_FRONT_CORNER] = refined_poi[left_bot_index];
			corners[LOWER_RIGHT_BACK_CORNER] = flag;
		}
		else {
			corners[LOWER_RIGHT_BACK_CORNER] = refined_poi[right_bot_index];
			corners[LOWER_RIGHT_FRONT_CORNER] = flag;
		}
	}

	return corners;
}

std::vector<Point> findIntersections(Rect boundRect, std::vector<std::vector<Point> > contours, int index) {
	std::vector<Point> points_of_interest;
	std::vector<Point> refined_poi;
	std::vector<Point> corners;
	corners.resize(8);

	for (unsigned i = 0; i < corners.size(); i++) {
		corners[i].x = -1;
		corners[i].y = -1;
	}
	points_of_interest.resize(0);


	int x = boundRect.x;
	int y = boundRect.y;
	for (; x < boundRect.x + boundRect.width; x++) {
		// Search for intersection with the bounding rectangle
		if (pointPolygonTest(contours[index], Point(x, y), false) == 0) {
			points_of_interest.resize(points_of_interest.size() + 1);
			points_of_interest[points_of_interest.size() - 1] = Point(x, y);
		}
	}

	int max_jump = 10;

	Point top(-1, -1);
	bool jump = false;
	if (points_of_interest.size() <= 10 && points_of_interest.size() > 0) {
		int avg;
		int sum = points_of_interest[0].x;
		for (unsigned i = 1; i < points_of_interest.size(); i++) {
			if (points_of_interest[i].x - points_of_interest[i-1].x > max_jump) {
				jump = true;
			}
			sum += points_of_interest[i].x;
		}
		avg = sum / points_of_interest.size();
		top.x = avg;
		top.y = y;
	}
	if (jump && points_of_interest.size() > 0) {
		// Loop to see if there are multiple points at the top
		std::vector<int> indexes;
		indexes.push_back(0);
		for (unsigned i = 1; i < points_of_interest.size(); i++) {
			if ((points_of_interest[i].x - points_of_interest[i-1].x) > max_jump) {
				indexes.push_back(i-1);

			}
		}
		indexes.push_back(points_of_interest.size() - 1);
		//		std::cout << "Number of Indexes: " << indexes.size() << std::endl;
		// Now we have a vector containing all the last indexes of jumps

		// If we only have one interval, let's make sure it's not the whole top
		if ((points_of_interest[0].x - points_of_interest[points_of_interest.size()-1].x)
				>= GOAL_WIDTH) {
			refined_poi.push_back(points_of_interest[0]);
			refined_poi.push_back(points_of_interest[points_of_interest.size()]);
		}
		else {
			// Take the average of these new intervals
			for (unsigned i = 0; i < indexes.size()-1; i++) {
				int avg;
				int sum = 0;
				std::cout << "\t" << indexes[i] << " - " << indexes[i+1] << std::endl;
				for (int j = indexes[i]; j <= indexes[i+1]; j++) {
					sum += points_of_interest[j].x;
				}
				//				std::cout << sum << "/" << (indexes[i+1]+1) - indexes[i] << std::endl;
				avg = sum / (indexes[i+1] - indexes[i] + 1);
				refined_poi.push_back(Point(avg, y));
			}
		}
	}

	/* TODO Added */
	if (refined_poi.size() == 1) {
		// We only have one point to deal with
		if ((refined_poi[0].x - boundRect.x) < boundRect.width/2) {
			// Point is the upper left corner
			corners[UPPER_LEFT_CORNER] = refined_poi[0];
		}
		else {
			corners[UPPER_RIGHT_CORNER] = refined_poi[0];
		}
	}
	else if (refined_poi.size() > 1) {
		if (getDistance(refined_poi[0], refined_poi[refined_poi.size()-1]) > GOAL_WIDTH) {
			corners[UPPER_LEFT_CORNER] = refined_poi[0];
			corners[UPPER_RIGHT_CORNER] = refined_poi[refined_poi.size()-1];
		}
		else if ((refined_poi[0].x - boundRect.x) > boundRect.width/2) {
			// RIGHT
			corners[UPPER_RIGHT_CORNER] = refined_poi[refined_poi.size()-1];
		}
		else {
			corners[UPPER_LEFT_CORNER] = refined_poi[0];
		}
	}

	refined_poi.clear();



	//	for (unsigned i = 0; i < points_of_interest.size(); i++) {
	//		std::cout << "(" << points_of_interest[i].x << ", " << points_of_interest[i].y << ")" << std::endl;
	//
	//	}
	//	std::cout << std::endl;

	/* TODO End added */

	points_of_interest.resize(0);
	y = boundRect.y + boundRect.height - 1;
	x = boundRect.x;
	for (; x < boundRect.x + boundRect.width; x++) {
		// Search for intersection with the bounding rectangle
		if (pointPolygonTest(contours[index], Point(x, y), false) == 0) {
			points_of_interest.resize(points_of_interest.size() + 1);
			points_of_interest[points_of_interest.size() - 1] = Point(x, y);
		}
	}

	Point bot(-1, -1);
	if (points_of_interest.size() <= 15 && points_of_interest.size() > 0) {
		int avg;
		int sum = 0;
		for (unsigned i = 0; i < points_of_interest.size(); i++) {
			sum += points_of_interest[i].x;
		}
		avg = sum / points_of_interest.size();
		bot.x = avg;
		bot.y = y;
	}
	else if (points_of_interest.size() > 0) {
		// Loop to see if there are multiple points at the top
		std::vector<int> indexes;
		indexes.push_back(0);
		for (unsigned i = 1; i < points_of_interest.size(); i++) {
			if ((points_of_interest[i].x - points_of_interest[i-1].x) > max_jump) {
				indexes.push_back(i-1);

			}
		}
		indexes.push_back(points_of_interest.size() - 1);
		//		std::cout << "Number of Indexes: " << indexes.size() << std::endl;
		// Now we have a vector containing all the last indexes of jumps

		// If we only have one interval, let's make sure it's not the whole top
		if ((points_of_interest[0].x - points_of_interest[points_of_interest.size()-1].x)
				>= GOAL_WIDTH) {
			refined_poi.push_back(points_of_interest[0]);
			refined_poi.push_back(points_of_interest[points_of_interest.size()]);
		}
		else {
			// Take the average of these new intervals
			for (unsigned i = 0; i < indexes.size()-1; i++) {
				int avg;
				int sum = 0;
				for (int j = indexes[i]; j <= indexes[i+1]; j++) {
					sum += points_of_interest[j].x;
				}
				avg = sum / (indexes[i+1] - indexes[i] + 1);
				refined_poi.push_back(Point(avg, y));
			}
		}
	}

	/* TODO Added */

	if (refined_poi.size() == 1) {
		// We are only dealing with one point
		if ((refined_poi[0].x - boundRect.x) < boundRect.width/2) {
			corners[LOWER_LEFT_FRONT_CORNER] = refined_poi[0];
		}
		else {
			corners[LOWER_RIGHT_FRONT_CORNER] = refined_poi[0];
		}
	}
	else if (refined_poi.size() > 1) {
		if ((refined_poi[refined_poi.size()-1].x- boundRect.x) < boundRect.width/2) {
			// All points are on the left
			corners[LOWER_LEFT_FRONT_CORNER] = refined_poi[0];
		}
		else {
			corners[LOWER_RIGHT_FRONT_CORNER] = refined_poi[refined_poi.size()-1];
		}
	}

	refined_poi.clear();

	//	for (unsigned i = 0; i < points_of_interest.size(); i++) {
	//		std::cout << "(" << points_of_interest[i].x << ", " << points_of_interest[i].y << ")" << std::endl;
	//	}
	//	std::cout << std::endl;

	/* TODO End Added */

	points_of_interest.resize(0);
	y = boundRect.y;
	x = boundRect.x;
	for(; y < boundRect.y + boundRect.height; y++) {
		// Search for intersection with the bounding rectangle
		if (pointPolygonTest(contours[index], Point(x, y), false) == 0) {
			points_of_interest.resize(points_of_interest.size() + 1);
			points_of_interest[points_of_interest.size() - 1] = Point(x, y);
		}
	}

	Point left(-1, -1);
	if (points_of_interest.size() <= 15 && points_of_interest.size() > 0) {
		int avg;
		int sum = 0;
		for (unsigned i = 0; i < points_of_interest.size(); i++) {
			sum += points_of_interest[i].y;
		}
		avg = sum / points_of_interest.size();
		left.x = x;
		left.y = avg;
	}
	else if (points_of_interest.size() > 0) {
		// Loop to see if there are multiple points at the top
		std::vector<int> indexes;
		indexes.push_back(0);
		for (unsigned i = 1; i < points_of_interest.size(); i++) {
			if ((points_of_interest[i].y - points_of_interest[i-1].y) > max_jump) {
				indexes.push_back(i-1);

			}
		}
		indexes.push_back(points_of_interest.size() - 1);
		//		std::cout << "Number of Indexes: " << indexes.size() << std::endl;
		// Now we have a vector containing all the last indexes of jumps

		// If we only have one interval, let's make sure it's not the whole top
		if ((points_of_interest[0].y - points_of_interest[points_of_interest.size()-1].y)
				>= GOAL_WIDTH) {
			refined_poi.push_back(points_of_interest[0]);
			refined_poi.push_back(points_of_interest[points_of_interest.size()-1]);
		}
		else {
			// Take the average of these new intervals
			for (unsigned i = 0; i < indexes.size()-1; i++) {
				int avg;
				int sum = 0;
				for (int j = indexes[i]; j <= indexes[i+1]; j++) {
					sum += points_of_interest[j].y;
				}
				avg = sum / (indexes[i+1] - indexes[i] + 1);
				refined_poi.push_back(Point(x, avg));
			}
		}
	}

	/* TODO Added */

	if (refined_poi.size() == 1) {
		if ((refined_poi[0].y - boundRect.y) < boundRect.height/2) {
			corners[UPPER_LEFT_CORNER] = refined_poi[0];
		}
		else {
			corners[LOWER_LEFT_BACK_CORNER] = refined_poi[0];
		}
	}
	else if (refined_poi.size() > 1) {
		if ((refined_poi[refined_poi.size()-1].y - boundRect.y) < boundRect.height/2) {
			corners[UPPER_LEFT_CORNER] = refined_poi[0];
		}
		else {
			corners[LOWER_LEFT_BACK_CORNER] = refined_poi[refined_poi.size()-1];
		}
	}

	refined_poi.clear();

	//	for (unsigned i = 0; i < points_of_interest.size(); i++) {
	//		std::cout << "(" << points_of_interest[i].x << ", " << points_of_interest[i].y << ")" << std::endl;
	//	}
	//	std::cout << std::endl;

	/* TODO End Added */
	points_of_interest.resize(0);
	x = boundRect.x + boundRect.width - 1;
	y = boundRect.y;
	for (; y < boundRect.y + boundRect.height; y++) {
		// Search for intersection with the bounding rectangle
		if (pointPolygonTest(contours[index], Point(x, y), false) == 0) {
			points_of_interest.resize(points_of_interest.size() + 1);
			points_of_interest[points_of_interest.size() - 1] = Point(x, y);
		}
	}

	Point right(-1, -1);
	if (points_of_interest.size() <= 15 && points_of_interest.size() > 0) {
		int avg;
		int sum = 0;
		for (unsigned i = 0; i < points_of_interest.size(); i++) {
			sum += points_of_interest[i].y;
		}
		avg = sum / points_of_interest.size();
		right.x = x;
		right.y = avg;
	}
	else if (points_of_interest.size() > 0) {
		// Loop to see if there are multiple points at the top
		std::vector<int> indexes;
		indexes.push_back(0);
		for (unsigned i = 1; i < points_of_interest.size(); i++) {
			if ((points_of_interest[i].y - points_of_interest[i-1].y) > max_jump) {
				indexes.push_back(i-1);

			}
		}
		indexes.push_back(points_of_interest.size() - 1);
		//		std::cout << "Number of Indexes: " << indexes.size() << std::endl;
		//		for (unsigned i = 0; i < points_of_interest.size(); i++) {
		//			std::cout << "(" << points_of_interest[i].x << ", " << points_of_interest[i].y << ")" << std::endl;
		//		}
		//		std::cout << std::endl;

		// Now we have a vector containing all the last indexes of jumps

		// If we only have one interval, let's make sure it's not the whole top
		if ((points_of_interest[0].y - points_of_interest[points_of_interest.size()-1].y)
				>= GOAL_WIDTH) {
			refined_poi.push_back(points_of_interest[0]);
			refined_poi.push_back(points_of_interest[points_of_interest.size()]);
		}
		else {
			// Take the average of these new intervals
			for (unsigned i = 0; i < indexes.size()-1; i++) {
				int avg;
				int sum = 0;
				for (int j = indexes[i]; j <= indexes[i+1]; j++) {
					sum += points_of_interest[j].y;
				}
				avg = sum / (indexes[i+1] - indexes[i] + 1);
				refined_poi.push_back(Point(x, avg));
			}
		}
	}

	/* TODO Added */
	if (refined_poi.size() == 1) {
		if ((refined_poi[0].y - boundRect.y) < boundRect.height/2) {
			corners[UPPER_RIGHT_CORNER] = refined_poi[0];
		}
		else {
			corners[LOWER_RIGHT_BACK_CORNER] = refined_poi[0];
		}
	}
	else if (refined_poi.size() > 1) {
		if ((refined_poi[refined_poi.size()-1].y - boundRect.y) < boundRect.height/2) {
			corners[UPPER_RIGHT_CORNER] = refined_poi[0];
		}
		else {
			corners[LOWER_RIGHT_BACK_CORNER] = refined_poi[refined_poi.size()-1];
		}
	}

	/* TODO End Added */

	//	std::cout << "Refined corners size: " << refined_poi.size() << std::endl;
	//	for (unsigned i = 0; i < refined_poi.size(); i++) {
	//
	//		std::cout << i << ": (" << refined_poi[i].x << ", " << refined_poi[i].y << ")" << std::endl;
	//	}
	points_of_interest.resize(0);
	refined_poi.clear();
	//std::vector<Point> result = determineCorners(refined_poi);
	//	Point top;
	//	Point left;
	//	Point bot;
	//	Point right;
	//	result[LEFT] = left;
	//	result[RIGHT] = right;
	//	result[TOP] = top;
	//	result[BOT] = bot;

	return corners;
}

double getLength(double distance, double pixels) {
	double length = (distance+1.653881);
	length /= 1332.565623;
	length *= pixels;
	return length;

}

double getTheta(double known, double observed) {
	double theta = asin(observed/known);
	return (90 - toDegrees(theta));
}

double long getDistanceFromCamera(double cm_per_pixel) {
	double long result = 0;
	double x = cm_per_pixel;
	/* Quartic Approximation
	result += 221726.739595*(x*x*x*x);
	result -= 107994.521049*(x*x*x);
	result += 18077.65535*(x*x);
	result += 107.892022*(x);
	result += 25.917722;*/

	/* Quadratic Equation */
	result += 1332.565623*x;
	result -= 1.653881;

	return result;
}

double getDistanceToCenter(double known, double observed, double d, double angle_from_center) {
	double bigTheta = 180 - toDegrees(asin(toRadians(observed/d))) - getTheta(known, observed);
	double result = d;
	result *= toDegrees(sin(toRadians(bigTheta)));
	result /= toDegrees(sin(toRadians(angle_from_center)));
	return result;
}

double getAngleFromCenter(double known, double observed, double d) {
	double bigTheta = 180 - toDegrees(asin(toRadians(observed/d))) - getTheta(known, observed);
	std::cout << "Angle from back post: " << bigTheta-90 << std::endl;
	double g = GOAL_WIDTH/2;
	double result = sin(toRadians(180-bigTheta))/((g/d) + cos(toRadians(180 - bigTheta)));
	result = toDegrees(atan(result));
	return (/*90 - */result);
}

//Detect the largest red objects location in the webcam view
double detectBallDistance(Mat img){

	//	Mat src_gray;
	//	// Convert it to gray
	//	cvtColor( img, src_gray, CV_BGR2GRAY );
	//
	//	// Reduce the noise so we avoid false circle detection
	//	medianBlur( src_gray, src_gray, kernel_size );
	//
	//	//equalizeHist(src_gray, src_gray);
	//
	//	std::vector<Vec3f> circles;
	//
	//	Canny(src_gray, src_gray, 21/*thresh1/*45*/, 115/*thresh2/*410*/);
	//
	////	std::vector<Mat> BGR;
	////	split(img, BGR);
	//
	//
	//	// Apply the Hough Transform to find the circles
	//	HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 1000/*thresh1*/, 36/*thresh2*/, 1, 75 );
	//
	//	std::cout << "Circles size: " << circles.size() << std::endl;
	//
	//
	//	Mat mask = Mat::zeros(src_gray.rows, src_gray.cols, CV_8UC1);
	//	// Draw the circles detected
	//
	//	if (circles.size() > 3) {
	//		rectangle(mask, Point (0, 120), Point (640, 480), Scalar(255, 255, 255), -1);
	//		Mat temp;
	//		src_gray.copyTo(temp, mask);
	//		src_gray = temp.clone();
	//		~temp;
	//		mask.zeros(mask.rows, mask.cols, CV_8UC1);
	//		Canny(src_gray, src_gray, 21/*thresh1/*45*/, 115/*thresh2/*410*/);
	//
	//
	//		// Apply the Hough Transform to find the circles
	//		HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 1000/*thresh1*/, 36/*thresh2*/, 1, 75 );
	//	}
	//
	//
	//	for( size_t i = 0; i < circles.size(); i++ )
	//	{
	//		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
	//		int radius = cvRound(circles[i][2]);
	//		// circle center
	//		circle( img, center, 3, Scalar(0,255,0), -1, 8, 0 );
	//		// circle outline
	//		circle( img, center, radius, Scalar(0,0,255), 3, 8, 0 );
	//		Vec3b c = img.at<Vec3b>(center.x, center.y);
	//		if (c[0] >= 200 && c[1] < 0xA5) {
	//			circle(mask, center, radius, Scalar(255, 255, 255), -1, 8, 0);
	//		}
	//	}
	//	Mat dst;
	//	src_gray.copyTo(dst, mask);
	//
	//	namedWindow("Grayscale", WINDOW_NORMAL);
	//	imshow("Grayscale", src_gray);
	//	namedWindow("Mask", WINDOW_NORMAL);
	//	imshow("Mask", dst);
	//	imshow("Circle Detect", img);
	int largest_area=90;
	int largest_contour_index=0;
	//	int moveBody = 0;
	Rect boundRect;

	Mat imgHSV;
	cvtColor(img, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV

	// Get Image Thresh
	//Set the thershold color (ball color) for image tracking
	Mat imgThresh;
	inRange(img, Scalar(/*lowerH/*/0/*9*/,/*lowerS/*/31/*0*/, /*lowerV/*/158/*72*/), Scalar(/*upperH/*/256/*207*/, /*upperS/*/108/*256*/, /*upperV/*/256/*226*/), imgThresh);

	medianBlur(imgThresh, imgThresh, kernel_size);

	vector<vector <Point> > contours; // Vector for storing contour
	vector<Vec4i> hierarchy;
	findContours(imgThresh, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE ); // Find the contours in the image

	std::vector<Vec3f> circles;


	HoughCircles( imgThresh, circles, CV_HOUGH_GRADIENT, 1, imgThresh.rows/8, 1000/*thresh1*/, 36/*thresh2*/, 1, 75 );


	for( unsigned i = 0; i< contours.size(); i++ ) // iterate through each contour.
	{
		double a=contourArea( contours[i],false);  //Find the area of contour

		if(a>largest_area){
			largest_area=a; //Reset largest area
			largest_contour_index=i;  //Store the index of largest contour
			boundRect=boundingRect(contours[i]); // Find the bounding rectangle for biggest contour

			// For drawing image
			//			drawContours(img, contours, i, Scalar(255,255,255), CV_FILLED, 8, hierarchy);
		}

	}

	for( size_t i = 0; i < circles.size(); i++ )
	{
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		// circle center
		circle( img, center, 3, Scalar(0,255,0), -1, 8, 0 );
		// circle outline
		circle( img, center, radius, Scalar(0,0,255), 3, 8, 0 );
	}

	//	drawContours(img, contours, largest_contour_index, Scalar(0, 255, 255), CV_FILLED, 8, hierarchy);

	Point centerRec = Point((boundRect.x + boundRect.width/2), (boundRect.y + boundRect.height/2));
	circle(img, cvPoint(centerRec.x,centerRec.y), boundRect.width/2, CV_RGB(255,0,0), -1, 8, 0 );

	Scalar color(255,255,255);

	imshow("Circle Detect", img);
	//        drawContours( imgThresh, contours,largest_contour_index, color, CV_FILLED, 1, hierarchy ); // Draw the largest contour using previously stored index.

	// TODO For showing image
	//      imshow("imgThreshMat", imgThreshMat);
	//      rectangle(imgMat, boundRect, Scalar(255,255,255), 3, 8, 0);

	//      cvClearMemStorage(storage);



	//return the grid position the ball is in to main.
	double pixels = getDistance(Point(boundRect.x, boundRect.y), Point(boundRect.x + boundRect.width, boundRect.y));
	double distance = DOUBLE_FLAG;
	if (pixels > 0 && pixels < 800) {
		distance = getDistanceFromCamera(BALL_DIAMETER / pixels);
	}
	return distance;
}

RotatedRect processImage(Mat &frame, Mat &preprocessedImg, Mat &imgDraw) {

	std::vector<std::vector<Point> > contours;
	std::vector<Vec4i> heirarchy;
	imgDraw = preprocessedImg.clone();
	findContours(preprocessedImg, contours, heirarchy, RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	//		dilate(canny.clone(), canny, lowerH);
	//				erode(canny.clone(), canny, upperH);
	for (unsigned i = 0; i < contours.size(); i++) {
		drawContours(imgDraw, contours, i, Scalar(200, 255, 0), 2, 8, heirarchy, 0, Point());
	}

	//		findContours(imgDraw, contours, heirarchy, RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
	//
	//		for (unsigned i = 0; i < contours.size(); i++) {
	//			drawContours(imgDraw, contours, i, Scalar(200, 255, 0), 2, 8, heirarchy, 0, Point());
	//		}

	//		std::vector<cv::Vec4i> lines;
	//		HoughLinesP(imgDraw, lines, 3, CV_PI/180, 100, 75, 3);
	//
	//		for (unsigned i = 0; i < lines.size(); i++)
	//		{
	//			Vec4i l = lines[i];
	//			line( /*imgDraw*/frame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,255,255), 3, CV_AA);
	//		}

	findContours(imgDraw, contours, heirarchy, RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	int largest_area = 1500;
	int largest_contour_index = -1;
	RotatedRect boundRect(Point(-1,-1), Size(1,1), 45);
	//std::cout << "Number of contours found: " << contours.size() << std::endl;

	for( unsigned i = 0; i< contours.size(); i++ ) // iterate through each contour.
	{
		double a=contourArea( contours[i],false);  //Find the area of contour

		//std::cout << "Area of contour " << i << ": " << a << std::endl;
		if(a>largest_area){
			largest_area=a; //Reset largest area1
			largest_contour_index=i;  //Store the index of largest contour
			boundRect=minAreaRect(contours[i]); // Find the bounding rectangle for biggest contour

			// For drawing image
			//				std::cout << "Found new largest contour: " << a << std::endl;
		}
		drawContours(imgDraw, contours, i, Scalar(200, 255, 0), 2, 8, heirarchy, 0, Point());
	}

	return boundRect;

}

void determinePosition(RotatedRect boundRect, Mat &imgDraw, Mat &frame){
	std::vector<Point> corners;
	// rotated rectangle
	Point2f rect_points[4]; boundRect.points( rect_points );
	for( int j = 0; j < 4; j++ ) {
		for (int k = rect_points[j].x-5; k <= rect_points[j].x+5; k++) {
			for (int l = rect_points[j].y-5; l <= rect_points[j].y+5; l++) {
				if (imgDraw.at<unsigned char>(Point(k, l)) > 175) {
					if (corners.size() == 0) {
						corners.push_back(rect_points[j]);
					}
					else if (!(corners[corners.size()-1].x == rect_points[j].x && corners[corners.size()-1].y == rect_points[j].y)) {
						//							if (corners.size() > 0 && !((corners[corners.size()-1].x == rect_points[j].x) && (corners[corners.size()-1].y == rect_points[j].y))) {
						corners.push_back(rect_points[j]);
					}
					//							else if (corners.size() == 0) {
					//								corners.push_back(rect_points[j]);
					//							}
				}
			}
		}
	}
	std::vector<Point> refined;
	if (corners.size() > 0) {
		refined.push_back(corners[0]);
	}

	for (unsigned i = 1; i < corners.size(); i++) {
		if (corners[i-1].x != corners[i].x && corners[i-1].y != corners[i].y) {
			refined.push_back(corners[i]);
		}
	}
	Point center = boundRect.center;
	int left_bot = -1;
	int left_top = -1;
	int right_bot = -1;
	int right_top = -1;
	for (int i = 0; i < 4; i++) {
		//line( imgDraw, rect_points[i], rect_points[(i+1)%4], Scalar(255, 0, 0) , 1, 8 );
		if (rect_points[i].x < center.x) {
			if (rect_points[i].y < center.y) {
				// Top left
				left_top = i;
				rect_points[left_top].x = (int) (rect_points[left_top].x + 0.5);
				rect_points[left_top].y = (int) (rect_points[left_top].y + 0.5);
				//						std::cout << "Top left = " << i << std::endl;
				//						std::cout << "\t(" << rect_points[left_top].x << ", " << rect_points[left_top].y << ")" << std::endl;
			}
			else {
				// Bottom Left
				left_bot = i;
				rect_points[left_bot].x = (int) (rect_points[left_bot].x + 0.5);
				rect_points[left_bot].y = (int) (rect_points[left_bot].y + 0.5);
				//						std::cout << "Bottom left = " << i << std::endl;
				//						std::cout << "\t(" << rect_points[left_bot].x << ", " << rect_points[left_bot].y << ")" << std::endl;
			}
		}
		else {
			if (rect_points[i].y < center.y) {
				// Top right
				right_top = i;
				rect_points[right_top].x = (int) (rect_points[right_top].x + 0.5);
				rect_points[right_top].y = (int) (rect_points[right_top].y + 0.5);
				//						std::cout << "Top Right = " << i << std::endl;
				//						std::cout << "\t(" << rect_points[right_top].x << ", " << rect_points[right_top].y << ")" << std::endl;
			}
			else {
				// Bottom right
				right_bot = i;
				rect_points[right_bot].x = (int) (rect_points[right_bot].x + 0.5);
				rect_points[right_bot].y = (int) (rect_points[right_bot].y + 0.5);
				//						std::cout << "Bottom right = " << i << std::endl;
				//						std::cout << "\t(" << rect_points[right_bot].x << ", " << rect_points[right_bot].y << ")" << std::endl;
			}
		}
	}



	int top_left_index = -1;
	int top_right_index = -1;
	int bot_left_index = -1;
	int bot_right_index = -1;
	//			std::cout << "Size of corners vector: " << refined.size() << std::endl;
	for (unsigned i = 0; i < refined.size(); i++) {
		//				std::cout << "(" << refined[i].x << ", " << refined[i].y << ")" << std::endl;
		if (refined[i].x == rect_points[left_top].x && refined[i].y == rect_points[left_top].y) {
			top_left_index = i;
			//					std::cout << "Top Left Index = " << i << std::endl;
		}
		else if (refined[i].x == rect_points[right_top].x && refined[i].y == rect_points[right_top].y) {
			top_right_index = i;
			//					std::cout << "Top Right Index = " << i << std::endl;
		}
		else if (refined[i].x == rect_points[left_bot].x && refined[i].y == rect_points[left_bot].y) {
			bot_left_index = i;
			//					std::cout << "Bottom Left Index = " << i << std::endl;
		}
		else if (refined[i].x == rect_points[right_bot].x && refined[i].y == rect_points[right_bot].y) {
			bot_right_index = i;
			//					std::cout << "Bottom Right Index = " << i << std::endl;
		}
		circle(/*imgDraw*/frame, refined[i], 3, Scalar(255/2*i, 255/2*i, 0), 2, 8, 0);
	}

	if (top_left_index >= 0)
		std::cout << "Top Left: (" << refined[top_left_index].x << ", " << refined[top_left_index].y << ")" << std::endl;
	if (top_right_index >= 0)
		std::cout << "Top right: (" << refined[top_right_index].x << ", " << refined[top_right_index].y << ")" << std::endl;
	if (bot_left_index >= 0)
		std::cout << "Bottom Left: (" << refined[bot_left_index].x << ", " << refined[bot_left_index].y << ")" << std::endl;
	if (bot_right_index >= 0)
		std::cout << "Bottom Right: (" << refined[bot_right_index].x << ", " << refined[bot_right_index].y << ")" << std::endl;

	int known_points = (int)refined.size();
	double distance_from_cam = -1;
	double observed_length = -1;
	double theta = -100;
	//			double distance_center = -1;
	if (top_left_index >= 0 && bot_left_index >= 0) {
		distance_from_cam = getDistanceFromCamera(GOAL_HEIGHT/getDistance(refined[top_left_index], refined[bot_left_index]));
		if (top_right_index >= 0) {
			observed_length = getLength(distance_from_cam, getDistance(refined[top_left_index], refined[top_right_index]));
			theta = -1 * getAngleFromCenter(GOAL_WIDTH, observed_length, distance_from_cam);
		}
	}
	else if (top_right_index >= 0 && bot_right_index >= 0) {
		distance_from_cam = getDistanceFromCamera(GOAL_HEIGHT/getDistance(refined[top_right_index], refined[bot_right_index]));
		if (top_left_index >= 0) {
			observed_length = getLength(distance_from_cam, getDistance(refined[top_left_index], refined[top_right_index]));
			theta = getAngleFromCenter(GOAL_WIDTH, observed_length, distance_from_cam);
		}
	}

	/* Try even rougher estimations */
	double distance_from_cam_2 = getDistanceFromCamera(GOAL_HEIGHT/getDistance(rect_points[left_top], rect_points[left_bot]));
	double observed_length2 = getLength(distance_from_cam_2, getDistance(rect_points[left_top], rect_points[right_top]));
	double theta2 = getAngleFromCenter(GOAL_WIDTH, observed_length2, distance_from_cam_2);
	distance[distance_index] = distance_from_cam_2;
	distance_confidence[distance_index] = known_points/4.0;
	distance_index = (distance_index + 1) % history_size;
	angle[angle_index] = theta2;
	angle_confidence[angle_index] = known_points/4.0;
	angle_index = (angle_index + 1) % history_size;

//	char text[255];
//	sprintf(text, "Distance to goal: %fm", (distance_from_cam_2/100.0));
//	putText(frame, text, Point(10, 200), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(200, 200, 250), 1, CV_AA);
//	sprintf(text, "Angle from goal: %fm", (theta2));
//	putText(frame, text, Point(10, 240), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(200, 200, 250), 1, CV_AA);



	if (distance_from_cam > 0) {
		std::cout << "Distance from camera to near post: " << distance_from_cam/100.0 << "m" << std::endl;
		distance[distance_index-1] = distance_from_cam;
		distance_confidence[distance_index-1] = (known_points == 3) ? 0.75 : 0.25;
	}
	if (theta > -100) {
		std::cout << "Angle from goal: " << theta << "°" << std::endl;
		angle[angle_index-1] = theta;
		angle_confidence[angle_index-1] = (known_points == 3) ? 0.75 : 0.25;
	}
	//			if (distance_from_cam > 0 && theta > -100) {
	//				distance_center = getDistanceToCenter(GOAL_WIDTH, observed_length, distance_from_cam, theta);
	//				distance_from_cam = distance_center;
	//				distance[distance_index-1] = distance_from_cam;
	//			}

	imshow("Ball Detect", imgDraw);
}

double averageOf(double vec[], int size) {
	double sum = 0;
	int count = 0;
	for (int i = 0; i < size; i++) {
		if (vec[i] != 0.0) {
			sum += vec[i];
			count++;
		}
	}
	double avg = sum/((double)count);
	return avg;
}

Point convertToPolar(double distance, double theta) {
	double y = sin(toRadians(theta)) * distance;
	double x = cos(toRadians(theta)) * distance;
	return Point(x, y);
}

double weightedAverageOf(double vec[], double confidence[], int size) {
	double sum = 0;
	double count = 0;
	for (int i = 0; i < size; i++) {
		sum += confidence[i]*vec[i];
		count += confidence[i];
	}
	double avg = sum/(count);
	return avg;
}

/** @function main */
int main ( int argc, char** argv )
{
	VideoCapture cap = setup();

	bool continuous = true;
	bool flipped = false;

	double distance_to_goal = averageOf(distance, history_size);
	double confidence_distance_to_goal = weightedAverageOf(distance, distance_confidence, history_size);
	double angle_from_goal = averageOf(angle, history_size);
	double confidence_angle_from_goal = weightedAverageOf(angle, angle_confidence, history_size);

	Point position = convertToPolar(confidence_distance_to_goal, confidence_angle_from_goal);

	std::cout << "Would you like to go frame by frame? (Y/N) ";
	string input;
	std::cin >> input;
	if (input == "Y" || input == "y") {
		continuous = false;
	}

	int c;

	Mat frame;
	cap.read(frame);

	for (int i = 0; i < history_size; i++) {
		distance[i] = 0;
		angle[i] = 0;
		distance_confidence[i] = 0;
		angle_confidence[i] = 0;
	}

	clock_t timer;

	/// Initialize arguments for the filter
	//	//  anchor = Point( -1, -1 );
	//	Mat imgHSV;
	//	cvtColor(src, imgHSV, CV_BGR2HSV);
	//	medianBlur(imgHSV, imgHSV, kernel_size);

	/// Loop - Will filter the image with different kernel sizes each 0.5 seconds
	while( cap.isOpened() )
	{
		bool ballFound = false;
		bool goalFound = false;
		c = waitKey(80);
		/// Press 'ESC' to exit the program
		if( (char)c == 27 )
		{ break; }
		else if (continuous == true ||(char)c == 'n') {

			//			std::cout /*<< "One Iteration" */<< std::endl;
			if (!cap.read(frame)) {
				std::cout << "I broke!" << std::endl;
				break;
			}
			if (flipped) {
				flip(frame, frame, 0);
			}
		}

		double distance_to_ball = detectBallDistance(frame);

		if (distance_to_ball != DOUBLE_FLAG) {
			ballFound = true;
			std::cout << "Distance to ball: " << distance_to_ball << std::endl;
		}

		Mat preprocessedImg;
		//		NoBlue = RGB[0] + RGB[1];

		// Threshold the image and detect edges
		preprocessedImg = preprocess(frame);

		Mat imgDraw;
		RotatedRect boundRect = processImage(frame, preprocessedImg, imgDraw);

		if (boundRect.center.x >= 0) {
			goalFound = true;
			determinePosition(boundRect, imgDraw, frame);
			distance_to_goal = averageOf(distance, history_size);
			confidence_distance_to_goal = weightedAverageOf(distance, distance_confidence, history_size);
			angle_from_goal = averageOf(angle, history_size);
			confidence_angle_from_goal = weightedAverageOf(angle, angle_confidence, history_size);

			position = convertToPolar(confidence_distance_to_goal, confidence_angle_from_goal);
		}


		//		imshow( "Draw" , imgThresh );
		//		imshow( "Canny", canny );
		//		imshow( "Frame", frame );
		//		imshow( "ImgHSV", imgHSV );
		char text[255];
		sprintf(text, "Distance to goal: %fm    Confidence Distance: %fm", (distance_to_goal/100.0), (confidence_distance_to_goal/100.0));
		putText(frame, text, Point(10, 400), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(200, 200, 250), 1, CV_AA);
		sprintf(text, "Angle from goal: %f    Confidence Angle: %f", angle_from_goal, confidence_angle_from_goal);
		putText(frame, text, Point(10, 420), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(200, 200, 250), 1, CV_AA);
		sprintf(text, "%f fps", (1/(((float)clock()-timer)/CLOCKS_PER_SEC)));
		timer = clock();
		putText(frame, text, Point(10, 460), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(200, 200, 250), 1, CV_AA);
		sprintf(text, "Coordinates: (%d, %d)", position.x, position.y);
		putText(frame, text, Point(10, 20), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(0, 255, 255), 1, CV_AA);


		imshow( "Ball", /*imgDraw*/ /*grey*/ /*preprocessedImg*/ frame);

		if ( (char)c == 's') {
			//imwrite("/home/unicorn/Pictures/test.png", imgDraw);
		}
		if ( (char)c == 'f') {
			flipped = !(flipped);
		}
	}

	//	std::cout << "Distance from camera = " << getDistanceFromCamera(90/389.391) << std::endl;

	return 0;
}


void findCornersAndStuff() {
	//corners = findIntersections(boundRect, contours, largest_contour_index);


	//
	//		std::cout << "Points of interest:" << std::endl;
	//		std::cout << "\t Top: \t(" << intersections[TOP].x << ", " << intersections[TOP].y << ")" << std::endl;
	//		std::cout << "\t Left: \t(" << intersections[LEFT].x << ", " << intersections[LEFT].y << ")" << std::endl;
	//		std::cout << "\t Right: \t(" << intersections[RIGHT].x << ", " << intersections[RIGHT].y << ")" << std::endl;
	//		std::cout << "\t Bottom:\t(" << intersections[BOT].x << ", " << intersections[BOT].y << ")" << std::endl;


	//		int validInput = 0;
	//
	//		for (unsigned i = 0; i < corners.size(); i++) {
	//			Point poi = corners[i];
	//			std::cout << "Point of interest: (" << poi.x << ", " << poi.y << ")" << std::endl;
	//			circle(/*imgDraw*/frame, poi, 3, Scalar(255, 255, 0), 2, 8, 0);
	//			if (poi.x >= 0 || poi.y >= 0) {
	//				validInput++;
	//			}
	//		}
	//		if (validInput >= 3) {
	//
	//			if (corners[UPPER_LEFT_CORNER].x >= 0 && corners[UPPER_RIGHT_CORNER].x >= 0) {
	//				std::cout << "Top Left to Top Right" << std::endl;
	//				double length = getDistance(corners[UPPER_LEFT_CORNER], corners[UPPER_RIGHT_CORNER]);
	//				double long distance_from_cam = getDistanceFromCamera(GOAL_WIDTH/length);
	//
	//				std::cout << "Distance to goal: " << distance_from_cam/100.0 << "m" << std::endl << std::endl;
	//			}
	//			else if (corners[UPPER_LEFT_CORNER].x >= 0 && corners[LOWER_LEFT_FRONT_CORNER].x >= 0) {
	//				std::cout << "Top Left to Bottom Left Front" << std::endl;
	//				double length = getDistance(corners[UPPER_LEFT_CORNER], corners[LOWER_LEFT_FRONT_CORNER]);
	//				double long distance_from_cam = getDistanceFromCamera(GOAL_HEIGHT/length);
	//
	//				std::cout << "Distance to goal: " << distance_from_cam/100.0 << "m " << std::endl << std::endl;
	//			}
	//			else if (corners[UPPER_RIGHT_CORNER].x >= 0 && corners[LOWER_RIGHT_FRONT_CORNER].x >= 0) {
	//				std::cout << "Top Right to Bottom Right Front" << std::endl;
	//				double length = getDistance(corners[UPPER_RIGHT_CORNER], corners[LOWER_RIGHT_FRONT_CORNER]);
	//				double long distance_from_cam = getDistanceFromCamera(GOAL_HEIGHT/length);
	//
	//				std::cout << "Distance to goal: " << distance_from_cam/100.0 << "m " << std::endl << std::endl;
	//			}
	//			else if (corners[UPPER_LEFT_CORNER].x >= 0 && corners[LOWER_LEFT_BACK_CORNER].x >= 0) {
	//				std::cout << "Top Left to Bottom Left Back" << std::endl;
	//				double length = getDistance(corners[UPPER_LEFT_CORNER], corners[LOWER_LEFT_BACK_CORNER]);
	//				double long distance_from_cam = getDistanceFromCamera(sqrt(GOAL_HEIGHT*GOAL_HEIGHT + GOAL_DEPTH*GOAL_DEPTH)/length);
	//
	//				std::cout << "Distance to goal: " << distance_from_cam/100.0 << "m " << std::endl << std::endl;
	//			}
	//			else if (corners[UPPER_RIGHT_CORNER].x >= 0 && corners[LOWER_RIGHT_BACK_CORNER].x >= 0) {
	//				std::cout << "Top Right to Bottom Right Back" << std::endl;
	//				double length = getDistance(corners[UPPER_RIGHT_CORNER], corners[LOWER_RIGHT_BACK_CORNER]);
	//				double long distance_from_cam = getDistanceFromCamera(sqrt(GOAL_HEIGHT*GOAL_HEIGHT + GOAL_DEPTH*GOAL_DEPTH)/length);
	//
	//				std::cout << "Distance to goal: " << distance_from_cam/100.0 << "m " << std::endl << std::endl;
	//			}
	//			else if (corners[LOWER_LEFT_FRONT_CORNER].x >= 0 && corners[LOWER_LEFT_BACK_CORNER].x >= 0) {
	//				std::cout << "Bottom Left Front to Bottom Left Back" << std::endl;
	//				double length = getDistance(corners[LOWER_LEFT_FRONT_CORNER], corners[LOWER_LEFT_BACK_CORNER]);
	//				double long distance_from_cam = getDistanceFromCamera(GOAL_DEPTH/length);
	//
	//				std::cout << "Distance to goal: " << distance_from_cam/100.0 << "m " << std::endl << std::endl;
	//			}
	//			else if (corners[LOWER_RIGHT_FRONT_CORNER].x >= 0 && corners[LOWER_RIGHT_BACK_CORNER].x >= 0) {
	//				std::cout << "Bottom Right Front to Bottom Right Back" << std::endl;
	//				double length = getDistance(corners[LOWER_RIGHT_FRONT_CORNER], corners[LOWER_RIGHT_BACK_CORNER]);
	//				double long distance_from_cam = getDistanceFromCamera(GOAL_DEPTH/length);
	//
	//				std::cout << "Distance to goal: " << distance_from_cam/100.0 << "m " << std::endl << std::endl;
	//			}
	//
	//		}
}
