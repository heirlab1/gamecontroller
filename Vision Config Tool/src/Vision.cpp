/*
 * Vision.cpp
 *
 *  Created on: Jun 20, 2014
 *      Author: Kellen Carey
 */

#include "Vision.h"
//#include "dynamixel.h"
#include <cmath>

bool visionDisplayEnabled = false;

int vis_frequency = 0;
int max_frequency = 1000;

int searching_speed = 125;

int history_size = 8;

int lowerH=22;//13;//15;//5;//15;//15;//11;//20;//10;//18;
int lowerS=115;//77;//160;//202;//134;//202;//165;//100;//227;//158;
int lowerV=146;//172;//83;//255;//144;//255;//145;//100;//141;//80;

int ballLowerH = 7;//115;//0;//158;//0;//157;//148;//0;//0;//0;//0;//0;
int ballLowerS = 160;//145;//177;//72;//218;//133;//58;//187;//137;//189;//31;//0;
int ballLowerV = 209;//144;//194;//102;//96;//85;//190;//164;//21;//158;//0;
// BALL LOWER
//0
//189
//21

// BALL UPPER
//13
//255
//255

bool headLeft;
bool headRight;
bool headUp;
bool headDown;



int thresh1 = 5;
int thresh2 = 5;
int thresh3 = 5;

int motorTime = 2;
int vision = 8;

int upperH=42;//60;//47;//47;//47;//47;//30;//30;//31;//23;
int upperS=255;//255;//255;//255;//255;//255;//250;//255;//256;//220;
int upperV=255;//255;//255;//255;//175;//255;//255;//255;//256;//178;

int ballUpperH = 16;//255;//83;//203;//5;//187;//255;//13;//21;//13;//256;//255;
int ballUpperS = 255;//255;//255;//255;//253;//192;//255;//255;//256;//255;//108;//255;
int ballUpperV = 255;//255;//255;//255;//222;//141;//255;//255;//241;//255;//256;//255;

int kernel_size_erode = 4;

int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
const char* window_name = "Edge Map";


int pixels_to_black_out = 0;


int action = WAIT;

double distance[8];
double distance_confidence[8];
double angle[8];
double angle_confidence[8];
double ball_distance[8];
int ball_index = 0;
int distance_index = 0;
int angle_index = 0;
double position_confidence[8];

double thisRobotTheta = 0;

bool ballPositionKnown = false;
bool goalPositionKnown = false;

double distance_to_goal;
double confidence_distance_to_goal;
double angle_from_goal;
double confidence_angle_from_goal;

Point position;
VideoCapture cap;
double timer;

bool Vision::motorControllerIsExecuting = false;

//MotorController Vision::motorController;


double Vision::getUnixTime() {
	struct timespec tv;

	if (clock_gettime(CLOCK_REALTIME, &tv) != 0) {
		return 0;
	}

	return (((double)tv.tv_sec) + (tv.tv_nsec / 1000000000.0));
}

Vision::Vision() {
	motorController = NULL;
	position.x = 0;
	position.y = 0;
}

void Vision::init(MotorController& controller) {
	motorController = &controller;
	cap = setup();
	setwindowSettings();
//	calibrateThresholds();

	// Initialize the arrays
	for (int i = 0; i < history_size; i++) {
		distance[i] = 0;
		angle[i] = 0;
		distance_confidence[i] = 0;
		angle_confidence[i] = 0;
		ball_distance[i] = 0;
	}

	// Initialize movement records
	// Used when searching for the ball or goal
	headLeft = false;
	headRight = false;
	headUp = false;
	headDown = false;

	// Initialize where MUL8 thinks it is to zero
	distance_to_goal = DOUBLE_FLAG;
	confidence_distance_to_goal = DOUBLE_FLAG;
	angle_from_goal = DOUBLE_FLAG;
	confidence_angle_from_goal = DOUBLE_FLAG;
	position = Point(0, 0);
}

Vision::~Vision() {
	// Release the capture
	cap.release();
}

Mat Vision::grabFrame() {
	Mat frame;
	cap.read(frame);
	flip(frame, frame, 0);
	if (visionDisplayEnabled) {
		imshow("Ball", frame);
	}
	return frame;
}

int getAction() {
	return action;
}

void Vision::executeMotion(Vision* vis) {
	while (!vis->motorController->step(false)) {
		for (int i = 0; i < 100; i++) {}
	}
}

double Vision::getVisionTime() {
	return ((double) (0.0005 * vision));
}

double Vision::getMotorsTime() {
	return ((double) (0.0005 * motorTime));
}

void Vision::startMotorController() {
	motorControllerIsExecuting = true;
}

void Vision::stopMotorController() {
	motorControllerIsExecuting = false;
}

bool Vision::motorControllerIsRunning() {
	bool result = motorControllerIsExecuting;
	return result;
}

bool Vision::setMotion(std::string motion) {
	bool result = motorController->setMotion(motion);
	return result;
}

int Vision::getRobotX() {
	for (int i = 0; i < history_size; i++) {
//		std::cout << i << ": (" << distance[i] << ", " << angle[i] << ")  ";
	}
//	std::cout << std::endl;
	return position.x;
}
int Vision::getRobotY() {
	return position.y;
}

double Vision::getRobotTheta() {
	return thisRobotTheta;
}
double Vision::getRobotHeadTheta() {
	// Get the head angle
	double robotTheta = motorController->getHeadAngle();
	// Subtract 90 degrees to get offset from goal
//	robotTheta -= 90;
	// Get  the angle to the goal from the camera, and add that to our head orientation angle
	robotTheta += weightedAverageOf(angle, angle_confidence, history_size);
	// Update our global variable
//	thisRobotTheta = robotTheta;
	return robotTheta;
}
int Vision::getBallX() {
	// Get the head orientation
	double ballTheta = thisRobotTheta + motorController->getHeadAngle();
	// Figure out how far away along the X-axis the ball is
	double ballX = averageOf(ball_distance, history_size) * cos(toRadians(ballTheta));
	// Round to the nearest Integer
	int result = (int)(ballX + 0.5);
	// Return the global position by adding MUL8's position offset
	return result + getRobotX();
}
int Vision::getBallDistance() {
	if (knowsBallPosition()) {
		return (int)averageOf(ball_distance, history_size);
	}
	else return 1000;
}
void Vision::updateRobotTheta(double dTheta) {
	// Add the estimated rotation to previous rotation estimation
	thisRobotTheta += dTheta;
}
void Vision::updateRobotPosition(double distance, double theta) {
	// Update MUL8's position estimation by adding estimated movement and rotation
	position.x += distance * cos(toRadians(theta));
	position.y += distance * sin(toRadians(theta));
	// Reduce the confidence we have in the location, as the movement isn't guaranteed to be absolutely accurate
	reduceConfidence(position_confidence, 0.05);
}
double Vision::getRobotThetaToPoint(Point p) {
	// Determine simple angle to point
	int dx = getRobotX() - p.x;
	int dy = getRobotY() - p.y;

	// TODO Should this be dy/dx?
	double theta = tan(toRadians(dx/dy));

	// Subtract MUL8's estimated rotation from calculated angle
	theta -= thisRobotTheta;
	return theta;
}

double Vision::getRobotDistanceToPoint(Point p) {
	// Determine X and Y offsets
	int dx = getRobotX() - p.x;
	int dy = getRobotY() - p.y;

	// Calculate and return Euclidean distance
	double distance = dx*dx + dy*dy;
	return sqrt(distance);
}
int Vision::getBallY() {
	// Get the head orientation
	double ballTheta = thisRobotTheta + motorController->getHeadAngle();
	// Figure out how far away along the Y-axis the ball is
	double ballY = averageOf(ball_distance, history_size) * sin(toRadians(ballTheta));
	// Round to the nearest integer
	int result = (int)(ballY + 0.5);
	// Return the global position by adding MUL8's position offset
	return result + getRobotY();
}

void Vision::calibrateThresholds() {
	// Create the window where the thresholded image will be displayed
	namedWindow("Goal Thresholding", WINDOW_NORMAL);
	// Create the trackbars
	cvCreateTrackbar("LowerH", "Ball", &lowerH, 255, NULL);
	cvCreateTrackbar("UpperH", "Ball", &upperH, 255, NULL);

	cvCreateTrackbar("LowerS", "Ball", &lowerS, 255, NULL);
	cvCreateTrackbar("UpperS", "Ball", &upperS, 255, NULL);

	cvCreateTrackbar("LowerV", "Ball", &lowerV, 255, NULL);
	cvCreateTrackbar("UpperV", "Ball", &upperV, 255, NULL);


	// Keep updating the image with the current thresholds until the user presses 'g'
	while ((char)waitKey(80) != 'g') {
		Mat frame;
		cap.read(frame);
		Mat preprocessed = preprocess(frame);

		// Show the thresholded image
		if (visionDisplayEnabled) {
			imshow("Goal Thresholding", preprocessed);
		}
	}

	// Once the goal has been thresholded, we no longer need the frame
	destroyWindow("Goal Thresholding");

	// Create the window where the thresholded image will be displayed
	namedWindow("Ball Thresholding", WINDOW_NORMAL);

	// Create the sliding bars used to adjust the threshold levels
	cvCreateTrackbar("LowerR", "Ball", &ballLowerH, 255, NULL);
	cvCreateTrackbar("UpperR", "Ball", &ballUpperH, 255, NULL);

	cvCreateTrackbar("LowerG", "Ball", &ballLowerS, 255, NULL);
	cvCreateTrackbar("UpperG", "Ball", &ballUpperS, 255, NULL);

	cvCreateTrackbar("LowerB", "Ball", &ballLowerV, 255, NULL);
	cvCreateTrackbar("UpperB", "Ball", &ballUpperV, 255, NULL);

	// Keep updating the image with the current thresholds until the user presses 'b'
	while ((char)waitKey(80) != 'b') {
		Mat frame;
		cap.read(frame);

		// Apply some preprocessing to the image
		Mat imgHSV = frame.clone();
		medianBlur(imgHSV, imgHSV, kernel_size);

		cvtColor(frame, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV

		// Get Image Thresh
		//Set the thershold color (ball color) for image tracking
		Mat imgThresh;
		inRange(imgHSV, Scalar(ballLowerH, ballLowerS, ballLowerV), Scalar(ballUpperH, ballUpperS, ballUpperV), imgThresh);

		// Show the thresholded image
		if (visionDisplayEnabled) {
			imshow("Ball Thresholding", imgThresh);
		}
	}

	// Once the ball has been thresholded, we no longer need the frame
	destroyWindow("Ball Thresholding");
}

void Vision::setAction(int newAction) {
	// Set the new action
	action = newAction;

	// Reset the movement markers, so if we have to search we start over fresh
	headLeft = false;
	headRight = false;
	headUp = false;
	headDown = false;
}

bool Vision::knowsBallPosition() {
	bool result = true;

//	std:: cout << "Ball distance values: ";
	// If any of the distance indexes are 0, we don't have enough information to say we know where the ball is
	for (int i = 0; i < history_size; i++) {
		if (ball_distance[i] <= 0) {
			result = false;
			break;
		}
//		std::cout << "(" << ball_distance[i] << ") ";
	}
//	std::cout << std::endl;
	return result;
}

bool Vision::knowsRobotPosition() {
	bool result = true;
	// Keep track of how many confidence levels are below 0.5
//	int count = 0;

	// For each of the confidence levels, if it's 0 return false immediately, otherwise keep a total of the confidences below 0.5
	for (int i = 0; i < history_size; i++) {
		if (angle_confidence[i] == 0) {
			result = false;
			break;
		}
//		else if (angle_confidence[i] < 0.5 || distance_confidence[i] < 0.5) {
//			count++;
//		}
//		std::cout << "(" << angle_confidence[i] << ", " << distance_confidence[i] << ") ";
	}
	// If we have too many confidences below 0.5, we can't say we know MUL8's position
//	if (count > 7) {
//		result = false;
//	}
//	std::cout << std::endl;
	return result;
}

double Vision::toRadians(double degrees) {
	// Convert degrees to radians and return the results
	double result = (degrees * PI / 180.0);
	return result;
}

double Vision::toDegrees(double radians) {
	// Convert the radians to degrees, and return the result
	double result = (radians * 180.0 / PI);
	return result;
}

/*
 * This function is not used in the current vision approach.
 */
std::vector<double> Vision::getLine(Point p1, Point p2) {
	std::vector<double> arr;

	double m = ((double)p2.y-p1.y)/((double)p2.x-p1.x);
	if (p1.x == p2.x) {
		m = DOUBLE_FLAG;
	}
	double b = p1.y - m*p1.x;
	arr.push_back(m);
	arr.push_back(b);

//	std::cout << "M: " << arr[0] << "\tb: " << arr[1] << std::endl;
	return arr;
}
double Vision::getDistance(Point p1, Point p2) {
	double result;

	// Translate the coordinates to different resolutions
	// This is because my calibration images were taken at the higher resolution
	int x1 = (int)p1.x/640.0*1600.0;
	int y1 = (int)p1.y/480.0*1200.0;

	int x2 = (int)p2.x/640.0*1600.0;
	int y2 = (int)p2.y/480.0*1200.0;

	// Calculate the distance between the two points
	long int dx = (x2 - x1);
	long int dy = (y2 - y1);
	result = sqrt(dx*dx + dy*dy);
	return result;
}

VideoCapture Vision::setup() {
	/// Declare variables
	Mat src;

	bool web_zero = true;

	// To open live webcam
	VideoCapture cap(0);
	VideoCapture cap2(1);


	Mat test;
	if (!cap.read(test)) {
		std::cout << "Switching to cap(1)" << std::endl;
		cap.release();
		web_zero = false;


		if (!cap2.read(test)) {
			std::cout << "Is the webcam connected? If so, it's not video0 or video1.\n" <<
					"\nExiting program" << std::endl;
			exit(1);
		}
	}

	// To open file
	//VideoCapture cap("/home/unicorn/Videos/Webcam_view_goal.webm");

	/// Create window
	setwindowSettings();

	// Return the VideoCapture

	if (web_zero) {
		return cap;
	}
	else {
		return cap2;
	}
}

int Vision::getAction() {
	return action;
}

Mat Vision::GetThresholdedImage(Mat imgHSV) {
	Mat imgThresh;

	// Generate the threshold levels
	Scalar lower(lowerH, lowerS, lowerV);
	Scalar upper(upperH, upperS, upperV);

	// Threshold the image
	inRange(imgHSV, lower, upper, imgThresh);

	// Return thresholded image
	return imgThresh;
}

Mat func_dilate(Mat image) {
	// Create structures necessary for the dilation
	Mat result;
	Mat element = getStructuringElement(MORPH_RECT, Size(2*kernel_size_erode+1, 2*kernel_size_erode+1), Point(kernel_size_erode, kernel_size_erode));

	// Perform the dilation
	dilate(image, result, element);

	// Return the dilated image
	return result;
}

Mat func_erode(Mat image) {
	// Create the structures necessary for the erosion
	Mat result;
	Mat element = getStructuringElement(MORPH_RECT, Size(2*kernel_size_erode+1, 2*kernel_size_erode+1), Point(kernel_size_erode, kernel_size_erode));

	// Perform the erosion
	erode(image, result, element);

	// Return the eroded image
	return result;
}
void Vision::reduceConfidence(double *vec, double reduction) {
	// We reduce the confidence level for the given array
	for (int i = 0; i < history_size; i++) {
		vec[i] -= reduction;
		if (vec[i] < 0) {
			vec[i] = 0;
		}
	}
}
// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
//static double angle( Point pt1, Point pt2, Point pt0 )
//{
//    double dx1 = pt1.x - pt0.x;
//    double dy1 = pt1.y - pt0.y;
//    double dx2 = pt2.x - pt0.x;
//    double dy2 = pt2.y - pt0.y;
//    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
//}

//This function create two windows and 6 trackbars for the "Ball" window
void Vision::setwindowSettings(){

	// Create two windows
	if (visionDisplayEnabled) {
		cvNamedWindow("Ball"/*, WINDOW_NORMAL*/);
//		cvCreateTrackbar("Searching speed", "Ball", &searching_speed, 1023, NULL);
	}
//	cvNamedWindow("imgThresh", WINDOW_NORMAL);
//	cvCreateTrackbar("Vision", "Ball", &vision, 20, NULL);
//	cvCreateTrackbar("Motions", "Ball", &motorTime, 20, NULL);

	// Create the trackbars
//	cvCreateTrackbar("LowerH", "Ball", &lowerH, 255, NULL);
//	cvCreateTrackbar("UpperH", "Ball", &upperH, 255, NULL);
//
//	cvCreateTrackbar("LowerS", "Ball", &lowerS, 255, NULL);
//	cvCreateTrackbar("UpperS", "Ball", &upperS, 255, NULL);
//
//	cvCreateTrackbar("LowerV", "Ball", &lowerV, 255, NULL);
//	cvCreateTrackbar("UpperV", "Ball", &upperV, 255, NULL);
//	cvCreateTrackbar("Vision Frequency", "Ball", &vis_frequency, max_frequency, NULL);
}

int Vision::getFrequency() {
	return vis_frequency;
}

Mat Vision::preprocess(Mat original) {

	// Blur the original image into a temporary container
	Mat temp;
	medianBlur(original, temp, kernel_size);

	double headAngle = motorController->getHeadDownAngle();

	// TODO For testing purposes only
	if (headAngle > 90 || headAngle < 0) {
		headAngle = 0;
	}

	int maskPixelHeight = 160 - (8 * headAngle);
	if (maskPixelHeight < 0) {
		maskPixelHeight = 0;
	}
//
//	Mat mask(temp.rows, temp.cols, );
//	mask.zeros(temp.rows, temp.cols, CV_8UC1);
//
	Rect rect;
	rect.x = 0;
	rect.y = 0;
	rect.width = temp.cols;
	rect.height = maskPixelHeight;

	rectangle(temp, rect, Scalar(0, 0, 0), -1, 8, 0);

	// Convert the image into HSV Color coding
	Mat imgHSV = temp.clone();
	cvtColor(temp, imgHSV, CV_BGR2HSV);

	if (visionDisplayEnabled) {
		imshow("IMG HSV", imgHSV);
	}

	// Get the thresholded image
	Mat imgThresh = GetThresholdedImage(imgHSV);

	// Show the thresholded image
	if (visionDisplayEnabled) {
		imshow("imgThresh", imgThresh);
	}

	// Detect the edges using Canny Edge Detection
	Mat canny;
	Canny(imgThresh, canny, 900, 1000);

	// Dilate and Erode the image to close up any holes
	canny = func_dilate(canny);
	canny = func_erode(canny);

	// Return the resulting image
	return canny;
}

/*
 * This function isn't used in the current implementation of goal detection
 */
std::vector<Point> Vision::determineCorners(std::vector<Point> refined_poi) {
	// Create an array to store the corners in
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
		int ymin = 100000;        // Some large number that is greater than the resolution size
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
					//    int lower_left_index = -1;
					xmax = refined_poi[yMaxIndex[i]].x;
					lower_right_index = yMaxIndex[i];
				}
				if (refined_poi[yMaxIndex[i]].x < xmin) {
					xmin = refined_poi[yMaxIndex[i]].x;
					lower_left_index = yMaxIndex[i];
				}
			}
		}



		/* lower_right_index now contains the bottom-right most index     */
		/* lower_left_index now contains the bottom-left most index     */


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

		/* upper_right_index now contains the top-right most index        */
		/* upper_left_index now contains the top-left most index        */


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

		/* left_top_index now contains the left-top most index        */
		/* left_bot_index now contains the left-bottom most index    */

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



		/* right_top_index now contains the right-top most index    */
		/* right_bot_index now contains the right-bottom most index    */

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

/*
 * This function isn't used in the current implementation of goal detection
 */
std::vector<Point> Vision::findIntersections(Rect boundRect, std::vector<std::vector<Point> > contours, int index) {
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
		//        std::cout << "Number of Indexes: " << indexes.size() << std::endl;
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
				//                std::cout << sum << "/" << (indexes[i+1]+1) - indexes[i] << std::endl;
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

	points_of_interest.resize(0);
	refined_poi.clear();

	return corners;
}

double Vision::getLength(double distance, double pixels) {
	// Returns the length of something pixels wide at distance away from the camera

	// Numbers are based on best-fit line
	double length = (distance+1.653881);
	length /= 1332.565623;
	length *= pixels;
	return length;

}

double Vision::getTheta(double known, double observed) {
	// Determines an object's rotation based on known length and observed length
	double theta = asin(observed/known);
	return (90 - toDegrees(theta));
}

double long Vision::getDistanceFromCamera(double cm_per_pixel) {
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

double Vision::getDistanceToCenter(double known, double observed, double d, double angle_from_center) {
	// Determine the distance to the center of the goal
	double bigTheta = 180 - toDegrees(asin(toRadians(observed/d))) - getTheta(known, observed);
	double result = d;
	result *= toDegrees(sin(toRadians(bigTheta)));
	result /= toDegrees(sin(toRadians(angle_from_center)));
	return result;
}

double Vision::getAngleFromCenter(double known, double observed, double d) {
	// Determine the angle to the center of the goal
	double bigTheta = 180 - toDegrees(asin(toRadians(observed/d))) - getTheta(known, observed);
	std::cout << "Angle from back post: " << bigTheta-90 << std::endl;
	double g = GOAL_WIDTH/2;
	double result = sin(toRadians(180-bigTheta))/((g/d) + cos(toRadians(180 - bigTheta)));
	result = toDegrees(atan(result));
	return (/*90 - */result);
}
//Detect the largest red objects location in the webcam view
double Vision::detectBallDistance(Rect boundRect){

	// Create the color white
//	Scalar color(255,255,255);

	//Determine the number of pixels tall the bounding box is
	double pixels = getDistance(Point(boundRect.x, boundRect.y), Point(boundRect.x + boundRect.width, boundRect.y));

	// Set the distance to a flagged value so we can determine if we really know the distance
	double distance = DOUBLE_FLAG;

	// If the number of pixels seems realistic, get the distance
	if (pixels > 0 && pixels < 800) {
		distance = getDistanceFromCamera(BALL_DIAMETER / pixels);
	}

//	double distance = DOUBLE_FLAG;
//	std::cout << "Determining The Nominator" << std::endl;
//	double denominator = tan(toRadians(/*90-*/motorController->getHeadDownAngle()/*pixels_to_black_out*/));
//	std::cout << "Denominator = " << denominator << std::endl;
//	if (denominator != 0) {
//		distance = ((double)MUL8_HEIGHT_TO_WEBCAM)/denominator;
//	}

	// Return the value for distance
	return distance;
}

RotatedRect Vision::processImage(Mat &frame, Mat &preprocessedImg, Mat &imgDraw) {

	// Find the external contours in the preprocessed image
	std::vector<std::vector<Point> > contours;
	std::vector<Vec4i> heirarchy;
	imgDraw = preprocessedImg.clone();
	findContours(preprocessedImg, contours, heirarchy, RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	// Draw the contours on imgDraw
	for (unsigned i = 0; i < contours.size(); i++) {
		drawContours(imgDraw, contours, i, Scalar(200, 255, 0), 2, 8, heirarchy, 0, Point());
	}

	// After drawing contours once, we again find the contours of the image
	findContours(imgDraw, contours, heirarchy, RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	// Initialize the largest contour area and corresponding bounding rectangle
	int largest_area = 1500;
	RotatedRect boundRect(Point(-1,-1), Size(1,1), 45);

	// Cycle through all the contours, finding the largest one, which we will assume is the goal
	for( unsigned i = 0; i< contours.size(); i++ ) // iterate through each contour.
	{
		double a=contourArea( contours[i],false);  //Find the area of contour

		// If the current area is the largest, update the necessary variables
		if(a>largest_area){
			largest_area=a; //Reset largest area1
			//            largest_contour_index=i;  //Store the index of largest contour
			boundRect=minAreaRect(contours[i]); // Find the bounding rectangle for biggest contour

			// For drawing image
			//                std::cout << "Found new largest contour: " << a << std::endl;
		}
		// Draw the contours on imgDraw
		drawContours(imgDraw, contours, i, Scalar(200, 255, 0), 2, 8, heirarchy, 0, Point());
	}

	// Draw the smallest area rectangle surrounding the largest area contour (the goal)
	Point2f rect_points[4]; boundRect.points( rect_points );
	for( int j = 0; j < 4; j++ ) {
		line( frame, rect_points[j], rect_points[(j+1)%4], Scalar(255, 255, 255), 1, 8 );
	}

	// Return the bounding rectangle
	return boundRect;

}

void Vision::determinePosition(RotatedRect boundRect, Mat &imgDraw, Mat &frame) {
	std::vector<Point> corners;
	// Get the points on the rotated rectangle
	Point2f rect_points[4]; boundRect.points( rect_points );

	// For each of the corners of the bounding rectangle, determine if there is a corner of the goal within 5 pixels
	for( int j = 0; j < 4; j++ ) {
		for (int k = rect_points[j].x-7; k <= rect_points[j].x+7; k++) {
			for (int l = rect_points[j].y-7; l <= rect_points[j].y+7; l++) {
				if (imgDraw.at<unsigned char>(Point(k, l)) > 175) {
					if (corners.size() == 0) {
						corners.push_back(rect_points[j]);
					}
					else if (!(corners[corners.size()-1].x == rect_points[j].x && corners[corners.size()-1].y == rect_points[j].y)) {
						//                            if (corners.size() > 0 && !((corners[corners.size()-1].x == rect_points[j].x) && (corners[corners.size()-1].y == rect_points[j].y))) {
						corners.push_back(rect_points[j]);
					}
					//                            else if (corners.size() == 0) {
					//                                corners.push_back(rect_points[j]);
					//                            }
				}
			}
		}
	}

	// Go through all the points that were added as corners and refine them so they each only appear once
	std::vector<Point> refined;
	if (corners.size() > 0) {
		refined.push_back(corners[0]);
	}

	for (unsigned i = 1; i < corners.size(); i++) {
		if (corners[i-1].x != corners[i].x && corners[i-1].y != corners[i].y) {
			refined.push_back(corners[i]);
		}
	}

	// Determine if each of the points is the top left, top right, bottom left, or bottom right
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
			}
			else {
				// Bottom Left
				left_bot = i;
				rect_points[left_bot].x = (int) (rect_points[left_bot].x + 0.5);
				rect_points[left_bot].y = (int) (rect_points[left_bot].y + 0.5);
			}
		}
		else {
			if (rect_points[i].y < center.y) {
				// Top right
				right_top = i;
				rect_points[right_top].x = (int) (rect_points[right_top].x + 0.5);
				rect_points[right_top].y = (int) (rect_points[right_top].y + 0.5);
			}
			else {
				// Bottom right
				right_bot = i;
				rect_points[right_bot].x = (int) (rect_points[right_bot].x + 0.5);
				rect_points[right_bot].y = (int) (rect_points[right_bot].y + 0.5);
			}
		}
	}



	// Get the index of the respective corners from the array 'refined'
	int top_left_index = -1;
	int top_right_index = -1;
	int bot_left_index = -1;
	int bot_right_index = -1;
	//            std::cout << "Size of corners vector: " << refined.size() << std::endl;
	for (unsigned i = 0; i < refined.size(); i++) {
		if (refined[i].x == rect_points[left_top].x && refined[i].y == rect_points[left_top].y) {
			top_left_index = i;
		}
		else if (refined[i].x == rect_points[right_top].x && refined[i].y == rect_points[right_top].y) {
			top_right_index = i;
		}
		else if (refined[i].x == rect_points[left_bot].x && refined[i].y == rect_points[left_bot].y) {
			bot_left_index = i;
		}
		else if (refined[i].x == rect_points[right_bot].x && refined[i].y == rect_points[right_bot].y) {
			bot_right_index = i;
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
//	double observed_length = -1;
//	double theta = -100;

	// If we know both points on the left, we are probably to the left of the goal, so use those points to determine our distance and angle
//	if (top_left_index >= 0 && bot_left_index >= 0) {
//		distance_from_cam = getDistanceFromCamera(GOAL_POST_WIDTH/getDistance(refined[top_left_index], refined[top_right_index]));
//		if (top_right_index >= 0) {
//			observed_length = getLength(distance_from_cam, getDistance(refined[top_left_index], refined[top_right_index]));
//			theta = -1 * getAngleFromCenter(GOAL_WIDTH, observed_length, distance_from_cam);
//		}
//	}

	// Otherwise, if we know both points on the right, we are probably to the right of the goal, so use those points to determine our distance and angle
//	else if (top_right_index >= 0 && bot_right_index >= 0) {
//		distance_from_cam = getDistanceFromCamera(GOAL_HEIGHT/getDistance(refined[top_right_index], refined[bot_right_index]));
//		if (top_left_index >= 0) {
//			observed_length = getLength(distance_from_cam, getDistance(refined[top_left_index], refined[top_right_index]));
//			theta = getAngleFromCenter(GOAL_WIDTH, observed_length, distance_from_cam);
//		}
//	}

	/* Try even rougher estimations */
	double heightDistance = getDistance(rect_points[left_top], rect_points[left_bot]);
	double widthDistance = getDistance(rect_points[left_top], rect_points[right_top]);
	double distance_from_cam_2;
	if ((heightDistance / widthDistance) > 2) {
		distance_from_cam_2 = getDistanceFromCamera(GOAL_POST_WIDTH/getDistance(rect_points[left_top], rect_points[right_top]));
	}
	else {
		distance_from_cam_2 = getDistanceFromCamera(GOAL_HEIGHT/getDistance(rect_points[left_top], rect_points[left_bot]));
	}
//	double observed_length2 = getLength(distance_from_cam_2, getDistance(rect_points[left_top], rect_points[right_top]));
//	double theta2 = getAngleFromCenter(GOAL_WIDTH, observed_length2, distance_from_cam_2);
	distance[distance_index] = distance_from_cam_2;
	distance_confidence[distance_index] = known_points/4.0;
	distance_index = (distance_index + 1) % history_size;
//	angle[angle_index] = theta2;
//	angle_confidence[angle_index] = known_points/4.0;
//	angle_index = (angle_index + 1) % history_size;


	// If we have a reading on the distance, update our position estimation
	if (distance_from_cam > 0) {
		std::cout << "Distance from camera to near post: " << distance_from_cam/100.0 << "m" << std::endl;
		distance[distance_index-1] = distance_from_cam;
		distance_confidence[distance_index-1] = (known_points == 3) ? 0.75 : 0.25;
	}
	// If we have a reading on the angle, update our angle estimation
//	if (theta > -100) {
//		std::cout << "Angle from goal: " << theta << "°" << std::endl;
//		angle[angle_index-1] = theta;
//		angle_confidence[angle_index-1] = (known_points == 3) ? 0.75 : 0.25;
//	}
}

double Vision::averageOf(double vec[], int size) {
	// Determine the average value of the given array
	double sum = 0;
	int count = 0;
	for (int i = 0; i < size; i++) {
		if (vec[i] != 0.0) {
			sum += vec[i];
			count++;
		}
	}
	double avg = sum/((double)count);

	// Return the calculated average
	return avg;
}

Point Vision::convertToPolar(double distance, double theta) {
	// Convert the polar coordinates to Cartesian coordinates (Function name is misleading)
	double y = sin(toRadians(theta)) * distance;
	double x = cos(toRadians(theta)) * distance;
	return Point(x, y);
}

double Vision::weightedAverageOf(double vec[], double confidence[], int size) {
	// Determine the weighted average of the given array
	// Weight is determined by confidence levels, where confidence has a value between 0 and 1
	double sum = 0;
	double count = 0;
	for (int i = 0; i < size; i++) {
		sum += confidence[i]*vec[i];
		count += confidence[i];
	}
	double avg = sum/(count);

	// Return the calculated average
	return avg;
}

Rect Vision::processBall(Mat frame) {

	// Initialize variables
	int largest_area=90;
	Rect boundRect;
	boundRect.x = -1;
	boundRect.y = -1;

	// Blur the image and convert to HSV Color coding
	Mat imgHSV = frame.clone();
	medianBlur(frame, imgHSV, kernel_size);
	cvtColor(frame, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV

	double headAngle = motorController->getHeadDownAngle();

	// TODO For testing purposes only
		if (headAngle > 90 || headAngle < 0) {
			headAngle = 0;
		}

		int maskPixelHeight = 160 - (8 * headAngle);
		if (maskPixelHeight < 0) {
			maskPixelHeight = 0;
		}
	//
	//	Mat mask(temp.rows, temp.cols, );
	//	mask.zeros(temp.rows, temp.cols, CV_8UC1);
	//
		Rect rect;
		rect.x = 0;
		rect.y = 0;
		rect.width = imgHSV.cols;
		rect.height = maskPixelHeight;

		rectangle(imgHSV, rect, Scalar(0, 0, 0), -1, 8, 0);
	// Get the thresholded image
	Mat imgThresh;
	inRange(imgHSV, Scalar(ballLowerH, ballLowerS, ballLowerV), Scalar(ballUpperH, ballUpperS, ballUpperV), imgThresh);


	// Find the contours in the image
	vector<vector <Point> > contours; // Vector for storing contour
	vector<Vec4i> hierarchy;
	findContours(imgThresh, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE ); // Find the contours in the image

	// Find circular shapes in the image
	std::vector<Vec3f> circles;

	HoughCircles( imgThresh, circles, CV_HOUGH_GRADIENT, 1, imgThresh.rows/8, 1000/*thresh1*/, 36/*thresh2*/, 1, 75 );


	// Cycle through all the contours to find the contour with the largest area
	for( unsigned i = 0; i< contours.size(); i++ ) // iterate through each contour.
	{
		double a=contourArea( contours[i],false);  //Find the area of contour

		if(a>largest_area){
			largest_area=a; //Reset largest area
			//            largest_contour_index=i;  //Store the index of largest contour
			boundRect=boundingRect(contours[i]); // Find the bounding rectangle for biggest contour

			// For drawing image
			//            drawContours(img, contours, i, Scalar(255,255,255), CV_FILLED, 8, hierarchy);
		}

	}

	// FOR DRAWING IMAGE
	// Draw circles on detected circles
	for( size_t i = 0; i < circles.size(); i++ )
	{
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		// circle center
		circle( frame, center, 3, Scalar(0,255,0), -1, 8, 0 );
		// circle outline
		circle( frame, center, radius, Scalar(0,0,255), 3, 8, 0 );
	}

	//    drawContours(img, contours, largest_contour_index, Scalar(0, 255, 255), CV_FILLED, 8, hierarchy);

	Point centerRec = Point((boundRect.x + boundRect.width/2), (boundRect.y + boundRect.height/2));
	circle(frame, cvPoint(centerRec.x, centerRec.y), boundRect.width/2, CV_RGB(255,0,0), -1, 8, 0 );
	// END DRAWING IMAGE

	// Return the bounding rectangle
	return boundRect;
}

void Vision::nextFrame() {

	Mat frame;
//	timer = getUnixTime();

	// Grab the next frame. If there is an error, return to calling function
	if (!cap.read(frame)) {
		std::cout << "I broke!" << std::endl;
		return;
	}
	// Flip the image if it needs to be flipped
	if (FLIPPED) {
		flip(frame, frame, 0);
	}

	switch(action) {
	case SEARCH_FOR_BALL:
		search_for_ball(frame);
		break;
	case SEARCH_FOR_GOAL:
		search_for_goal(frame);
		break;
	case SEARCH_FOR_BOTH:
		search_for_both(frame);
		break;
	case CENTER_GOAL:
		center_goal(frame);
		break;
	case CENTER_BALL:
		center_ball(frame);
		break;
	case LOCALIZE_GOAL:
		localize_goal(frame);
		break;
	default:
		break;
	}

//	// Do this if MUL8 is searching for the ball
//	if (action == SEARCH_FOR_BALL) {
//		search_for_ball(frame);
//	}
//
//	// Do this if MUL8 is searching for the goal
//	else if (action == SEARCH_FOR_GOAL) {
//		search_for_goal(frame);
//	}
//
//	// Do this if MUL8 is searching for both the goal and the ball at the same time
//	else if (action == SEARCH_FOR_BOTH) {
//		search_for_both(frame);
//	}
//
//	// Do this is MUL8 needs to center the goal in the frame
//	else if (action == CENTER_GOAL) {
//		center_goal(frame);
//	}
//
//	// Do this if MUL8 needs to center the ball in the frame
//	else if (action == CENTER_BALL) {
//		center_ball(frame);
//	}
//
//	// Do this if we need to determine where we are based on the goal's position
//	else if (action == LOCALIZE_GOAL) {
//		localize_goal(frame);
//	}

	// Put information on the screen for the user to see and debug
	char text[255];
	sprintf(text, "Distance to goal: %fm    Confidence Distance: %fm", (distance_to_goal/100.0), (confidence_distance_to_goal/100.0));
	putText(frame, text, Point(10, 400), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(200, 200, 250), 1, CV_AA);
	sprintf(text, "Angle from goal: %f    Confidence Angle: %f", angle_from_goal, confidence_angle_from_goal);
	putText(frame, text, Point(10, 420), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(200, 200, 250), 1, CV_AA);
//	sprintf(text, "%f fps", (1/((getUnixTime()-timer))));
//	timer = getUnixTime();
	putText(frame, text, Point(10, 460), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(200, 200, 250), 1, CV_AA);
	sprintf(text, "Coordinates: (%d, %d)", position.x, position.y);
	putText(frame, text, Point(10, 20), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(0, 255, 255), 1, CV_AA);

	sprintf(text, "Distance to ball: %d", getBallDistance());
	putText(frame, text, Point(10, 100), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(0, 255, 255), 1, CV_AA);

	// Wait allows OpenCV to update the image
	waitKey(1);
	if (visionDisplayEnabled) {
		imshow( "Ball", /*imgDraw*/ /*grey*/ /*preprocessedImg*/ frame);
	}
	//    }
}

void Vision::center_goal(Mat frame) {
//	std::cout << "Centering Goal" << std::endl;

	Mat preprocessedImg;

	// Threshold the image and detect edges
	preprocessedImg = preprocess(frame);

	// Get the bounding rectangle for the goal
	Mat imgDraw;
	RotatedRect boundRect = processImage(frame, preprocessedImg, imgDraw);

	// Determine how MUL8 needs to move its head in order to get the ball in the center of the frame
	if (boundRect.center.x >= 0) {
		if (boundRect.center.x < 213/*310*/) {
			if (boundRect.center.y < 160/*230*/) {
				motorController->moveHead(MUL8_HEAD_UP_RIGHT, searching_speed);
			}
			else if (boundRect.center.y > 320/*250*/) {
				motorController->moveHead(MUL8_HEAD_DOWN_RIGHT, searching_speed);
			}
			else {
				motorController->moveHead(MUL8_HEAD_RIGHT, searching_speed);
			}
		}
		else if (boundRect.center.x > 427/*330*/) {
			if (boundRect.center.y < 160/*230*/) {
				motorController->moveHead(MUL8_HEAD_UP_LEFT, searching_speed);
			}
			else if (boundRect.center.y > 320/*250*/) {
				motorController->moveHead(MUL8_HEAD_DOWN_LEFT, searching_speed);
			}
			else {
				motorController->moveHead(MUL8_HEAD_LEFT, searching_speed);
			}
		}
		else {
			if (boundRect.center.y < 160/*230*/) {
				motorController->moveHead(MUL8_HEAD_UP, searching_speed);
			}
			else if (boundRect.center.y > 320/*250*/) {
				motorController->moveHead(MUL8_HEAD_DOWN, searching_speed);
			}
			else {
				if (motorController->headLeftRightIsMoving() || motorController->headUpDownIsMoving()) {
					motorController->stopHead();
				}
				setAction(LOCALIZE_GOAL);
			}
		}
	}

	// If the goal isn't in the frame, we need to start searching for it again
	else {
		setAction(SEARCH_FOR_GOAL);
	}
}

void Vision::center_ball(Mat frame) {
	// Get the bounding rectangle for the ball
	Rect centerRec = processBall(frame);
	centerRec.x += centerRec.width/2;
	centerRec.y += centerRec.height/2;
	int centerX = centerRec.x;
	int centerY = centerRec.y;

	// Determine how MUL8 needs to move its head in order to get the ball centered in the frame
	if (centerX >= 0) {
		// Calculate the moving speed based on the distance between the center of the ball and the center of the screen
		int speed = (int)getDistance(Point(centerX, centerY), Point(320, 240));
		speed /= 7;

		if (centerX < /*213*/310) {
			if (centerY < /*160*/230) {
				motorController->moveHead(MUL8_HEAD_UP_RIGHT, speed);
			}
			else if (centerY > /*320*/250) {
				motorController->moveHead(MUL8_HEAD_DOWN_RIGHT, speed);
			}
			else {
				motorController->moveHead(MUL8_HEAD_RIGHT, speed);
			}
		}
		else if (centerX > /*427*/330) {
			if (centerY < /*160*/230) {
				motorController->moveHead(MUL8_HEAD_UP_LEFT, speed);
			}
			else if (centerY > /*320*/250) {
				motorController->moveHead(MUL8_HEAD_DOWN_LEFT, speed);
			}
			else {
				motorController->moveHead(MUL8_HEAD_LEFT, speed);
			}
		}
		else {
			if (centerY < /*160*/230) {
				motorController->moveHead(MUL8_HEAD_UP, speed);
			}
			else if (motorController->readMotorPosition(24) < 510) {


				if (centerY > /*320*/250 && centerY < 365) {
					// The ball is on the ground right by the robot
					if (motorController->headLeftRightIsMoving() || motorController->headUpDownIsMoving()) {
						motorController->stopHead();
					}
					// This is the measured distance from the camera to the ball when it's located at MUL8's feet
					std::cout << "I am setting the ball distance to 75" << std::endl;
					ball_distance[ball_index] = 75;
					ball_index = (ball_index + 1) % history_size;
				}
				else if (centerY > 365 && centerY < 440) {
					// The ball is on the ground right by the robot
					if (motorController->headLeftRightIsMoving() || motorController->headUpDownIsMoving()) {
						motorController->stopHead();
					}
					// This is the measured distance from the camera to the ball when it's located at MUL8's feet
					ball_distance[ball_index] = 65;
					std::cout << "I set the ball distance to 65" << std::endl;
					ball_index = (ball_index + 1) % history_size;
				}
				else if (centerY >= 440) {
					// The ball is on the ground right by the robot
					if (motorController->headLeftRightIsMoving() || motorController->headUpDownIsMoving()) {
						motorController->stopHead();
					}
					// This is the measured distance from the camera to the ball when it's located at MUL8's feet
					ball_distance[ball_index] = 55;
					std::cout << "I set the ball distance to 55" << std::endl;
					ball_index = (ball_index + 1) % history_size;
				}

			}
			else if (centerY >/* 320*/250) {
				motorController->moveHead(MUL8_HEAD_DOWN, speed);
//				std::cout << "I am going to detect the ball distance" << std::endl;
				ball_distance[ball_index] = detectBallDistance(centerRec);
				ball_index = (ball_index + 1) % history_size;
			}
			else {
				if (motorController->headLeftRightIsMoving() || motorController->headUpDownIsMoving()) {
					motorController->stopHead();
				}
//				std::cout << "I am going to detect the ball distance" << std::endl;
				ball_distance[ball_index] = detectBallDistance(centerRec);
				ball_index = (ball_index + 1) % history_size;
			}
		}
	}

	// If the ball isn't on screen, MUL8 needs to start searching for it again
	else {
		setAction(SEARCH_FOR_BALL);
	}
}

void Vision::localize_goal(Mat frame) {

	Mat preprocessedImg;

	// Threshold the image and detect edges
	preprocessedImg = preprocess(frame);

	// Get the bounding rectangle for the goal
	Mat imgDraw;
	RotatedRect boundRect = processImage(frame, preprocessedImg, imgDraw);

	// Determine MUL8's position based on bounding rectangle
	determinePosition(boundRect, imgDraw, frame);
	distance_to_goal = averageOf(distance, history_size);
	confidence_distance_to_goal = weightedAverageOf(distance, distance_confidence, history_size);
	angle_from_goal = averageOf(angle, history_size);
	confidence_angle_from_goal = weightedAverageOf(angle, angle_confidence, history_size);

	position = convertToPolar(confidence_distance_to_goal, confidence_angle_from_goal);

	thisRobotTheta = getRobotHeadTheta();

	// We don't want consecutive frames for our position, so set the action to searching for goal in order to get non-sequential frames
	setAction(SEARCH_FOR_GOAL);
}

void Vision::search_for_both(Mat frame) {
	// Get the bounding rectangle for the ball
	Rect ball = processBall(frame.clone());

	// If the ball is onscreen, update its location information
	if (ball.x >= 0) {
		ball_distance[ball_index] = detectBallDistance(ball);
		ball_index = (ball_index + 1) % history_size;
	}

	// Begin searching for the goal
	Mat preprocessedImg;

	// Threshold the image and detect edges
	preprocessedImg = preprocess(frame);

	// Get the bounding rectangle for the goal
	Mat imgDraw;
	RotatedRect boundRect = processImage(frame, preprocessedImg, imgDraw);

	// If the goal is on screen, set the action to centering the goal
	if (boundRect.center.x >= 0) {
		setAction(CENTER_GOAL);
		//goalFound = true;
	}
}

void Vision::search_for_goal(Mat frame) {
	Mat preprocessedImg;

	// Threshold the image and detect edges
	preprocessedImg = preprocess(frame);

	// Get the bounding rectangle for the goal
	Mat imgDraw;
	RotatedRect boundRect = processImage(frame, preprocessedImg, imgDraw);

	// If the goal is on screen, set the action to centering the goal
	if (boundRect.center.x >= 0) {
		motorController->stopHead();
		setAction(CENTER_GOAL);
	}
	// Otherwise search for the goal using a simple searching algorithm
	else {
		// Here we implement a searching algorithm since we don't know where the goal is
		if (!headLeft) {
			motorController->moveHead(MUL8_HEAD_LEFT, searching_speed);
			if (!motorController->headLeftRightIsMoving()) {
				headLeft = true;
			}
		}
		else if (!headRight) {
			motorController->moveHead(MUL8_HEAD_RIGHT, searching_speed);
			if (!motorController->headLeftRightIsMoving()) {
				headRight = true;
			}
		}
		else if (!headUp) {
			motorController->moveHead(MUL8_HEAD_UP, searching_speed);
			if (!motorController->headUpDownIsMoving()) {
				headLeft = false;
				headRight = false;
				headUp = true;
			}
		}
		else if (!headDown) {
			motorController->moveHead(MUL8_HEAD_DOWN, searching_speed);
			if (!motorController->headUpDownIsMoving()) {
				headLeft = false;
				headRight = false;
				headDown = true;
			}
		}
		else {
			motorController->moveHead(MUL8_HEAD_CENTER, searching_speed);
		}
	}

}

void Vision::search_for_ball(Mat frame) {
	// Get the bounding rectangle for the ball
	Rect ball = processBall(frame);

	// If the ball is on screen, switch modes to centering the ball
	if (ball.x >= 0) {
		motorController->stopHead();
		setAction(CENTER_BALL);
	}
	// Otherwise search for the ball using a simple head moving algorithm
	else {
		// Here we implement a searching algorithm since we don't know where the ball is
		if (!headLeft) {
			motorController->moveHead(MUL8_HEAD_LEFT, searching_speed);
			if (!motorController->headLeftRightIsMoving()) {
				headLeft = true;
			}
		}
		else if (!headRight) {
			motorController->moveHead(MUL8_HEAD_RIGHT, searching_speed);
			if (!motorController->headLeftRightIsMoving()) {
				headRight = true;
			}
		}
		else if (!headDown) {
			motorController->moveHead(MUL8_HEAD_DOWN, searching_speed);
			if (!motorController->headUpDownIsMoving()) {
				headLeft = false;
				headRight = false;
				headDown = true;
			}
		}
		else if (!headUp) {
			motorController->moveHead(MUL8_HEAD_UP, searching_speed);
			if (!motorController->headUpDownIsMoving()) {
				headLeft = false;
				headRight = false;
				headUp = true;
			}
		}
		else {
			motorController->moveHead(MUL8_HEAD_CENTER, searching_speed);
		}
	}
}

std::string Vision::getMotionRequest() {
//	Mat frame;
//	cap.read(frame);
	std::string result;
//
//	Rect ball = processBall(frame);
//	Point ballCenter;
//	ballCenter.x = ball.x + ball.width;
//	ballCenter.y = ball.y + ball.height;
//
//	if (ball.x < 213) {
//		result = "SSr";
//	}
//	else if (ball.x > 427) {
//		result = "SSl";
//	}
//	else {
//		if (ball.x < 320) {
//			result = "KickR";
//		}
//		else {
//			result = "KickL";
//		}
		double headAngle = motorController->getHeadAngle();
		if (headAngle < 0) {
			result = "KickL";
			std::cout << "Requesting KickL" << std::endl;
		}
		else {
			result = "KickR";
			std::cout << "Requesting KickR" << std::endl;
		}
//	}
	return result;

}
