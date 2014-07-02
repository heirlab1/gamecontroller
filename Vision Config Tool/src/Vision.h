/*
 * Vision.h
 *
 *  Created on: Jun 20, 2014
 *      Author: Kellen Carey
 */

#ifndef VISION_H_
#define VISION_H_

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
#include "MotorController.h"

//#include "MotorController.h"

#define LEFT						0
#define RIGHT						1
#define TOP							2
#define BOT							3


#define GOAL_HEIGHT					90.0	//180.0 // FOR ROBOCUP
#define GOAL_WIDTH					167.0	//300.0 // FOR ROBOCUP
#define GOAL_DEPTH					44.0	//60.0  // FOR ROBOCUP

#define UPPER_LEFT_CORNER			0
#define UPPER_RIGHT_CORNER			1
#define LOWER_LEFT_FRONT_CORNER		2
#define LOWER_RIGHT_FRONT_CORNER	3
#define LOWER_LEFT_BACK_CORNER		4
#define LOWER_RIGHT_BACK_CORNER		5

#define PI							3.141592
#define INT_FLAG					-100
#define DOUBLE_FLAG					-0.12345

#define BALL_DIAMETER				19.735//13.0507

#define WAIT						0
#define SEARCH_FOR_GOAL				1
#define SEARCH_FOR_BALL				2
#define SEARCH_FOR_BOTH				3
#define CENTER_GOAL					4
#define LOCALIZE_GOAL				5
#define CENTER_BALL					6

#define FLIPPED						1

using namespace cv;

class Vision {
public:
	Vision();
	virtual ~Vision();
	void init(MotorController&);
//	bool seesGoal();
	void setAction(int);
	int getRobotX();
	int getRobotY();
	double getRobotHeadTheta();
	int getBallX();
	int getBallY();
	int getBallDistance();
	bool knowsRobotPosition();
	bool knowsBallPosition();
	void nextFrame();
	void calibrateThresholds();
	double getRobotThetaToPoint(Point);
	double getRobotDistanceToPoint(Point);
	void updateRobotTheta(double);
	void updateRobotPosition(double distance, double theta);
	double getRobotTheta();
	double getDistance(Point, Point);
	int getFrequency();

private:
	Mat GetThresholdedImage(Mat);
	void setwindowSettings();
	Mat preprocess(Mat);
	std::vector<Point> determineCorners(std::vector<Point>);
	std::vector<Point> findIntersections(Rect, std::vector<std::vector<Point> >, int);
	double getLength(double, double);
	double getTheta(double, double);
	double long getDistanceFromCamera(double);
	double getDistanceToCenter(double, double, double, double);
	double getAngleFromCenter(double, double, double);
	//Detect the largest red objects location in the webcam view
	double detectBallDistance(Rect);
	RotatedRect processImage(Mat&, Mat&, Mat&);
	void determinePosition(RotatedRect, Mat&, Mat&);
	double averageOf(double[], int);
	Point convertToPolar(double, double);
	double weightedAverageOf(double[], double[], int);
	double toRadians(double);
	double toDegrees(double);
	VideoCapture setup();
	std::vector<double> getLine(Point, Point);
	Rect processBall(Mat);
	void reduceConfidence(double*, double);
	void center_goal(Mat);
	void center_ball(Mat);
	void localize_goal(Mat);
	void search_for_both(Mat);
	void search_for_goal(Mat);
	void search_for_ball(Mat);
};

#endif /* VISION_H_ */
