/*
 * MUL8.cpp
 *
 *  Created on: Jul 2, 2014
 *      Author: unicorn
 */

#include "MUL8.h"
#include "Vision.h"
#include "GameController.h"
#include <pthread.h>

#define MY_TEAM		0
#define MY_NUMBER	0

MotorController motorController;
Vision vis;
GameController gameController;
int MUL8_action = 0;
int MUL8_state = -1;

int currentThreshold = 80;
int firstThreshold = 80;
int secondThreshold = 70;

bool MUL8_initialized = false;
bool MUL8_ready = false;
bool MUL8_set = false;
bool MUL8_play_start = false;
bool MUL8_finished = false;
bool second_half = false;


/* Boolean used for search function */
bool mul8_knows_position = false;

struct RoboCupGameControlData myData;

MUL8::MUL8() {
	// TODO Auto-generated constructor stub
}

MUL8::~MUL8() {
	// TODO Auto-generated destructor stub
}

bool MUL8::search() {
	bool position = false;
		bool done = false;

		vis.setAction(SEARCH_FOR_GOAL);
		//	vis.setAction(CENTER_BALL);

		while (!done) {
			if (!position && vis.knowsRobotPosition()) {
				vis.setAction(SEARCH_FOR_BALL);
				std::cout << "Searching for Ball" << std::endl;
				position = true;
			}
			else if (vis.knowsBallPosition()) {
				std::cout << "Searching for both" << std::endl;
				setAction(MUL8_ACTION_WALK_TOWARDS_BALL);
				done = true;
			}

			int c = waitKey(1);

			if ((char)c == 27) {
				done = true;
			}

			vis.nextFrame();

			done = true;
		}
		// Robot knows where both the ball and the field are
	//	std::cout << "Vision knows where it is, and where the ball is.\n" <<
	//			"We would now normally wait for the robot to move," <<
	//			" and then update our world." << std::endl;
	//	std::cout << std::endl;
	//	std::cout << "The robot is located: (" << vis.getRobotX() <<
	//			", " << vis.getRobotY() << ") @ " << vis.getRobotTheta()
	//			<< std::endl;
	//	std::cout << "The ball is located: (" << vis.getBallX() << ", "
	//			<< vis.getBallY() << ")" << std::endl;

		return done;
}
/*
 * This function returns true when it is near the ball, or false if it loses track of the ball.
 */
bool MUL8::walkTowardsBall(int distance_to_ball) {
	bool done = false;

	std::cout << "Walking towards the ball time" << std::endl;

	vis.setAction(CENTER_BALL);
//	double temp_time = getUnixTime();
	// Search for ball for 6 seconds
//	while ((getUnixTime()-temp_time) < 6) {
//		vis.nextFrame();
//	}
//	temp_time = getUnixTime();


//	double waitTimer = getUnixTime();
	// TODO I forgot how the robot's X and Y are oriented, might have to change some vision code if this doesn't work
	//	while (!done) {

	double turningTheta = motorController.getHeadAngle()*-1;
	double previous_distance = vis.getBallDistance();

	// TODO We assume here that each turn left motion turns us approximately 45 degrees
	if (turningTheta > 23) {
		std::cout << "Robot turning left. New theta = " << vis.getRobotTheta() << std::endl;
		if (motorController.getMotion() == "W1") {
			motorController.setMotion("W1i");
		}
		else if (motorController.getMotion() == "W2") {
			motorController.setMotion("W2i");
		}
		doMotion();
		// Don't need to update odometry as these motions should not affect any odometry data
		motorController.setMotion("Tl30");
		doMotion();
		vis.updateRobotPosition(ODOMETRY_TL30_DISTANCE, ODOMETRY_TL30_THETA);
		vis.updateRobotTheta(ODOMETRY_TL30_TURN);
	}
	// TODO We assume here that each turn right motion turns us approximately 45 degrees
	else if (turningTheta < -23) {
		std::cout << "Robot turning right. New theta = " << vis.getRobotTheta() << std::endl;
		if (motorController.getMotion() == "W1") {
			motorController.setMotion("W1i");
		}
		else if (motorController.getMotion() == "W2") {
			motorController.setMotion("W2i");
		}
		doMotion();
		std::cout << "Setting motion to TR" << std::endl;
		motorController.setMotion("Tr30");
		doMotion();
		vis.updateRobotPosition(ODOMETRY_TR30_DISTANCE, ODOMETRY_TR30_THETA);
		vis.updateRobotTheta(ODOMETRY_TR30_TURN);
		std::cout << "Robot turning right. New theta = " << vis.getRobotTheta() << std::endl;
	}
	// Here we assume that we are facing in the correct direction, so let's walk forward
	else if (previous_distance > distance_to_ball) { //
		std::string currMo = motorController.getMotion();
		if (currMo == "Wi") {
			std::cout << "Position W0_m" << std::endl;
			motorController.setMotion("W0_m");
			vis.updateRobotPosition(ODOMETRY_W0_M_DISTANCE, ODOMETRY_W0_M_THETA);
			vis.updateRobotTheta(ODOMETRY_W0_M_TURN);
//			waitTimer = getUnixTime();
		}
		else if (currMo == "W0_m"/* && ((getUnixTime()-waitTimer) > 1)*/) {
			motorController.setMotion("W2");
			vis.updateRobotPosition(ODOMETRY_W2_DISTANCE, ODOMETRY_W2_THETA);
			vis.updateRobotTheta(ODOMETRY_W2_TURN);
			std::cout << "Position W2" << std::endl;
		}
//		else if (currMo == "W0_m") {
//
//		}
		else if (currMo == "W1") {
			motorController.setMotion("W2");
			vis.updateRobotPosition(ODOMETRY_W2_DISTANCE, ODOMETRY_W2_THETA);
			vis.updateRobotTheta(ODOMETRY_W2_TURN);
			std::cout << "Position W2" << std::endl;
		}
		else if (currMo == "W2") {
			motorController.setMotion("W1");
			vis.updateRobotPosition(ODOMETRY_W1_DISTANCE, ODOMETRY_W1_THETA);
			vis.updateRobotTheta(ODOMETRY_W1_TURN);
			std::cout << "Position W1" << std::endl;
		}
		else {
			std::cout << "Old motion was " << currMo << std::endl;
			motorController.setMotion("Wi");
			// No need to update odometry, as no foot movements are involved
			std::cout << "Position Wi" << std::endl;
		}
		doMotion();
		//				motorController.setMotion("Walk Forward");
		std::cout << "Robot walking forward. New coordinates: (" << vis.getRobotX() << ", " << vis.getRobotY() << ")" << std::endl;
		std::cout << "Distance to ball: " << previous_distance << std::endl;
	}
	else {
		// We are within 1 meter of the ball
		//should call getBehindBall();
		// At this point, we would determine how to move, whether to move to the far side of the ball, or walk forward and kick it

		std::string currMo = motorController.getMotion();
		if (currMo == "W2") {
			motorController.setMotion("W2i");
			std::cout << "Returning from Position W2" << std::endl;
		}
		else if (currMo == "W1") {
			motorController.setMotion("W1i");
			std::cout << "Returning from Position W1" << std::endl;
		}

		doMotion();
		// No need to update odometry, as these motions should not affect any odometry data
		done = true;
	}
	//	}

	return done;
}
/*
 * This function returns true when it is behind the ball, or false if it loses sight of the ball or the ball moves too far away.
 */
bool MUL8::getBehindBall() {
	bool result;

	double theta = vis.getRobotTheta();

	std::cout << "Received Vision reference has theta of " << theta << std::endl;

	std::string motion;


	// If theta is negative, we have to side step right
	if (theta < -7) {
		motion = "SSr";
	}
	// If theta is positive, we have to side step left
	else {
		motion = "SSl";
	}

	// Need to keep side stepping and turning until our theta is zero
	if (theta < -12 || theta > 12 ) {
		motorController.setMotion(motion);
		doMotion();
		if (motion == "SSr") {
			vis.updateRobotPosition(ODOMETRY_SSR_DISTANCE, ODOMETRY_SSR_THETA);
			vis.updateRobotTheta(ODOMETRY_SSR_TURN);
		}
		else {
			vis.updateRobotPosition(ODOMETRY_SSL_LONG_DISTANCE, ODOMETRY_SSL_LONG_THETA);
			vis.updateRobotTheta(ODOMETRY_SSL_LONG_TURN);
		}
		double robot_head_theta = motorController.getHeadAngle();
		if (robot_head_theta < -7) {
			turn(robot_head_theta);
			//			doMotion(vis, motorController);
			robot_head_theta = motorController.getHeadAngle();
		}
		else if (robot_head_theta > 7) {
			turn(robot_head_theta);
			//			doMotion(vis, motorController);
			robot_head_theta = motorController.getHeadAngle();
		}
		theta = vis.getRobotTheta();
		std::cout << "My new theta is: " << theta << std::endl;
	}
	else {
		result = true;
		std::cout << "Got behind ball!" << std::endl;

		// Set the second threshold value so that when walkTowardsBall is called, MUL8 gets close enough to kick
		currentThreshold = secondThreshold;
	}

	return result;
}
/*
 * This function returns true if it is ready to kick or false if something else happens
 */
bool MUL8::alignToKick() {
	bool result = true;

	vis.nextFrame();
	std::string motion = vis.getMotionRequest();
	if (motion != "KickL" && motion != "KickR") {
		motorController.setMotion(motion);
		doMotion();
		motion = vis.getMotionRequest();
		result = false;
	}
	else {
		motorController.moveHead(MUL8_HEAD_UP, 500);
		motorController.setMotion(motion);
		doMotion();
	}

	// Reset the threshold so MUL8 gets behind the ball
	currentThreshold = firstThreshold;

	return result;
}

bool MUL8::checkLocation() {

	bool result = false;

	vis.nextFrame();
	double robotTheta = vis.getRobotTheta();
	int motor23pos = motorController.getMotorPositionReadWord(23);
	int motor24pos = motorController.getMotorPositionReadWord(24);
	double robotHeadTheta = motorController.getHeadAngle();
	vis.setAction(WAIT);
	if (robotTheta < 0) {
		while (robotTheta < robotHeadTheta) {
			// MUL8 needs to turn head left in order to find the goal
			motorController.moveHead(MUL8_HEAD_UP_LEFT, 256);
			vis.nextFrame();
			robotHeadTheta = motorController.getHeadAngle();
		}
		motorController.stopHead();
	}
	else {
		while (robotHeadTheta < robotTheta) {
			// MUL8 needs to turn head right in order to find the goal
			motorController.moveHead(MUL8_HEAD_UP_RIGHT, 256);
			vis.nextFrame();
			robotHeadTheta = motorController.getHeadAngle();
		}
		motorController.stopHead();
	}

	// The head should be pointing towards the goal, so set the vision action to find the goal
	vis.setAction(CENTER_GOAL);

	// Cycle through once, if the goal is centered, then the action should remain LOCALIZE_GOAL
	vis.nextFrame();

//	while (!localized) {
		if (vis.getAction() == LOCALIZE_GOAL) {
			double headAngle = motorController.getHeadAngle();
			std::cout << "Robot Head Theta: " << headAngle << std::endl;
			double robotAngle = vis.getRobotTheta();
			std::cout << "Robot Theta: " << robotAngle << std::endl;

			vis.updateRobotTheta(headAngle-robotAngle);

			motorController.setMotorPosition(23, motor23pos, 256);
			motorController.setMotorPosition(24, motor24pos, 256);

			result = true;
		}

		return result;
//		vis.nextFrame();
//	}

//	// This should update the robot's position estimation
//	for (int i = 0; i < 20; i++) {
//		vis.nextFrame();
//	}



}

double MUL8::getUnixTime() {
	struct timespec tv;

	if (clock_gettime(CLOCK_REALTIME, &tv) != 0) {
		return 0;
	}

	return (((double)tv.tv_sec) + (tv.tv_nsec / 1000000000.0));
}

void MUL8::doMotion() {
	bool done = false;
	double startTime = getUnixTime();
	double currentTime = getUnixTime();
	double stepTime;
	while (!done) {
		int step_result = motorController.step(false);
		switch (step_result) {
		case MUL8_STEP_NULL:
			break;
		case MUL8_STEP_FINISHED:
			// Loop through vision until it's time to execute the next motion
			startTime = getUnixTime();
			currentTime = getUnixTime();
			stepTime = motorController.getStepTime();
			stepTime -= 0.15;	// Subtract 150 mS to deal with frame processing time
			do {
				vis.nextFrame();
				currentTime = getUnixTime();
			} while ((currentTime-startTime) < stepTime);
			break;
		case MUL8_MOTION_FINISHED:
			done = true;
			break;
		}
	}
}

void MUL8::turn(int theta){

	if(theta<0){
		//turn left
		if(theta<0 && theta>=-15){
			//between 0 and -15
			motorController.setMotion("Tl15");
			vis.updateRobotPosition(ODOMETRY_TL15_DISTANCE, ODOMETRY_TL15_THETA);
			vis.updateRobotTheta(ODOMETRY_TL15_TURN);
		}
		else if(theta<-15 && theta>=-30){
			//between -15 and -30
			motorController.setMotion("Tl30");
			vis.updateRobotPosition(ODOMETRY_TL30_DISTANCE, ODOMETRY_TL30_THETA);
			vis.updateRobotTheta(ODOMETRY_TL30_TURN);
		}
		else if(theta<-30 && theta>-45){
			//between -30 and -45
			motorController.setMotion("Tl45");
			vis.updateRobotPosition(ODOMETRY_TL45_DISTANCE, ODOMETRY_TL45_THETA);
			vis.updateRobotTheta(ODOMETRY_TL45_TURN);
		}
	}

	else if(theta>=0){
		if(theta>=0 && theta<15){
			//between 0 and 15
			motorController.setMotion("Tr15");
			vis.updateRobotPosition(ODOMETRY_TR15_DISTANCE, ODOMETRY_TR15_THETA);
			vis.updateRobotTheta(ODOMETRY_TR15_TURN);
		}
		else if(theta>=15 && theta<30){
			//between 15 and 30
			motorController.setMotion("Tr30");
			vis.updateRobotPosition(ODOMETRY_TR30_DISTANCE, ODOMETRY_TR30_THETA);
			vis.updateRobotTheta(ODOMETRY_TR30_TURN);
		}
		else if(theta>=30 && theta<45){
			//between 30 and 45
			motorController.setMotion("Tr30");
			vis.updateRobotPosition(ODOMETRY_TR30_DISTANCE, ODOMETRY_TR30_THETA);
			vis.updateRobotTheta(ODOMETRY_TR30_TURN);

		}
	}
	else{
		//do nothing?
	}
	doMotion();
}

void MUL8::init() {
	if (!MUL8_initialized) {
		motorController.init();
		vis.init(motorController);
		motorController.setMotion("Wi");
		motorController.step(false);
		vis.setAction(SEARCH_FOR_GOAL);
		MUL8_initialized = true;
	}
}

void MUL8::ready() {
	if (!MUL8_ready) {
		std::cout << "I'm ready!" << std::endl;
		MUL8_ready = true;
	}
}

void MUL8::set() {
	if (!MUL8_set) {
		std::cout << "I'm set!" << std::endl;
		MUL8_set = true;
	}
}

void MUL8::play() {
	if (!MUL8_play_start) {
		std::cout << "Let's play!!!!!" << std::endl;
		MUL8_play_start = true;
		setAction(MUL8_ACTION_SEARCH);
	}
	//	setAction(MUL8_ACTION_SEARCH);
	actionStep();
}

void MUL8::penalty() {
	gameController.getGCData(myData);
	std::cout << "Waiting in penalty mode" << std::endl;
	while (myData.teams[MY_TEAM].players[MY_NUMBER].secsTillUnpenalised > 0) {
		gameController.getGCData(myData);
		vis.nextFrame();
		if (myData.teams[MY_TEAM].players[MY_NUMBER].penalty == 0) {
			break;
		}
	}
}

void MUL8::finish() {
	if (!MUL8_finished) {
		std::cout << "Done playing :(" << std::endl;
		MUL8_finished = true;
		//		MUL8_ready = false;
		//		MUL8_set = false;
		//		MUL8_play_start = false;
	}
}

void MUL8::actionStep() {
	switch (MUL8_action) {
	case MUL8_ACTION_SEARCH:
		if (search()) {
			setAction(MUL8_ACTION_WALK_TOWARDS_BALL);
		}
		break;
	case MUL8_ACTION_WALK_TOWARDS_BALL:
		if(walkTowardsBall(currentThreshold)) {
			if(currentThreshold == firstThreshold) {
				setAction(MUL8_ACTION_CHECK_LOCATION);
			}
			else {
				setAction(MUL8_ACTION_ALIGN_TO_KICK);
			}
		}
		break;
	case MUL8_ACTION_ALIGN_TO_KICK:
		if(alignToKick()) {
			setAction(MUL8_ACTION_WALK_TOWARDS_BALL);
			currentThreshold = firstThreshold;
		}
		break;
	case MUL8_ACTION_GET_BEHIND_BALL:
		if (getBehindBall()) {
			setAction(MUL8_ACTION_WALK_TOWARDS_BALL);
			currentThreshold = secondThreshold;
		}
		break;
	case MUL8_ACTION_CHECK_LOCATION:
		if (checkLocation()) {
			setAction(MUL8_ACTION_GET_BEHIND_BALL);
		}
		break;
	default:
		break;
	}
}

void MUL8::setAction(int new_action) {
	MUL8_action = new_action;
}

void MUL8::setState(int new_state) {
	MUL8_state = new_state;
}

void MUL8::step() {
	gameController.getGCData(myData);

	int MUL8_state = myData.state;
	int penalty_occured = myData.teams[0].players[0].penalty;
	if (!second_half && myData.firstHalf == 0) {
		second_half = true;
		MUL8_ready = false;
		MUL8_set = false;
		MUL8_play_start = false;
	}
	if (penalty_occured == 0) {

		switch(MUL8_state) {
		case MUL8_STATE_INIT:
			init();
			break;
		case MUL8_STATE_READY:
			ready();
			break;
		case MUL8_STATE_SET:
			set();
			break;
		case MUL8_STATE_PLAY:
			play();
			break;
		case MUL8_STATE_FINISH:
			finish();
			break;
		case MUL8_STATE_PENALTY:
			penalty();
			break;
		default:
			break;
		}
	}

	else {
		penalty();
	}
}

