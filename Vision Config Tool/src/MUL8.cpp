/*
 * MUL8.cpp
 *
 *  Created on: Jul 2, 2014
 *      Author: unicorn
 */

#include "MUL8.h"
#include "Vision.h"
#include <pthread.h>

MotorController mC;
Vision vis;
int MUL8_action = 0;
int MUL8_state = -1;

bool MUL8_initialized = false;
bool MUL8_ready = false;
bool MUL8_set = false;
bool MUL8_play_start = false;
bool MUL8_finished = false;

MUL8::MUL8() {
	// TODO Auto-generated constructor stub
}

MUL8::~MUL8() {
	// TODO Auto-generated destructor stub
}

void MUL8::search() {
	bool position = false;
	bool done = false;

	vis.setAction(SEARCH_FOR_GOAL);
	//	vis.setAction(CENTER_BALL);

//	while (!done) {
		if (!position && vis.knowsRobotPosition()) {
			vis.setAction(SEARCH_FOR_BALL);
			std::cout << "Searching for Ball" << std::endl;
			position = true;
		}
		else if (vis.knowsBallPosition()) {
			std::cout << "Searching for both" << std::endl;
			done = true;
		}

		int c = waitKey(1);

		if ((char)c == 27) {
			done = true;
		}

		vis.nextFrame();
//	}
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
}
/*
 * This function returns true when it is near the ball, or false if it loses track of the ball.
 */
/*bool*/void MUL8::walkTowardsBall() {
	bool done = false;

	std::cout << "Walking towards the ball time" << std::endl;

	vis.setAction(CENTER_BALL);
	clock_t temp_time = clock();

	// Search for ball for 6 seconds
	while ((clock()-temp_time)/CLOCKS_PER_SEC < 6) {
		vis.nextFrame();
	}
	temp_time = clock();

	clock_t last_clock;
	clock_t waitTimer = clock();
	last_clock = clock();
	bool waitVision = false;
	int count = 1;
	// TODO I forgot how the robot's X and Y are oriented, might have to change some vision code if this doesn't work
//	while (!done) {
		vis.nextFrame();
		double robotTheta = vis.getRobotTheta();

		std::cout << "One Main Loop" << std::endl;

		double turningTheta = 0;
		if (!mC.isExecuting()) {
			turningTheta = mC.getHeadAngle()*-1;
			std::cout << "Got turning theta" << std::endl;
		}
		double previous_distance = vis.getBallDistance();

		// TODO We assume here that each turn left motion turns us approximately 45 degrees
		if (turningTheta > 23) {
			std::cout << "Need to turn left" << std::endl;
			//			vis.nextFrame();
			// FIXME Timer for testing without motions
			//			if ((clock()-last_clock)/CLOCKS_PER_SEC > 1) {
			// Need to turn left
			if (!mC.isExecuting()) {
				std::cout << "Robot turning left. New theta = " << vis.getRobotTheta() << std::endl;
				if (mC.getMotion() == "W1") {
					mC.setMotion("W1i");
				}
				else if (mC.getMotion() == "W2") {
					mC.setMotion("W2i");
				}
				while (mC.isExecuting()) {
					vis.nextFrame();
				}
				mC.setMotion("Tl");
				while (mC.isExecuting()) {
					vis.nextFrame();
				}
				mC.setMotion("Wi");
				vis.nextFrame();
				vis.updateRobotTheta(45);
//				last_clock = clock();
				//				vis.nextFrame();
			}
		}
		// TODO We assume here that each turn right motion turns us approximately 45 degrees
		else if (turningTheta < -23) {
			std::cout << "Need to turn right" << std::endl;
			//			vis.nextFrame();
			// FIXME Timer for testing without motions
			// Need to turn right
			if (!mC.isExecuting()) {
				std::cout << "Robot turning right. New theta = " << vis.getRobotTheta() << std::endl;
				if (mC.getMotion() == "W1") {
					mC.setMotion("W1i");
				}
				else if (mC.getMotion() == "W2") {
					mC.setMotion("W2i");
				}
				while (mC.isExecuting()) {
					vis.nextFrame();
				}
				mC.setMotion("Tr");
				while (mC.isExecuting()) {
					vis.nextFrame();
				}
//				std::cout << "Motion finished executing" << std::endl;
				mC.setMotion("Wi");
				vis.nextFrame();
				vis.updateRobotTheta(-45);
				std::cout << "Robot turning right. New theta = " << vis.getRobotTheta() << std::endl;
//				last_clock = clock();
			}
		}
		// Here we assume that we are facing in the correct direction, so let's walk forward
		else if (previous_distance > 60) {
			std::cout << "Need to walk forward" << std::endl;
			// FIXME Timer for testing without motions
			//			if ((clock()-last_clock)/CLOCKS_PER_SEC > 1) {
			//std::cout << "Entered Walking step" << std::endl;
			if (!mC.isExecuting()) {
				//				vis.nextFrame();
				std::string currMo = mC.getMotion();
				if (currMo == "Wi") {
					std::cout << "Position W0" << std::endl;
					mC.setMotion("W0");
					waitTimer = clock();
				}
				else if (currMo == "W0" && ((clock()-waitTimer)/CLOCKS_PER_SEC > 1)) {
					mC.setMotion("W1");
					std::cout << "Position W1" << std::endl;
				}
				else if (currMo == "W1") {
					mC.setMotion("W2");
					std::cout << "Position W2" << std::endl;
				}
				else if (currMo == "W2") {
					mC.setMotion("W1");
					std::cout << "Position W1" << std::endl;
				}
				else {
					mC.setMotion("Wi");
					std::cout << "Position Wi" << std::endl;
				}
				//				motorController.setMotion("Walk Forward");
				std::cout << "Robot walking forward. New coordinates: (" << vis.getRobotX() << ", " << vis.getRobotY() << ")" << std::endl;
				std::cout << "Distance to ball: " << previous_distance << std::endl;
				// TODO We assume that the robot moves 3 cm every time it takes a step
				vis.updateRobotPosition(3, robotTheta);
			}
			//			if ((clock()-temp_time)/CLOCKS_PER_SEC > 0.1) {
			////				vis.nextFrame();
			//				temp_time = clock();
			//			}
		}
		else {
			std::cout << "Stopping and standing" << std::endl;
			// We are within 1 meter of the ball
			// At this point, we would determine how to move, whether to move to the far side of the ball, or walk forward and kick it

			// TODO Add check to make sure robot has finished moving
			//std::cout << "We are close enough to the ball" << std::endl;
			if (!mC.isExecuting()) {
				std::string currMo = mC.getMotion();
				if (currMo == "W2") {
					mC.setMotion("W2i");
					std::cout << "Returning from Position W2" << std::endl;
				}
				else if (currMo == "W1") {
					mC.setMotion("W1i");
					std::cout << "Returning from Position W1" << std::endl;
				}

				while (mC.isExecuting()) {}
				done = true;
			}
		}
//	}
}
/*
 * This function returns true when it is behind the ball, or false if it loses sight of the ball or the ball moves too far away.
 */
/*bool*/void MUL8::getBehindBall() {
	bool done = false;
	bool turning = false;
	bool turned = false;
	bool walking = false;

	clock_t last_clock = clock();

//	while (!done) {
		vis.nextFrame();
		if ((clock()-last_clock)/CLOCKS_PER_SEC > 1) {
			last_clock = clock();
			double robotTheta = vis.getRobotTheta();

			if (turned) {
				// Walk forward until we're past the ball
				if (!walking) {
					// motorController.setMotion("Walk");
					walking = true;
				}
				else if (!mC.isExecuting()) {
					// We need to loop until we pass the ball
					double headTheta = vis.getRobotHeadTheta();
					if (headTheta < -100) {
						// We need to turn left to face the ball
						//motorController.setMotion("Turn Left");
					}
					else if (headTheta > 100) {
						// Here we need to turn right to face the ball
						// motorController.setMotion("Turn Right");
					}
					else if (/* motorController.getMotion() != "Walk" */ false) {
						// Test to see if we need to keep turning
						if (headTheta < 23 && headTheta > -23) {
							// Don't need to keep turning
							done = true;
						}
						else {
							// Need to keep turning
							// std::string currentMotion = motorController.getMotion();
							// motorController.setMotion(currentMotion);
						}
					}
					else {
						// motorController.setMotion("Walk");
					}
				}
			}


			if (robotTheta < -90) {
				// Robot should turn right to get behind the ball
				std::cout << "Need to turn right to get around ball" << std::endl;
				// After turning, robot should walk forward around 1.5 meters, then turn to face the ball
				if (!turning) {
					// motorController.setMotion("Turn Right");
					turning = true;
				}
				else if (!mC.isExecuting()) {
					// FIXME Assuming -45 degrees per right turn
					vis.updateRobotTheta(-45);
					turned = false;
				}
			}
			else if (robotTheta > 90) {
				// Robot should turn left to get behind the ball
				std::cout << "Need to turn left to get around ball" << std::endl;
				// After turning, robot should walk forward around 1.5 meters, then turn to face the ball
				if (!turning) {
					// motorController.setMotion("Turn Left");
					// turning = true;
				}
				else if (!mC.isExecuting()) {
					// FIXME Assuming 45 degrees per left turn
					vis.updateRobotTheta(45);
					turned = false;
				}
			}
			else {
				// We are facing in the correct direction, so we need to line up the ball and the goal
				std::cout << "Need to line up ball and goal" << std::endl;
				done = true;
			}
		}
//	}
}
/*
 * This function returns true if it is ready to kick or false if something else happens
 */
/*bool*/void MUL8::alignToKick() {
	vis.setAction(SEARCH_FOR_BOTH);
}

void MUL8::init() {
	if (!MUL8_initialized) {
		mC.init();
		vis.init(mC);
		mC.setMotion("Wi");
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
	}
	setAction(MUL8_ACTION_SEARCH);
	actionStep();
}

void MUL8::penalty() {

}

void MUL8::finish() {
	if (!MUL8_finished) {
	std::cout << "Done playing :(" << std::endl;
		MUL8_finished = true;
	}
}

void MUL8::actionStep() {
	switch (MUL8_action) {
	case MUL8_ACTION_SEARCH:
		search();
		break;
	case MUL8_ACTION_WALK_TOWARDS_BALL:
		walkTowardsBall();
		break;
	case MUL8_ACTION_ALIGN_TO_KICK:
		alignToKick();
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

