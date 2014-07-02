/*
 * MotorController.h
 *
 *  Created on: Nov 13, 2013
 *      Author: Kellen Carey
 */

#ifndef MOTORCONTROLLER_H_
#define MOTORCONTROLLER_H_

#define MAINFILE

#include <vector>
#include <ctime>
#include <iostream>
#include <fstream>
#include <cmath>

//#define MOTIONS_PREPATH                       "/home/mul8/Documents/MU-L8_Motion_Files_limb_select/"
#define TEMP_MOTION_FILE		"/home/unicorn/Documents/motions_text_file_select_limbs.mtn"
#define TEMP_MOTION_QUEUE_FILE	"/home/unicorn/Documents/motions_queue_text_file.mtnq"
#define ZERO_POSE_FILE			"/home/unicorn/Documents/MU-L8_Motion_Files_limb_select/zero_pose_file.txt"
#define MOTIONS_PREPATH	 "/home/unicorn/Documents/MU-L8_Motion_Files_limb_select/"
#define DATA_PREPATH	 "/home/unicorn/Dropbox/MUL8_onboard/Motion_testing/Position_testing/"



#define NUM_MOTORS      12
#define NUM_LEG_MOTORS  12
#define NUM_ARM_MOTORS  6
#define TOTAL_MOTORS    24

#define MOTIONS_TOTAL 9
#define SPEED_CONSTANT  0.1285
#define INITIAL_SPEED   15

#define CHECK_POSITIONS         6
#define POSITION_TOLERANCE      25
#define P_GAIN          28      // This is the address of the P value, DO NOT CHANGE

#define DEGREES_PER_POSITION    0.08789
#define RADIANS_PER_DEGREE              0.01745
#define P_FOR_PID       32      // Change this value to change the P value
#define NEW_RETURN_DELAY_TIME   10
#define STATUS_RETURN                   1       // 0 returns nothing, 1 returns only read, 2 returns everything
#define LEG_CUTOFF                              12


#define TORQUE_ENABLE                   24
#define MOVING_SPEED                    32
#define GOAL_ACCELERATION               73

#define DEFAULT_MOVING_SPEED    50
#define DEFAULT_ACCELERATION    15
#define LOW_ACCELERATION                15
#define GOAL_POSITION                   30
#define PRESENT_POSITION                36
#define RETURN_DELAY_TIME               5
#define STATUS_RETURN_LEVEL             16
#define MAX_TORQUE                              14
#define TORQUE_LIMIT                    34

#define BYTE                                    1
#define WORD                                    2

#define MUL8_HEAD_UP					0
#define MUL8_HEAD_DOWN					1
#define MUL8_HEAD_LEFT					2
#define MUL8_HEAD_RIGHT					3
#define MUL8_HEAD_UP_LEFT				4
#define MUL8_HEAD_DOWN_LEFT				5
#define MUL8_HEAD_UP_RIGHT				6
#define MUL8_HEAD_DOWN_RIGHT			7
#define MUL8_HEAD_CENTER				8

#define BATTERY_TIMEOUT                 20
#define VOLTAGE_THRESHHOLD              105

#define M1_CW   100
#define M1_CCW  1575

#define M2_CW   2750
#define M2_CCW  3730

#define M3_CW   660
#define M3_CCW  2578

#define M4_CW   525
#define M4_CCW  2430

#define M5_CW   1923
#define M5_CCW  3147

#define M6_CW   1700
#define M6_CCW  2460

#define M7_CW   500
#define M7_CCW  2029

#define M8_CW   30
#define M8_CCW  1560

#define M9_CW   1976
#define M9_CCW  3860

#define M10_CW  455
#define M10_CCW 1850

#define M11_CW  1376
#define M11_CCW 2303

#define M12_CW  486
#define M12_CCW 1380

#define M13_CW  5
#define M13_CCW 3600

#define M14_CW  5
#define M14_CCW 3600

#define M15_CW  78
#define M15_CCW 1941

#define M16_CW  484
#define M16_CCW 2641

#define M17_CW  946
#define M17_CCW 1912

#define M18_CW  127
#define M18_CCW 1100

#define M19_CW  500
#define M19_CCW 3200

#define M20_CW  800
#define M20_CCW 3500

#define M21_CW  2010
#define M21_CCW 2840

#define M22_CW  210
#define M22_CCW 1116

#define M23_CW  635
#define M23_CCW 3360

#define M24_CW  383
#define M24_CCW 1017





struct Motion {
	std::string friendlyName;
	std::vector<double> time;                                               //Array of times for the motions
	std::vector<std::vector<int> > motorPositions;  //Array of motor positions for the motions
	std::vector<std::vector<int> > motorVelocities; //Array of motor velocities for the motions
	std::vector<std::vector<int> > motorIDs;                        //Array of motor IDs for motions.

	std::vector<int>  currPos;              //Array for holding potential new positions
	std::vector<int>  recoverPos;           //Array for recovering a single step's positions
	std::vector<int>  recoverVel;           //Array for recovering a single step's velocity
	int recoverTime;                //recover a single step's time

	int currentIndex;       //step index. 0-n                                                       //Integer marking the currently indexed step
	int length;                     //number of steps in a motion                           //Integer representing the size of the arrays
	int num_motors;
	int limb_select;
	//1--> ID 1-18  All
	//2--> ID 1-12  Legs
	//3--> ID 1-12(odd) right leg
	//4--> ID 1-12(even) left leg
	//5--> ID 13-18 arms
	//6--> ID 13-18(odd) right arm
	//7--> ID 13-18(even) left arm
	//8--> ID 23-24 head
};

struct MotionQueue {
	std::string friendlyName; //name of the queue
	std::vector<std::string>  motionList;   //names of the motions inside the queue
	std::vector<double> pauseTime;
	int currentIndex;       //index of the motionList vector
	int length;                     //number of motions in the motionList vector
};

class MotorController {
private:
	float timeSince(float);
	void initializeMotions(void);
	void initializeMotionQueues(void);
	void initialize();
	Motion getInitializedMotion(std::string, std::string);
	MotionQueue getInitializedMotionQueue(std::string, std::string);
	void changePID();
	void setReturnDelayTime();
	static void* executeMotion(void*);
public:
	MotorController();
	virtual ~MotorController();

	void init(void);
	bool step(bool);
	bool step(bool, unsigned long);
	bool setMotion(std::string);
	bool setMotionQueue(std::string);
	void getZeroPose();
	std::string getMotion(void);
	void setStatusReturnLevel(int);
	Motion getMotionFile();
	std::string getCurrentMotionName(void);


	void sendSyncWrite(std::vector<int>, int, int, std::vector<int>);
	void executeNext(Motion);
	void executePrevious(void);

	bool incrementStep(void);
	void incrementIndex(void);
	void decrementIndex(void); // Decrement step index
	bool incrementMotioninQueue(void);

	void disableMotionExecution(void);
	void enableMotionExecution(void);

	void disableMotor(int);
	void enableMotor(int);
	void disableAllMotors(void);
	void enableAllMotors(void);
	void lockHead(void);

	void disableLegs(void);
	void disableLeftLeg(void);
	void disableRightLeg(void);
	void disableSwayMotors(void);
	void enableLegs(void);
	void enableRightLeg(void);
	void enableLeftLeg(void);

	void disableArms(void);
	void disableLeftArm(void);
	void disableRightArm(void);
	void enableArms(void);
	void enableRightArm(void);
	void enableLeftArm(void);

	int readMotorPosition(int);
	int mainController(std::string in);
	int convDegrees(int, int);
	int getMotorPositionReadWord(int);
	std::vector<int> getMotorPositionSyncRead(int);
	void getCurrentPose();
	int errorCheck(int[], int);
	void getMirroredPose();
	void setMirroredPose();
	void mirrorCurrentMotion(void); //this function will take the mirror the current motion across the sagital plane and save it to a
	//new file with the <current_motion_name>_m.mtn

	void setTime(double); //takes new time input and passes it to editStep
	void setPauseTime(double); //sets the pause time after a motion in the motion queue
	void changeTime(double);//takes new time input for changing a step's time (without changing positions)
	void editStep(Motion&, int, double, std::vector<int>); /*edit motion struct to replace a step and calc new vel with time input*/
	void editStepTime(Motion&, int, double);//just change the time (speed) of a step
	void insertStep(void);
	void addLimbtoMotion(int); //this will resize the current motion to include a specific limb
	void printMotion(int); //this will print the current motion in "motion reading" format so the user can paste them into a txt file.
	void printMotionQueue(void);
	void addMotionStep(void); //Add a step to the current motion
	bool addMotion(std::string); // Add a new motion
	void setLimbSelection(int);
	bool addMotionQueue(std::string); //add a new motion queue to be filled with motion names
	void addMotionToQueue(std::string);

	void decrementQueueIndex(void);
	void incrementQueueIndex(void);
	void increaseQueueLength(void);
	int getBoundedPosition(int, int);
	int getQueueLength(void);
	int getQueuePause(void);



	void displayMotionStatus(void); //show the user which step we are on in a graphical manner
	void recalculateCurrentMotionSpeeds(void);
	void deleteCurrentStep(void);   //manipulate struct to delete step


	void moveHead(int, int);
	void stopHead(void);
	bool headLeftRightIsMoving();
	bool headUpDownIsMoving();
	double getHeadAngle();

	bool isExecuting();
};

#endif /* MOTORCONTROLLER_H_ */
