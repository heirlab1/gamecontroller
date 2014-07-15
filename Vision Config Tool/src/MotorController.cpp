/*
 * MotorController.cpp
 *
 *  Created on: Nov 13, 2013
 *      Author: Kellen Carey
 */

/*Belongs to MotionPlanner*/
#include "MotorController.h"
#include "balanceServer.h"
#include <string>
#include <sstream>
#include <stdlib.h>
#include <vector>
#include <iomanip>
#include <cmath>
#include <iostream>
#include <fstream>
#include <map>

//#include "Dyna.h"

#include <dynamixel.h>

balanceServer balance_server;
bool motionExecutionDisabled= true;//todo

double lastClock, batteryClock;
std::map<std::string, Motion> motions;
std::map<std::string, Motion>::iterator motions_iterator;

std::map<std::string, MotionQueue> motionQueues;
std::map<std::string, MotionQueue>::iterator queue_iterator;

Motion currMo;
MotionQueue currQueue;
int zeroPose[NUM_MOTORS];
int mirrorSigns[]= {1, -1,  1, -1, -1, 1, -1, 1, 1, 1, 1, -1};
int mirroredPose[NUM_MOTORS];
std::vector<int> currentPose;

int currentTorque= 1023;        //0-1023
std::string currentMotion = "";
std::string currentMotionQueue = "";


std::vector<std::string> machineQueueNames;     //a list of all the motion queues that can be accessed by name (for writing to .mtnq)
std::vector<std::string> friendlyQueueNames;    //a list of all the motion queues that can be accessed by name (for writing to .mtnq)

std::vector<std::string> machineNames;  //a list of all the motions that can be accessed by name (for writing to .mtn)
std::vector<std::string> friendlyNames; //a list of all the motions that can be accessed by name (by the user with a friendly name)


// TODO add these motor status/battery checks to the final framework
double batteryLevel= 0.00;
int temperature= 0;
int checkBattery;
int result;

int balance_updates=0;
double balance_slowdown= 0;
int previous_balance= 10;

std::vector<int> motors;
std::vector<int> data;


double MotorController::getUnixTime() {
        struct timespec tv;

        if (clock_gettime(CLOCK_REALTIME, &tv) != 0) {
                return 0;
        }

        return (((double)tv.tv_sec) + (tv.tv_nsec / 1000000000.0));
}

// Default Constructor
MotorController::MotorController() {
        executing = false;
}

void MotorController::init() {
    // Initialize USB2Dynamixel
    //      motor_dynamixel.dxl_initialize(0,1);
    dxl_initialize(0,1);
    //Initialize the motors
    initialize();
    // Set the current motion to standing
    currentMotion = "Stand";


    motors.resize(TOTAL_MOTORS);
    for (int i = 0; i < TOTAL_MOTORS; i++) {
            motors[i] = i + 1;
    }
    //std::cout<<"resized motors array"<<std::endl;
    data.resize(TOTAL_MOTORS);
    //std::cout<<"resized data for syncwrite array"<<std::endl;
}

// Default Destructor
MotorController::~MotorController() {
        //      motor_dynamixel.dxl_terminate();
        dxl_terminate();
}
void MotorController::initialize() {
        lastClock = getUnixTime();
        batteryClock = getUnixTime();

        //changePID(P_FOR_PID);
        setStatusReturnLevel(STATUS_RETURN);
        setReturnDelayTime();
        initializeMotions();
        initializeMotionQueues();
        getZeroPose();

        // We have to initialize the currentMotion, so we'll let it be STAND
        currentMotion = "Stand";
        currentMotionQueue= "initial_queue";
}
//Initialize the Motions. Called from initialize()
void MotorController::initializeMotions() {


        // Define an input file stream, and open the number of motions file
        std::ifstream motionFile;

        std::string tab = "\t";
        std::string enter = "\n";

        // String variable to hold one line at a time from the text files
        std::string temp;


        // The number of steps in a particular motion
        int numMotions = 0;

        //Open a file that specifies the number of motions we need to initialize
        motionFile.open(TEMP_MOTION_FILE);

        while (std::getline(motionFile, temp)) {
                numMotions++;
        }
        motionFile.close();

        motionFile.open(TEMP_MOTION_FILE);

        //std::cout << "Number of lines found: " << numMotions << std::endl;

        //Let's get the total number of motion files
        //Currently this is not used, but should be later on for knowing how many motions to add to motions vector
        //getline(motionFile, temp);
        //std::stringstream(temp)>>numMotions;

        machineNames.resize(numMotions);
        friendlyNames.resize(numMotions);


        //machineNames.resize(1);
        //friendlyNames.resize(1);

        for (int i = 0; i < numMotions; i++) {
                //while ( getline(motionFile, temp)) {
                //      int i = 0;

                //machineNames.resize(machineNames.size() + 1);
                //friendlyNames.resize(friendlyNames.size() + 1);
                // Initialize the motor positions and velocities from the text file
                getline(motionFile, temp);
                while (temp.find(tab) == 0 || temp.find(enter) == 0)
                {
                        getline(motionFile, temp);
                }

                // Find the location of the space character
                unsigned space = temp.find(tab);

                // Push the first number into the motor position. In [i=rows][j=columns], i is the step number of the motion we're in and j is the motor number
                machineNames[i] = /*MOTIONS_PREPATH + */temp.substr(0, space);

                std::cout << "Motion Path:\n\t" << machineNames[i] << std::endl;

                // Push the second number into the velocity position
                friendlyNames[i] = temp.substr(space+1);

                std::cout << "Friendly Name:\n\t" << friendlyNames[i] << std::endl;

                // This way allows the motions to be overwritten, useful when adding new motions and re-initializing.
                //here the parameters are the real file path and the friendly name for that file
                motions[friendlyNames[i]] = getInitializedMotion((std::string)(MOTIONS_PREPATH) + machineNames[i], friendlyNames[i]);


                // This way does not allow the motions to be overwritten, which produces errors unless code is restarted
                //motions.insert(std::pair<std::string, Motion>(friendlyNames[i], getInitializedMotion((std::string)(MOTIONS_PREPATH + machineNames[i]), friendlyNames[i])));

                //i++;
        }
}

void MotorController::initializeMotionQueues(){

        // Define an input file stream, and open the number of motions file
        std::ifstream motionQueueFile;

        std::string tab = "\t";
        std::string enter = "\n";

        // String variable to hold one line at a time from the text files
        std::string temp;


        // The number of steps in a particular motion
        int numMotionQueues = 0;

        //Open a file that specifies the number of motions we need to initialize
        motionQueueFile.open(TEMP_MOTION_QUEUE_FILE);

        while (std::getline(motionQueueFile, temp)) {
                numMotionQueues++;
        }
        motionQueueFile.close();

        motionQueueFile.open(TEMP_MOTION_QUEUE_FILE);

        //std::cout << "Number of lines found: " << numMotions << std::endl;

        //Let's get the total number of motion files
        //Currently this is not used, but should be later on for knowing how many motions to add to motions vector
        //getline(motionFile, temp);
        //std::stringstream(temp)>>numMotions;

        machineQueueNames.resize(numMotionQueues);
        friendlyQueueNames.resize(numMotionQueues);


        //machineNames.resize(1);
        //friendlyNames.resize(1);

        for (int i = 0; i < numMotionQueues; i++) {
                //while ( getline(motionFile, temp)) {
                //      int i = 0;

                //machineNames.resize(machineNames.size() + 1);
                //friendlyNames.resize(friendlyNames.size() + 1);
                // Initialize the motor positions and velocities from the text file
                getline(motionQueueFile, temp);
                while (temp.find(tab) == 0 || temp.find(enter) == 0)
                {
                        getline(motionQueueFile, temp);
                }

                // Find the location of the space character
                unsigned space = temp.find(tab);

                // Push the first name into the machine name column.  i is the step number of the motion we're in
                machineQueueNames[i] = /*MOTIONS_PREPATH + */temp.substr(0, space);

                std::cout << "Motion Queue Path:\n\t" << machineQueueNames[i] << std::endl;

                // Push the second number into the velocity position
                friendlyQueueNames[i] = temp.substr(space+1);

                std::cout << "Friendly Queue Name:\n\t" << friendlyQueueNames[i] << std::endl;

                // This way allows the motions to be overwritten, useful when adding new motions and re-initializing
                motionQueues[friendlyQueueNames[i]] = getInitializedMotionQueue((std::string)(MOTIONS_PREPATH) + machineQueueNames[i], friendlyQueueNames[i]);

                // This way does not allow the motions to be overwritten, which produces errors unless code is restarted
                //motions.insert(std::pair<std::string, Motion>(friendlyNames[i], getInitializedMotion((std::string)(MOTIONS_PREPATH + machineNames[i]), friendlyNames[i])));

                //i++;
        }
        std::cout<< "SUCCESS LOADING ALL MOTIONS AND QUEUES"<<std::endl;
}
/* reads from the zero pose of the robot for reading positions as degrees/radians as well as mirroring. */
void MotorController::getZeroPose(){

        std::ifstream file;
        file.open(ZERO_POSE_FILE);
        std::string aString;

        for(int i=0; i<LEG_CUTOFF; i++){
                // each line holds the zero position of a motor. Read each line to the index of the zeroPose array
                getline(file, aString);
                std::stringstream(aString)>>zeroPose[i];

        }
        std::cout<< "SUCCESS LOADING ZERO POSE CONFIGURATION "<<std::endl;

}

/*
 * Initializes the motion associated with the file path passed in.
 * @param motion_file The file path for the motion file
 * @return tempMotion The initialized motion struct
 * Called from initializeMotions().
 */
Motion MotorController::getInitializedMotion(std::string motion_file, std::string friendlyName) {

        Motion tempMotion;
        std::ifstream file;
        // Example: Open the WALK_FILE from which to read the WALK motion

        file.open(motion_file.c_str());
        int numSteps = 0;
        std::string aString;
        double tempTime;

        std::string spaceChar   (" ");
        std::string tabChar     ("\t");
        std::string enterChar   ("\n");

        // Code for this motion
        getline(file, aString);                         // The first number in the file is the total number of steps
        std::stringstream(aString)>>numSteps;

        //find the number of motors used

        getline(file, aString);
        // if there is a tab character or new line, move on to the next line

        if (aString.find(tabChar) == 0 || aString.find(enterChar) == 0)
        {
                getline(file, aString);
        }

        std::stringstream(aString) >> tempMotion.num_motors;


        std::cout << "Motion Contains "<< tempMotion.num_motors << " motors" <<std::endl;
        std::cout << "Motion Contains "<< numSteps << " steps" <<std::endl;


        tempMotion.friendlyName = friendlyName;

        // Set the length, and resize the time and first position and velocity vectors
        tempMotion.length = numSteps;
        tempMotion.currentIndex = 0;
        tempMotion.motorPositions.resize(numSteps);
        tempMotion.motorVelocities.resize(numSteps);
        tempMotion.motorIDs.resize(numSteps);
        tempMotion.time.resize(numSteps);
        tempMotion.torqueReadings.resize(numSteps);
        tempMotion.motorCompliance.resize(numSteps);

        // Loop through the second position and velocity vectors, setting their size equal to the number of motors
        for (int i = 0; i < numSteps; i++)
        {
                tempMotion.motorPositions[i].resize(tempMotion.num_motors);
                tempMotion.motorVelocities[i].resize(tempMotion.num_motors);
                tempMotion.motorIDs[i].resize(tempMotion.num_motors);
                tempMotion.torqueReadings[i].resize(tempMotion.num_motors);
                tempMotion.motorCompliance[i].resize(tempMotion.num_motors); //TODO unless testing, torque should be 100%

                for(int j= 0; j<tempMotion.num_motors; j++){
                        tempMotion.motorCompliance[i][j]= DEFAULT_MAX_TORQUE; //TODO set to default 100%
                }
        }

        // Initialize the time, position, and velocity data fields
        for (int i = 0; i < numSteps; i++)
        {
                // Read and store the time
                getline(file, aString);
                if (aString.find(tabChar) == 0 || aString.find(enterChar) == 0)
                {
                        getline(file, aString);
                }

                std::stringstream(aString) >> tempTime;
                tempMotion.time[i] = tempTime;

                for (int j = 0; j < tempMotion.num_motors; j++)
                {
                        // Initialize the motor positions and velocities from the text file
                        getline(file, aString);
                        if (aString.find(tabChar) == 0 || aString.find(enterChar) == 0)
                        {
                                getline(file, aString);
                        }

                        // Find the location of the space character
                        int space = aString.find(tabChar);

                        std::string split_after_column_1 = aString.substr(space+1);

                        int space2= split_after_column_1.find(tabChar);

                        std::string split_after_column_2 = split_after_column_1.substr(space2+1);

                        int space3= split_after_column_2.find(tabChar);


                        // Push the first number into the motor position. In [i=rows][j=columns], i is the step number of the motion we're in and j is the motor number
                        std::stringstream(aString.substr(0, space))>> tempMotion.motorPositions[i][j];

                        if(space2> 0){
                        // Push the second number into the velocity position

                        std::stringstream(split_after_column_1.substr(0, space2)) >> tempMotion.motorVelocities[i][j];
                        }
                        if(space3>0){

                                // Push the third number into the motor ID position
                                // Push the fourth number into the compliancy position
                                std::stringstream(split_after_column_2.substr(0, space3)) >> tempMotion.motorIDs[i][j];

                                std::stringstream(split_after_column_2.substr(space3+1)) >> tempMotion.motorCompliance[i][j];

                        }


                        else{
                                std::stringstream(aString.substr(space+1)) >> tempMotion.motorVelocities[i][j];
                        }

                }

        }

        file.close();

        // Return the newly initialized motion
        return tempMotion;

}


/*
 * Initializes the motionQueue associated with the file path passed in.
 * @param motion_queue_file The file path for the motionQueue file
 * @return tempMotionQueue The initialized motionQueue struct
 * Called from initializeMotionQueue().
 */
MotionQueue MotorController::getInitializedMotionQueue(std::string motion_queue_file, std::string friendlyQueueName) {

        MotionQueue tempMotionQueue;
        std::ifstream file;
        // Example: Open the WALK_FILE from which to read the WALK motion

        file.open(motion_queue_file.c_str());
        int numMotions = 0;
        std::string aString;
        //double tempTime;
        std::string spaceChar   (" ");
        std::string tabChar     ("\t");
        std::string enterChar   ("\n");

        // Code for this motion
        getline(file, aString);                         // The first number in the file is the total number of steps
        std::stringstream(aString)>>numMotions;

        std::cout << "Queue Contains "<< numMotions << " steps" <<std::endl;

        tempMotionQueue.friendlyName = friendlyQueueName;

        // Set the length, and resize the time and first position and velocity vectors
        tempMotionQueue.length = numMotions;
        tempMotionQueue.currentIndex = 0;
        tempMotionQueue.motionList.resize(numMotions);
        tempMotionQueue.pauseTime.resize(numMotions);



        // Initialize the time, position, and velocity data fields
        for (int i = 0; i < numMotions; i++)
        {

                // Initialize the motor positions and velocities from the text file
                getline(file, aString);
                if (aString.find(tabChar) == 0 || aString.find(enterChar) == 0)
                {
                        getline(file, aString);
                }

                // Find the location of the space character
                unsigned space = aString.find(tabChar);

                // Push the first number into the motor position. In [i=rows] i is the step number of the motion we're in
                //std::stringstream(aString.substr(0, space))>> tempMotionQueue.motionList[i];
                tempMotionQueue.motionList[i] = aString.substr(0, space);
                // Push the second number into the velocity position
                std::stringstream(aString.substr(space+1)) >> tempMotionQueue.pauseTime[i];

                std::cout << "Motion "<< i << " added" <<std::endl;
        }
        std::cout << "Queue Built ^\n "<<std::endl;
        // Close the text file
        file.close();

        // Return the newly initialized motion
        return tempMotionQueue;

}

// Returns the time in seconds since the lastClock parameter
float MotorController::timeSince(double lastClock) {
        // Return the number of seconds since the last clock was taken
        return (((getUnixTime() - lastClock)));
}

/**
 *  Code to actually execute the next step
 *      called from MotorController.step(false)
 */
/**
 *  Code to actually execute the next step
 *	called from MotorController.step(false)
 */
void MotorController::executeNext(Motion motion) {
	std::vector<int> tempMotors;
	tempMotors.resize(motion.num_motors);//will take >0 positions from here

	int active_joints= motion.num_motors;	//default number of motors used for this motion
	int temp_counter=0;


	// Let the user know that the next step is being executed
	if(!motionExecutionDisabled){	//is robot in passive or active mode

		//list of the motor IDs to which the data will be written for this motion
		for(int j= 0; j<motion.num_motors; j++){

			//if the goal position is 0 that means we don't execute it-->exclude from motors[]
			if(motion.motorPositions[motion.currentIndex][j]>0){
				//std::cout<<" this is step "<<motion.currentIndex<<std::endl;
				tempMotors[j]= motion.motorIDs[motion.currentIndex][j];
			}
			//else, the position was a passive one for this joint--> decrement active joints
			else{
				active_joints--;
			}
		}

		for(int j= 0; j<motion.num_motors; j++){

			if(tempMotors[j]>0){
				motors[temp_counter]= tempMotors[j];
				temp_counter++;
			}
		}
		motors.resize(active_joints);


		if (motion.currentIndex == 0)
		{
			for (int i = 0; i < motion.num_motors; i++) {
				data[i] = LOW_ACCELERATION;
			}
			sendSyncWrite(motors,  GOAL_ACCELERATION, BYTE, data);
		}
		else if (motion.currentIndex == 1)
		{
			for (int i = 0; i < motion.num_motors; i++) {
				data[i] = DEFAULT_ACCELERATION;
			}
			sendSyncWrite(motors, GOAL_ACCELERATION, WORD, data);
		}


		if(motion.currentIndex==0){

			for (int i = 0; i < motion.num_motors; i++) {

				int finalPos= motion.motorPositions[0][i];
				data[i]= SPEED_CONSTANT*abs(dxl_read_word(i+1,PRESENT_POSITION)-finalPos)/(motion.time[0]);//+balance_slowdown); //find proper motor speeds to meet the time of the first step+ balance offset
			}
			balance_slowdown = 0;	//after the balance offset has been applied to this motion, set it back to zero- start fresh
		}

		else if(motion.currentIndex==1){

			std::cout<<"### SLOW DOWN LEAN: "<<balance_slowdown<<std::endl;

			for(int j=0; j<motion.num_motors; j++){
				data[j]= SPEED_CONSTANT*abs(currMo.motorPositions[motion.currentIndex-1][j]-currMo.motorPositions[motion.currentIndex][j])/(currMo.time[currMo.currentIndex]+balance_slowdown);
			}
			balance_slowdown = 0;	//after the balance offset has been applied to this motion, set it back to zero- start fresh

		}
		else if(motion.currentIndex==4){//this is the part when it falls back to recover after making the turn
			if(currentMotion == "Tl15" || currentMotion == "Tl30" || currentMotion == "Tl45" || currentMotion == "Tr15" || currentMotion == "Tr30" || currentMotion == "Tr45" ){

				std::cout<<"###  SLOW DOWN TURN: "<<balance_slowdown<<std::endl;

				for(int j=0; j<motion.num_motors; j++){
					data[j]= SPEED_CONSTANT*abs(currMo.motorPositions[motion.currentIndex-1][j]-currMo.motorPositions[motion.currentIndex][j])/(currMo.time[currMo.currentIndex]+balance_slowdown);
				}
			}
		}

		else{
			for (int i = 0; i < motion.num_motors; i++) {

				data[i] = motion.motorVelocities[motion.currentIndex][i];
			}

		}
		sendSyncWrite(motors,  MOVING_SPEED, WORD, data);

		//TODO for setting compliancy in the motors

		for (int i = 0; i < motion.num_motors; i++) {
//			if(currMo.currentIndex==0){
//			//try changing the torque back to something less abrupt
//			}
			data[i] = motion.motorCompliance[motion.currentIndex][i];
//			std::cout<<"Setting Motor "<<motion.motorIDs[motion.currentIndex][i]<<" to be compliant. "<<motion.motorCompliance[motion.currentIndex][i]<< " of 1023"<<std::endl;

		}
		sendSyncWrite(motors, MAX_TORQUE, WORD, data);

		// TODO Change this so it includes the whole body

		for (int i = 0; i < motion.num_motors; i++) {
			data[i] = motion.motorPositions[motion.currentIndex][i];

		}
		sendSyncWrite(motors,  GOAL_POSITION, WORD, data);
	}

	for(int i= 0; i < active_joints; i++ ){
		if(i==0){
//			std::cout<<"\n"<< motors.size() <<" ACTIVE JOINTS! "<<std::endl;
		}
		std::cout<<" "<<motors[i];
	}
	std::cout<<"\n\n";
	// Reset the clock
	lastClock = getUnixTime();
}



/*
 * Executes one non-blocking check to see if next step should be executed.
 * @param hasFallen A boolean value to check whether the robot has fallen,
 * and if so, if the proper procedure has been followed.
 * @return Returns true if motion has finished, otherwise returns false.
 */
int MotorController::step(bool isFalling) {
        // TODO Add error checking to see if robot has fallen
        // If it hasint  fallen, make sure that it's currently standing steady

        // What we are going to return
                executing = true;
        int returnVar = MUL8_STEP_NULL;

        if (isFalling)
        {
                // Relax all motors
                for (int i = 0; i < NUM_MOTORS; i++)
                {
                        //                      motor_dynamixel.dxl_write_byte(i+1, TORQUE_ENABLE, 0);
                        dxl_write_byte(i+1, TORQUE_ENABLE, 0);
                }

                // Wait for fall to finish
                sleep(5000);

                // TODO add code for getting back up

                return MUL8_STEP_NULL;

        }
        else //do this is if robot is not falling
        {
                motions_iterator = motions.find(currentMotion);
                // If it's been longer than the time we have to wait, let's do the next movement or step
                if (timeSince(lastClock) >= currMo.time[currMo.currentIndex-1])

                {

                        //If motion is over
                        if (currMo.currentIndex == currMo.length)
                        {

                                //printTorqueReadings();
                                //TODO check the temp of each motor and the battery level. Battery level is not very accurate

                                if(checkBattery==1){
                                        batteryLevel=0; //start fresh
                                        dxl_write_byte(23, TORQUE_ENABLE, 0); //for neck pan motor disable torque
                                        double batteryCheckSamples=5;
                                        for(int i = 0; i<batteryCheckSamples; i++){
                                                batteryLevel+= (double) (dxl_read_byte(23, 42)*.1);
                                        }

                                        std::cout <<std::endl<< "Battery Voltage ~ ";
                                        std::cout.setf( std::ios::fixed, std:: ios::floatfield ); // floatfield set to fixed
                                        std::cout << batteryLevel/batteryCheckSamples << "\n\n";
                                        /*Battery readings here are across motors, not at the leads of the power source
                                         *so the motors will read at about 0.5-0.7 V lower than the leads of the
                                         *power source. The 14.8 V LiPo batteries that we are using cannot go below
                                         *2.5V/cell. Our batteries have 4 cells, so the battery should not fall below 10V.
                                         *
                                         *To be safe, we should tell the robot to move into a stable position
                                         *when the battery reading (from the motors) falls below 10.0 V*/

                                        for (int i = 0; i < NUM_MOTORS; i++)
                                        {
                                                //                      motor_dynamixel.dxl_write_byte(i+1, TORQUE_ENABLE, 0);

                                                temperature= (int) dxl_read_byte(i+1, 43);
                                                if(i<9){
                                                        std::cout <<"Motor  "<<i+1<<": "<< temperature <<"C"<<std::endl;
                                                }
                                                else{
                                                        std::cout <<"Motor "<<i+1<<": "<< temperature <<"C"<<std::endl;

                                                }
                                        }
                                        std::cout <<std::endl;

                                        checkBattery=0; //don't allow the battery to be checked until it is set to 1 again(when another motion executes)
                                        //return voltage > VOLTAGE_THRESHHOLD;
                                }
                                // We have finished moving, so we'll let the caller know
                                returnVar =  MUL8_MOTION_FINISHED;
                                executing = false;
                        }
                        //Motion is not over
                        else
                        {

                                checkBattery= 1;        //lets allow the battery to be checked

                                //Print the status of the current motion.

                                if(currMo.currentIndex!=0){
                                        std::cout <<"\nMotion: "<< currMo.friendlyName <<std::endl;

                                        std::cout << "Step " << currMo.currentIndex-1<< " took " <<  timeSince( lastClock) << " seconds." << std::endl;
                                }
                                std::cout <<"\nStep "<< currMo.currentIndex<< " of "<< currMo.length-1<< " should take " << currMo.time[currMo.currentIndex] << " seconds: " <<std::endl;

                                // Execute the next step
                                balance_updates=1;
                                executeNext(currMo);
                                //getTorqueReadings();
                                // Increment the motion counter
                                currMo.currentIndex++;
                                returnVar = MUL8_STEP_FINISHED;
                        }

                        // Return the correct value;
                        //std::cout << "Returning " << returnVar << " from MotorController" << std::endl;
                        return returnVar;
                }
//        		if(balance_updates>0){
//        		correctBalance(balance_server.checkBalance());
//        		balance_updates--;
//        		}
                return returnVar;

        }
}

double MotorController::getStepTime() {
        return currMo.time[currMo.currentIndex-1];
}

void MotorController::correctBalance(int y_accel){
	balance_slowdown= 0;
	//	previous_balance= y_accel;

	if(y_accel<5 && y_accel>14){
		//robot is falling
		//step(true); disable all motors or set torque to 0 for all, let robot fall
	}
	else if(y_accel== 0){//server not working

		return;
	}

	else{

		if(y_accel> 8 && y_accel<12){	//robot is relatively balanced
			//do nothing?
			previous_balance=y_accel;
		}
		//if leaning to the right, incrementally adjust balance
		else if(previous_balance<=10 ){
			//			printf("got to right side");
			if(y_accel<9){
				previous_balance=y_accel;
				if(currentMotion=="Tr15" || currentMotion=="Tr30" || currentMotion=="Tr45"){
					balance_slowdown=.2;
				}
				else{
					balance_slowdown=.6;
				}

			}

		}
		//if leaning to the left
		else if(previous_balance>=10 ){
			//			printf("got to left side");
			if(y_accel>11){

				previous_balance=y_accel;
				if(currentMotion=="Tl15" || currentMotion=="Tl30" || currentMotion=="Tl45"){
					balance_slowdown=.2;
				}
				else{
					balance_slowdown=.6;
				}

			}

		}

	}
}

/**
 * Motion is set here from Main
 */
bool MotorController::setMotion(std::string newMotion) {
        currentMotion = newMotion;
        bool result = false;
        if (motions.find(currentMotion) != motions.end()) {
                // Motion found, so set the current motion
                currMo = motions.find(currentMotion)->second;
                currMo.currentIndex = 0;
                std::cout << "Set motion to: " << currentMotion << std::endl;

                result = true;
        }
        //std::cout<<"setMotion works"<<std::endl;
        return result;
}
bool MotorController::setMotionQueue(std::string newMotionQueue) {
        currentMotionQueue = newMotionQueue;
        bool result = false;
        if (motionQueues.find(currentMotionQueue) != motionQueues.end()) {
                // Motion found, so set the current motion
                currQueue = motionQueues.find(currentMotionQueue)->second;
                currQueue.currentIndex = 0;
                result = true;
        }
        return result;
}

// Get the currently executing motion
std::string MotorController::getMotion() {
        return currentMotion;
}

bool MotorController::isExecuting() {
        return executing;
}

void MotorController::sendSyncWrite(std::vector<int> ids, int address, int instruction_length, std::vector<int> data) {
        // Make syncwrite packet
        int idsLength= ids.size();

        dxl_set_txpacket_id(BROADCAST_ID);//ID of destination
        dxl_set_txpacket_instruction(INST_SYNC_WRITE);// packet type
        dxl_set_txpacket_parameter(0, address);//which data to manipulate. speed, goal pos, ect
        dxl_set_txpacket_parameter(1, instruction_length);//how long the instruction will be. 2 for word, 1 for byte,
        for(int i=0; i<idsLength; i++ )
        {
                // Set the ID number
                dxl_set_txpacket_parameter(2+(instruction_length+1)*i, ids[i]);
                // Set the data values
                if (instruction_length == 1) {
                        dxl_set_txpacket_parameter(2+2*i+1, data[i]);
                }

                else if (instruction_length == 2) {
                        dxl_set_txpacket_parameter(2+3*i+1, dxl_get_lowbyte(data[i]));
                        dxl_set_txpacket_parameter(2+3*i+2, dxl_get_highbyte(data[i]));
                }
        }
        dxl_set_txpacket_length((instruction_length+1)*idsLength+4);//(2+1) for writing a word, (1+1) for writing a byte, 4 is a constant

        dxl_txrx_packet();//sends the packet
        result = dxl_get_result( );

        if( result == COMM_TXSUCCESS )

        {
                std::cout << "COMM_TXSUCCESS "<<std::endl;
        }

        else if( result == COMM_RXSUCCESS )

        {
                std::cout << " "<<std::endl;
        }

        else if( result == COMM_TXFAIL )

        {
                std::cout << "COMM_TXFAIL "<<std::endl;
        }

        else if( result == COMM_RXFAIL)

        {
                std::cout << "COMM_RXFAIL "<<std::endl;
        }

        else if( result == COMM_TXERROR )

        {
                std::cout << "COMM_TXERROR "<<std::endl;
        }

        else if( result == COMM_RXWAITING )

        {
                std::cout << "COMM_RXWAITING "<<std::endl;
        }
}

Motion MotorController::getMotionFile() {
        return motions[getMotion()];
}

void MotorController::lockHead(){
        dxl_write_byte(24, TORQUE_ENABLE, 1);
}

bool MotorController::incrementStep() {
        // TODO Add error checking to see if robot has fallen
        // If it hasint  fallen, make sure that it's currently standing steady

        // What we are going to return
        bool returnVar1 = false;

        //If motion is over
        if (currMo.currentIndex == currMo.length)
        {
                returnVar1 =  true;
        }

        //Motion is not over
        else
        {

                // Execute the next step
                executeNext(currMo);
                // Increment the motion counter
                currMo.currentIndex++;
        }

        // Return the correct value;
        return returnVar1;

}

void MotorController::incrementIndex() {
        currMo.currentIndex++;
}

void MotorController::decrementIndex() {
        currMo.currentIndex--;
}

void MotorController::incrementQueueIndex() {
        currQueue.currentIndex++;
}

void MotorController::decrementQueueIndex() {
        currQueue.currentIndex--;
}
void MotorController::increaseQueueLength(){
        currQueue.length++;
}
std::string MotorController::getCurrentMotionName(){
        return currQueue.motionList[currQueue.currentIndex];
}

int MotorController::getQueueLength() {
        return currQueue.length;
}

int MotorController::getQueuePause() {
        return currQueue.pauseTime[currQueue.currentIndex-1];
}

int MotorController::getMotorPositionReadWord(int id) {
        return dxl_read_word(id, PRESENT_POSITION);
}

void MotorController::calibrateMotor(int motor, int adjust){
        for(int i = 0; i<currMo.length; i++){
                std::cout<< "Changing Motor "<< motor << " from "<<currMo.motorPositions[i][motor-1];
                currMo.motorPositions[i][motor-1]+= adjust;
                std::cout<< "to"  <<currMo.motorPositions[i][motor-1]<<std::endl;
        }
        recalculateCurrentMotionSpeeds();

}

void MotorController::getTorqueReadings(){


        for(int j= 0; j<currMo.num_motors; j++){
                currMo.torqueReadings[currMo.currentIndex][j]= dxl_read_word(PRESENT_LOAD, j+1);
        }

}
void MotorController::printTorqueReadings(){
        std::ofstream outFile;
        std::string extension = currMo.friendlyName +"_torque.data";

        std::string wholePath = DATA_PREPATH + extension; //show the path of the motion file

        outFile.open(wholePath.c_str()); //open this path

        //**********************************Writing the current motion struct into this .mtn file

        for(int i= 0; i<currMo.num_motors; i++){

                for(int j= 0; j<currMo.length; j++){
                        outFile<< currMo.torqueReadings[j][i]<<"\t";
                }
                outFile<< "\n";

        }
        outFile.close();
}

std::vector<int> MotorController::getMotorPositionSyncRead(int numMotors) {
        std::vector<int> positions;
        positions.resize(numMotors);
        int ids[numMotors];

        for (int i = 0; i < numMotors; i++) {
                ids[i] = i+1;
                std::cout << ids[i] << " ";
        }

        std::cout << std::endl;


        dxl_set_txpacket_id(0xFD);//ID of destination
        dxl_set_txpacket_instruction(INST_SYNC_READ);// packet type
        dxl_set_txpacket_parameter(0, PRESENT_POSITION);//which data to manipulate. speed, goal pos, ect
        dxl_set_txpacket_parameter(1, BYTE);//how long the instruction will be.

        for(int i=0; i<numMotors; i++ )
        {
                // Set the ID number
                dxl_set_txpacket_parameter(2+i, ids[i]);

        }

        dxl_set_txpacket_length(numMotors+4);

        dxl_txrx_packet();//sends the packet

        int high;
        int low;
        std::cout << "Total packets: " << dxl_get_rxpacket_length() << std::endl;

        for (int i = 0; i < numMotors; i++) {
                low = dxl_get_rxpacket_parameter(2*i);
                std::cout << "Low: " << low << "\t\t";
                high = dxl_get_rxpacket_parameter(2*i+1);
                std::cout << "High: " << high << "\t\t" << i << std::endl;
                positions[i] = dxl_makeword(low, high);
        }



        return positions;
}



void MotorController::executePrevious() {
        // Let the user know that the next step is being executed
        // TODO Delete this functionality for final robot
        if (currMo.currentIndex ==0) { return; }
        currMo.currentIndex--; //This is because the currentIndex is already set to the next motion, so we need to set it to the previous motion

        // Write the new velocities to each of the motors
        // TODO Change this so it includes the whole body
        for (int i = 0; i < currMo.num_motors; i++) {
                data[i] = INITIAL_SPEED;
        }
        sendSyncWrite(motors,  MOVING_SPEED, WORD, data);

        // Write the goal positions to each of the motors
        // TODO Change this so it includes the whole body
        for (int i = 0; i < NUM_MOTORS; i++) {
                data[i] = currMo.motorPositions[currMo.currentIndex][i];
        }
        sendSyncWrite(motors,  GOAL_POSITION, WORD, data);
        if(currMo.currentIndex!=0){
                std::cout << "Step " << currMo.currentIndex-1<< " took " <<  timeSince( lastClock) << " seconds." << std::endl;
        }
        std::cout <<"\nStep "<< currMo.currentIndex<< " of "<< currMo.length-1<< " should take " << currMo.time[currMo.currentIndex] << " seconds: " <<std::endl;

        /*show results of the step. Did the motors go where they were supposed to? also show the speeds. formatted for easy reading*/
        for (int i = 0; i < currMo.num_motors; i++) {
                if(i<9){
                        std::cout <<"\tMotor  "<< currMo.motorIDs[0][i]<< "  Goal Position:\t" <<data[i]<<"\tActual Position:\t"<< getMotorPositionReadWord(currMo.motorIDs[0][i])<<"\tSpeed:\t"<<currMo.motorVelocities[currMo.currentIndex][i]<< std::endl;
                }
                else{
                        std::cout <<"\tMotor "<< currMo.motorIDs[0][i]<< "  Goal Position:\t" <<data[i]<<"\tActual Position:\t"<< getMotorPositionReadWord(currMo.motorIDs[0][i])<<"\tSpeed:\t"<<currMo.motorVelocities[currMo.currentIndex][i]<< std::endl;

                }
        }
        // Reset the clock
        lastClock = getUnixTime();
}

void MotorController::disableAllMotors() {
        std::vector<int> datas;
        datas.resize(TOTAL_MOTORS);
        motors.resize(TOTAL_MOTORS);
        for (int i = 0; i < TOTAL_MOTORS; i++) {
                motors[i] = i+1;        //give the name of each and every motor ID
                datas[i] = 0;   // 1 is enable. 0 is disable
        }
        sendSyncWrite(motors, TORQUE_ENABLE, BYTE, datas);
}

//TODO for editing new steps, the current positions are save to the struct if user wants to save step
void MotorController::enableAllMotors() {

        std::vector<int> datas;
        datas.resize(TOTAL_MOTORS);
        motors.resize(TOTAL_MOTORS);
        for (int i = 0; i < TOTAL_MOTORS; i++) {
                motors[i] = i+1;        //give the name of each and every motor ID
                datas[i] = 1;   // 1 is enable. 0 is disable
        }
        sendSyncWrite(motors, TORQUE_ENABLE, BYTE, datas);
        std::cout << "\nStep\n" << currMo.currentIndex << "\n\n";
        //sendSyncWrite(motors, NUM_MOTORS, TORQUE_ENABLE, BYTE, datas); //TODO double tapping this write method to see if motor positions are more accurate
        getCurrentPose();

}

/*Will run a error checking algorithm to make sure that the motors aren't reading back unrealistic values */
void MotorController::getCurrentPose(){
        int tempPositions[CHECK_POSITIONS];
        currentPose.resize(currMo.num_motors);
        std::vector<int> badMotorList;//hold a list of the motors that gave bad readings on the forward read
        int badMotorSize=1; //to resize the list
        bool reverseCheck;
        bool badPose;
        int tempRevPositions[currMo.num_motors];


        for (int i = 0; i < currMo.num_motors; i++) {
                /*Going to take a certain number of samples from this motor*/
                for(int j= 0; j<CHECK_POSITIONS; j++){
                        //compare indexes 0 and 1. Are they within 11 (1 degree) of eachother? good, lets compare 2 to be sure
                        tempPositions[j]= dxl_read_word(currMo.motorIDs[0][i], PRESENT_POSITION);
                        std::cout<<tempPositions[j]<<"  ";
                }
                currentPose[i] = errorCheck(tempPositions, CHECK_POSITIONS);//holds value of the error checking results


                if(currentPose[i]<0){
                        reverseCheck= true;
                        //lets signal that we're going to read the motors in the reverse order to get better values
                        badMotorList.resize(badMotorSize++);
                        badMotorList[badMotorSize-2]= i;
                }
                /*Check to see that motor position is within the CW CCW limits*/
                if(getBoundedPosition(currentPose[i], currMo.motorIDs[0][i])<0){
                        std::cout<<"Motor "<<currMo.motorIDs[0][i]<<" "<<"is outside of its range of motion!"<<std::endl;
                }

                std::cout<<"\n";
        }

        if(reverseCheck){
                std::cout<<"\nReverse readings"<<std::endl;

                for (int i = currMo.num_motors-1; i >-1; i--) {
                        for(int j= 0; j<CHECK_POSITIONS; j++){
                                //compare indexes 0 and 1. Are they within 11 (1 degree) of eachother? good, lets compare 2 to be sure

                                tempPositions[j]= dxl_read_word(currMo.motorIDs[0][i], PRESENT_POSITION);
                                std::cout<<tempPositions[j]<<"  ";
                        }
                        tempRevPositions[i] = errorCheck(tempPositions, CHECK_POSITIONS);//holds value of the error checking results

                        /*check to see if motor reading falls within CCW and CW limits*/
                        if(getBoundedPosition(tempRevPositions[i], i+1)<0){
                                std::cout<<"Motor "<<currMo.motorIDs[0][i]<<" "<<"is outside of its range of motion!"<<std::endl;
                        }
                        std::cout<<"\n";
                        // Needed so that we do not try to resize vector to a negative value
                }

                for(int i=0; i<badMotorSize-1; i++){
                        int toReplace= badMotorList[i];
                        currentPose[toReplace]= tempRevPositions[toReplace]; //each value of badMotor list specifies the an index inside currentPose qhich needs to be placed
                }
        }

        /*This loop will print out the final positions at the moment in position values and degrees, and radians*/
        for (int i = 0; i < currMo.num_motors; i++) {
                int degrees= convDegrees(currentPose[i], currMo.motorIDs[0][i]);
                double radians= degrees*RADIANS_PER_DEGREE;
                if(i==0){
                        std::cout<<"Motor\tPosition\tDegrees\tRadians"<<std::endl;
                }

                std::cout<<currMo.motorIDs[0][i] <<"\t"<<currentPose[i]<<"\t"<< degrees <<"\t"<<radians<<std::endl;
                if((currentPose[i]<0) || (currentPose[i]>4095)){
                        badPose= true;
                }
        }
        if(badPose){
                std::cout<<"UNSAFE MOTOR READINGS!\n Try again by releasing all (ra)\n and then enabling all (ea)\n"<<std::endl;
        }

}
/*algorithm for excluding outliers in "size" number of motor reads*/
int MotorController::errorCheck(int a[], int size){
        int prelimScan=0;
        int check1;
        int check2;
        for(int i= 0; i<size-3; i++){  // we use (i<size-3) because we are looking ahead 3 indices at most. i.e looking at i=0 to i+3.

                if((a[i]>=0) && (a[i]<=4095) && (a[i+1]>=0) && (a[i+1] <=4095) && (a[i+2]>=0) && (a[i+2] <=4095)){
                        check1= abs(a[i]-a[i+1]);
                        check2= abs(a[i+1]-a[i+2]);
                        if((check1<=POSITION_TOLERANCE) && (check2<=POSITION_TOLERANCE)){// We will fist check to see if any xxx are the same.
                                prelimScan= a[i];
                                return prelimScan;
                        }
                }
                else if((a[i+3]>=0) && (a[i+3] <=4095)) {
                        check1= abs(a[i]-a[i+1]);
                        check2= abs(a[i+1]-a[i+3]);
                        if((check1<=POSITION_TOLERANCE) && (check2<=POSITION_TOLERANCE)){       //check to see if any xx_x are the same.
                                prelimScan= a[i];
                                return prelimScan;
                        }
                        check1= abs(a[i]-a[i+2]);
                        check2= abs(a[i+2]-a[i+3]);
                        if((check1<=POSITION_TOLERANCE) && (check2<=POSITION_TOLERANCE)){ //check to see if any x_xx are the same.
                                prelimScan= a[i];
                                return prelimScan;
                        }
                }
        }
        std::cout<<"No combos of 3 matches"<<std::endl;

        if(prelimScan>0){
                return prelimScan;
        }

        else{           //if goodOnes array does not contain values, return error
                return -1;
        }

        return -1;
}

/*This function uses currentPose[] and zeroPose[] to mirror currentPose[] across the sagital plane*/
void MotorController::getMirroredPose(){

        getCurrentPose(); //this will put the current positions into the global array currentPose[]

        for(int i=1; i<NUM_MOTORS; i+= 2){  //start on motor 2, switch it with motor 1. end on Motor 12.
                //do the mirroring operation two motors at a time (swapping)
                mirroredPose[i-1]= zeroPose[i-1] + ((currentPose[i]-zeroPose[i])*mirrorSigns[i])*mirrorSigns[i-1];
                mirroredPose[i]= zeroPose[i] + ((currentPose[i-1]-zeroPose[i-1])*mirrorSigns[i-1])*mirrorSigns[i];
        }

        /*This loop will print out the final positions at the moment in position values and degrees, and radians*/
        for (int i = 0; i < NUM_MOTORS; i++) {

                int degrees= convDegrees(mirroredPose[i], i);//convert to degrees

                double radians= degrees*RADIANS_PER_DEGREE; //convert degrees to radians
                /*Printing the results in a nice format*/
                if(i==0){
                        std::cout<<"Motor         Position    Degrees    Radians"<<std::endl;
                }

                std::cout<<i+1 <<"          "<<mirroredPose[i]<<"          "<< degrees <<"          "<<radians<<std::endl;

                /************************************************/

                if(getBoundedPosition(mirroredPose[i], i+1)<0){
                        std::cout<<"Motor "<<i+1<<" "<<"is outside of its range of motion!"<<std::endl;
                }
        }
}

void MotorController::setMirroredPose(){        //set the pose to the one that was requested to be mirrored MOST RECENTLY
        int initialPos;
        int finalPos;
        int speeds[NUM_MOTORS];

        for (int i = 0; i < NUM_MOTORS; i++) {
                /*Calculate speeds to get from one pose to the next using a set slow transition */
                initialPos= currentPose[i];
                finalPos= mirroredPose[i];
                data[i]= SPEED_CONSTANT*abs(initialPos-finalPos)/2.5;
                speeds[i]=data[i];
        }

        sendSyncWrite(motors,  MOVING_SPEED, WORD, data); //set slow moving speed to transistion to mirrored pose

        for( int i =0; i<NUM_MOTORS; i++){
                data[i]= mirroredPose[i];
        }
        sendSyncWrite(motors,  GOAL_POSITION, WORD, data); //send that position command

        /*show results of the step*/
        for (int i = 0; i < NUM_MOTORS; i++) {

                int degrees= convDegrees(mirroredPose[i], i);//convert to degrees

                double radians= degrees*RADIANS_PER_DEGREE; //convert degrees to radians
                /*Printing the results in a nice format*/
                if(i==0){
                        std::cout<<"Motor         Position    Degrees    Radians        Speed"<<std::endl;
                }

                std::cout<<i+1 <<"          "<<mirroredPose[i]<<"          "<< degrees <<"          "<<radians<<"                       "<<speeds[i]<<std::endl;

                /************************************************/
        }
}
void MotorController::mirrorCurrentMotion(){
        std::vector<std::vector<int> > tempStep;

        tempStep.resize(2); //one row for pose one row for speeds
        tempStep[0].resize(NUM_MOTORS); //each row is num_motors long
        tempStep[1].resize(NUM_MOTORS);

        for(int i= 0; i<currMo.length; i++){

                //put current step into a temporary vector to pull from for mirroring
                for(int j= 0; j<NUM_MOTORS; j++){
                        tempStep[0][j]= currMo.motorPositions[i][j];    //pose step i, motor j
                        tempStep[1][j]= currMo.motorVelocities[i][j];   //vel step i, motor j
                }

                for(int j=1; j<NUM_MOTORS; j+= 2){  //start on motor 2, switch it with motor 1. end on Motor 12.
                        //do the mirroring operation two motors at a time (swapping)
                        currMo.motorPositions[i][j-1]= zeroPose[j-1] + ((tempStep[0][j]-zeroPose[j])*mirrorSigns[j])*mirrorSigns[j-1];
                        currMo.motorPositions[i][j]= zeroPose[j] + ((tempStep[0][j-1]-zeroPose[j-1])*mirrorSigns[j-1])*mirrorSigns[j];
                }
                for(int j=1; j<NUM_MOTORS; j+= 2){

                        currMo.motorVelocities[i][j-1]=tempStep[1][j];  //assign vel 1 to vel 0, etc
                        currMo.motorVelocities[i][j]=tempStep[1][j-1];  //assign vel 0 to vel 1, etc
                }

                std::cout<<"Results of mirrored step "<<i<<std::endl;

                /*This loop will print out the final positions at the moment in position values and degrees, and radians*/
                for (int j = 0; j < NUM_MOTORS; j++) {

                        int degrees= convDegrees(currMo.motorPositions[i][j], j);//convert to degrees

                        double radians= degrees*RADIANS_PER_DEGREE; //convert degrees to radians
                        /*Printing the results in a nice format*/
                        if(j==0){
                                std::cout<<"Motor         Position    Degrees    Radians"<<std::endl;
                        }

                        std::cout<<j+1 <<"          "<<currMo.motorPositions[i][j]<<"          "<< degrees <<"          "<<radians<<std::endl;

                        //************************************************
                        if(getBoundedPosition(currMo.motorPositions[i][j], j+1)<0){
                                std::cout<<"Motor "<<j+1<<" "<<"is outside of its range of motion!"<<std::endl;
                        }
                }
        }
        std::ofstream outFile;
        currMo.friendlyName= currMo.friendlyName+"_m";
        std::string extension = currMo.friendlyName;

        std::cout << "Friendly Name: " << MOTIONS_PREPATH << extension << std::endl;

        // Convert Friendly Name to Computer Name
        for (unsigned i = 0; i < extension.length(); i++) {
                //std::cout << extension.substr(i, 1) << " ";
                if (extension.substr(i, 1) == " ") {
                        extension = extension.substr(0,i) + "_" + extension.substr(i+1); //replaces any spaces with "_"
                }
        }

        extension += ".mtn";//adds extension to the end of the friendly name

        std::cout << "Machine Name: " << MOTIONS_PREPATH << extension << std::endl;

        std::string wholePath = MOTIONS_PREPATH + extension; //show the path of the motion file

        outFile.open(wholePath.c_str()); //open this path

        //**********************************Writing the current motion struct into this .mtn file
        outFile<< currMo.length<< "\t\n";

        //Go through all of the steps in this motion, first printing out the time
        for(int i= 0; i<currMo.length; i++){
                outFile<<"\t\t\n"<< currMo.time[i] <<"\t\n\t\t\n";

                //then print out each motor position and speed for this step
                for(int j=0; j<NUM_MOTORS; j++){
                        outFile<< currMo.motorPositions[i][j] <<"\t" <<currMo.motorVelocities[i][j]<<"\n";
                }

        }
        outFile.close();
        //***************************************done writing to this .mtn file

        // Update the motions "library". .../motion_text_file.mtn
        outFile.open(TEMP_MOTION_FILE); //this is the name of the library file
        machineNames.resize(machineNames.size() + 1);   //resize this vector to how many motion names there are
        friendlyNames.resize(friendlyNames.size() + 1); //resize this vecotr to how many motion names there are
        machineNames[machineNames.size() - 1] = extension; //the last thing in the library is the motion that we just saved.
        friendlyNames[friendlyNames.size() - 1] = currMo.friendlyName;  //the last thing in the library is the motion that we just saved.

        //outFile << machineNames.size() << "\n";
        for(unsigned i = 0; i < machineNames.size(); i++) {
                outFile << machineNames[i] << "\t" << friendlyNames[i] << "\n"; //write each motion name in both formats to the library
        }
        outFile.close();

        initialize(); //reinitialize all of the motions to account for the changes made to animation file
        setMotion("Animation");
        std::cout<<"Success! Edited motion saved to " << MOTIONS_PREPATH << extension << "\n\n";

}

int MotorController::convDegrees(int pos, int index){
        int degrees;
        index--;//parameter gives the actual motor number, so we'll change it to reflect an index for the array (i.e. motor#-1)

        switch(index){
        case 0:
                degrees= (pos-zeroPose[index])*DEGREES_PER_POSITION;
                break;

        case 1:
                degrees= -1*(pos-zeroPose[index])*DEGREES_PER_POSITION;
                break;
        case 2:
                degrees= (pos-zeroPose[index])*DEGREES_PER_POSITION;
                break;
        case 3:
                degrees= -1*(pos-zeroPose[index])*DEGREES_PER_POSITION;
                break;
        case 4:
                degrees= -1*(pos-zeroPose[index])*DEGREES_PER_POSITION;
                break;
        case 5:
                degrees= (pos-zeroPose[index])*DEGREES_PER_POSITION;
                break;
        case 6:
                degrees= -1*(pos-zeroPose[index])*DEGREES_PER_POSITION;
                break;
        case 7:
                degrees= (pos-zeroPose[index])*DEGREES_PER_POSITION;
                break;
        case 8:
                degrees= (pos-zeroPose[index])*DEGREES_PER_POSITION;
                break;
        case 9:
                degrees= (pos-zeroPose[index])*DEGREES_PER_POSITION;
                break;
        case 10:
                degrees= (pos-zeroPose[index])*DEGREES_PER_POSITION;
                break;
        case 11:
                degrees= -1*(pos-zeroPose[index])*DEGREES_PER_POSITION;
                break;
        default:
                break;
        }
        return degrees;
}

void MotorController::chooseCompliantLimb(int step, int motorID, int new_compliance){
        if(step== -1 && motorID==-1){
                for(int i= 0; i<currMo.length; i++){
                        for(int j=0; j<currMo.num_motors; j++){
                                currMo.motorCompliance[i][j]= new_compliance; //setting torque limits for all steps in motion
                        }
                }
        }
        else{
                currMo.motorCompliance[step][motorID-1]= new_compliance;
        }
}

void MotorController::setCompliantLimb(int compliancy){
        currentTorque= compliancy;
}
//void MotorController::setTorqueLimit(int id, int torque) {            //set new torque limits
//      dxl_write_word(id, TORQUE_LIMIT, torque);
//}
void MotorController::disableMotor(int motor) {
        dxl_write_byte(motor, TORQUE_ENABLE, 0);
}

void MotorController::enableMotor(int motor) {
        dxl_write_byte(motor, TORQUE_ENABLE, 1);
        //      std::cout << "Present Position of motor " << motor << ": " << dxl_read_word(motor, PRESENT_POSITION) << std::endl;
}

void MotorController::changePID(int motor, int newPID) {
        if(motor==0){

                for(int i= 0; i<TOTAL_MOTORS; i++){
                        dxl_write_byte(i+1, newPID, P_FOR_PID);
                }

        }
        else{

                dxl_write_byte(motor, newPID, P_FOR_PID);
        }
}
//functions for manipulating the legs
void MotorController::disableLeftLeg() {
        std::vector<int>  leftLeg;
        leftLeg.resize(6);
        std::vector<int> datas;
        datas.resize(NUM_LEG_MOTORS/2);
        for (int i = 0; i < NUM_LEG_MOTORS/2; i++) {
                leftLeg[i] = 2*i + 2;
                datas[i] = 0;
        }
        sendSyncWrite(leftLeg,  TORQUE_ENABLE, BYTE, datas);
}

void MotorController::disableRightLeg() {
        std::vector<int>  rightLeg;
        rightLeg.resize(6);
        std::vector<int> datas;
        datas.resize(NUM_LEG_MOTORS/2);
        for (int i = 0; i < NUM_LEG_MOTORS/2; i++) {
                rightLeg[i] = 2*i + 1;
                datas[i] = 0;
        }
        sendSyncWrite(rightLeg,  TORQUE_ENABLE, BYTE, datas);
}

void MotorController::enableRightLeg() {
        std::vector<int>  rightLeg;
        rightLeg.resize(6);
        std::vector<int> datas;
        datas.resize(NUM_LEG_MOTORS/2);
        for (int i = 0; i < NUM_LEG_MOTORS/2; i++) {
                rightLeg[i] = 2*i + 1;
                datas[i] = 1;
        }
        sendSyncWrite(rightLeg,  TORQUE_ENABLE, BYTE, datas);
}

void MotorController::enableLeftLeg() {
        std::vector<int>  leftLeg;
        leftLeg.resize(6);
        std::vector<int> datas;
        datas.resize(NUM_MOTORS/2);
        for (int i = 0; i < NUM_LEG_MOTORS/2; i++) {
                leftLeg[i] = 2*i + 2;
                datas[i] = 1;
        }
        sendSyncWrite(leftLeg,  TORQUE_ENABLE, BYTE, datas);
}

//release only the sagital motors in the ankles and hips
void MotorController::disableSwayMotors() {
        std::vector<int>  swayMotors;
        swayMotors.resize(6);
        std::vector<int> datas;

        swayMotors[0] = 5;
        swayMotors[1] = 6;
        swayMotors[2] = 11;
        swayMotors[3] = 12;
        datas.resize(4);
        for(int i=0; i<4; i++){
                datas[i] = 0;
        }
        sendSyncWrite(swayMotors, TORQUE_ENABLE, BYTE, datas);
}

//functions for manipulating the arms
void MotorController::disableLeftArm() {
        std::vector<int>  leftArm;
        leftArm.resize(NUM_ARM_MOTORS/2);
        std::vector<int> datas;
        datas.resize(NUM_ARM_MOTORS/2);
        for (int i = 0; i < NUM_ARM_MOTORS/2; i++) {
                leftArm[i] = 2*i + 2+ 12;
                datas[i] = 0;
        }
        std::cout<<"RELEASING LEFT ARM"<<std::endl;
        sendSyncWrite(leftArm,  TORQUE_ENABLE, BYTE, datas);
}

void MotorController::disableRightArm() {
        std::vector<int>  rightArm;
        rightArm.resize(NUM_ARM_MOTORS/2);
        std::vector<int> datas;
        datas.resize(NUM_ARM_MOTORS/2);
        for (int i = 0; i < NUM_ARM_MOTORS/2; i++) {
                rightArm[i] = 2*i + 1 + 12;
                datas[i] = 0;
        }
        std::cout<<"RELEASING RIGHT ARM"<<std::endl;
        sendSyncWrite(rightArm,  TORQUE_ENABLE, BYTE, datas);
}

void MotorController::enableRightArm() {
        std::vector<int>  rightArm;
        rightArm.resize(NUM_ARM_MOTORS/2);
        std::vector<int> datas;
        datas.resize(NUM_ARM_MOTORS/2);
        for (int i = 0; i < NUM_ARM_MOTORS/2; i++) {
                rightArm[i] = 2*i + 1 + 12;
                datas[i] = 1;
        }
        std::cout<<"ENABLING RIGHT ARM"<<std::endl;
        sendSyncWrite(rightArm,  TORQUE_ENABLE, BYTE, datas);
}

void MotorController::enableLeftArm() {
        std::vector<int> leftArm;
        leftArm.resize(NUM_ARM_MOTORS/2);
        std::vector<int> datas;
        datas.resize(NUM_ARM_MOTORS/2);
        for (int i = 0; i < NUM_ARM_MOTORS/2; i++) {
                leftArm[i] = 2*i + 2 + 12;
                datas[i] = 1;
        }
        std::cout<<"ENABLING LEFT ARM"<<std::endl;
        sendSyncWrite(leftArm,  TORQUE_ENABLE, BYTE, datas);
}


int MotorController::readMotorPosition(int motor) {
        return ( (int)dxl_read_word(motor, PRESENT_POSITION) );
}
void MotorController::setStatusReturnLevel(int level) {
        for (int i = 0; i < NUM_MOTORS; i++) {
                dxl_write_byte(i+1, STATUS_RETURN_LEVEL, level);
        }

}

/*this is supposed to be a high level call. It takes the new time that a step should take (user input) and passes it to editStep()*/
void MotorController::setTime(double stepTime){
        std::cout <<"Setting time for new step..." << currMo.currentIndex-1<<std::endl;
        editStep(currMo, currMo.currentIndex-1, stepTime, currentPose);
}

/*this is supposed to be a high level call. It takes the new time that a step should take (user input) and passes it to editStep()*/
void MotorController::setPauseTime(double pause){
        std::cout<<"the current index when setting pause time is "<<currQueue.currentIndex-2<<::std::endl;
        std::cout <<"Setting pause time to " << pause <<" seconds"<<std::endl;
        currQueue.pauseTime[currQueue.currentIndex-2]=pause;
}

/*this is supposed to be a high level call. It takes the new time that a step should take (user input) and passes it to editStep()*/
void MotorController::changeTime(double stepTime){
        std::cout <<"Setting time for new step..." << currMo.currentIndex-1<<std::endl;
        editStepTime(currMo, currMo.currentIndex-1, stepTime);
}

/*edits a step inside of a motion by taking a new pose and calculating the new speeds*/
void MotorController::editStep(Motion &tempMotion, int stepIndex, double newTime, std::vector<int> currPos) {
        //[rows= steps][columns=motors]

        int finalPos;
        int initialPos;

        std::cout << "This is the step index: " << stepIndex << std::endl;
        std::cout << "This is the time: " << newTime << std::endl;

        /*Book keeping to recover the previous step. Need pos's, vel's, and time*/
        std::cout << tempMotion.time[stepIndex] << std::endl;

        tempMotion.time[stepIndex]= newTime;

        for(int i=0; i<tempMotion.num_motors; i++){
                /* Move the current step into the recovery vectors in case the new step doesn't work out*/
                //std::cout << tempMotion.motorPositions[stepIndex][i] << std::endl;

                tempMotion.motorPositions[stepIndex][i]= currPos[i]; //replace current step's motor positions with the input motor positions

                /*calculate change in position from previous step to current step*/
                if(stepIndex == 0){
                        tempMotion.motorVelocities[stepIndex][i]= INITIAL_SPEED;
                        std::cout <<"Step "<< stepIndex<< "...Motor "<< currMo.motorIDs[stepIndex][i]<< " position saved..." <<std::endl;
                }
                else {
                        /*Calaculate speeds to get from one pose to the next*/
                        initialPos= tempMotion.motorPositions[stepIndex-1][i];
                        finalPos= tempMotion.motorPositions[stepIndex][i];
                        tempMotion.motorVelocities[stepIndex][i]= SPEED_CONSTANT*abs(initialPos-finalPos)/newTime;

                        std::cout <<"Step "<< stepIndex<< "...Motor "<< tempMotion.motorIDs[stepIndex][i]<< " position saved..." <<std::endl;
                }
        }
}
void MotorController::editStepTime(Motion &tempMotion, int stepIndex, double newTime){
        /*calculate change in position from previous step to current step*/
        int finalPos;
        int initialPos;

        std::cout << "This is the step index: " << stepIndex << std::endl;
        std::cout << "This is the time: " << newTime << std::endl;
        tempMotion.time[stepIndex]= newTime;

        for(int i=0; i<NUM_MOTORS; i++){
                if(stepIndex == 0){
                        tempMotion.motorVelocities[stepIndex][i]= INITIAL_SPEED;
                        std::cout <<"Step "<< stepIndex<< "...Motor "<< i+1<< " position saved..." <<std::endl;
                }
                else {
                        /*Calaculate speeds to get from one pose to the next*/
                        initialPos= tempMotion.motorPositions[stepIndex-1][i];
                        finalPos= tempMotion.motorPositions[stepIndex][i];
                        tempMotion.motorVelocities[stepIndex][i]= SPEED_CONSTANT*abs(initialPos-finalPos)/newTime;

                        std::cout <<"Step "<< stepIndex<< "...Motor "<< i+1<< " new speed = "<< tempMotion.motorVelocities[stepIndex][i]<<std::endl;
                }
        }
}
void MotorController::insertStep(){ //this function puts a new step right in the middle of the current Motion at the next index

        std::cout << "Inserting new step: " << currMo.currentIndex << std::endl;//incrementing index becuase step will be inserted after the current step
        //currMo.currentIndex++;
        currMo.length++;
        std::cout << "New length of motion: \n" << currMo.length << std::endl;
        //currMo.currPos.resize(currMo.length); //TODO not sure why this is done. currPos is a 1D vector with length of NUM_MOTORS
        currMo.motorPositions.resize(currMo.length);
        currMo.motorVelocities.resize(currMo.length);
        currMo.motorIDs.resize(currMo.length);
        currMo.time.resize(currMo.length);
        //std::cout<<"resized vectors"<<std::endl;
        if(currMo.currentIndex==currMo.length-1){
                std::cout<<"putting new step on the end of this motion\n";
                //need to initialize the allocated space for data for each motor
                currMo.time[currMo.currentIndex]= currMo.time[currMo.currentIndex-1];


                currMo.motorPositions[currMo.length-1].resize(currMo.num_motors);
                //std::cout<<"resized pos vector\n";
                currMo.motorVelocities[currMo.length-1].resize(currMo.num_motors);

                currMo.motorIDs[currMo.length-1].resize(currMo.num_motors);

                //std::cout<<"resized velvector\n";


                //std::cout<<"resized position and velocity vectors\n";
                for(int i=0; i<currMo.num_motors; i++){
                        currMo.motorPositions[currMo.currentIndex][i]= currMo.motorPositions[currMo.currentIndex-1][i];
                        //std::cout<<"shifted pos vector\n";

                        currMo.motorVelocities[currMo.currentIndex][i]= currMo.motorVelocities[currMo.currentIndex-1][i];
                        //std::cout<<"shifted velocity vectors\n";
                        currMo.motorIDs[currMo.currentIndex][i]= currMo.motorIDs[currMo.currentIndex-1][i];
                }
                std::cout<<"SUCCESS!\n";
        }
        else{
                //shift the times, poses, and speeds to the next spot. This will make space for a new step to be entered in the current index of the vectors
                for(int i = currMo.length-1; i>currMo.currentIndex-1; i--){
                        //std::cout<<"this is the time +1 :  "<<currMo.time[i+1]<<std::endl;
                        currMo.time[i]= currMo.time[i-1];

                        //need to initialize the allocated space for data for each motor
                        if( i== currMo.length-1){
                                currMo.motorPositions[i].resize(currMo.num_motors);
                                currMo.motorVelocities[i].resize(currMo.num_motors);
                                currMo.motorIDs[i].resize(currMo.num_motors);
                        }

                        for(int j= 0; j<currMo.num_motors;j++){
                                //take put stuff from currentIndex into currentIndex+1
                                currMo.motorPositions[i][j]= currMo.motorPositions[i-1][j];
                                currMo.motorVelocities[i][j]= currMo.motorVelocities[i-1][j];
                                currMo.motorIDs[i][j]= currMo.motorIDs[i-1][j];

                        }
                }
        }
        for(int j= 0; j<currMo.num_motors; j++){
                std::cout<< currMo.motorIDs[currMo.currentIndex][j]<<"\t"<< currMo.motorPositions[currMo.currentIndex][j]<<"\t"<<currMo.motorVelocities[currMo.currentIndex][j]<<std::endl;
        }
}


void MotorController::deleteCurrentStep(){ //this function puts a new step right in the middle of the current Motion at the next index

        //shift the times, poses, and speeds to the next spot. This will make space for a new step to be entered in the current index of the vectors
        for(int i = currMo.currentIndex; i<currMo.length-1; i++){

                //move step 0 into 1, 2 into 1, etc...
                currMo.time[i]= currMo.time[i+1];

                //move step 0 into 1, 2 into 1, etc...
                for(int j= 0; j<NUM_MOTORS;j++){
                        //take put stuff from currentIndex into currentIndex+1
                        currMo.motorPositions[i][j]= currMo.motorPositions[i+1][j];
                        currMo.motorVelocities[i][j]= currMo.motorVelocities[i+1][j];
                }

                //need to chop off the last step of the motion
                if( i== currMo.length-2){
                        currMo.length--;
                        currMo.time.resize(currMo.length);
                        currMo.motorPositions.resize(currMo.length);
                        currMo.motorVelocities.resize(currMo.length);
                }
        }
        std::cout << "Deleting step: " << currMo.currentIndex << std::endl;//incrementing index becuase step will be inserted after the current step

        std::cout << "New length of motion: \n" << currMo.length << std::endl;
        //currMo.currPos.resize(currMo.length); //TODO not sure why this is done. currPos is a 1D vector with length of NUM_MOTORS

        std::cout<<"SUCCESS deleting step!\n";
}
void MotorController::printMotion(int printCode){

        switch(printCode){

        case 1: //print to the console
                std::cout<< currMo.length <<std::endl;

                std::cout<< "\n"<< currMo.num_motors  <<std::endl;

                /*Go through all of the steps in this motion, first printing out the time*/
                for(int i= 0; i<currMo.length; i++){
                        std::cout<<"\n"<< currMo.time[i] <<"\n" << std::endl;

                        /*then print out each motor position and speed for this step*/
                        for(int j=0; j<currMo.num_motors; j++){
                                std::cout<< currMo.motorPositions[i][j] <<"\t" <<currMo.motorVelocities[i][j]<<"\t"<<currMo.motorIDs[i][j]<<"\t"<<currMo.motorCompliance[i][j]<<std::endl;
                        }
                        std::cout<<"+---------+"<<std::endl;
                        std::cout<<"| step "<<i<<"  |"<<std::endl;
                        std::cout<<"+---------+"<<std::endl;
                }
                break;

        case 2: //print this motion to a .mtn file and update the motion library to include this motion.

                std::ofstream outFile;
                std::string extension = currMo.friendlyName;

                std::cout << "Friendly Name: " << MOTIONS_PREPATH << extension << std::endl;

                // Convert Friendly Name to Computer Name
                for (unsigned i = 0; i < extension.length(); i++) {
                        //std::cout << extension.substr(i, 1) << " ";
                        if (extension.substr(i, 1) == " ") {
                                extension = extension.substr(0,i) + "_" + extension.substr(i+1); //replaces any spaces with "_"
                        }
                }

                extension += ".mtn";//adds extension to the end of the friendly name

                std::cout << "Machine Name: " << MOTIONS_PREPATH << extension << std::endl;

                std::cout<<currMo.num_motors<<std::endl;
                std::string wholePath = MOTIONS_PREPATH + extension; //show the path of the motion file


                outFile.open(wholePath.c_str()); //open this path
                //**********************************Writing the current motion struct into this .mtn file
                std::cout << "found file "  << std::endl;

                outFile<< currMo.length<< "\t\t\n";
                std::cout << "printed length "  << std::endl;

                outFile<<"\t\t\t\n"<< currMo.num_motors <<"\t\t\n";

                std::cout << "printed used motors "  << std::endl;

                //Go through all of the steps in this motion, first printing out the time
                for(int i= 0; i<currMo.length; i++){
                        outFile<<"\t\t\t\n"<< currMo.time[i] <<"\t\t\n\t\t\t\n";
                        std::cout << "printed time "  <<i<< std::endl;

                        //then print out each motor position and speed for this step
                        for(int j=0; j<currMo.num_motors; j++){

                                outFile<< currMo.motorPositions[i][j] <<"\t" <<currMo.motorVelocities[i][j]<<"\t"<<currMo.motorIDs[i][j]<<"\t"<<currMo.motorCompliance[i][j]<<"\n";
                                std::cout << "printed pose speed and motor for step " <<i << std::endl;

                        }

                }
                outFile.close();
                //***************************************done writing to this .mtn file

                // Update the motions "library". .../motion_text_file.mtn
                outFile.open(TEMP_MOTION_FILE); //this is the name of the library file
                machineNames.resize(machineNames.size() + 1);   //resize this vector to how many motion names there are
                friendlyNames.resize(friendlyNames.size() + 1); //resize this vecotr to how many motion names there are
                machineNames[machineNames.size() - 1] = extension; //the last thing in the library is the motion that we just saved.
                friendlyNames[friendlyNames.size() - 1] = currMo.friendlyName;  //the last thing in the library is the motion that we just saved.

                //outFile << machineNames.size() << "\n";
                for(unsigned i = 0; i < machineNames.size(); i++) {
                        outFile << machineNames[i] << "\t" << friendlyNames[i] << "\n"; //write each motion name in both formats to the library
                }
                outFile.close();

                initialize(); //reinitialize all of the motions to account for the changes made to animation file
                setMotion("Animation");
                std::cout<<"Success! Edited motion saved to " << MOTIONS_PREPATH << extension << "\n\n";
                break;
        }
}

void MotorController::printMotionQueue(){

        std::ofstream outFile;
        std::string extension = currQueue.friendlyName;

        std::cout << "Friendly Name: " << MOTIONS_PREPATH << extension << std::endl;

        // Convert Friendly Name to Computer Name
        for (unsigned i = 0; i < extension.length(); i++) {
                //std::cout << extension.substr(i, 1) << " ";
                if (extension.substr(i, 1) == " ") {
                        extension = extension.substr(0,i) + "_" + extension.substr(i+1); //replaces any spaces with "_"
                }
        }

        extension += ".mtnq";//adds extension to the end of the friendly name

        std::cout << "Machine Name: " << MOTIONS_PREPATH << extension << std::endl;

        std::string wholePath = MOTIONS_PREPATH + extension; //show the path of the motion file

        outFile.open(wholePath.c_str()); //open this path

        //**********************************Writing the current motion struct into this .mtn file
        outFile<< currQueue.length<< "\t\n";

        //Go through all of the motions in the queue, printing the name and pause time
        for(int i= 0; i<currQueue.length; i++){

                std::cout<< currQueue.motionList[i]<<"\t" <<currQueue.pauseTime[i]<<std::endl;
                outFile<< currQueue.motionList[i] <<"\t" <<currQueue.pauseTime[i]<<"\n";

        }
        outFile.close();
        //***************************************done writing to this .mtn file

        // Update the motions "library". .../motion_text_file.mtn
        outFile.open(TEMP_MOTION_QUEUE_FILE);   //this is the name of the library file
        machineQueueNames.resize(machineQueueNames.size() + 1); //resize this vector to how many motion names there are
        friendlyQueueNames.resize(friendlyQueueNames.size() + 1);       //resize this vecotr to how many motion names there are
        machineQueueNames[machineQueueNames.size() - 1] = extension; //the last thing in the library is the motion that we just saved.
        friendlyQueueNames[friendlyQueueNames.size() - 1] = currQueue.friendlyName;     //the last thing in the library is the motion that we just saved.

        //outFile << machineNames.size() << "\n";
        for(unsigned i = 0; i < machineQueueNames.size(); i++) {
                outFile << machineQueueNames[i] << "\t" << friendlyQueueNames[i] << "\n";       //write each motion name in both formats to the library
        }
        outFile.close();

        initialize(); //reinitialize all of the motions to account for the changes made to animation file
        setMotion("Animation");
        std::cout<<"Success! Motion Queue saved to " << MOTIONS_PREPATH << extension << "\n\n";

}


void MotorController::setReturnDelayTime() {
        std::vector<int> datas;
        datas.resize(NUM_MOTORS);
        for (int i = 0; i < NUM_MOTORS; i++) {
                datas[i] = NEW_RETURN_DELAY_TIME;
        }
        sendSyncWrite(motors, RETURN_DELAY_TIME, BYTE, datas);
}

bool MotorController::addMotion(std::string animationName) {
        bool result = true;
        if (motions.find(animationName) != motions.end()) {
                result = false;
        }
        else {
                currentMotion = animationName;
                Motion newMotion;
                newMotion.friendlyName = animationName;
                newMotion.length = 1;
                newMotion.currentIndex = 0;
                newMotion.currPos.resize(NUM_MOTORS);

                newMotion.motorPositions.resize(newMotion.length);
                newMotion.motorVelocities.resize(newMotion.length);
                newMotion.motorIDs.resize(newMotion.length);

                newMotion.recoverPos.resize(newMotion.length);
                newMotion.recoverVel.resize(newMotion.length);

                newMotion.time.resize(newMotion.length);

                newMotion.motorPositions[newMotion.currentIndex].resize(NUM_MOTORS);
                newMotion.motorVelocities[newMotion.currentIndex].resize(NUM_MOTORS);
                newMotion.motorIDs[newMotion.currentIndex].resize(NUM_MOTORS);

                newMotion.time[0] = -1;
                newMotion.motorPositions[0][0] = -1;
                newMotion.motorVelocities[0][0] = -1;
                newMotion.motorIDs[0][0] = -1;
                newMotion.currPos[0] = -1;

                motions.insert(std::pair<std::string, Motion>(animationName, newMotion));

                currMo = motions.find(animationName)->second; //sets the current motion to the value of the key in the hashmap
        }

        return result;

        //      std::cout << motions.find(animationName)->second.time[0] << std::endl;
}

void MotorController::addMotionStep() {

        //currMo = motions.find(currentMotion)->second;

        std::cout << "Adding new step: " << currMo.length << std::endl;
        currMo.length = currMo.length + 1;
        std::cout << "Time for previous step: " << currMo.time[currMo.currentIndex] << std::endl;
        std::cout << "New length: " << currMo.length << std::endl;
        currMo.currPos.resize(currMo.length); //TODO not sure why this is done. currPos is a 1D vector with length of NUM_MOTORS
        currMo.currentIndex++;

        currMo.motorPositions.resize(currMo.length);
        currMo.motorVelocities.resize(currMo.length);
        currMo.motorIDs.resize(currMo.length);

        currMo.recoverPos.resize(currMo.length);
        currMo.recoverVel.resize(currMo.length);
        currMo.time.resize(currMo.length);

        currMo.motorPositions[currMo.currentIndex].resize(currMo.num_motors);
        currMo.motorVelocities[currMo.currentIndex].resize(currMo.num_motors);
        currMo.motorIDs[currMo.currentIndex].resize(currMo.num_motors);


        //set a -1 flag for the first motor on the last step to see if the step is empty or not
        currMo.time[currMo.length-1] = -1;
        currMo.motorPositions[currMo.length-1][0] = -1;
        currMo.motorVelocities[currMo.length-1][0] = -1;

        //set the motor IDs to the same as last step (same motor IDs for all steps in a motion)
        for(int j= 0; j< currMo.num_motors; j++){
                currMo.motorIDs[currMo.currentIndex][j] = currMo.motorIDs[currMo.currentIndex-1][j] ;
        }
        for(int j= 0; j< currMo.num_motors; j++){
                currMo.motorIDs[currMo.currentIndex][j] = currMo.motorIDs[currMo.currentIndex-1][j] ;
        }



        currMo.currPos[currMo.length-1] = -1;

        motions[currentMotion] = currMo;
        //debugging tp see if motorIDs vector works
        std::cout << "motors in this step: " << currMo.num_motors << std::endl;
        std::cout << "Current step: " << currMo.currentIndex << std::endl;

        for(int j= 0; j< currMo.num_motors; j++){
                std::cout<<currMo.motorPositions[currMo.currentIndex][j]<<"\t"<<currMo.motorVelocities[currMo.currentIndex][j]<<"\t"<<currMo.motorIDs[currMo.currentIndex][j]<<std::endl;
        }

}

void MotorController::setMotorLimits(int motor, int CW, int CCW){
        dxl_write_word(motor, CW_LIMIT, CW);
        dxl_write_word(motor, CCW_LIMIT, CCW);


}

void MotorController::setLimbSelection(int select_code){
        currMo.limb_select= select_code;

        switch (select_code){
        case 1:         //all motors

                currMo.num_motors=18;//NUM_LEG_MOTORS+NUM_ARM_MOTORS;   //this motion will use 20 motors

                currMo.motorPositions[currMo.currentIndex].resize(currMo.num_motors);   //re-adjust the size of the vectors for x motors
                currMo.motorVelocities[currMo.currentIndex].resize(currMo.num_motors);
                currMo.motorIDs[currMo.currentIndex].resize(currMo.num_motors);

                for(int i=0; i<currMo.num_motors; i++){
                        currMo.motorIDs[currMo.currentIndex][i]= i+1;
                }
                break;

        case 2: //legs
                currMo.num_motors=NUM_LEG_MOTORS;

                currMo.motorPositions[currMo.currentIndex].resize(currMo.num_motors);   //re-adjust the size of the vectors for x motors
                currMo.motorVelocities[currMo.currentIndex].resize(currMo.num_motors);
                currMo.motorIDs[currMo.currentIndex].resize(currMo.num_motors);

                for(int i=0; i<currMo.num_motors; i++){
                        currMo.motorIDs[currMo.currentIndex][i]= i+1;
                }
                break;

        case 3: //right leg
                currMo.num_motors=6;

                currMo.motorPositions[currMo.currentIndex].resize(currMo.num_motors);   //re-adjust the size of the vectors for x motors
                currMo.motorVelocities[currMo.currentIndex].resize(currMo.num_motors);
                currMo.motorIDs[currMo.currentIndex].resize(currMo.num_motors);

                for (int i = 0; i < currMo.num_motors; i++) {
                        currMo.motorIDs[currMo.currentIndex][i] = 2*i + 1;
                }
                break;

        case 4: //left leg
                currMo.num_motors=6;

                currMo.motorPositions[currMo.currentIndex].resize(currMo.num_motors);   //re-adjust the size of the vectors for x motors
                currMo.motorVelocities[currMo.currentIndex].resize(currMo.num_motors);
                currMo.motorIDs[currMo.currentIndex].resize(currMo.num_motors);

                for (int i = 0; i < currMo.num_motors; i++) {
                        currMo.motorIDs[currMo.currentIndex][i] = 2*i + 2;
                }
                break;

        case 5: //arms
                currMo.num_motors=NUM_ARM_MOTORS;

                currMo.motorPositions[currMo.currentIndex].resize(currMo.num_motors);   //re-adjust the size of the vectors for x motors
                currMo.motorVelocities[currMo.currentIndex].resize(currMo.num_motors);
                currMo.motorIDs[currMo.currentIndex].resize(currMo.num_motors);

                for (int i = 0; i < currMo.num_motors; i++) {
                        currMo.motorIDs[currMo.currentIndex][i] = i + 1 + 12;
                }
                break;

        case 6: //right arm
                currMo.num_motors=3;

                currMo.motorPositions[currMo.currentIndex].resize(currMo.num_motors);   //re-adjust the size of the vectors for x motors
                currMo.motorVelocities[currMo.currentIndex].resize(currMo.num_motors);
                currMo.motorIDs[currMo.currentIndex].resize(currMo.num_motors);

                for (int i = 0; i < currMo.num_motors; i++) {
                        currMo.motorIDs[currMo.currentIndex][i] = 2*i + 1 + 12;
                }
                break;

        case 7: //left arm
                currMo.num_motors=3;

                currMo.motorPositions[currMo.currentIndex].resize(currMo.num_motors);   //re-adjust the size of the vectors for x motors
                currMo.motorVelocities[currMo.currentIndex].resize(currMo.num_motors);
                currMo.motorIDs[currMo.currentIndex].resize(currMo.num_motors);

                for (int i = 0; i < currMo.num_motors; i++) {
                        currMo.motorIDs[currMo.currentIndex][i] = 2*i + 2 + 12;
                }
                break;

        case 8: //head
                currMo.num_motors=2;

                currMo.motorPositions[currMo.currentIndex].resize(currMo.num_motors);   //re-adjust the size of the vectors for x motors
                currMo.motorVelocities[currMo.currentIndex].resize(currMo.num_motors);
                currMo.motorIDs[currMo.currentIndex].resize(currMo.num_motors);

                for (int i = 0; i < 2; i++) {
                        currMo.motorIDs[currMo.currentIndex][i] = i + 23;
                }
                break;

        case 9: //process to add arms to an existing motion

                if(currMo.num_motors==NUM_LEG_MOTORS ||currMo.num_motors==NUM_LEG_MOTORS+2 || currMo.num_motors==2){


                        currMo.num_motors+=6;
                        for(int i=0; i<currMo.length; i++){
                                currMo.motorPositions[i].resize(currMo.num_motors);     //re-adjust the size of the vectors for x motors
                                currMo.motorVelocities[i].resize(currMo.num_motors);
                                currMo.motorIDs[i].resize(currMo.num_motors);
                        }
                        //these conditionals will assign the correct IDs in the correct location in the vector

                        //assign the arms when only legs are being used
                        if(currMo.num_motors-6==NUM_LEG_MOTORS){
                                for(int i= 0; i<currMo.length; i++){
                                        for (int j = currMo.num_motors-6; j < currMo.num_motors; j++) {
                                                currMo.motorIDs[i][j] = j + 1;
                                        }
                                }
                        }
                        //assign the arms when legs AND head are being used
                        if(currMo.num_motors-6==NUM_LEG_MOTORS+2){
                                for(int i= 0; i<currMo.length; i++){
                                        for (int j = currMo.num_motors-8; j < currMo.num_motors; j++) {
                                                currMo.motorIDs[i][j] = j + 1;
                                        }
                                }
                        }

                        //assign the arms when only head is being used
                        if(currMo.num_motors-6==2){
                                for(int i= 0; i<currMo.length; i++){
                                        for (int j = currMo.num_motors-6; j < currMo.num_motors; j++) {
                                                currMo.motorIDs[i][j] = j + 11;
                                        }
                                }
                        }
                }
                else{
                        std::cout<<"Arms are already being used!"<<std::endl;
                }
                break;

        case 10:        //process to add head to existing motion

                if(currMo.num_motors==NUM_LEG_MOTORS ||currMo.num_motors==NUM_LEG_MOTORS+NUM_ARM_MOTORS || currMo.num_motors==NUM_ARM_MOTORS){

                        currMo.num_motors+=2;

                        for(int i=0; i<currMo.length; i++){
                                currMo.motorPositions[i].resize(currMo.num_motors);     //re-adjust the size of the vectors for x motors
                                currMo.motorVelocities[i].resize(currMo.num_motors);
                                currMo.motorIDs[i].resize(currMo.num_motors);
                        }

                        //these conditionals will assign the correct IDs in the correct location in the vector

                        //assign the head when only legs are being used
                        if(currMo.num_motors-2==NUM_LEG_MOTORS){
                                for(int i= 0; i<currMo.length; i++){
                                        for (int j = currMo.num_motors-2; j < currMo.num_motors; j++) {
                                                currMo.motorIDs[i][j] = j + 11;
                                        }
                                }
                        }
                        //assign the head when legs AND arms are being used
                        if(currMo.num_motors-2==NUM_LEG_MOTORS+NUM_ARM_MOTORS){
                                for(int i= 0; i<currMo.length; i++){
                                        for (int j = currMo.num_motors-2; j < currMo.num_motors; j++) {
                                                currMo.motorIDs[i][j] = j + 5;
                                        }
                                }
                        }

                        //assign the head when only arms are being used
                        if(currMo.num_motors-2==NUM_LEG_MOTORS){
                                for(int i= 0; i<currMo.length; i++){
                                        for (int j = currMo.num_motors-2; j < currMo.num_motors; j++) {
                                                currMo.motorIDs[i][j] = j + 17;
                                        }
                                }
                        }
                }
                else{
                        std::cout<<"Head is already being used!"<<std::endl;
                }
                break;


        }
        //debugging tp see if motorIDs vector works
        for(int i= 0; i<currMo.length; i++){
                std::cout<<"STEP "<<i<<std::endl;
                for(int j= 0; j< currMo.num_motors; j++){
                        std::cout<<currMo.motorPositions[i][j]<<"\t"<<currMo.motorVelocities[i][j]<<"\t"<<currMo.motorIDs[i][j]<<std::endl;
                }
                std::cout<<"\n";
        }
}
bool MotorController::addMotionQueue(std::string animationName){
        bool result = true;
        if (motionQueues.find(animationName) != motionQueues.end()) {
                result = false;
        }
        else {
                //currentMotionQueue = animationName;

                MotionQueue newQueue;
                newQueue.friendlyName = animationName;
                newQueue.length = 0;
                newQueue.currentIndex = 0;
                newQueue.motionList.resize(1);
                newQueue.pauseTime.resize(1);


                std::cout << "inserted stuff into the struct \n  ";

                motionQueues.insert(std::pair<std::string, MotionQueue>(animationName, newQueue));
                std::cout << "inserted queue name into hashmap \n  ";

                currQueue = motionQueues.find(animationName)->second; //sets the current motion to the value of the key in the hashmap
        }

        return result;  //must deal with this in UI (motorController_test.cpp)
}

void MotorController::addMotionToQueue(std::string MotionName){

        while (motions.find(MotionName) == motions.end()) {
                std::cout << "That motion was not recognized, please try another" << std::endl;
                std::cin >> MotionName;
                std::getline(std::cin, MotionName);
        }
        std::cout << "Adding new Motion: " << MotionName << std::endl;
        currQueue.length++;
        std::cout << "New length: " << currQueue.length << std::endl;
        std::cout<< "current index = "<<currQueue.currentIndex<<std::endl;

        //currQueue.currentIndex++;// this should be done in MotorController_test.cpp
        currQueue.motionList.resize(currQueue.length);
        currQueue.pauseTime.resize(currQueue.length);

        currQueue.motionList[currQueue.currentIndex]= MotionName;
        std::cout<< "saved the motion to the vector"<<std::endl;

        motionQueues[currentMotionQueue] = currQueue;
}

void MotorController::recalculateCurrentMotionSpeeds(){
        int initialPos;
        int finalPos;
        for(int i= 0; i<currMo.length; i++){
                for(int j=0; j<currMo.num_motors; j++){
                        if(i == 0){
                                currMo.motorVelocities[i][j]= INITIAL_SPEED;
                                std::cout <<"Step "<< currMo.currentIndex<< "...Motor "<< j+1<< " position saved..." <<std::endl;

                        }
                        else {
                                /*Calaculate speeds to get from one pose to the next*/
                                initialPos= currMo.motorPositions[i-1][j];
                                finalPos= currMo.motorPositions[i][j];
                                currMo.motorVelocities[i][j]= SPEED_CONSTANT*abs(initialPos-finalPos)/currMo.time[currMo.currentIndex];
                        }
                }
                std::cout <<"Recalculating speed for step "<< i<<std::endl;

        }
}


void MotorController::displayMotionStatus(){

        std::cout<<"\n";
        if(!motionExecutionDisabled){
                std::cout<<"RECORDING MOTORS";
                for(int i=0; i<currMo.num_motors; i++){
                        std::cout<<" "<<currMo.motorIDs[0][i];
                }
                std::cout<<"\nROBOT IS IN *ACTIVE* MODE\n";
                std::cout<<currMo.friendlyName<<" [CURRENT STEP IS "<<currMo.currentIndex-1<<"]"<<std::endl;;
        }
        else{
                std::cout<<"RECORDING MOTORS";
                for(int i=0; i<currMo.num_motors; i++){
                        std::cout<<"\t"<<currMo.motorIDs[0][i];
                }
                std::cout<<"ROBOT IS IN *PASSIVE* MODE\n\n";
                std::cout<<"[CURRENT STEP IS "<<currMo.currentIndex-1<<"]"<<std::endl;;

        }
        //point at the current step
        for(int i=0; i<3; i++){
                for(int j=0; j<currMo.currentIndex+1; j++){

                        if(j==0){
                        }
                        else if(j== currMo.currentIndex){
                                if(i==1){
                                        std::cout<<"  @\t\n";
                                }
                                else if(i==2){
                                        std::cout<<"   \t\n";
                                }
                                else{
                                        std::cout<<" <->\t\n";
                                }
                        }
                        else{
                                std::cout<<"\t ";
                        }
                }
        }
        for(int i= 0; i<currMo.length; i++){
                std::cout<<"Step "<< i<<"\t";
                if(i==currMo.length-1){
                        std::cout<<"\n";
                }
        }

        for(int i= 0; i<currMo.length; i++){
                std::cout<<"["<<currMo.time[i]<<"]\t";
                if(i==currMo.length-1){
                        std::cout<<"\n";
                }
        }

}

void MotorController::disableMotionExecution(){
        //disableAllMotors();   //this will turn off the motors. active mode must be entered
        motionExecutionDisabled= true;  //this variable is applied in executeNext()
}
void MotorController::enableMotionExecution(){
        motionExecutionDisabled= false; //this variable is applied in executeNext()
}

int MotorController::getBoundedPosition(int pos, int id){

        switch(id){
        case 1:
                if(pos>=M1_CW && pos<=M1_CCW){
                        return pos;
                }
                else{
                        return -1;
                }
                break;
        case 2:
                if(pos>=M2_CW && pos<=M2_CCW){
                        return pos;
                }
                else{
                        return -1;
                }
                break;
        case 3:
                if(pos>=M3_CW && pos<=M3_CCW){
                        return pos;
                }
                else{
                        return -1;
                }
                break;
        case 4:
                if(pos>=M4_CW && pos<=M4_CCW){
                        return pos;
                }
                else{
                        return -1;
                }
                break;
        case 5:
                if(pos>=M5_CW && pos<=M5_CCW){
                        return pos;
                }
                else{
                        return -1;
                }
                break;
        case 6:
                if(pos>=M6_CW && pos<=M6_CCW){
                        return pos;
                }
                else{
                        return -1;
                }
                break;
        case 7:
                if(pos>=M7_CW && pos<=M7_CCW){
                        return pos;
                }
                else{
                        return -1;
                }
                break;
        case 8:
                if(pos>=M8_CW && pos<=M8_CCW){
                        return pos;
                }
                else{
                        return -1;
                }
                break;
        case 9:
                if(pos>=M9_CW && pos<=M9_CCW){
                        return pos;
                }
                else{
                        return -1;
                }
                break;
        case 10:
                if(pos>=M10_CW && pos<=M10_CCW){
                        return pos;
                }
                else{
                        return -1;
                }
                break;
        case 11:
                if(pos>=M11_CW && pos<=M11_CCW){
                        return pos;
                }
                else{
                        return -1;
                }
                break;
        case 12:
                if(pos>=M12_CW && pos<=M12_CCW){
                        return pos;
                }
                else{
                        return -1;
                }
                break;
        case 13:
                if(pos>=M13_CW && pos<=M13_CCW){
                        return pos;
                }
                else{
                        return -1;
                }
                break;
        case 14:
                if(pos>=M14_CW && pos<=M14_CCW){
                        return pos;
                }
                else{
                        return -1;
                }
                break;
        case 15:
                if(pos>=M15_CW && pos<=M15_CCW){
                        return pos;
                }
                else{
                        return -1;
                }
                break;
        case 16:
                if(pos>=M16_CW && pos<=M16_CCW){
                        return pos;
                }
                else{
                        return -1;
                }
                break;
        case 17:
                if(pos>=M17_CW && pos<=M17_CCW){
                        return pos;
                }
                else{
                        return -1;
                }
                break;
        case 18:
                if(pos>=M18_CW && pos<=M18_CCW){
                        return pos;
                }
                else{
                        return -1;
                }
                break;
        case 19:
                if(pos>=M19_CW && pos<=M19_CCW){
                        return pos;
                }
                else{
                        return -1;
                }
                break;
        case 20:
                if(pos>=M20_CW && pos<=M20_CCW){
                        return pos;
                }
                else{
                        return -1;
                }
                break;
        case 21:
                if(pos>=M21_CW && pos<=M21_CCW){
                        return pos;
                }
                else{
                        return -1;
                }
                break;
        case 22:
                if(pos>=M22_CW && pos<=M22_CCW){
                        return pos;
                }
                else{
                        return -1;
                }
                break;
        case 23:
                if(pos>=M23_CW && pos<=M23_CCW){
                        return pos;
                }
                else{
                        return -1;
                }
                break;
        case 24:
                if(pos>=M24_CW && pos<=M24_CCW){
                        return pos;
                }
                else{
                        return -1;
                }
                break;
        default:
                return -1;
                break;
        }
        return -1;
}

void MotorController::moveHead(int direction, int speed) {
        switch (direction) {
        case MUL8_HEAD_UP:
                // Write the moving speed to motor 24
                dxl_write_word(24, MOVING_SPEED, speed);
                // Set the goal position as the farthest back we can go
                dxl_write_word(24, GOAL_POSITION, M24_CCW);
                dxl_write_word(23, GOAL_POSITION, dxl_read_word(23, PRESENT_POSITION));
                break;
        case MUL8_HEAD_DOWN:
                // Write the moving speed to motor 24
                dxl_write_word(24, MOVING_SPEED, speed);
                // Set the goal position as the farthest forward we can go
                dxl_write_word(24, GOAL_POSITION, M24_CW);
                dxl_write_word(23, GOAL_POSITION, dxl_read_word(23, PRESENT_POSITION));
                break;
        case MUL8_HEAD_LEFT:
                // Write the moving speed to motor 23
                dxl_write_word(23, MOVING_SPEED, speed);
                // Write the goal position as the farthest left we can go
                dxl_write_word(23, GOAL_POSITION, M23_CCW);
                dxl_write_word(24, GOAL_POSITION, dxl_read_word(24, PRESENT_POSITION));
                break;
        case MUL8_HEAD_RIGHT:
                // Write the moving speed to motor 23
                dxl_write_word(23, MOVING_SPEED, speed);
                // Set the goal position as the farthest right we can go
                dxl_write_word(23, GOAL_POSITION, M23_CW);
                dxl_write_word(24, GOAL_POSITION, dxl_read_word(24, PRESENT_POSITION));
                break;
        case MUL8_HEAD_UP_LEFT:
                // Write the moving speed to motors 23 and 24
                dxl_write_word(23, MOVING_SPEED, speed/sqrt(2));
                dxl_write_word(24, MOVING_SPEED, speed/sqrt(2));
                // Set the goal position as the farthest left and up we can go
                dxl_write_word(23, GOAL_POSITION, M23_CCW);
                dxl_write_word(24, GOAL_POSITION, M24_CCW);
                break;
        case MUL8_HEAD_DOWN_LEFT:
                // Set the speed for both motors 23 and 24
                dxl_write_word(23, MOVING_SPEED, speed/sqrt(2));
                dxl_write_word(24, MOVING_SPEED, speed/sqrt(2));
                // Set the goal position as the farthest left and down we can go
                dxl_write_word(23, GOAL_POSITION, M23_CCW);
                dxl_write_word(24, GOAL_POSITION, M24_CW);
                break;

        case MUL8_HEAD_UP_RIGHT:
                // Write the moving speed to motors 23 and 24
                dxl_write_word(23, MOVING_SPEED, speed/sqrt(2));
                dxl_write_word(24, MOVING_SPEED, speed/sqrt(2));
                // Set the goal position as the farthest up and right we can go
                dxl_write_word(23, GOAL_POSITION, M23_CW);
                dxl_write_word(24, GOAL_POSITION, M24_CCW);
                break;
        case MUL8_HEAD_DOWN_RIGHT:
                // Write the moving speed to motors 23 and 24
                dxl_write_word(23, MOVING_SPEED, speed/sqrt(2));
                dxl_write_word(24, MOVING_SPEED, speed/sqrt(2));
                // Set the goal position as the farthest down and right we can go
                dxl_write_word(23, GOAL_POSITION, M23_CW);
                dxl_write_word(24, GOAL_POSITION, M24_CW);
                break;
        default:
                break;
        }
}

void MotorController::stopHead() {
        int motor23pos = dxl_read_word(23, PRESENT_POSITION);
        int motor24pos = dxl_read_word(24, PRESENT_POSITION);
        dxl_write_word(23, GOAL_POSITION, motor23pos);
        dxl_write_word(24, GOAL_POSITION, motor24pos);
}

bool MotorController::headLeftRightIsMoving() {
        bool result = false;
        int present23pos = dxl_read_word(23, PRESENT_POSITION);
        int goal23pos = dxl_read_word(23, GOAL_POSITION);
//      std::cout << "23: " << present23pos << " vs " << goal23pos << std::endl;
        if (present23pos < (goal23pos-25) || (present23pos > goal23pos+25)) {
//              std::cout << "Returning true" << std::endl;
                result = true;
        }
        return result;
}

bool MotorController::headUpDownIsMoving() {
        bool result = false;
        int present24pos = dxl_read_word(24, PRESENT_POSITION);
        int goal24pos = dxl_read_word(24, GOAL_POSITION);
//      std::cout << "24: " << present24pos << " vs " << goal24pos << std::endl;
        if (present24pos < (goal24pos-25) || present24pos > goal24pos+25) {
                result = true;
        }
        return result;
}

double MotorController::getHeadAngle() {
        int headPosition = readMotorPosition(23);
        headPosition -= 2048; //Subtract the center position to get the relative location
        double angle = headPosition * DEGREES_PER_POSITION * -1;
        return angle;
}

void MotorController::setMotorPosition(int motor, int position, int speed=-1) {
	if (speed != -1) {
		dxl_write_word(motor, MOVING_SPEED, speed);
	}
	dxl_write_word(motor, GOAL_POSITION, position);
}


