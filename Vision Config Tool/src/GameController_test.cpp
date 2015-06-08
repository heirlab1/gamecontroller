#include "MotorController.h"
#include "Vision.h"
#include "GameController.h"
#include "MUL8.h"

int main(int argc, char** argv) {
	MUL8 mul8;
//	GameController gameController;

//	MotorController motorController;
//	motorController.init();
//	motorController.setMotion("Wi");
//	motorController.step(false);
//	Vision vis;
//	vis.init(motorController);
//	vis.setAction(SEARCH_FOR_GOAL);


//	RoboCupGameControlData myData;

	int delayTime = 5;

	std::cout << "Start in how many seconds? ";
	//std::string input;
	//std::cin >> input;
	//std::stringstream(input)>>delayTime;

	mul8.setState(MUL8_STATE_INIT);
	mul8.step();
	mul8.setState(MUL8_STATE_READY);
	mul8.step();
	mul8.setState(MUL8_STATE_SET);
	mul8.step();
	sleep(delayTime);
	mul8.setState(MUL8_STATE_PLAY);


	while (1) {
//		gameController.getGCData(myData);
//		int gameState = myData.state;
//		mul8.setState(gameState);
		mul8.step();
//		vis.nextFrame();
	}
}
