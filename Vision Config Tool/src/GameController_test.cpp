#include "MotorController.h"
#include "Vision.h"
#include "GameController.h"
#include "MUL8.h"

int main(int argc, char** argv) {
	MUL8 mul8;
//	GameController gameController;
//	RoboCupGameControlData myData;

	while (1) {
//		gameController.getGCData(myData);
//		int gameState = myData.state;
//		mul8.setState(gameState);
		mul8.step();
	}
}
