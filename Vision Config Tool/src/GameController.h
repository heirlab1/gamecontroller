/*
 * GameController.h
 *
 *  Created on: Jul 11, 2014
 *      Author: unicorn
 */

#ifndef GAMECONTROLLER_H_
#define GAMECONTROLLER_H_

typedef unsigned char  uint8;
typedef unsigned short uint16;
typedef unsigned int   uint32;

extern "C" {

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "SPLCoachMessage.h"
#include "RoboCupGameControlData.h"
#include <stdint.h>
}
//#include "MUL8.h"


/* the port users will be connecting to */
#define MYPORT 3838
#define MAXBUFLEN 500
#define SIZE 256

class GameController {
public:
	GameController();
	virtual ~GameController();
	void getGCData(RoboCupGameControlData&);
};

#endif /* GAMECONTROLLER_H_ */
