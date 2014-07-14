/*
 * GameController.h
 *
 *  Created on: Jul 11, 2014
 *      Author: unicorn
 */


#ifndef BALANCE_SERVER_H_
#define BALANCE_SERVER_H_

typedef unsigned char  uint8;
typedef unsigned short uint16;
typedef unsigned int   uint32;

extern "C"{
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdint.h>
}


/* the port users will be connecting to */
#define MYPORT 3434
#define MAXBUFLEN 500
#define SIZE 256

//#include "MUL8.h"


class balanceServer {
public:
	balanceServer();
	virtual ~balanceServer();
	int checkBalance(void);
	void init();
};

#endif /* BALANCE_SERVER_H_ */
