/*
 * GameController.cpp
 *
 *  Created on: Jul 11, 2014
 *      Author: unicorn
 */

#include "GameController.h"


//MUL8 mul8;
int sockfd;
/* my address information */
struct sockaddr_in my_addr;
/* connector’s address information */
struct sockaddr_in their_addr;
int addr_len, numbytes;
char buf[MAXBUFLEN];

GameController::GameController() {
	// TODO Auto-generated constructor stub

//
//
//	if((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
//	{
//		perror("Server-socket() sockfd error lol!");
//		exit(1);
//	}
//	else
//		//    printf("Server-socket() sockfd is OK...\n");
//
//		/* host byte order */
//		my_addr.sin_family = AF_INET;
//	/* short, network byte order */
//	my_addr.sin_port = htons(MYPORT);
//	/* automatically fill with my IP */
//	my_addr.sin_addr.s_addr = INADDR_ANY;
//	/* zero the rest of the struct */
//	memset(&(my_addr.sin_zero), '\0', 8);
//
//	if(bind(sockfd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)) == -1)
//	{
//		perror("Server-bind() error ");
//		exit(1);
//	}

}

GameController::~GameController() {
	// TODO Auto-generated destructor stub
//	if(close(sockfd) != 0)
//		printf("Server-sockfd closing failed!\n");
//	else{
//		printf("Server-sockfd successfully closed!\n");
//	}
}

void GameController::getGCData(RoboCupGameControlData &myData){

	if((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
	{
		perror("Server-socket() sockfd error lol!");
		exit(1);
	}
	else{
		//    printf("Server-socket() sockfd is OK...\n");
	}
		/* host byte order */
		my_addr.sin_family = AF_INET;
	/* short, network byte order */
	my_addr.sin_port = htons(MYPORT);
	/* automatically fill with my IP */
	my_addr.sin_addr.s_addr = INADDR_ANY;
	/* zero the rest of the struct */
	memset(&(my_addr.sin_zero), '\0', 8);

	if(bind(sockfd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)) == -1)
	{
		perror("Server-bind() error ");
		exit(1);
	}


	//  else
	//    printf("Server-bind() is OK...\n");

	//loop to keep socket stream open
	//while(n==1){

	addr_len = sizeof(struct sockaddr);

	numbytes = recvfrom(sockfd, buf, MAXBUFLEN-1, 0, (struct sockaddr *)&their_addr, (socklen_t*)&addr_len);
	if(numbytes == -1)
	{
		perror("Server-recvfrom() error ");

		exit(1);
	}
	else
	{
		//      printf("Server-Waiting and listening...\n");
		//      printf("Server-recvfrom() is OK...\n");
	}

	if ((uint8_t)buf[4] == 8){

		//  printf("Server-Got packet from %s\n", inet_ntoa(their_addr.sin_addr));
		//  printf("Server-Packet is %d bytes long\n", numbytes);
		//  buf[numbytes] = '\0';
		//  printf("Server-Packet contains \"%s\"\n", buf);


		//struct RoboCupGameControlData myData;

		memcpy(&myData, buf, (size_t)numbytes);

		//  //variables for penalty states of each player
		//  int player1penalty = myData.teams[0].players[0].penalty;
		//  int ptime1 = myData.teams[0].players[0].secsTillUnpenalised;
		//
		//  int player2penalty = myData.teams[0].players[1].penalty;
		//  int ptime2 = myData.teams[0].players[1].secsTillUnpenalised;
		//
		//  int player3penalty = myData.teams[0].players[2].penalty;
		//  int ptime3 = myData.teams[0].players[2].secsTillUnpenalised;
		//
		//  int player4penalty = myData.teams[0].players[3].penalty;
		//  int ptime4 = myData.teams[0].players[3].secsTillUnpenalised;
		//
		//  int t2player1penalty = myData.teams[1].players[0].penalty;
		//  int t2ptime1 = myData.teams[1].players[0].secsTillUnpenalised;
		//
		//  int t2player2penalty = myData.teams[1].players[1].penalty;
		//  int t2ptime2 = myData.teams[1].players[1].secsTillUnpenalised;
		//
		//  int t2player3penalty = myData.teams[1].players[2].penalty;
		//  int t2ptime3 = myData.teams[1].players[2].secsTillUnpenalised;
		//
		//  int t2player4penalty = myData.teams[1].players[3].penalty;
		//  int t2ptime4 = myData.teams[1].players[3].secsTillUnpenalised;
		//
		//  //variable of package data
		//  int version = myData.version;
		//  int packetNumber = myData.packetNumber;
		//  int playersPerTeam = myData.playersPerTeam;
		//  int state = myData.state;
		//  int firstHalf = myData.firstHalf;
		//  int kickOffTeam = myData.kickOffTeam;
		//  int secondaryState = myData.secondaryState;
		//  int dropInTeam = myData.dropInTeam;
		//  int dropInTime = myData.dropInTime;
		//  int secsRemaining = myData.secsRemaining;
		//  int secondaryTime = myData.secondaryTime;
		//  int team1 = myData.teams[0].teamNumber;
		//  int team2 = myData.teams[1].teamNumber;

		//  mul8.setState(state);
		//  mul8.step();

		//displays packet info
		//  printf("Version: %u \n", myData.version);
		//  printf("Packet Number: %u \n", myData.packetNumber);
		//  printf("Players Per Team: %u \n", myData.playersPerTeam);
		//  printf("State: %u \n", myData.state);
		//  printf("Half: %u \n", myData.firstHalf);
		//  printf("Kick Off Team: %u\n", myData.kickOffTeam);
		//  printf("Secondary State: %u \n", myData.secondaryState);
		//  printf("Drop In Team: %u\n", myData.dropInTeam);
		//  printf("Drop In Time: %u \n", myData.dropInTime);
		//  printf("Time Remaining: %u \n", myData.secsRemaining);
		//  printf("Next state countdown: %u \n", myData.secondaryTime);
		//  printf("team 0 teamNumber: %u \n", myData.teams[0].teamNumber);
		//  printf("team 1 teamNumber: %u \n", myData.teams[1].teamNumber);
		//
		//  printf("Penalty on Team1 player 1: %u \n", player1penalty);
		//  printf("Penalty on Team1 player 2: %u \n", player2penalty);
		//  printf("Penalty on Team1 player 3: %u \n", player3penalty);
		//  printf("Penalty on Team1 player 4: %u \n", player4penalty);
		//
		//  printf("Penalty on Team2 player 1: %u \n", t2player1penalty);
		//  printf("Penalty on Team2 player 2: %u \n", t2player2penalty);
		//  printf("Penalty on Team2 player 3: %u \n", t2player3penalty);
		//  printf("Penalty on Team2 player 4: %u \n", t2player4penalty);

		//}
	}
	if(close(sockfd) != 0)
		printf("Server-sockfd closing failed!\n");
	else{
//		printf("Server-sockfd successfully closed!\n");
	}
}

