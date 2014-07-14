
#include "balanceServer.h"


//MUL8 mul8;
int balanceSockfd;
/* my address information */
struct sockaddr_in this_addr;
/* connector’s address information */
struct sockaddr_in sensor_addr;
int addr_len_1, numbytes_1;
char sensor_buf[MAXBUFLEN];


balanceServer::balanceServer(){


}

balanceServer::~balanceServer(){

}

void balanceServer::init(){

}
int balanceServer::checkBalance(){

	//printf( "starting server\n");

	if((balanceSockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
	{
		perror("Server-socket() sockfd error lol!");
		exit(1);
	}
	else{
		//printf("Server-socket() sockfd is OK...\n");
	}
	/* host byte order */
	this_addr.sin_family = AF_INET;
	/* short, network byte order */
	this_addr.sin_port = htons(MYPORT);
	/* automatically fill with my IP */
	this_addr.sin_addr.s_addr = INADDR_ANY;
	/* zero the rest of the struct */
	memset(&(this_addr.sin_zero), '\0', 8);

	if(bind(balanceSockfd, (struct sockaddr *)&this_addr, sizeof(struct sockaddr)) == -1)
	{
		perror("Server-bind() error ");
		exit(1);
	}
	else{


		addr_len_1 = sizeof(struct sockaddr);

		numbytes_1 = recvfrom(balanceSockfd, sensor_buf, MAXBUFLEN-1, 0, (struct sockaddr *)&sensor_addr,(socklen_t*)&addr_len_1);

		sensor_buf[numbytes_1] = '\0';
		//printf("Server-Packet contains \"%s\"\n", sensor_buf);
		char *pNext;
		int num = strtol(sensor_buf,&pNext, 10);
		printf("%d\n", num);

		//##########close socket################

		if(close(balanceSockfd) != 0){

			//printf("Server-sockfd closing failed!\n");
		}

		return num;


	}




}




