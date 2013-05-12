/*******************udpserver.c*****************/
/* Header files needed to use the sockets API. */
/* File contain Macro, Data Type and Structure */
/* definitions along with Function prototypes. */
/* header files */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "server.h"

// Global vars
char buff[100];
char *bufptr = buff;
int socket_d, irc;
int buflen = sizeof(buff);
struct sockaddr_in serveraddr, clientaddr;
int clientaddrlen = sizeof(clientaddr);
int serveraddrlen = sizeof(serveraddr);

/* Run the server without argument */
int init_server(int port) {
	/* The socket() function returns a socket */
	/* descriptor representing an endpoint. */
	/* The statement also identifies that the */
	/* INET (Internet Protocol) address family */
	/* with the UDP transport (SOCK_DGRAM) will */
	/* be used for this socket. */
	/******************************************/
	/* get a socket descriptor */
	if((socket_d = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("UDP server - socket() error");
		return -1;
	}
	else
		printf("UDP server - socket() is OK\n");

	printf("UDP server - try to bind...\n");

	/* After the socket descriptor is received, */
	/* a bind() is done to assign a unique name */
	/* to the socket. In this example, the user */
	/* set the s_addr to zero. This allows the */
	/* system to connect to any client that uses */
	/* port 3333. */
	/********************************************/
	/* bind to address */
	memset(&serveraddr, 0x00, serveraddrlen);
	serveraddr.sin_family = AF_INET;
	serveraddr.sin_port = htons(port);
	serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
	if((irc = bind(socket_d, (struct sockaddr *)&serveraddr, serveraddrlen)) < 0) {
		perror("UDP server - bind() error");
		shutdown(socket_d, 2);
		/* If something wrong with socket(), just exit lol */
		exit(-1);
	}
	else
		printf("UDP server - bind() is OK\n");

	printf("Using IP %s and port %d\n", inet_ntoa(serveraddr.sin_addr), port);
    return socket_d;
}

char* receive(int socket_d) {
	printf("UDP server - Listening...\n");

	/* Use the recvfrom() function to receive the */
	/* data. The recvfrom() function waits */
	/* indefinitely for data to arrive. */
	/************************************************/
	/* This example does not use flags that control */
	/* the reception of the data. */
	/************************************************/
	/* Wait on client requests. */
	irc = recvfrom(socket_d, bufptr, buflen, 0, 
        (struct sockaddr *)&clientaddr, (socklen_t*) &clientaddrlen);
	if(irc < 0) {
		perror("UDP Server - recvfrom() error");
		return NULL;
	}
	else
		printf("UDP Server - recvfrom() is OK...\n");

	printf("UDP Server received the following:\n \"%s\" message\n", bufptr);
	printf("from port %d and address %s.\n", ntohs(clientaddr.sin_port),
	inet_ntoa(clientaddr.sin_addr));

    return bufptr;
}
	
int reply() {
    /* Send a reply by using the sendto() function. */
	/* In this example, the system echoes the received */
	/* data back to the client. */
	/************************************************/
	/* This example does not use flags that control */
	/* the transmission of the data */
	/************************************************/
	/* Send a reply, just echo the request */
	printf("UDP Server replying to the UDP client...\n");
	irc = sendto(socket_d, bufptr, buflen, 0, (struct sockaddr *)&clientaddr, clientaddrlen);

	if(irc < 0) {
		perror("UDP server - sendto() error");
		return 1;
	}
	else {
		printf("UDP Server - sendto() is OK...\n");
        return 0;
    }

	/* When the data has been sent, close() the */
	/* socket descriptor. */
	/********************************************/
	/* close() the socket descriptor. */
	//close(sd);
	//exit(0);
}
