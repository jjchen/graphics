/************tcpclient.c************************/
/* Header files needed to use the sockets API. */
/* File contains Macro, Data Type and */
/* Structure definitions along with Function */
/* prototypes. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <errno.h>
#include "client.h"

/* BufferLength is 100 bytes */
#define BufferLength 100

/* Default host name of server system. Change it to your default */
/* server hostname or IP.  If the user do not supply the hostname */
/* as an argument, the_server_name_or_IP will be used as default*/
#define SERVER "The_server_name_or_IP"

int rc, length = sizeof(int);
char buffer[BufferLength];
char temp;
int totalcnt = 0;

/* Pass in 1 parameter which is either the */
/* address or host name of the server, or */
/* set the server name in the #define SERVER ... */
int client_init(char* server_addr, int port) {
    /* Variable and structure definitions. */
    int sd;
    struct sockaddr_in serveraddr;
    struct hostent *hostp;

    /* The socket() function returns a socket */
    /* descriptor representing an endpoint. */
    /* The statement also identifies that the */
    /* INET (Internet Protocol) address family */
    /* with the TCP transport (SOCK_STREAM) */
    /* will be used for this socket. */
    /******************************************/
    /* get a socket descriptor */
    if((sd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    	perror("Client-socket() error");
    	return -1;
    }
    else
    	printf("Client-socket() OK\n");

	memset(&serveraddr, 0x00, sizeof(struct sockaddr_in));
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_port = htons(port);
     
    if((serveraddr.sin_addr.s_addr = inet_addr(server_addr)) == (unsigned long)INADDR_NONE) {
		/* When passing the host name of the server as a */
		/* parameter to this program, use the gethostbyname() */
		/* function to retrieve the address of the host server. */
		/***************************************************/
		/* get host address */
		hostp = gethostbyname(server_addr);
		if(hostp == (struct hostent *)NULL) {
			printf("HOST NOT FOUND --> ");
			/* h_errno is usually defined */
			/* in netdb.h */
			printf("h_errno = %d\n",h_errno);
			printf("---This is a client program---\n");
		    return -1;
		}

    	memcpy(&serveraddr.sin_addr, hostp->h_addr, sizeof(serveraddr.sin_addr));
    }

    /* After the socket descriptor is received, the */
    /* connect() function is used to establish a */
    /* connection to the server. */
    /***********************************************/
    /* connect() to server. */
    if((rc = connect(sd, (struct sockaddr *)&serveraddr, sizeof(serveraddr))) < 0) {
		perror("Client-connect() error");
		return -1;
    }
    else
    	printf("Connection established...\n");
     
    return sd;
}

int client_write(char* data, int sd) {
    /* Send string to the server using */
    /* the write() function. */
    /*********************************************/
    /* Write() some string to the server. */
    rc = write(sd, data, sizeof(data));

    if(rc < 0) {
		perror("Client-write() error");
		rc = getsockopt(sd, SOL_SOCKET, SO_ERROR, &temp, (socklen_t*) &length);
		if(rc == 0) {
			/* Print out the asynchronously received error. */
			errno = temp;
			perror("SO_ERROR was");
		}

		//close(sd);
		//exit(-1);
        return -1;
    }
    else {
		printf("Client-write() is OK\n");
		printf("String successfully sent lol!\n");
    }

    return 0;
}

char *client_read(int sd) {
    totalcnt = 0;
    while (totalcnt < BufferLength) {
		/* Wait for the server to echo the */
		/* string by using the read() function. */
		/***************************************/
		/* Read data from the server. */
		rc = read(sd, &buffer[totalcnt], BufferLength-totalcnt);
		if(rc < 0) {
			perror("Client-read() error");
			//close(sd);
			//exit(-1);
            return NULL;
		}
		else if (rc == 0) {
			printf("Server program has issued a close()\n");
			//close(sd);
			//exit(-1);
            return NULL;
		}
		else
			totalcnt += rc;
    }

    printf("Client-read() is OK\n");
    printf("Echoed data from the f***ing server: %s\n", buffer);
    return buffer;
}

void client_close(int sd) {
    /* When the data has been read, close() */
    /* the socket descriptor. */
    /****************************************/
    /* Close socket descriptor from client side. */
    close(sd);
    //exit(0);
    //return 0;
}
