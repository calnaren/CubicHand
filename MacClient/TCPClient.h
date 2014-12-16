#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

#pragma once

class TCPClient
{
	public:
		TCPClient();
		~TCPClient();
		void Init(const char * Host, const char * Port);
		int Send(unsigned char * Buf, int Length);
		int Receive(unsigned char * Buf, int Length);
		void Close();
		void SetOptions();

	private:
	    int sockfd;
	    int portno;
	    struct sockaddr_in serv_addr;
	    struct hostent *server;
		timeval tv;
		bool OptionSet;
		char mHost[256];
		char mPort[256];

};