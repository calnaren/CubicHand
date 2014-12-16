#include "TCPClient.h"

static void error(const char *msg)
{
    perror(msg);
    exit(0);
}

TCPClient::TCPClient():OptionSet(false)
{

}

TCPClient::~TCPClient()
{

}

void TCPClient::Init(const char * Host, const char * Port)
{
	strcpy(mHost, Host);
	strcpy(mPort, Port);
    portno = atoi(Port);
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");
    server = gethostbyname(Host);
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
        error("ERROR connecting");
}

int TCPClient::Send(unsigned char * Buf, int Length)
{
	int n;
	n = write(sockfd,(char *)Buf,Length);
    if (n < 0) 
         error("ERROR writing to socket");
    return n;
}

int TCPClient::Receive(unsigned char * Buf, int Length)
{
	int n;
    n = read(sockfd,(char *)Buf,Length);
    if (n < 0)
    {
    	printf("Error Closing \r\n");
    	Close();
	    sockfd = socket(AF_INET, SOCK_STREAM, 0);
	    if (sockfd < 0) 
	        error("ERROR opening socket");
    	if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
        	error("ERROR connecting");//*/
        //Init((const char *)mHost, (const char *)mPort);
        if(OptionSet) SetOptions();
        usleep(1000000);
    } //error("ERROR reading from socket");
    return n;
}

void TCPClient::Close()
{
	close(sockfd);
}

void TCPClient::SetOptions()
{	
	tv.tv_sec  = 1;
	tv.tv_usec = 0;
	setsockopt (sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv));
	OptionSet = true;

}