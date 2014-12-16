#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

int main(int argc, char *argv[])
{
    int sockfd, portno, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    volatile unsigned char buffer[256];
    if (argc < 3) {
       fprintf(stderr,"usage %s hostname port\n", argv[0]);
       exit(0);
    }
    portno = atoi(argv[2]);
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");
    server = gethostbyname(argv[1]);
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
    //usleep(10000);
    printf("Please enter the message: ");
    bzero(buffer,256);
    //fgets(buffer,255,stdin);
    unsigned char SendBuf[6];
    bzero(SendBuf, 6);
    SendBuf[0] = '$';
    SendBuf[1] = 0x0A;
    SendBuf[2] = 0x03;
    // Streaming quaternion data
    SendBuf[3] = 2;
    SendBuf[4] = (SendBuf[0]+SendBuf[1]+SendBuf[2]+SendBuf[3])%256;
    SendBuf[5] = '#';
    //usleep(10000);
    //for(volatile int i = 0; i < 5000; i++)
    //{
        printf("test\r\n");
        n = write(sockfd,(char *)SendBuf,6);
        if (n < 0) error("ERROR writing to socket");
        //usleep(100000);
    //}
    bzero(buffer,256);
    n = 0;
    while(n <= 0)
    {
        n = read(sockfd,(char *)buffer,255);
    }
    printf("%d\r\n", n);
    //if (n < 0) error("ERROR reading from socket");
    printf("%s\n",buffer);
    close(sockfd);
    return 0;
}