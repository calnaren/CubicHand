#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include "TCPClient.h"


TCPClient CubeSocket;
TCPClient GloveSocket;

unsigned char numReceived;
int receiveCount = 0;
unsigned char pkglen;
unsigned char bcc;

unsigned char Buf[1000];

static void error(const char *msg)
{
    perror(msg);
    exit(0);
}


void SendStream()
{
    unsigned char SendB[10];
    SendB[0] = '$';
    SendB[1] = 0x0A;
    SendB[2] = 0x03;
    // Streaming quaternion data
    SendB[3] = 1;
    SendB[4] = (SendB[0]+SendB[1]+SendB[2]+SendB[3])%256;
    SendB[5] = '#';
    GloveSocket.Send(SendB, 7);
}

bool Receive()
{
    numReceived += GloveSocket.Receive((Buf + numReceived), 1);
    if (Buf[0] == '$' && numReceived > 0) 
    {
        bcc = '$';
        numReceived += GloveSocket.Receive((Buf+numReceived), 2);
        if ((numReceived > 2) && (Buf[1] == 0x0a)) 
        {
            pkglen = Buf[2];
            bcc += Buf[1];
            bcc += Buf[2];
            numReceived += GloveSocket.Receive((Buf+numReceived), pkglen - numReceived + 3);
            if ((numReceived  - 3 )< pkglen) 
            {
                //if(numReceived == 0) notConnectedCount++;
                return false;
            }
            for (int u = 3; u <= pkglen; u++)
            {
                bcc += Buf[u];
            }
            if ((numReceived - 3 == pkglen) && (bcc == Buf[pkglen+1])) 
            {
                //ReceiveCount++;
                receiveCount++;
                if(receiveCount > 1)
                {
                    receiveCount = 0; //*/
                    CubeSocket.Send(Buf,numReceived);
                }
                Buf[0] = 0; //Clear Token so no repeats.
                //notConnectedCount = 0;
                numReceived = 0;
                return true;
            }
            else
            {
                //if (bcc!=Buf[pkglen+1])
                    //notConnectedCount++;
            }
        }
    }
   else {
        numReceived = 0;
    }
    return false;
}//*/

int main(int argc, char *argv[])
{
    numReceived = 0;


    GloveSocket.Init("192.168.1.33", "2000");
    GloveSocket.SetOptions();
    printf("Connected to Glove \r\n");

    CubeSocket.Init("192.168.1.12", "2000");
    printf("Connected to Cube \r\n");
    unsigned char buffer[256];
 
    printf("Please enter the message: ");
    bzero(buffer,256);
    //fgets(buffer,255,stdin);
    SendStream();
    printf("Sent Start to Glove \r\n");
    unsigned char SendBuf[100];
    bzero(SendBuf, 100);
    SendBuf[0] = '$';
    SendBuf[1] = 0x0A;
    SendBuf[2] = 0x03+38;
    // Streaming quaternion data
    SendBuf[3] = 0;
    SendBuf[4+38] = (SendBuf[0]+SendBuf[1]+SendBuf[2]+SendBuf[3])%256;
    SendBuf[5+38] = '#';
    int i = 0;
    while(true) //for(volatile int i = 0; i < 5000; i++)
    {
        //printf("%d", Buf[0]);
        if(i++%100 == 0) printf("Running %d\r\n", i/100);
        if(Receive())
        {
            //printf("%d \r\n" , GloveSocket.Receive(Buf, 10));
            //printf("Packet! \r\n");
            //CubeSocket.Send(SendBuf,6+38);
        }//SendStream();
        //usleep(10000);
    }
    CubeSocket.Close();
    GloveSocket.Close();
    return 0;
}