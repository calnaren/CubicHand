/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "mbed.h"
#include "cc3000_hostdriver_mbedsocket/cc3000.h"
#include "main.h"
#include "MMA8451Q/MMA8451Q/MMA8451Q.h"

#include "cc3000_hostdriver_mbedsocket/Socket/TCPSocketConnection.h"
#include "cc3000_hostdriver_mbedsocket/Socket/TCPSocketServer.h"

using namespace mbed_cc3000;

//#define SSID "wifiisawesome"
//#define PASSWORD "A915FA24"

#define SSID "CubeNet"
#define PASSWORD "modelbased"

#define MMA8451_I2C_ADDRESS (0x1d<<1)

/* cc3000 module declaration specific for user's board. Check also init()
#if (MY_BOARD == WIGO)
cc3000 wifi(PTA16, PTA13, PTD0, SPI(PTD2, PTD3, PTC5), SSID, PASSWORD, WPA2, false);
Serial pc(USBTX, USBRX);
#elif (MY_BOARD == WIFI_DIPCORTEX)
cc3000 wifi(p28, p27, p30, SPI(p21, p14, p37), SSID, PASSWORD, WPA2, false);
Serial pc(UART_TX, UART_RX);
#elif (MY_BOARD == MBED_BOARD_EXAMPLE)
cc3000 wifi(p9, p10, p8, SPI(p5, p6, p7), SSID, PASSWORD, WPA2, false);
Serial pc(USBTX, USBRX);
#else

#endif
*/

cc3000 wifi(PTD4, PTC9, PTD0, SPI(PTD2, PTD3, PTD1), SSID, PASSWORD, WPA2, false);
Serial pc(USBTX, USBRX);

MMA8451Q accelerometer(PTE25, PTE24, MMA8451_I2C_ADDRESS);

/**
 *  \brief TCP client demo
 *  \param none
 *  \return int
 */
int main() {
    float data[3];
    
    init(); /* board dependent init */
    pc.baud(115200);

    printf("cc3000 tcp client demo. \r\n");
    wifi.init();
    if (wifi.connect() == -1) {
        printf("Failed to connect. Please verify connection details and try again. \r\n");
    } else {
        printf("IP address: %s \r\n", wifi.getIPAddress());
    }
    
    const char* ECHO_SERVER_ADDRESS = "192.168.1.12";
    const int ECHO_SERVER_PORT = 1895;
    
     TCPSocketConnection socket;
     while (socket.connect(ECHO_SERVER_ADDRESS, ECHO_SERVER_PORT) < 0) {
         printf("Unable to connect to (%s) on port (%d) \r\n", ECHO_SERVER_ADDRESS, ECHO_SERVER_PORT);
         wait(1);
     }
     
     while(true) {
        printf("trying to get accel data\r\n");
        accelerometer.getAccAllAxis(data);
        printf("got accel data\r\n");
        socket.send_all((char*)data, 12);
     }
     
     //char hello[] = "Hello World\n";
     //socket.send_all(hello, sizeof(hello) - 1);
     
     char buf[256];
     int n = socket.receive(buf, 256);
     buf[n] = '\0';
     printf("%s", buf);
     
     socket.close();
     printf("Completed. \r\n");
     wifi.disconnect();
}
