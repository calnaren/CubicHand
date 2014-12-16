/* Copyright (C) 2013 mbed.org, MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "UDPSocket.h"

#include <string>
#include <algorithm>

UDPSocket::UDPSocket() {

}

int UDPSocket::init(void) {
    return init_socket(SOCK_DGRAM, IPPROTO_UDP);
}

int UDPSocket::bind(int port) {
    if (init_socket(SOCK_DGRAM, IPPROTO_UDP) < 0) {
        return -1;
    }

    sockaddr_in localHost;
    std::memset(&localHost, 0, sizeof(sockaddr_in));

    localHost.sin_family = AF_INET;
    localHost.sin_port = htons(port);
    localHost.sin_addr.s_addr = 0;

    if (_cc3000_module->_socket.bind(_sock_fd, (sockaddr *)&localHost, sizeof(sockaddr_in)) != 0) {
        DBG_SOCKET("Failed to bind a socket (udp). Closing socket");
        _cc3000_module->_socket.closesocket(_sock_fd);
        _sock_fd = -1;
        return -1;
    }

    return 0;
}

int UDPSocket::sendTo(Endpoint &remote, char *packet, int length)
{
    if (_sock_fd < 0) {
        return -1;
    }
    // TODO - seems to be a bug, waiting for TI to respond
    // if (!_blocking) {
    //     TimeInterval timeout(_timeout);
    //     if (wait_writable(timeout) != 0) {
    //         DBG_SOCKET("The socket is not writeable. _sock_fd: %d", _sock_fd);
    //         return 0;
    //     }
    // }

    return _cc3000_module->_socket.sendto(_sock_fd, packet, length, 0, (sockaddr *)&remote._remote_host, sizeof(sockaddr));
}

int UDPSocket::receiveFrom(Endpoint &remote, char *buffer, int length)
{
    if (_sock_fd < 0) {
        return -1;
    }

    if (!_blocking) {
        TimeInterval timeout(_timeout);
        if (wait_readable(timeout) != 0) {
            DBG_SOCKET("The socket is not readable. _sock_fd: %d", _sock_fd);
            return 0;
        }
    }

    remote.reset_address();
    socklen_t remote_host_length = sizeof(remote._remote_host);

    return _cc3000_module->_socket.recvfrom(_sock_fd, buffer, length, 0, (sockaddr *)&remote._remote_host, &remote_host_length);
}


