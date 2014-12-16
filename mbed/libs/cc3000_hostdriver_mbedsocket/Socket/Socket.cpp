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

#include "Socket.h"
#include <cstring>

Socket::Socket() : _sock_fd(-1), _blocking(true), _timeout(1500) {
    _cc3000_module = cc3000::get_instance();
    if (_cc3000_module == NULL) {
        error("Socket constructor error: no cc3000 instance available!\r\n");
    }
}

int Socket::init_socket(int type, int protocol) {
    if (_sock_fd != -1) {
        DBG_SOCKET("Socket was initialized previously");
        return -1;
    }

    int fd = _cc3000_module->_socket.socket(AF_INET, type, protocol);
    if (fd < -1) {
        DBG_SOCKET("Failed to create new socket (type: %d, protocol: %d)",type, protocol);
        return -1;
    }

    DBG_SOCKET("Socket created (fd: %d type: %d, protocol: %d)",fd, type, protocol);
    _sock_fd = fd;

    return 0;
}

void Socket::set_blocking(bool blocking, unsigned int timeout) {
    _blocking = blocking;
    _timeout = timeout;
}

int Socket::set_option(int level, int optname, const void *optval, socklen_t optlen) {
#ifndef CC3000_TINY_DRIVER
    return _cc3000_module->_socket.setsockopt(_sock_fd, level, optname, optval, optlen);
#else
    return -1;
#endif
}

int Socket::get_option(int level, int optname, void *optval, socklen_t *optlen) {
    return _cc3000_module->_socket.getsockopt(_sock_fd, level, optname, optval, optlen);
}

int Socket::select(struct timeval *timeout, bool read, bool write) {
    if (_sock_fd < 0) {
        return -1;
    }

    fd_set fdSet;
    FD_ZERO(&fdSet);
    FD_SET(_sock_fd, &fdSet);

    fd_set* readset  = (read ) ? (&fdSet) : (NULL);
    fd_set* writeset = (write) ? (&fdSet) : (NULL);

    int ret = _cc3000_module->_socket.select(_sock_fd+1, readset, writeset, NULL, timeout);

    DBG_SOCKET("Select on sock_fd: %d, returns %d. fdSet: %d", _sock_fd, ret, FD_ISSET(_sock_fd, &fdSet));

    // TODO
    //return (ret <= 0 || !FD_ISSET(_sock_fd, &fdSet)) ? (-1) : (0);
    if (FD_ISSET(_sock_fd, &fdSet)) {
        return 0;
    } else {
        return -1;
    }
}

int Socket::wait_readable(TimeInterval& timeout) {
    return select(&timeout._time, true, false);
}

int Socket::wait_writable(TimeInterval& timeout) {
    return select(&timeout._time, false, true);
}

int Socket::close() {
    if (_sock_fd < 0 ) {
        return -1;
    }

    _cc3000_module->_socket.closesocket(_sock_fd);
    _sock_fd = -1;
    return 0;
}

Socket::~Socket() {
    close();
}

TimeInterval::TimeInterval(unsigned int ms) {
    _time.tv_sec = ms / 1000;
    _time.tv_usec = (ms - (_time.tv_sec * 1000)) * 1000;
}
