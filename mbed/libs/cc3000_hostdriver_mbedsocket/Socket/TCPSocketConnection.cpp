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

#include "TCPSocketConnection.h"
#include <algorithm>

TCPSocketConnection::TCPSocketConnection() : _is_connected(false) {
    _cc3000_module = cc3000::get_instance();
    if (_cc3000_module == NULL) {
        error("Endpoint constructor error: no cc3000 instance available!\r\n");
    }
}

int TCPSocketConnection::connect(const char *host, const int port) {
    if (init_socket(SOCK_STREAM, IPPROTO_TCP) < 0) {
        DBG_SOCKET("Failed to create tcp socket");
        return -1;
    }

    if (set_address(host, port) != 0) {
        DBG_SOCKET("Failed to set address (tcp)");
        return -1;
    }

    if (_cc3000_module->_socket.connect(_sock_fd, (const sockaddr *)&_remote_host, sizeof(_remote_host)) < 0) {
        DBG_SOCKET("Failed to connect (tcp)");
        close();
        return -1;
    }

    _is_connected = true;

    return 0;
}

bool TCPSocketConnection::is_connected(void) {
    return _is_connected;
}

int TCPSocketConnection::send(char* data, int length) {
    if ((_sock_fd < 0) || !_is_connected) {
        return -1;
    }

    if (!_blocking) {
        TimeInterval timeout(_timeout);
        if (wait_writable(timeout) != 0) {
            return -1;
        }
    }

    int n = _cc3000_module->_socket.send(_sock_fd, data, length, 0);
    _is_connected = (n != 0);

    return n;
}

int TCPSocketConnection::send_all(char *data, int length) {
    if ((_sock_fd < 0) || !_is_connected) {
        return -1;
    }

    int writtenLen = 0;
    TimeInterval timeout(_timeout);
    while (writtenLen < length) {
        if (!_blocking) {
            // Wait for socket to be writeable
            if (wait_writable(timeout) != 0) {
                return writtenLen;
            }
        }

        int ret = _cc3000_module->_socket.send(_sock_fd, data + writtenLen, length - writtenLen, 0);
        if (ret > 0) {
            writtenLen += ret;
            continue;
        } else if (ret == 0) {
            _is_connected = false;
            return writtenLen;
        } else {
            return -1; //Connnection error
        }
    }

    return writtenLen;
}

int TCPSocketConnection::receive(char *data, int length) {
    if ((_sock_fd < 0) || !_is_connected) {
        return -1;
    }

    if (!_blocking) {
        TimeInterval timeout(_timeout);
        if (wait_readable(timeout) != 0)
            return -1;
    }

    int n = _cc3000_module->_socket.recv(_sock_fd, data, length, 0);
    if (n >= 0) {
        _is_connected = 1;
    } else {
        _is_connected = 0;
    }

    return n;
}

int TCPSocketConnection::receive_all(char *data, int length) {
    if ((_sock_fd < 0) || !_is_connected) {
        return -1;
    }

    int readLen = 0;
    TimeInterval timeout(_timeout);
    while (readLen < length) {
        if (!_blocking) {
            //Wait for socket to be readable
            if (wait_readable(timeout) != 0)
                return readLen;
        }

        int ret = _cc3000_module->_socket.recv(_sock_fd, data + readLen, length - readLen, 0);
        if (ret > 0) {
            readLen += ret;
        } else if (ret == 0) {
            _is_connected = false;
            return readLen;
        } else {
            return -1; //Connnection error
        }
    }
    return readLen;
}
