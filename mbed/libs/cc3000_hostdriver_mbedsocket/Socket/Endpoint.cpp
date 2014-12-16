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
#include "Socket/Socket.h"
#include "Socket/Endpoint.h"
#include "Helper/def.h"
#include <cstring>

 #include "cc3000.h"

/* Copied from lwip */
static char *cc3000_inet_ntoa_r(const in_addr addr, char *buf, int buflen)
{
  uint32_t s_addr;
  char inv[3];
  char *rp;
  uint8_t *ap;
  uint8_t rem;
  uint8_t n;
  uint8_t i;
  int len = 0;

  s_addr = addr.s_addr;

  rp = buf;
  ap = (uint8_t *)&s_addr;
  for(n = 0; n < 4; n++) {
    i = 0;
    do {
      rem = *ap % (uint8_t)10;
      *ap /= (uint8_t)10;
      inv[i++] = '0' + rem;
    } while(*ap);
    while(i--) {
      if (len++ >= buflen) {
        return NULL;
      }
      *rp++ = inv[i];
    }
    if (len++ >= buflen) {
      return NULL;
    }
    *rp++ = '.';
    ap++;
  }
  *--rp = 0;
  return buf;
}

Endpoint::Endpoint()  {
    _cc3000_module = cc3000::get_instance();
    if (_cc3000_module == NULL) {
        error("Endpoint constructor error: no cc3000 instance available!\r\n");
    }
    reset_address();
}
Endpoint::~Endpoint() {}

void Endpoint::reset_address(void) {
    _ipAddress[0] = '\0';
    std::memset(&_remote_host, 0, sizeof(sockaddr_in));
}

int Endpoint::set_address(const char* host, const int port) {
    reset_address();

    char address[5];
    char *p_address = address;

    signed int add[5];

    // Dot-decimal notation
    int result = std::sscanf(host, "%3u.%3u.%3u.%3u", &add[0], &add[1], &add[2], &add[3]);
    for (int i=0;i<4;i++) {
      address[i] = add[i];
    }
    std::memset(_ipAddress,0,sizeof(_ipAddress));

    if (result != 4) {
 #ifndef CC3000_TINY_DRIVER
        //Resolve DNS address or populate hard-coded IP address
        uint32_t address_integer;
        int resolveRetCode;
        resolveRetCode = _cc3000_module->_socket.gethostbyname((uint8_t *)host, strlen(host) , &address_integer);

        if ((resolveRetCode > -1) && (0 != address_integer)) {
            _remote_host.sin_addr.s_addr = htonl(address_integer);
            cc3000_inet_ntoa_r(_remote_host.sin_addr, _ipAddress, sizeof(_ipAddress));
        } else {
            // Failed to resolve the address
            DBG_SOCKET("Failed to resolve the hostname : %s",host);
            return (-1);
        }
#else
        return -1;
#endif
    } else {
        std::memcpy((char*)&_remote_host.sin_addr.s_addr, p_address, 4);
    }

    _remote_host.sin_family = AF_INET;
    _remote_host.sin_port = htons(port);

    DBG_SOCKET("remote host address (string): %s",get_address());
    DBG_SOCKET("remote host address from s_addr : %d.%d.%d.%d",
            int(_remote_host.sin_addr.s_addr & 0xFF),
            int((_remote_host.sin_addr.s_addr & 0xFF00) >> 8),
            int((_remote_host.sin_addr.s_addr & 0xFF0000) >> 16),
            int((_remote_host.sin_addr.s_addr & 0xFF000000) >> 24));
    DBG_SOCKET("port: %d", port);

    return 0;
}

char* Endpoint::get_address() {
    if ((_ipAddress[0] == '\0') && (_remote_host.sin_addr.s_addr != 0))
            cc3000_inet_ntoa_r(_remote_host.sin_addr, _ipAddress, sizeof(_ipAddress));
    return _ipAddress;
}


int   Endpoint::get_port() {
    return ntohs(_remote_host.sin_port);
}
