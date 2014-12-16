/*****************************************************************************
*
*  C++ interface/implementation created by Martin Kojtal (0xc0170). Thanks to
*  Jim Carver and Frank Vannieuwkerke for their inital cc3000 mbed port and
*  provided help.
*
*  This version of "host driver" uses CC3000 Host Driver Implementation. Thus
*  read the following copyright:
*
*  Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*****************************************************************************/
#ifndef CC3000_SOCKET_H
#define CC3000_SOCKET_H

#define SOCKET_MAX_FREE_BUFFERS    6

#define SOCKET_STATUS_ACTIVE       0
#define SOCKET_STATUS_INACTIVE     1

#define SOCKET_STATUS_INIT_VAL  0xFFFF
#define M_IS_VALID_SD(sd) ((0 <= (sd)) && ((sd) <= 7))
#define M_IS_VALID_STATUS(status) (((status) == SOCKET_STATUS_ACTIVE)||((status) == SOCKET_STATUS_INACTIVE))

#ifdef _API_USE_BSD_CLOSE
    #define close(sd) closesocket(sd)
#endif

//Enable this flag if and only if you must comply with BSD socket read() and
//write() functions
#ifdef _API_USE_BSD_READ_WRITE
    #define read(sd, buf, len, flags) recv(sd, buf, len, flags)
    #define write(sd, buf, len, flags) send(sd, buf, len, flags)
#endif

#define SOCKET_OPEN_PARAMS_LEN                 (12)
#define SOCKET_CLOSE_PARAMS_LEN                (4)
#define SOCKET_ACCEPT_PARAMS_LEN               (4)
#define SOCKET_BIND_PARAMS_LEN                 (20)
#define SOCKET_LISTEN_PARAMS_LEN               (8)
#define SOCKET_GET_HOST_BY_NAME_PARAMS_LEN     (9)
#define SOCKET_CONNECT_PARAMS_LEN              (20)
#define SOCKET_SELECT_PARAMS_LEN               (44)
#define SOCKET_SET_SOCK_OPT_PARAMS_LEN         (20)
#define SOCKET_GET_SOCK_OPT_PARAMS_LEN         (12)
#define SOCKET_RECV_FROM_PARAMS_LEN            (12)
#define SOCKET_SENDTO_PARAMS_LEN               (24)
#define SOCKET_MDNS_ADVERTISE_PARAMS_LEN       (12)

//#define NULL 0

// The legnth of arguments for the SEND command: sd + buff_offset + len + flags,
// while size of each parameter is 32 bit - so the total length is 16 bytes;

#define HCI_CMND_SEND_ARG_LENGTH                    (16)
#define SELECT_TIMEOUT_MIN_MICRO_SECONDS            5000
#define HEADERS_SIZE_DATA                           (SPI_HEADER_SIZE + 5)
#define SIMPLE_LINK_HCI_CMND_TRANSPORT_HEADER_SIZE  (SPI_HEADER_SIZE + SIMPLE_LINK_HCI_CMND_HEADER_SIZE)
#define MDNS_DEVICE_SERVICE_MAX_LENGTH              (32)


#define HOSTNAME_MAX_LENGTH (230)  // 230 bytes + header shouldn't exceed 8 bit value

//--------- Address Families --------

#define  AF_INET                2
#define  AF_INET6               23

//------------ Socket Types ------------

#define  SOCK_STREAM            1
#define  SOCK_DGRAM             2
#define  SOCK_RAW               3           // Raw sockets allow new IPv4 protocols to be implemented in user space. A raw socket receives or sends the raw datagram not including link level headers
#define  SOCK_RDM               4
#define  SOCK_SEQPACKET         5

//----------- Socket Protocol ----------

#define IPPROTO_IP              0           // dummy for IP
#define IPPROTO_ICMP            1           // control message protocol
#define IPPROTO_IPV4            IPPROTO_IP  // IP inside IP
#define IPPROTO_TCP             6           // tcp
#define IPPROTO_UDP             17          // user datagram protocol
#define IPPROTO_IPV6            41          // IPv6 in IPv6
#define IPPROTO_NONE            59          // No next header
#define IPPROTO_RAW             255         // raw IP packet
#define IPPROTO_MAX             256

//----------- Socket retunr codes  -----------

#define SOC_ERROR                (-1)        // error
#define SOC_IN_PROGRESS          (-2)        // socket in progress

//----------- Socket Options -----------
#define  SOL_SOCKET             0xffff       //  socket level
#define  SOCKOPT_RECV_TIMEOUT   1            //  optname to configure recv and recvfromtimeout
#define  SOCKOPT_NONBLOCK       2            // accept non block mode set SOCK_ON or SOCK_OFF (default block mode )
#define  SOCK_ON                0            // socket non-blocking mode    is enabled
#define  SOCK_OFF               1            // socket blocking mode is enabled

#define  TCP_NODELAY            0x0001
#define  TCP_BSDURGENT          0x7000

#define  MAX_PACKET_SIZE        1500
#define  MAX_LISTEN_QUEUE       4

#define  IOCTL_SOCKET_EVENTMASK

#define __FD_SETSIZE            32

#define  ASIC_ADDR_LEN          8

#define NO_QUERY_RECIVED        -3


typedef struct _in_addr_t
{
    uint32_t s_addr;                   // load with inet_aton()
} in_addr;

/*typedef struct _sockaddr_t
{
    unsigned short int  sa_family;
    unsigned char       sa_data[14];
} sockaddr;*/

typedef struct _sockaddr_in_t
{
    int16_t  sin_family;            // e.g. AF_INET
    uint16_t sin_port;              // e.g. htons(3490)
    in_addr  sin_addr;              // see struct in_addr, below
    uint8_t  sin_zero[8];           // zero this if you want to
} sockaddr_in;

typedef uint32_t socklen_t;

// The fd_set member is required to be an array of longs.
typedef int32_t __fd_mask;

// It's easier to assume 8-bit bytes than to get CHAR_BIT.
#define __NFDBITS               (8 * sizeof (__fd_mask))
#define __FDELT(d)              ((d) / __NFDBITS)
#define __FDMASK(d)             ((__fd_mask) 1 << ((d) % __NFDBITS))

#ifndef FD_SET
//not used in the current code
#define ENOBUFS                 55          // No buffer space available

// Access macros for 'fd_set'.
#define FD_SET(fd, fdsetp)      __FD_SET (fd, fdsetp)
#define FD_CLR(fd, fdsetp)      __FD_CLR (fd, fdsetp)
#define FD_ISSET(fd, fdsetp)    __FD_ISSET (fd, fdsetp)
#define FD_ZERO(fdsetp)         __FD_ZERO (fdsetp)

// fd_set for select and pselect.
typedef struct
{
    __fd_mask fds_bits[__FD_SETSIZE / __NFDBITS];
#define __FDS_BITS(set)        ((set)->fds_bits)
} fd_set;

#endif /* FD_SET */

// We don't use `memset' because this would require a prototype and
//   the array isn't too big.
#define __FD_ZERO(set)                               \
  do {                                                \
    uint32_t __i;                                 \
    fd_set *__arr = (set);                            \
    for (__i = 0; __i < sizeof (fd_set) / sizeof (__fd_mask); ++__i) \
      __FDS_BITS (__arr)[__i] = 0;                    \
  } while (0)
#define __FD_SET(d, set)       (__FDS_BITS (set)[__FDELT (d)] |= __FDMASK (d))
#define __FD_CLR(d, set)       (__FDS_BITS (set)[__FDELT (d)] &= ~__FDMASK (d))
#define __FD_ISSET(d, set)     (__FDS_BITS (set)[__FDELT (d)] & __FDMASK (d))

//Use in case of Big Endian only

#define htonl(A)    ((((uint32_t)(A) & 0xff000000) >> 24) | \
                     (((uint32_t)(A) & 0x00ff0000) >> 8) | \
                     (((uint32_t)(A) & 0x0000ff00) << 8) | \
                     (((uint32_t)(A) & 0x000000ff) << 24))

#define ntohl                   htonl

//Use in case of Big Endian only
#define htons(A)     ((((uint32_t)(A) & 0xff00) >> 8) | \
                      (((uint32_t)(A) & 0x00ff) << 8))


#define ntohs                   htons

// mDNS port - 5353    mDNS multicast address - 224.0.0.251
#define SET_mDNS_ADD(sockaddr) sockaddr.sa_data[0] = 0x14; \
                               sockaddr.sa_data[1] = 0xe9; \
                               sockaddr.sa_data[2] = 0xe0; \
                               sockaddr.sa_data[3] = 0x0;  \
                               sockaddr.sa_data[4] = 0x0;  \
                               sockaddr.sa_data[5] = 0xfb;

#endif
