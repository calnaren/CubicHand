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
#include "cc3000.h"
#include "cc3000_socket.h"
#include "cc3000_event.h" //TODO - remove this
#include "cc3000_common.h"

namespace mbed_cc3000 {

cc3000_socket::cc3000_socket(cc3000_simple_link &simplelink, cc3000_hci &hci, cc3000_event &event)
    : _simple_link(simplelink), _hci(hci), _event(event) {

}

cc3000_socket::~cc3000_socket() {

}

int32_t cc3000_socket::HostFlowControlConsumeBuff(int32_t sd) {
#ifndef SEND_NON_BLOCKING
    /* wait in busy loop */
    do {
        // When the last transmission failed, return the last failure reason.
        // Note that the buffer will not be allocated in this case
        if (_simple_link.get_transmit_error() != 0) {
            errno = _simple_link.get_transmit_error();
            _simple_link.set_transmit_error(0);
            return errno;
        }

        if(SOCKET_STATUS_ACTIVE != _event.get_socket_active_status(sd))
            return -1;
    } while (0 == _simple_link.get_number_free_buffers());

    uint16_t free_buffer = _simple_link.get_number_free_buffers();
    free_buffer--;
    _simple_link.set_number_free_buffers(free_buffer);

    return 0;
#else

    // When the last transmission failed, return the last failure reason.
    // Note that the buffer will not be allocated in this case
    if (_simple_link.get_transmit_error() != 0) {
        errno = _simple_link.get_transmit_error();
        _simple_link.set_transmit_error(0);
        return errno;
    }
    if (SOCKET_STATUS_ACTIVE != _event.get_socket_active_status(sd))
        return -1;

    // If there are no available buffers, return -2. It is recommended to use
    // select or receive to see if there is any buffer occupied with received data
    // If so, call receive() to release the buffer.
    if (0 == _simple_link.get_number_free_buffers()) {
        return -2;
    } else {
        uint16_t free_buffer = _simple_link.get_number_free_buffers();
        free_buffer--;
        _simple_link.set_number_free_buffers(free_buffer);
        return 0;
    }
#endif
}

int32_t cc3000_socket::socket(int32_t domain, int32_t type, int32_t protocol) {
    int32_t ret;
    uint8_t *ptr, *args;

    ret = EFAIL;
    ptr = _simple_link.get_transmit_buffer();
    args = (ptr + HEADERS_SIZE_CMD);

    // Fill in HCI packet structure
    args = UINT32_TO_STREAM(args, domain);
    args = UINT32_TO_STREAM(args, type);
    args = UINT32_TO_STREAM(args, protocol);

    // Initiate a HCI command
    _hci.command_send(HCI_CMND_SOCKET, ptr, SOCKET_OPEN_PARAMS_LEN);

    // Since we are in blocking state - wait for event complete
    _event.simplelink_wait_event(HCI_CMND_SOCKET, &ret);

    // Process the event
    errno = ret;

    _event.set_socket_active_status(ret, SOCKET_STATUS_ACTIVE);

    return ret;
}

int32_t cc3000_socket::closesocket(int32_t sd) {
    int32_t ret;
    uint8_t *ptr, *args;

    while (_simple_link.get_number_free_buffers() != SOCKET_MAX_FREE_BUFFERS);
    ret = EFAIL;
    ptr = _simple_link.get_transmit_buffer();
    args = (ptr + HEADERS_SIZE_CMD);

    // Fill in HCI packet structure
    args = UINT32_TO_STREAM(args, sd);

    // Initiate a HCI command
    _hci.command_send(HCI_CMND_CLOSE_SOCKET, ptr, SOCKET_CLOSE_PARAMS_LEN);

    // Since we are in blocking state - wait for event complete
    _event.simplelink_wait_event(HCI_CMND_CLOSE_SOCKET, &ret);
    errno = ret;

    // since 'close' call may result in either OK (and then it closed) or error, mark this socket as invalid
    _event.set_socket_active_status(sd, SOCKET_STATUS_INACTIVE);

    return ret;
}

int32_t cc3000_socket::accept(int32_t sd, sockaddr *addr, socklen_t *addrlen) {
    int32_t ret;
    uint8_t *ptr, *args;
    tBsdReturnParams tAcceptReturnArguments;

    ret = EFAIL;
    ptr = _simple_link.get_transmit_buffer();
    args = (ptr + HEADERS_SIZE_CMD);

    // Fill in temporary command buffer
    args = UINT32_TO_STREAM(args, sd);

    // Initiate a HCI command
    _hci.command_send(HCI_CMND_ACCEPT, ptr, SOCKET_ACCEPT_PARAMS_LEN);

    // Since we are in blocking state - wait for event complete
    _event.simplelink_wait_event(HCI_CMND_ACCEPT, &tAcceptReturnArguments);


    // need specify return parameters!!!
    memcpy(addr, &tAcceptReturnArguments.tSocketAddress, ASIC_ADDR_LEN);
    *addrlen = ASIC_ADDR_LEN;
    errno = tAcceptReturnArguments.iStatus;
    ret = errno;

    // if succeeded, iStatus = new socket descriptor. otherwise - error number
    if(M_IS_VALID_SD(ret)) {
        _event.set_socket_active_status(ret, SOCKET_STATUS_ACTIVE);
    } else {
        _event.set_socket_active_status(sd, SOCKET_STATUS_INACTIVE);
    }

    return ret;
}

int32_t cc3000_socket::bind(int32_t sd, const sockaddr *addr, int32_t addrlen) {
    int32_t ret;
    uint8_t *ptr, *args;

    ret = EFAIL;
    ptr = _simple_link.get_transmit_buffer();
    args = (ptr + HEADERS_SIZE_CMD);

    addrlen = ASIC_ADDR_LEN;

    // Fill in temporary command buffer
    args = UINT32_TO_STREAM(args, sd);
    args = UINT32_TO_STREAM(args, 0x00000008);
    args = UINT32_TO_STREAM(args, addrlen);
    ARRAY_TO_STREAM(args, ((uint8_t *)addr), addrlen);

    // Initiate a HCI command
    _hci.command_send(HCI_CMND_BIND, ptr, SOCKET_BIND_PARAMS_LEN);

    // Since we are in blocking state - wait for event complete
    _event.simplelink_wait_event(HCI_CMND_BIND, &ret);

    errno = ret;

    return ret;
}

int32_t cc3000_socket::listen(int32_t sd, int32_t backlog) {
    int32_t ret;
    uint8_t *ptr, *args;

    ret = EFAIL;
    ptr = _simple_link.get_transmit_buffer();
    args = (ptr + HEADERS_SIZE_CMD);

    // Fill in temporary command buffer
    args = UINT32_TO_STREAM(args, sd);
    args = UINT32_TO_STREAM(args, backlog);

    // Initiate a HCI command
    _hci.command_send(HCI_CMND_LISTEN, ptr, SOCKET_LISTEN_PARAMS_LEN);

    // Since we are in blocking state - wait for event complete
    _event.simplelink_wait_event(HCI_CMND_LISTEN, &ret);
    errno = ret;

    return(ret);
}

int32_t cc3000_socket::connect(int32_t sd, const sockaddr *addr, int32_t addrlen) {
    int32_t ret;
    uint8_t *ptr, *args;

    ret = EFAIL;
    ptr = _simple_link.get_transmit_buffer();
    args = (ptr + SIMPLE_LINK_HCI_CMND_TRANSPORT_HEADER_SIZE);
    addrlen = 8;

    // Fill in temporary command buffer
    args = UINT32_TO_STREAM(args, sd);
    args = UINT32_TO_STREAM(args, 0x00000008);
    args = UINT32_TO_STREAM(args, addrlen);
    ARRAY_TO_STREAM(args, ((uint8_t *)addr), addrlen);

    // Initiate a HCI command
    _hci.command_send(HCI_CMND_CONNECT, ptr, SOCKET_CONNECT_PARAMS_LEN);

    // Since we are in blocking state - wait for event complete
    _event.simplelink_wait_event(HCI_CMND_CONNECT, &ret);

    errno = ret;

    return((int32_t)ret);
}

int32_t cc3000_socket::select(int32_t nfds, fd_set *readsds, fd_set *writesds, fd_set *exceptsds, struct timeval *timeout) {
    uint8_t *ptr, *args;
    tBsdSelectRecvParams tParams;
    uint32_t is_blocking;

    if (timeout == NULL) {
        is_blocking = 1; /* blocking , infinity timeout */
    } else {
        is_blocking = 0; /* no blocking, timeout */
    }

    // Fill in HCI packet structure
    ptr = _simple_link.get_transmit_buffer();
    args = (ptr + HEADERS_SIZE_CMD);

    // Fill in temporary command buffer
    args = UINT32_TO_STREAM(args, nfds);
    args = UINT32_TO_STREAM(args, 0x00000014);
    args = UINT32_TO_STREAM(args, 0x00000014);
    args = UINT32_TO_STREAM(args, 0x00000014);
    args = UINT32_TO_STREAM(args, 0x00000014);
    args = UINT32_TO_STREAM(args, is_blocking);
    args = UINT32_TO_STREAM(args, ((readsds) ? *(uint32_t*)readsds : 0));
    args = UINT32_TO_STREAM(args, ((writesds) ? *(uint32_t*)writesds : 0));
    args = UINT32_TO_STREAM(args, ((exceptsds) ? *(uint32_t*)exceptsds : 0));

    if (timeout) {
        if ( 0 == timeout->tv_sec && timeout->tv_usec < SELECT_TIMEOUT_MIN_MICRO_SECONDS) {
            timeout->tv_usec = SELECT_TIMEOUT_MIN_MICRO_SECONDS;
        }
        args = UINT32_TO_STREAM(args, timeout->tv_sec);
        args = UINT32_TO_STREAM(args, timeout->tv_usec);
    }

    // Initiate a HCI command
    _hci.command_send(HCI_CMND_BSD_SELECT, ptr, SOCKET_SELECT_PARAMS_LEN);

    // Since we are in blocking state - wait for event complete
    _event.simplelink_wait_event(HCI_EVNT_SELECT, &tParams);

    // Update actually read FD
    if (tParams.iStatus >= 0) {
        if (readsds) {
            memcpy(readsds, &tParams.uiRdfd, sizeof(tParams.uiRdfd));
        }

        if (writesds) {
            memcpy(writesds, &tParams.uiWrfd, sizeof(tParams.uiWrfd));
        }

        if (exceptsds) {
            memcpy(exceptsds, &tParams.uiExfd, sizeof(tParams.uiExfd));
        }

        return(tParams.iStatus);

    } else {
        errno = tParams.iStatus;
        return -1;
    }
}

int32_t cc3000_socket::getsockopt (int32_t sd, int32_t level, int32_t optname, void *optval, socklen_t *optlen) {
    uint8_t *ptr, *args;
    tBsdGetSockOptReturnParams  tRetParams;

    ptr = _simple_link.get_transmit_buffer();
    args = (ptr + HEADERS_SIZE_CMD);

    // Fill in temporary command buffer
    args = UINT32_TO_STREAM(args, sd);
    args = UINT32_TO_STREAM(args, level);
    args = UINT32_TO_STREAM(args, optname);

    // Initiate a HCI command
    _hci.command_send(HCI_CMND_GETSOCKOPT, ptr, SOCKET_GET_SOCK_OPT_PARAMS_LEN);

    // Since we are in blocking state - wait for event complete
    _event.simplelink_wait_event(HCI_CMND_GETSOCKOPT, &tRetParams);

    if (((int8_t)tRetParams.iStatus) >= 0) {
        *optlen = 4;
        memcpy(optval, tRetParams.ucOptValue, 4);
        return (0);
    } else {
        errno = tRetParams.iStatus;
        return errno;
    }
}

int32_t cc3000_socket::simple_link_recv(int32_t sd, void *buf, int32_t len, int32_t flags, sockaddr *from, socklen_t *fromlen, int32_t opcode) {
    uint8_t *ptr, *args;
    tBsdReadReturnParams tSocketReadEvent;

    ptr = _simple_link.get_transmit_buffer();
    args = (ptr + HEADERS_SIZE_CMD);

    // Fill in HCI packet structure
    args = UINT32_TO_STREAM(args, sd);
    args = UINT32_TO_STREAM(args, len);
    args = UINT32_TO_STREAM(args, flags);

    // Generate the read command, and wait for the
    _hci.command_send(opcode,  ptr, SOCKET_RECV_FROM_PARAMS_LEN);

    // Since we are in blocking state - wait for event complete
    _event.simplelink_wait_event(opcode, &tSocketReadEvent);

    // In case the number of bytes is more then zero - read data
    if (tSocketReadEvent.iNumberOfBytes > 0) {
        // Wait for the data in a synchronous way. Here we assume that the bug is
        // big enough to store also parameters of receive from too....
        _event.simplelink_wait_data((uint8_t *)buf, (uint8_t *)from, (uint8_t *)fromlen);
    }

    errno = tSocketReadEvent.iNumberOfBytes;

    return(tSocketReadEvent.iNumberOfBytes);
}

int32_t cc3000_socket::recv(int32_t sd, void *buf, int32_t len, int32_t flags) {
    return(simple_link_recv(sd, buf, len, flags, NULL, NULL, HCI_CMND_RECV));
}

int32_t cc3000_socket::recvfrom(int32_t sd, void *buf, int32_t len, int32_t flags, sockaddr *from, socklen_t *fromlen) {
    return(simple_link_recv(sd, buf, len, flags, from, fromlen, HCI_CMND_RECVFROM));
}

int32_t cc3000_socket::simple_link_send(int32_t sd, const void *buf, int32_t len, int32_t flags, const sockaddr *to, int32_t tolen, int32_t opcode) {
    uint8_t uArgSize = 0x00,  addrlen = 0x00;
    uint8_t *ptr, *pDataPtr = NULL, *args;
    uint32_t addr_offset = 0x00;
    int32_t res;
    tBsdReadReturnParams tSocketSendEvent;

    // Check the bsd_arguments
    if (0 != (res = HostFlowControlConsumeBuff(sd))) {
        return res;
    }

    //Update the number of sent packets
    uint16_t sent_packets = _simple_link.get_sent_packets();
    sent_packets++;
    _simple_link.set_sent_packets(sent_packets);

    // Allocate a buffer and construct a packet and send it over spi
    ptr = _simple_link.get_transmit_buffer();
    args = (ptr + HEADERS_SIZE_DATA);

    // Update the offset of data and parameters according to the command
    switch(opcode)
    {
        case HCI_CMND_SENDTO:
        {
            addr_offset = len + sizeof(len) + sizeof(len);
            addrlen = 8;
            uArgSize = SOCKET_SENDTO_PARAMS_LEN;
            pDataPtr = ptr + HEADERS_SIZE_DATA + SOCKET_SENDTO_PARAMS_LEN;
            break;
        }

        case HCI_CMND_SEND:
        {
            tolen = 0;
            to = NULL;
            uArgSize = HCI_CMND_SEND_ARG_LENGTH;
            pDataPtr = ptr + HEADERS_SIZE_DATA + HCI_CMND_SEND_ARG_LENGTH;
            break;
        }

        default:
        {
            break;
        }
    }

    // Fill in temporary command buffer
    args = UINT32_TO_STREAM(args, sd);
    args = UINT32_TO_STREAM(args, uArgSize - sizeof(sd));
    args = UINT32_TO_STREAM(args, len);
    args = UINT32_TO_STREAM(args, flags);

    if (opcode == HCI_CMND_SENDTO) {
        args = UINT32_TO_STREAM(args, addr_offset);
        args = UINT32_TO_STREAM(args, addrlen);
    }

    // Copy the data received from user into the TX Buffer
    ARRAY_TO_STREAM(pDataPtr, ((uint8_t *)buf), len);

    // In case we are using SendTo, copy the to parameters
    if (opcode == HCI_CMND_SENDTO) {
        ARRAY_TO_STREAM(pDataPtr, ((uint8_t *)to), tolen);
    }

    // Initiate a HCI command
    _hci.data_send(opcode, ptr, uArgSize, len,(uint8_t*)to, tolen);
    if (opcode == HCI_CMND_SENDTO)
       _event.simplelink_wait_event(HCI_EVNT_SENDTO, &tSocketSendEvent);
    else
       _event.simplelink_wait_event(HCI_EVNT_SEND, &tSocketSendEvent);

    return (len);
}

int32_t cc3000_socket::send(int32_t sd, const void *buf, int32_t len, int32_t flags) {
    return(simple_link_send(sd, buf, len, flags, NULL, 0, HCI_CMND_SEND));
}

int32_t cc3000_socket::sendto(int32_t sd, const void *buf, int32_t len, int32_t flags, const sockaddr *to, socklen_t tolen) {
    return(simple_link_send(sd, buf, len, flags, to, tolen, HCI_CMND_SENDTO));
}

int32_t cc3000_socket::mdns_advertiser(uint16_t mdns_enabled, uint8_t *device_service_name, uint16_t device_service_name_length) {
    int32_t ret;
     uint8_t *pTxBuffer, *pArgs;

    if (device_service_name_length > MDNS_DEVICE_SERVICE_MAX_LENGTH) {
        return EFAIL;
    }

    pTxBuffer = _simple_link.get_transmit_buffer();
    pArgs = (pTxBuffer + SIMPLE_LINK_HCI_CMND_TRANSPORT_HEADER_SIZE);

    // Fill in HCI packet structure
    pArgs = UINT32_TO_STREAM(pArgs, mdns_enabled);
    pArgs = UINT32_TO_STREAM(pArgs, 8);
    pArgs = UINT32_TO_STREAM(pArgs, device_service_name_length);
    ARRAY_TO_STREAM(pArgs, device_service_name, device_service_name_length);

    // Initiate a HCI command
    _hci.command_send(HCI_CMND_MDNS_ADVERTISE, pTxBuffer, SOCKET_MDNS_ADVERTISE_PARAMS_LEN + device_service_name_length);

    // Since we are in blocking state - wait for event complete
    _event.simplelink_wait_event(HCI_EVNT_MDNS_ADVERTISE, &ret);

    return ret;
}


#ifndef CC3000_TINY_DRIVER
int32_t cc3000_socket::gethostbyname(uint8_t *hostname, uint16_t name_length, uint32_t *out_ip_addr) {
    tBsdGethostbynameParams ret;
    uint8_t *ptr, *args;

    errno = EFAIL;

    if (name_length > HOSTNAME_MAX_LENGTH) {
        return errno;
    }

    ptr = _simple_link.get_transmit_buffer();
    args = (ptr + SIMPLE_LINK_HCI_CMND_TRANSPORT_HEADER_SIZE);

    // Fill in HCI packet structure
    args = UINT32_TO_STREAM(args, 8);
    args = UINT32_TO_STREAM(args, name_length);
    ARRAY_TO_STREAM(args, hostname, name_length);

    // Initiate a HCI command
    _hci.command_send(HCI_CMND_GETHOSTNAME, ptr, SOCKET_GET_HOST_BY_NAME_PARAMS_LEN + name_length - 1);

    // Since we are in blocking state - wait for event complete
    _event.simplelink_wait_event(HCI_EVNT_BSD_GETHOSTBYNAME, &ret);

    errno = ret.retVal;

    (*((int32_t*)out_ip_addr)) = ret.outputAddress;

    return (errno);
}

int32_t cc3000_socket::setsockopt(int32_t sd, int32_t level, int32_t optname, const void *optval, socklen_t optlen) {
    int32_t ret;
    uint8_t *ptr, *args;

    ptr = _simple_link.get_transmit_buffer();
    args = (ptr + HEADERS_SIZE_CMD);

    // Fill in temporary command buffer
    args = UINT32_TO_STREAM(args, sd);
    args = UINT32_TO_STREAM(args, level);
    args = UINT32_TO_STREAM(args, optname);
    args = UINT32_TO_STREAM(args, 0x00000008);
    args = UINT32_TO_STREAM(args, optlen);
    ARRAY_TO_STREAM(args, ((uint8_t *)optval), optlen);

    // Initiate a HCI command
    _hci.command_send(HCI_CMND_SETSOCKOPT, ptr, SOCKET_SET_SOCK_OPT_PARAMS_LEN  + optlen);

    // Since we are in blocking state - wait for event complete
    _event.simplelink_wait_event(HCI_CMND_SETSOCKOPT, &ret);

    if (ret >= 0) {
        return (0);
    } else {
        errno = ret;
        return ret;
    }
}

#endif

char* cc3000_socket::inet_ntoa_r(uint32_t s_addr, char *buf, int buflen)
{
    char inv[3];
    char *rp;
    uint8_t *ap;
    uint8_t rem;
    uint8_t n;
    uint8_t i;
    int len = 0;

    rp = buf;
    ap = (uint8_t *)&s_addr;
    for (n = 0; n < 4; n++) {
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

} // mbed_cc3000 namespace
