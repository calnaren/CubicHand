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
#ifndef CC3000_EVENT_H
#define CC3000_EVENT_H

typedef struct _bsd_read_return_t
{
    int32_t             iSocketDescriptor;
    int32_t             iNumberOfBytes;
    uint32_t    uiFlags;
} tBsdReadReturnParams;

typedef struct _bsd_getsockopt_return_t
{
    uint8_t ucOptValue[4];
    uint8_t          iStatus;
} tBsdGetSockOptReturnParams;

typedef struct _bsd_accept_return_t
{
    int32_t             iSocketDescriptor;
    int32_t             iStatus;
    sockaddr         tSocketAddress;

} tBsdReturnParams;

typedef struct _bsd_select_return_t
{
    int32_t            iStatus;
    uint32_t           uiRdfd;
    uint32_t           uiWrfd;
    uint32_t           uiExfd;
} tBsdSelectRecvParams;

typedef struct _bsd_gethostbyname_return_t
{
    int32_t  retVal;
    int32_t  outputAddress;
} tBsdGethostbynameParams;

#define FLOW_CONTROL_EVENT_HANDLE_OFFSET        (0)
#define FLOW_CONTROL_EVENT_BLOCK_MODE_OFFSET    (1)
#define FLOW_CONTROL_EVENT_FREE_BUFFS_OFFSET    (2)
#define FLOW_CONTROL_EVENT_SIZE                 (4)

#define BSD_RSP_PARAMS_SOCKET_OFFSET            (0)
#define BSD_RSP_PARAMS_STATUS_OFFSET            (4)

#define GET_HOST_BY_NAME_RETVAL_OFFSET          (0)
#define GET_HOST_BY_NAME_ADDR_OFFSET            (4)

#define ACCEPT_SD_OFFSET                        (0)
#define ACCEPT_RETURN_STATUS_OFFSET             (4)
#define ACCEPT_ADDRESS__OFFSET                  (8)

#define SL_RECEIVE_SD_OFFSET                    (0)
#define SL_RECEIVE_NUM_BYTES_OFFSET             (4)
#define SL_RECEIVE__FLAGS__OFFSET               (8)


#define SELECT_STATUS_OFFSET                    (0)
#define SELECT_READFD_OFFSET                    (4)
#define SELECT_WRITEFD_OFFSET                   (8)
#define SELECT_EXFD_OFFSET                      (12)


#define NETAPP_IPCONFIG_IP_OFFSET               (0)
#define NETAPP_IPCONFIG_SUBNET_OFFSET           (4)
#define NETAPP_IPCONFIG_GW_OFFSET               (8)
#define NETAPP_IPCONFIG_DHCP_OFFSET             (12)
#define NETAPP_IPCONFIG_DNS_OFFSET              (16)
#define NETAPP_IPCONFIG_MAC_OFFSET              (20)
#define NETAPP_IPCONFIG_SSID_OFFSET             (26)

#define NETAPP_IPCONFIG_IP_LENGTH               (4)
#define NETAPP_IPCONFIG_MAC_LENGTH              (6)
#define NETAPP_IPCONFIG_SSID_LENGTH             (32)


#define NETAPP_PING_PACKETS_SENT_OFFSET         (0)
#define NETAPP_PING_PACKETS_RCVD_OFFSET         (4)
#define NETAPP_PING_MIN_RTT_OFFSET              (8)
#define NETAPP_PING_MAX_RTT_OFFSET              (12)
#define NETAPP_PING_AVG_RTT_OFFSET              (16)

#define GET_SCAN_RESULTS_TABlE_COUNT_OFFSET              (0)
#define GET_SCAN_RESULTS_SCANRESULT_STATUS_OFFSET        (4)
#define GET_SCAN_RESULTS_ISVALID_TO_SSIDLEN_OFFSET       (8)
#define GET_SCAN_RESULTS_FRAME_TIME_OFFSET               (10)
#define GET_SCAN_RESULTS_SSID_MAC_LENGTH                 (38)

#define M_BSD_RESP_PARAMS_OFFSET(hci_event_hdr)((uint8_t *)(hci_event_hdr) + HCI_EVENT_HEADER_SIZE)

#define SOCKET_STATUS_ACTIVE       0
#define SOCKET_STATUS_INACTIVE     1
/* Init socket_active_status = 'all ones': init all sockets with SOCKET_STATUS_INACTIVE.
   Will be changed by 'set_socket_active_status' upon 'connect' and 'accept' calls */
#define SOCKET_STATUS_INIT_VAL  0xFFFF
#define M_IS_VALID_SD(sd) ((0 <= (sd)) && ((sd) <= 7))
#define M_IS_VALID_STATUS(status) (((status) == SOCKET_STATUS_ACTIVE)||((status) == SOCKET_STATUS_INACTIVE))

#define BSD_RECV_FROM_FROMLEN_OFFSET    (4)
#define BSD_RECV_FROM_FROM_OFFSET       (16)

#endif
