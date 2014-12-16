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
#ifndef CC3000_NETAPP_H
#define CC3000_NETAPP_H

#define MIN_TIMER_VAL_SECONDS      20
#define MIN_TIMER_SET(t)    if ((0 != t) && (t < MIN_TIMER_VAL_SECONDS)) \
                            { \
                                t = MIN_TIMER_VAL_SECONDS; \
                            }


#define NETAPP_DHCP_PARAMS_LEN                 (20)
#define NETAPP_SET_TIMER_PARAMS_LEN            (20)
#define NETAPP_SET_DEBUG_LEVEL_PARAMS_LEN      (4)
#define NETAPP_PING_SEND_PARAMS_LEN            (16)


typedef struct _netapp_dhcp_ret_args_t
{
    uint8_t aucIP[4];
    uint8_t aucSubnetMask[4];
    uint8_t aucDefaultGateway[4];
    uint8_t aucDHCPServer[4];
    uint8_t aucDNSServer[4];
}tNetappDhcpParams;

typedef struct _netapp_ipconfig_ret_args_t
{
    uint8_t aucIP[4];
    uint8_t aucSubnetMask[4];
    uint8_t aucDefaultGateway[4];
    uint8_t aucDHCPServer[4];
    uint8_t aucDNSServer[4];
    uint8_t uaMacAddr[6];
    uint8_t uaSSID[32];
}tNetappIpconfigRetArgs;


/*Ping send report parameters*/
typedef struct _netapp_pingreport_args
{
    uint32_t packets_sent;
    uint32_t packets_received;
    uint32_t min_round_time;
    uint32_t max_round_time;
    uint32_t avg_round_time;
} netapp_pingreport_args_t;

#endif
