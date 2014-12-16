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
#include "cc3000_netapp.h"

namespace mbed_cc3000 {

cc3000_netapp::cc3000_netapp(cc3000_simple_link &simple_link, cc3000_nvmem &nvmem, cc3000_hci &hci , cc3000_event &event) :
    _simple_link(simple_link), _nvmem(nvmem), _hci(hci), _event(event) {

}

cc3000_netapp::~cc3000_netapp() {

}

int32_t cc3000_netapp::config_mac_adrress(uint8_t * mac) {
    return  _nvmem.set_mac_address(mac);
}

int32_t cc3000_netapp::dhcp(uint32_t *ip, uint32_t *subnet_mask,uint32_t *default_gateway, uint32_t *dns_server) {
    int8_t scRet;
    uint8_t *ptr;
    uint8_t *args;

    scRet = EFAIL;
    ptr = _simple_link.get_transmit_buffer();
    args = (ptr + HEADERS_SIZE_CMD);

    // Fill in temporary command buffer
    ARRAY_TO_STREAM(args,ip,4);
    ARRAY_TO_STREAM(args,subnet_mask,4);
    ARRAY_TO_STREAM(args,default_gateway,4);
    args = UINT32_TO_STREAM(args, 0);
    ARRAY_TO_STREAM(args,dns_server,4);

    // Initiate a HCI command
    _hci.command_send(HCI_NETAPP_DHCP, ptr, NETAPP_DHCP_PARAMS_LEN);

    // Wait for command complete event
    _event.simplelink_wait_event(HCI_NETAPP_DHCP, &scRet);

    return scRet;
}

#ifndef CC3000_TINY_DRIVER
void cc3000_netapp::ipconfig( tNetappIpconfigRetArgs * ipconfig ) {
    uint8_t *ptr;

    ptr = _simple_link.get_transmit_buffer();

    // Initiate a HCI command
    _hci.command_send(HCI_NETAPP_IPCONFIG, ptr, 0);

    // Wait for command complete event
    _event.simplelink_wait_event(HCI_NETAPP_IPCONFIG, ipconfig );
}


int32_t cc3000_netapp::timeout_values(uint32_t *dhcp, uint32_t *arp,uint32_t *keep_alive, uint32_t *inactivity) {
    int8_t scRet;
    uint8_t *ptr;
    uint8_t *args;

    scRet = EFAIL;
    ptr = _simple_link.get_transmit_buffer();
    args = (ptr + HEADERS_SIZE_CMD);

    // Set minimal values of timers
    MIN_TIMER_SET(*dhcp)
    MIN_TIMER_SET(*arp)
    MIN_TIMER_SET(*keep_alive)
    MIN_TIMER_SET(*inactivity)

    // Fill in temporary command buffer
    args = UINT32_TO_STREAM(args, *dhcp);
    args = UINT32_TO_STREAM(args, *arp);
    args = UINT32_TO_STREAM(args, *keep_alive);
    args = UINT32_TO_STREAM(args, *inactivity);

    // Initiate a HCI command
    _hci.command_send(HCI_NETAPP_SET_TIMERS, ptr, NETAPP_SET_TIMER_PARAMS_LEN);

    // Wait for command complete event
    _event.simplelink_wait_event(HCI_NETAPP_SET_TIMERS, &scRet);

    return scRet;
}

int32_t cc3000_netapp::ping_send(uint32_t *ip, uint32_t ping_attempts, uint32_t ping_size, uint32_t ping_timeout) {
    int8_t scRet;
    uint8_t *ptr, *args;

    scRet = EFAIL;
    ptr = _simple_link.get_transmit_buffer();
    args = (ptr + HEADERS_SIZE_CMD);

    // Fill in temporary command buffer
    args = UINT32_TO_STREAM(args, *ip);
    args = UINT32_TO_STREAM(args, ping_attempts);
    args = UINT32_TO_STREAM(args, ping_size);
    args = UINT32_TO_STREAM(args, ping_timeout);

    // Initiate a HCI command
    _hci.command_send(HCI_NETAPP_PING_SEND, ptr, NETAPP_PING_SEND_PARAMS_LEN);

    // Wait for command complete event
    _event.simplelink_wait_event(HCI_NETAPP_PING_SEND, &scRet);

    return scRet;
}

void cc3000_netapp::ping_report() {
    uint8_t *ptr;
    int8_t scRet;
    ptr = _simple_link.get_transmit_buffer();


    scRet = EFAIL;

    // Initiate a HCI command
    _hci.command_send(HCI_NETAPP_PING_REPORT, ptr, 0);

    // Wait for command complete event
    _event.simplelink_wait_event(HCI_NETAPP_PING_REPORT, &scRet);
}

int32_t cc3000_netapp::ping_stop() {
    int8_t scRet;
    uint8_t *ptr;

    scRet = EFAIL;
    ptr = _simple_link.get_transmit_buffer();

    // Initiate a HCI command
    _hci.command_send(HCI_NETAPP_PING_STOP, ptr, 0);

    // Wait for command complete event
    _event.simplelink_wait_event(HCI_NETAPP_PING_STOP, &scRet);

    return scRet;
}

int32_t cc3000_netapp::arp_flush() {
    int8_t scRet;
    uint8_t *ptr;

    scRet = EFAIL;
    ptr = _simple_link.get_transmit_buffer();

    // Initiate a HCI command
    _hci.command_send(HCI_NETAPP_ARP_FLUSH, ptr, 0);

    // Wait for command complete event
    _event.simplelink_wait_event(HCI_NETAPP_ARP_FLUSH, &scRet);

    return scRet;
}
#endif

} // mbed_cc3000
