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
#include "cc3000_event.h"
#include "cc3000_netapp.h"

namespace mbed_cc3000 {

#if (CC3000_DEBUG_HCI_RX == 1)
const char *HCI_EVENT_STR[] =
{
    "Socket",
    "Bind",
    "Send",
    "Recv",
    "Accept",
    "Listen",
    "Connect",
    "BSD Select",
    "Set Socket Options",
    "Get Socket Options",
    "Close Socket",
    "Unknown",
    "Recv From",
    "Write",
    "Send To",
    "Get Hostname",
    "mDNS Advertise"
};

const char *HCI_NETAPP_STR[] =
{
    "DHCP",
    "Ping Sent",
    "Ping Report",
    "Ping Stop",
    "IP Config",
    "ARP Flush",
    "Unknown",
    "Set Debug level",
    "Set Timers"
};

// from 0-7
const char *HCI_MISC_STR[] =
{
    "BASE - Error?",
    "Connecting",
    "Disconnect",
    "Scan Param",
    "Connect Policy",
    "Add Profile",
    "Del Profile",
    "Get Scan Res",
    "Event Mask",
    "Status Req",
    "Config Start",
    "Config Stop",
    "Config Set Prefix",
    "Config Patch",
};
#endif

cc3000_event::cc3000_event(cc3000_simple_link &simplelink, cc3000_hci &hci, cc3000_spi &spi, cc3000 &cc3000)
    : socket_active_status(SOCKET_STATUS_INIT_VAL), _simple_link(simplelink), _hci(hci), _spi(spi), _cc3000(cc3000) {

}

cc3000_event::~cc3000_event() {

}

/* TODO removed buffer, set it in init */
void cc3000_event::received_handler(uint8_t *buffer) {
    _simple_link.set_data_received_flag(1);
    _simple_link.set_received_data(buffer);

    hci_unsolicited_event_handler();
}

void cc3000_event::hci_unsol_handle_patch_request(uint8_t *event_hdr) {
    uint8_t *params = (uint8_t *)(event_hdr) + HCI_EVENT_HEADER_SIZE;
    uint32_t length = 0;
    uint8_t *patch;

    switch (*params)
    {
        case HCI_EVENT_PATCHES_DRV_REQ:
        {
            tDriverPatches func_pointer = (tDriverPatches)_simple_link.get_func_pointer(DRIVER_PATCHES);
            if (func_pointer)
            {
                patch = func_pointer(&length);
                if (patch)
                {
                    _hci.patch_send(HCI_EVENT_PATCHES_DRV_REQ, _simple_link.get_transmit_buffer(), patch, length);
                    return;
                }
            }

            // Send 0 length Patches response event
            _hci.patch_send(HCI_EVENT_PATCHES_DRV_REQ, _simple_link.get_transmit_buffer(), 0, 0);
            break;
        }
        case HCI_EVENT_PATCHES_FW_REQ:
        {
            tFWPatches func_pointer = (tFWPatches)_simple_link.get_func_pointer(FW_PATCHES);
            if (func_pointer)
            {
                patch = func_pointer(&length);
                // Build and send a patch
                if (patch)
                {
                    _hci.patch_send(HCI_EVENT_PATCHES_FW_REQ, _simple_link.get_transmit_buffer(), patch, length);
                    return;
                }
            }
            // Send 0 length Patches response event
            _hci.patch_send(HCI_EVENT_PATCHES_FW_REQ, _simple_link.get_transmit_buffer(), 0, 0);
            break;
        }
        case HCI_EVENT_PATCHES_BOOTLOAD_REQ:
        {
            tBootLoaderPatches func_pointer = (tBootLoaderPatches)_simple_link.get_func_pointer(BOOTLOADER_PATCHES);
            if (func_pointer)
            {
                patch = func_pointer(&length);
                if (patch)
                {
                    _hci.patch_send(HCI_EVENT_PATCHES_BOOTLOAD_REQ, _simple_link.get_transmit_buffer(), patch, length);
                    return;
                }
            }
            // Send 0 length Patches response event
            _hci.patch_send(HCI_EVENT_PATCHES_BOOTLOAD_REQ, _simple_link.get_transmit_buffer(), 0, 0);
            break;
        }
    }
}

static void hci_event_debug_print(uint16_t hciEventNo)
{
#if (CC3000_DEBUG_HCI_RX == 1)
    if ((hciEventNo > HCI_CMND_SOCKET_BASE) && ( hciEventNo <= HCI_CMND_MDNS_ADVERTISE))
    {
        DBG_HCI("Event Received : 0x%04X - %s", hciEventNo, HCI_EVENT_STR[hciEventNo-HCI_CMND_SOCKET]);
    }
    else if ((hciEventNo > HCI_CMND_NETAPP_BASE) && ( hciEventNo <= HCI_NETAPP_SET_TIMERS))
    {
        DBG_HCI("Event Received : 0x%04X - %s", hciEventNo, HCI_NETAPP_STR[hciEventNo-HCI_NETAPP_DHCP]);
    }
    else if (hciEventNo < HCI_CMND_WLAN_CONFIGURE_PATCH+1)
    {
        DBG_HCI("Event Received : 0x%04X - %s", hciEventNo, HCI_MISC_STR[hciEventNo]);
    }
    else
    {
        DBG_HCI("Event Received : 0x%04X", hciEventNo);
    }
#endif
}

uint8_t *cc3000_event::hci_event_handler(void *ret_param, uint8_t *from, uint8_t *fromlen) {
    uint8_t *received_data, argument_size;
    uint16_t length;
    uint8_t *pucReceivedParams;
    uint16_t received_op_code = 0;
    uint32_t return_value;
    uint8_t * RecvParams;
    uint8_t *RetParams;

    while (1)
    {
        if (_simple_link.get_data_received_flag() != 0)
        {
            received_data = _simple_link.get_received_data();
            if (*received_data == HCI_TYPE_EVNT)
            {
                // Event Received
                STREAM_TO_UINT16((uint8_t *)received_data, HCI_EVENT_OPCODE_OFFSET,received_op_code);
                pucReceivedParams = received_data + HCI_EVENT_HEADER_SIZE;
                RecvParams = pucReceivedParams;
                RetParams = (uint8_t *)ret_param;

                // unsolicited event received - finish handling
                if (hci_unsol_event_handler((uint8_t *)received_data) == 0)
                {
                    STREAM_TO_UINT8(received_data, HCI_DATA_LENGTH_OFFSET, length);

                    hci_event_debug_print( received_op_code );

                    switch(received_op_code)
                    {
                    case HCI_CMND_READ_BUFFER_SIZE:
                        {
                            uint16_t temp = _simple_link.get_number_free_buffers();
                            STREAM_TO_UINT8((uint8_t *)pucReceivedParams, 0, temp);
                            _simple_link.set_number_free_buffers(temp);

                            temp = _simple_link.get_buffer_length();
                            STREAM_TO_UINT16((uint8_t *)pucReceivedParams, 1, temp);
                            _simple_link.set_buffer_length(temp);
                        }
                        break;

                    case HCI_CMND_WLAN_CONFIGURE_PATCH:
                    case HCI_NETAPP_DHCP:
                    case HCI_NETAPP_PING_SEND:
                    case HCI_NETAPP_PING_STOP:
                    case HCI_NETAPP_ARP_FLUSH:
                    case HCI_NETAPP_SET_DEBUG_LEVEL:
                    case HCI_NETAPP_SET_TIMERS:
                    case HCI_EVNT_NVMEM_READ:
                    case HCI_EVNT_NVMEM_CREATE_ENTRY:
                    case HCI_CMND_NVMEM_WRITE_PATCH:
                    case HCI_NETAPP_PING_REPORT:
                    case HCI_EVNT_MDNS_ADVERTISE:

                        STREAM_TO_UINT8(received_data, HCI_EVENT_STATUS_OFFSET, *(uint8_t *)ret_param);
                        break;

                    case HCI_CMND_SETSOCKOPT:
                    case HCI_CMND_WLAN_CONNECT:
                    case HCI_CMND_WLAN_IOCTL_STATUSGET:
                    case HCI_EVNT_WLAN_IOCTL_ADD_PROFILE:
                    case HCI_CMND_WLAN_IOCTL_DEL_PROFILE:
                    case HCI_CMND_WLAN_IOCTL_SET_CONNECTION_POLICY:
                    case HCI_CMND_WLAN_IOCTL_SET_SCANPARAM:
                    case HCI_CMND_WLAN_IOCTL_SIMPLE_CONFIG_START:
                    case HCI_CMND_WLAN_IOCTL_SIMPLE_CONFIG_STOP:
                    case HCI_CMND_WLAN_IOCTL_SIMPLE_CONFIG_SET_PREFIX:
                    case HCI_CMND_EVENT_MASK:
                    case HCI_EVNT_WLAN_DISCONNECT:
                    case HCI_EVNT_SOCKET:
                    case HCI_EVNT_BIND:
                    case HCI_CMND_LISTEN:
                    case HCI_EVNT_CLOSE_SOCKET:
                    case HCI_EVNT_CONNECT:
                    case HCI_EVNT_NVMEM_WRITE:

                        STREAM_TO_UINT32((uint8_t *)pucReceivedParams,0, *(uint32_t *)ret_param);
                        break;

                    case HCI_EVNT_READ_SP_VERSION:

                        STREAM_TO_UINT8(received_data, HCI_EVENT_STATUS_OFFSET, *(uint8_t *)ret_param);
                        ret_param = ((uint8_t *)ret_param) + 1;
                        STREAM_TO_UINT32((uint8_t *)pucReceivedParams, 0, return_value);
                        UINT32_TO_STREAM((uint8_t *)ret_param, return_value);
                        break;

                    case HCI_EVNT_BSD_GETHOSTBYNAME:

                        STREAM_TO_UINT32((uint8_t *)pucReceivedParams,GET_HOST_BY_NAME_RETVAL_OFFSET,*(uint32_t *)ret_param);
                        ret_param = ((uint8_t *)ret_param) + 4;
                        STREAM_TO_UINT32((uint8_t *)pucReceivedParams,GET_HOST_BY_NAME_ADDR_OFFSET,*(uint32_t *)ret_param);
                        break;

                    case HCI_EVNT_ACCEPT:
                        {
                            STREAM_TO_UINT32((uint8_t *)pucReceivedParams,ACCEPT_SD_OFFSET,*(uint32_t *)ret_param);
                            ret_param = ((uint8_t *)ret_param) + 4;
                            STREAM_TO_UINT32((uint8_t *)pucReceivedParams,ACCEPT_RETURN_STATUS_OFFSET,*(uint32_t *)ret_param);
                            ret_param = ((uint8_t *)ret_param) + 4;

                            //This argument returns in network order
                            memcpy((uint8_t *)ret_param, pucReceivedParams + ACCEPT_ADDRESS__OFFSET, sizeof(sockaddr));
                            break;
                        }

                    case HCI_EVNT_RECV:
                    case HCI_EVNT_RECVFROM:
                        {
                            STREAM_TO_UINT32((uint8_t *)pucReceivedParams,SL_RECEIVE_SD_OFFSET ,*(uint32_t *)ret_param);
                            ret_param = ((uint8_t *)ret_param) + 4;
                            STREAM_TO_UINT32((uint8_t *)pucReceivedParams,SL_RECEIVE_NUM_BYTES_OFFSET,*(uint32_t *)ret_param);
                            ret_param = ((uint8_t *)ret_param) + 4;
                            STREAM_TO_UINT32((uint8_t *)pucReceivedParams,SL_RECEIVE__FLAGS__OFFSET,*(uint32_t *)ret_param);

                            if(((tBsdReadReturnParams *)ret_param)->iNumberOfBytes == ERROR_SOCKET_INACTIVE)
                            {
                                set_socket_active_status(((tBsdReadReturnParams *)ret_param)->iSocketDescriptor,SOCKET_STATUS_INACTIVE);
                            }
                            break;
                        }

                    case HCI_EVNT_SEND:
                    case HCI_EVNT_SENDTO:
                        {
                            STREAM_TO_UINT32((uint8_t *)pucReceivedParams,SL_RECEIVE_SD_OFFSET ,*(uint32_t *)ret_param);
                            ret_param = ((uint8_t *)ret_param) + 4;
                            STREAM_TO_UINT32((uint8_t *)pucReceivedParams,SL_RECEIVE_NUM_BYTES_OFFSET,*(uint32_t *)ret_param);
                            ret_param = ((uint8_t *)ret_param) + 4;

                            break;
                        }

                    case HCI_EVNT_SELECT:
                        {
                            STREAM_TO_UINT32((uint8_t *)pucReceivedParams,SELECT_STATUS_OFFSET,*(uint32_t *)ret_param);
                            ret_param = ((uint8_t *)ret_param) + 4;
                            STREAM_TO_UINT32((uint8_t *)pucReceivedParams,SELECT_READFD_OFFSET,*(uint32_t *)ret_param);
                            ret_param = ((uint8_t *)ret_param) + 4;
                            STREAM_TO_UINT32((uint8_t *)pucReceivedParams,SELECT_WRITEFD_OFFSET,*(uint32_t *)ret_param);
                            ret_param = ((uint8_t *)ret_param) + 4;
                            STREAM_TO_UINT32((uint8_t *)pucReceivedParams,SELECT_EXFD_OFFSET,*(uint32_t *)ret_param);
                            break;
                        }

                    case HCI_CMND_GETSOCKOPT:

                        STREAM_TO_UINT8(received_data, HCI_EVENT_STATUS_OFFSET,((tBsdGetSockOptReturnParams *)ret_param)->iStatus);
                        //This argument returns in network order
                        memcpy((uint8_t *)ret_param, pucReceivedParams, 4);
                        break;

                    case HCI_CMND_WLAN_IOCTL_GET_SCAN_RESULTS:

                        STREAM_TO_UINT32((uint8_t *)pucReceivedParams,GET_SCAN_RESULTS_TABlE_COUNT_OFFSET,*(uint32_t *)ret_param);
                        ret_param = ((uint8_t *)ret_param) + 4;
                        STREAM_TO_UINT32((uint8_t *)pucReceivedParams,GET_SCAN_RESULTS_SCANRESULT_STATUS_OFFSET,*(uint32_t *)ret_param);
                        ret_param = ((uint8_t *)ret_param) + 4;
                        STREAM_TO_UINT16((uint8_t *)pucReceivedParams,GET_SCAN_RESULTS_ISVALID_TO_SSIDLEN_OFFSET,*(uint32_t *)ret_param);
                        ret_param = ((uint8_t *)ret_param) + 2;
                        STREAM_TO_UINT16((uint8_t *)pucReceivedParams,GET_SCAN_RESULTS_FRAME_TIME_OFFSET,*(uint32_t *)ret_param);
                        ret_param = ((uint8_t *)ret_param) + 2;
                        memcpy((uint8_t *)ret_param, (uint8_t *)(pucReceivedParams + GET_SCAN_RESULTS_FRAME_TIME_OFFSET + 2), GET_SCAN_RESULTS_SSID_MAC_LENGTH);
                        break;

                    case HCI_CMND_SIMPLE_LINK_START:
                        break;

                    case HCI_NETAPP_IPCONFIG:

                        //Read IP address
                        STREAM_TO_STREAM(RecvParams,RetParams,NETAPP_IPCONFIG_IP_LENGTH);
                        RecvParams += 4;

                        //Read subnet
                        STREAM_TO_STREAM(RecvParams,RetParams,NETAPP_IPCONFIG_IP_LENGTH);
                        RecvParams += 4;

                        //Read default GW
                        STREAM_TO_STREAM(RecvParams,RetParams,NETAPP_IPCONFIG_IP_LENGTH);
                        RecvParams += 4;

                        //Read DHCP server
                        STREAM_TO_STREAM(RecvParams,RetParams,NETAPP_IPCONFIG_IP_LENGTH);
                        RecvParams += 4;

                        //Read DNS server
                        STREAM_TO_STREAM(RecvParams,RetParams,NETAPP_IPCONFIG_IP_LENGTH);
                        RecvParams += 4;

                        //Read Mac address
                        STREAM_TO_STREAM(RecvParams,RetParams,NETAPP_IPCONFIG_MAC_LENGTH);
                        RecvParams += 6;

                        //Read SSID
                        STREAM_TO_STREAM(RecvParams,RetParams,NETAPP_IPCONFIG_SSID_LENGTH);
                        break;

                    default :
                        DBG_HCI("UNKNOWN Event Received : 0x%04X ", received_op_code);
                        break;
                    }

                }
                if (received_op_code == _simple_link.get_op_code())
                {
                    _simple_link.set_op_code(0);
                }
            }
            else
            {
                pucReceivedParams = received_data;
                STREAM_TO_UINT8((uint8_t *)received_data, HCI_PACKET_ARGSIZE_OFFSET, argument_size);

                STREAM_TO_UINT16((uint8_t *)received_data, HCI_PACKET_LENGTH_OFFSET, length);

                // Data received: note that the only case where from and from length
                // are not null is in recv from, so fill the args accordingly
                if (from)
                {
                    STREAM_TO_UINT32((uint8_t *)(received_data + HCI_DATA_HEADER_SIZE), BSD_RECV_FROM_FROMLEN_OFFSET, *(uint32_t *)fromlen);
                    memcpy(from, (received_data + HCI_DATA_HEADER_SIZE + BSD_RECV_FROM_FROM_OFFSET) ,*fromlen);
                }

                memcpy(ret_param, pucReceivedParams + HCI_DATA_HEADER_SIZE + argument_size, length - argument_size);

                _simple_link.set_pending_data(0);
            }

            _simple_link.set_data_received_flag(0);
            _spi.wlan_irq_enable();

            // Since we are going to TX - we need to handle this event after the ResumeSPi since we need interrupts
            if ((*received_data == HCI_TYPE_EVNT) && (received_op_code == HCI_EVNT_PATCHES_REQ))
            {
                hci_unsol_handle_patch_request((uint8_t *)received_data);
            }
            if ((_simple_link.get_op_code() == 0) && (_simple_link.get_pending_data() == 0))
            {
                return NULL;
            }
        }
    }
}

int32_t cc3000_event::hci_unsol_event_handler(uint8_t *event_hdr) {
    uint8_t *data = NULL;
    int32_t event_type;
    uint32_t number_of_released_packets;
    uint32_t number_of_sent_packets;

    STREAM_TO_UINT16(event_hdr, HCI_EVENT_OPCODE_OFFSET,event_type);

    if (event_type & HCI_EVNT_UNSOL_BASE) {
        switch(event_type) {
            case HCI_EVNT_DATA_UNSOL_FREE_BUFF:
            {
                hci_event_unsol_flowcontrol_handler(event_hdr);

                number_of_released_packets = _simple_link.get_released_packets();
                number_of_sent_packets = _simple_link.get_sent_packets();

                if (number_of_released_packets == number_of_sent_packets)
                {
                    if (_simple_link.get_tx_complete_signal())
                    {
                        //tWlanCB func_pointer = (tWlanCB)_simple_link.get_func_pointer(WLAN_CB);
                        _cc3000.usync_callback(HCI_EVENT_CC3000_CAN_SHUT_DOWN, NULL, 0);
                    }
                }
                return 1;
            }
        }
    }

    if (event_type & HCI_EVNT_WLAN_UNSOL_BASE) {
        switch(event_type) {
            case HCI_EVNT_WLAN_KEEPALIVE:
            case HCI_EVNT_WLAN_UNSOL_CONNECT:
            case HCI_EVNT_WLAN_UNSOL_DISCONNECT:
            case HCI_EVNT_WLAN_UNSOL_INIT:
            case HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE:
                 _cc3000.usync_callback(event_type, 0, 0);
                break;
            case HCI_EVNT_WLAN_UNSOL_DHCP:
            {
                uint8_t params[NETAPP_IPCONFIG_MAC_OFFSET + 1]; // extra byte is for the status
                uint8_t *recParams = params;
                data = (uint8_t *)(event_hdr) + HCI_EVENT_HEADER_SIZE;

                //Read IP address
                STREAM_TO_STREAM(data,recParams,NETAPP_IPCONFIG_IP_LENGTH);
                data += 4;
                //Read subnet
                STREAM_TO_STREAM(data,recParams,NETAPP_IPCONFIG_IP_LENGTH);
                data += 4;
                //Read default GW
                STREAM_TO_STREAM(data,recParams,NETAPP_IPCONFIG_IP_LENGTH);
                data += 4;
                //Read DHCP server
                STREAM_TO_STREAM(data,recParams,NETAPP_IPCONFIG_IP_LENGTH);
                data += 4;
                //Read DNS server
                STREAM_TO_STREAM(data,recParams,NETAPP_IPCONFIG_IP_LENGTH);
                // read the status
                STREAM_TO_UINT8(event_hdr, HCI_EVENT_STATUS_OFFSET, *recParams);

                _cc3000.usync_callback(event_type, (uint8_t  *)params, sizeof(params));

                break;
            }
            case HCI_EVNT_WLAN_ASYNC_PING_REPORT:
            {
                netapp_pingreport_args_t params;
                data = (uint8_t *)(event_hdr) + HCI_EVENT_HEADER_SIZE;
                STREAM_TO_UINT32(data, NETAPP_PING_PACKETS_SENT_OFFSET, params.packets_sent);
                STREAM_TO_UINT32(data, NETAPP_PING_PACKETS_RCVD_OFFSET, params.packets_received);
                STREAM_TO_UINT32(data, NETAPP_PING_MIN_RTT_OFFSET, params.min_round_time);
                STREAM_TO_UINT32(data, NETAPP_PING_MAX_RTT_OFFSET, params.max_round_time);
                STREAM_TO_UINT32(data, NETAPP_PING_AVG_RTT_OFFSET, params.avg_round_time);

                _cc3000.usync_callback(event_type, (uint8_t  *)&params, sizeof(params));
                break;
            }
            case HCI_EVNT_BSD_TCP_CLOSE_WAIT:
            {
                _cc3000.usync_callback(event_type, NULL, 0);
                break;
            }

            //'default' case which means "event not supported"
            default:
                return (0);
        }
        return(1);
    }

    if ((event_type == HCI_EVNT_SEND) || (event_type == HCI_EVNT_SENDTO) || (event_type == HCI_EVNT_WRITE)) {
        uint8_t *pArg;
        int32_t status;
        pArg = M_BSD_RESP_PARAMS_OFFSET(event_hdr);
        STREAM_TO_UINT32(pArg, BSD_RSP_PARAMS_STATUS_OFFSET,status);
        if (ERROR_SOCKET_INACTIVE == status) {
            // The only synchronous event that can come from SL device in form of
            // command complete is "Command Complete" on data sent, in case SL device
            // was unable to transmit
            int32_t transmit_error  = _simple_link.get_transmit_error();
            STREAM_TO_UINT8(event_hdr, HCI_EVENT_STATUS_OFFSET, transmit_error);
            _simple_link.set_transmit_error(transmit_error);
            update_socket_active_status(M_BSD_RESP_PARAMS_OFFSET(event_hdr));
            return (1);
        }
        else {
            return (0);
        }
    }
    return(0);
}

int32_t cc3000_event::hci_unsolicited_event_handler(void) {
    uint32_t res = 0;
    uint8_t *received_data;

    if (_simple_link.get_data_received_flag() != 0) {
        received_data = (_simple_link.get_received_data());

        if (*received_data == HCI_TYPE_EVNT) {
            // unsolicited event received - finish handling
            if (hci_unsol_event_handler((uint8_t *)received_data) == 1) {
                // An unsolicited event was received:
                // release the buffer and clean the event received
                _simple_link.set_data_received_flag(0);

                res = 1;
                _spi.wlan_irq_enable();
            }
        }
    }
    return res;
}

void cc3000_event::set_socket_active_status(int32_t sd, int32_t status) {
    if (M_IS_VALID_SD(sd) && M_IS_VALID_STATUS(status)) {
        socket_active_status &= ~(1 << sd);      /* clean socket's mask */
        socket_active_status |= (status << sd); /* set new socket's mask */
    }
}

int32_t cc3000_event::hci_event_unsol_flowcontrol_handler(uint8_t *event) {
    int32_t temp, value;
    uint16_t i;
    uint16_t pusNumberOfHandles=0;
    uint8_t *pReadPayload;

    STREAM_TO_UINT16((uint8_t *)event,HCI_EVENT_HEADER_SIZE,pusNumberOfHandles);
    pReadPayload = ((uint8_t *)event + HCI_EVENT_HEADER_SIZE + sizeof(pusNumberOfHandles));
    temp = 0;

    for(i = 0; i < pusNumberOfHandles; i++) {
        STREAM_TO_UINT16(pReadPayload, FLOW_CONTROL_EVENT_FREE_BUFFS_OFFSET, value);
        temp += value;
        pReadPayload += FLOW_CONTROL_EVENT_SIZE;
    }

    _simple_link.set_number_free_buffers(_simple_link.get_number_free_buffers() + temp);
    _simple_link.set_number_of_released_packets(_simple_link.get_released_packets() + temp);

    return(ESUCCESS);
}

int32_t cc3000_event::get_socket_active_status(int32_t sd) {
    if (M_IS_VALID_SD(sd)) {
        return (socket_active_status & (1 << sd)) ? SOCKET_STATUS_INACTIVE : SOCKET_STATUS_ACTIVE;
    } else {
        return SOCKET_STATUS_INACTIVE;
    }
}

void cc3000_event::update_socket_active_status(uint8_t *resp_params) {
    int32_t status, sd;

    STREAM_TO_UINT32(resp_params, BSD_RSP_PARAMS_SOCKET_OFFSET,sd);
    STREAM_TO_UINT32(resp_params, BSD_RSP_PARAMS_STATUS_OFFSET,status);

    if (ERROR_SOCKET_INACTIVE == status) {
        set_socket_active_status(sd, SOCKET_STATUS_INACTIVE);
    }
}

void cc3000_event::simplelink_wait_event(uint16_t op_code, void *ret_param) {
    // In the blocking implementation the control to caller will be returned only
    // after the end of current transaction
    _simple_link.set_op_code(op_code);
    hci_event_handler(ret_param, 0, 0);
}

void cc3000_event::simplelink_wait_data(uint8_t *pBuf, uint8_t *from, uint8_t *fromlen) {
    // In the blocking implementation the control to caller will be returned only
    // after the end of current transaction, i.e. only after data will be received
    _simple_link.set_pending_data(1);
    hci_event_handler(pBuf, from, fromlen);
}


} // end of cc3000
