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

namespace mbed_cc3000 {

/* TODO this prefix remove? verify */
static uint8_t cc3000_prefix[] = {'T', 'T', 'T'};
cc3000 *cc3000::_inst;

cc3000::cc3000(PinName cc3000_irq, PinName cc3000_en, PinName cc3000_cs, SPI cc3000_spi)
             : _event(_simple_link, _hci, _spi, *this), _socket(_simple_link, _hci, _event),
               _spi(cc3000_irq, cc3000_en, cc3000_cs, cc3000_spi, _event, _simple_link), _hci(_spi),
               _nvmem(_hci, _event, _simple_link), _netapp(_simple_link, _nvmem, _hci, _event),
               _wlan(_simple_link, _event, _spi, _hci) {
    _simple_link.set_tx_complete_signal(1);
    memset(&_status, 0, sizeof(_status));
    _inst = this;
}

cc3000::~cc3000() {
}

#if (CC3000_ETH_COMPAT == 1)
cc3000::cc3000(PinName cc3000_irq, PinName cc3000_en, PinName cc3000_cs, SPI cc3000_spi, const char *ssid,
               const char *phrase, Security sec, bool smart_config)
             : _event(_simple_link, _hci, _spi, *this), _socket(_simple_link, _hci, _event),
               _spi(cc3000_irq, cc3000_en, cc3000_cs, cc3000_spi, _event, _simple_link), _hci(_spi),
               _nvmem(_hci, _event, _simple_link), _netapp(_simple_link, _nvmem, _hci, _event),
               _wlan(_simple_link, _event, _spi, _hci), _sec(sec), _smart_config(smart_config) {
    _simple_link.set_tx_complete_signal(1);
    memset(&_status, 0, sizeof(_status));
    strcpy((char *)_ssid, ssid);
    strcpy((char *)_phrase, phrase);
    _inst = this;
}

// Ethernet library compatible, functions return strings
// Caches the ipconfig from the usync callback
static char mac_addr[19]= "\0";
static char ip_addr[17] = "\0";
static char gateway[17] = "\0";
static char networkmask[17] = "\0";

void cc3000::init() {
    _wlan.start(0);

    uint32_t subnet[4] = {0};
    uint32_t ip[4] = {0};
    uint32_t getway[4] = {0};
    uint32_t dns[4] = {0};

    _netapp.dhcp(ip, subnet, getway, dns);
    _wlan.stop();
    wait(1);
    _wlan.start(0);

    _status.enabled = 1;
    _wlan.set_event_mask(HCI_EVNT_WLAN_UNSOL_INIT | HCI_EVNT_WLAN_KEEPALIVE);
}

void cc3000::init(const char *ip, const char *mask, const char *gateway) {
    _netapp.dhcp((uint32_t *)ip, (uint32_t *)mask, (uint32_t *)gateway, (uint32_t *)ip); //dns = ip
    _wlan.stop();
    wait(1);
    _wlan.start(0);

    _status.enabled = 1;
    _wlan.set_event_mask(HCI_EVNT_WLAN_UNSOL_INIT | HCI_EVNT_WLAN_KEEPALIVE);
}

int cc3000::connect(unsigned int timeout_ms) {
    Timer t;
    int ret = 0;

    if (_smart_config == false) {
        _wlan.ioctl_set_connection_policy(0, 0, 0);
    } else {
        tUserFS user_info;
        get_user_file_info((uint8_t *)&user_info, sizeof(user_info));
        if (user_info.FTC == 1) {
            _wlan.ioctl_set_connection_policy(0, 1, 1);
        } else {
            DBG_CC("Smart config is not set. Please run the first time configuration.");
            return -1;
        }
    }

    t.start();
    while (is_connected() == false) {
        if (strlen((const char *)_phrase) < 8) {
            if (connect_open(_ssid)) {
                break;
            }
        } else {
#ifndef CC3000_TINY_DRIVER
            if (connect_secure(_ssid,_phrase, _sec)) {
                break;
            }
#else
            return -1; /* secure connection not supported with TINY_DRIVER */
#endif
        }

        if (t.read_ms() > timeout_ms) {
            ret = -1;
            DBG_CC("Connection to AP failed");
            break;
        }
    }

    while (is_dhcp_configured() == false)
    {
        if (t.read_ms() > timeout_ms) {
            ret = -1;
            DBG_CC("Connection to AP failed");
            break;
        }
    }

    return ret;
}

char* cc3000::getMACAddress() {
    return mac_addr;
}

char* cc3000::getIPAddress() {
    return ip_addr;
}

char* cc3000::getGateway() {
    return gateway;
}

char* cc3000::getNetworkMask() {
    return networkmask;
}

int cc3000::disconnect(void){
    if (_wlan.disconnect()) {
        return -1;
    } else {
        return 0;
    }
}

#endif

void cc3000::usync_callback(int32_t event_type, uint8_t *data, uint8_t length) {
    if (event_type == HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE) {
        DBG_CC("Callback : HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE");
        _status.smart_config_complete = 1;
        _status.stop_smart_config = 1;
    }

    if (event_type == HCI_EVNT_WLAN_UNSOL_CONNECT) {
        DBG_CC("Callback : HCI_EVNT_WLAN_UNSOL_CONNECT");
        _status.connected = 1;
        // Connect message is always followed by a DHCP message, connection is not useable until then
        _status.dhcp      = 0;
    }

    if (event_type == HCI_EVNT_WLAN_UNSOL_DISCONNECT) {
        DBG_CC("Callback : HCI_EVNT_WLAN_UNSOL_DISCONNECT");
        _status.connected = 0;
        _status.dhcp      = 0;
        _status.dhcp_configured = 0;
    }

    if (event_type == HCI_EVNT_WLAN_UNSOL_DHCP) {
#if (CC3000_ETH_COMPAT == 1)
        _socket.inet_ntoa_r( htonl(*((uint32_t *)(&data[NETAPP_IPCONFIG_IP_OFFSET]))), ip_addr, 17);
        _socket.inet_ntoa_r( htonl(*((uint32_t *)(&data[NETAPP_IPCONFIG_GW_OFFSET]))), gateway, 17);
        _socket.inet_ntoa_r( htonl(*((uint32_t *)(&data[NETAPP_IPCONFIG_SUBNET_OFFSET]))), networkmask, 17);
        _socket.inet_ntoa_r( htonl(*((uint32_t *)(&data[NETAPP_IPCONFIG_MAC_OFFSET]))), mac_addr, 19);
#endif
        if (*(data + NETAPP_IPCONFIG_MAC_OFFSET) == 0) {
            _status.dhcp = 1;
            DBG_CC("Callback : HCI_EVNT_WLAN_UNSOL_DHCP %i.%i.%i.%i", data[3], data[2], data[1], data[0]);
        } else {
            DBG_CC("Callback : HCI_EVNT_WLAN_UNSOL_DHCP - Disconnecting");
            _status.dhcp = 0;
        }
    }

    if (event_type == HCI_EVENT_CC3000_CAN_SHUT_DOWN) {
        // Note this means the modules is idle, so it could be shutdown..
        //DBG_CC("Callback : HCI_EVENT_CC3000_CAN_SHUT_DOWN");
        _status.ok_to_shut_down = 1;
    }

    if (event_type == HCI_EVNT_WLAN_ASYNC_PING_REPORT) {
        DBG_CC("Callback : HCI_EVNT_WLAN_ASYNC_PING_REPORT");
        memcpy(&_ping_report, data, length);
    }

    if (event_type == HCI_EVNT_BSD_TCP_CLOSE_WAIT) {
        uint8_t socketnum = data[0];
        DBG_CC("Callback : HCI_EVNT_BSD_TCP_CLOSE_WAIT - Socket : %d", socketnum);
        if (socketnum < MAX_SOCKETS) {
            _closed_sockets[socketnum] = true; /* clients socket is closed */
        }
    }
}

void cc3000::start_smart_config(const uint8_t *smart_config_key) {
    _status.smart_config_complete = 0;
    _wlan.ioctl_set_connection_policy(0, 0, 0);

    if (_status.connected == 1) {
        disconnect();
    }

    //Wait until CC3000 is disconected
    while (_status.connected == 1) {
        wait_us(5);
        _event.hci_unsolicited_event_handler();
    }

    // Trigger the Smart Config process
    _wlan.smart_config_set_prefix(cc3000_prefix);
    // Start the Smart Config process with AES disabled
    _wlan.smart_config_start(0);

    DBG_CC("Waiting for smartconfig to be completed");

    // Wait for Smart config finished
    while (_status.smart_config_complete == 0) {
        wait_ms(100);
    }

    DBG_CC("Smartconfig finished");

#ifndef CC3000_UNENCRYPTED_SMART_CONFIG
    // create new entry for AES encryption key
    _nvmem.create_entry(NVMEM_AES128_KEY_FILEID, 16);
    // write AES key to NVMEM
    _security.aes_write_key((uint8_t *)(&smart_config_key[0]));
    // Decrypt configuration information and add profile
    _wlan.smart_config_process();
#endif

    // Configure to connect automatically to the AP retrieved in the
    // Smart config process
    _wlan.ioctl_set_connection_policy(0, 0, 1);

    // reset the CC3000
    _wlan.stop();
    _status.enabled = 0;
    wait(5);
    _wlan.start(0);
    _status.enabled = 1;

    // Mask out all non-required events
    _wlan.set_event_mask(HCI_EVNT_WLAN_KEEPALIVE | HCI_EVNT_WLAN_UNSOL_INIT);
}

bool cc3000::connect_secure(const uint8_t *ssid, const uint8_t *key, int32_t security_mode) {
#ifdef CC3000_TINY_DRIVER
    return false; /* not supported*/
#else
    uint32_t ret;

    //_wlan.disconnect();
    wait_ms(3);
    ret = _wlan.connect(security_mode, ssid, strlen((const char *)ssid), 0, (uint8_t *)key, strlen((const char *)key));
    if (ret == 0) { /* TODO static internal cc3000 state 0 to TRUE */
      ret = true;
    } else {
      ret = false;
    }
    return ret;
#endif
}

bool cc3000::connect_non_blocking(const uint8_t *ssid, const uint8_t *key, int32_t security_mode)
{
    bool ret = false;

    if (key == 0) {
        if (connect_open(ssid)) {
            ret = true;
        }
    } else {
    #ifndef CC3000_TINY_DRIVER
        if (connect_secure(ssid,key,security_mode)) {
            ret = true;
        }
    #else
        /* secure connection not supported with TINY_DRIVER */
    #endif
    }

    return ret;
}

bool cc3000::connect_to_AP(const uint8_t *ssid, const uint8_t *key, int32_t security_mode) {
    Timer t;
    bool ret = true;

    t.start();
    while (is_connected() == false) {
        if (key == 0) {
            if (connect_open(ssid)) {
                break;
            }
        } else {
#ifndef CC3000_TINY_DRIVER
            if (connect_secure(ssid,key,security_mode)) {
                break;
            }
#else
            return false; /* secure connection not supported with TINY_DRIVER */
#endif
        }

        /* timeout 10 seconds */
        if (t.read_ms() > 10000) {
            ret = false;
            DBG_CC("Connection to AP failed");
            break;
        }
    }

    return ret;
}

void cc3000::start(uint8_t patch) {
    _wlan.start(patch);
    _status.enabled = 1;
    _wlan.set_event_mask(HCI_EVNT_WLAN_UNSOL_INIT | HCI_EVNT_WLAN_KEEPALIVE);
}

void cc3000::stop(void) {
    _wlan.stop();
    _status.enabled = 0;
}

void cc3000::restart(uint8_t patch) {
    _wlan.stop();
    _status.enabled = 0;
    wait_ms(500);
    _wlan.start(patch);
    _status.enabled = 1;
}

bool cc3000::connect_open(const uint8_t *ssid) {
    _wlan.disconnect();
    wait_ms(3);
    uint32_t ret;
#ifndef CC3000_TINY_DRIVER
    ret = _wlan.connect(0,ssid, strlen((const char *)ssid), 0, 0, 0);
#else
    ret = _wlan.connect(ssid, strlen((const char *)ssid));
#endif
    if (ret == 0) {
        ret = true;
    } else {
        ret = false;
    }
    return ret;
}

bool cc3000::is_enabled()
{
    return _status.enabled;
}

bool cc3000::is_connected() {
    if (( _status.connected ) && ( _status.dhcp )) {
        return 1;
    } else {
        return 0;
    }
}

bool cc3000::is_dhcp_configured() {
    return _status.dhcp;
}

bool cc3000::is_smart_confing_completed() {
    return _status.smart_config_complete;
}

uint8_t cc3000::get_mac_address(uint8_t address[6]) {
    return _nvmem.get_mac_address(address);
}

uint8_t cc3000::set_mac_address(uint8_t address[6]) {
    return _nvmem.set_mac_address(address);
}

void cc3000::get_user_file_info(uint8_t *info_file, size_t size) {
    _nvmem.read( NVMEM_USER_FILE_1_FILEID, size, 0, info_file);
}

#ifndef CC3000_TINY_DRIVER
bool cc3000::get_ip_config(tNetappIpconfigRetArgs *ip_config) {
    if ((_status.dhcp == false) || (_status.connected == false)) {
        return false;
    }

    _netapp.ipconfig(ip_config);
    return true;
}
#endif

void cc3000::delete_profiles(void) {
    _wlan.ioctl_set_connection_policy(0, 0, 0);
    _wlan.ioctl_del_profile(255);

    tUserFS user_info;
    get_user_file_info((uint8_t *)&user_info, sizeof(user_info));
    user_info.FTC = 0;
    set_user_file_info((uint8_t *)&user_info, sizeof(user_info));
}

void cc3000::set_user_file_info(uint8_t *info_file, size_t size) {
    _nvmem.write( NVMEM_USER_FILE_1_FILEID, size, 0, info_file);
}

uint32_t cc3000::ping(uint32_t ip, uint8_t attempts, uint16_t timeout, uint8_t size) {
#ifndef CC3000_TINY_DRIVER
    uint32_t reversed_ip = (ip >> 24) | ((ip >> 8) & 0xFF00) | ((ip << 8) & 0xFF0000) | (ip << 24);

    _ping_report.packets_received = 0;
    if (_netapp.ping_send(&reversed_ip, attempts, size, timeout) == -1) {
        DBG_CC("Failed to send ping");
        return 0;
    }
    wait_ms(timeout*attempts*2);

    /* known issue of cc3000 - sent number is send + received */
    // TODO : Remove the Sent/recv'd counts until ti fix the firmware issue?
    DBG_CC("Sent: %d",_ping_report.packets_sent);
    DBG_CC("Received: %d",_ping_report.packets_received);
    DBG_CC("Min time: %d",_ping_report.min_round_time);
    DBG_CC("Max time: %d",_ping_report.max_round_time);
    DBG_CC("Avg time: %d",_ping_report.avg_round_time);

    return _ping_report.packets_received;
#else
    return 0;
#endif
}

/* Conversion between uint types and C strings */
uint8_t* UINT32_TO_STREAM_f (uint8_t *p, uint32_t u32)
{
    *(p)++ = (uint8_t)(u32);
    *(p)++ = (uint8_t)((u32) >> 8);
    *(p)++ = (uint8_t)((u32) >> 16);
    *(p)++ = (uint8_t)((u32) >> 24);
    return p;
}


uint8_t* UINT16_TO_STREAM_f (uint8_t *p, uint16_t u16)
{
    *(p)++ = (uint8_t)(u16);
    *(p)++ = (uint8_t)((u16) >> 8);
    return p;
}


uint16_t STREAM_TO_UINT16_f(uint8_t *p, uint16_t offset)
{
    return (uint16_t)((uint16_t)((uint16_t)
           (*(p + offset + 1)) << 8) + (uint16_t)(*(p + offset)));
}


uint32_t STREAM_TO_UINT32_f(uint8_t *p, uint16_t offset)
{
    return (uint32_t)((uint32_t)((uint32_t)
           (*(p + offset + 3)) << 24) + (uint32_t)((uint32_t)
           (*(p + offset + 2)) << 16) + (uint32_t)((uint32_t)
           (*(p + offset + 1)) << 8) + (uint32_t)(*(p + offset)));
}

} // mbed_cc3000 namespace

