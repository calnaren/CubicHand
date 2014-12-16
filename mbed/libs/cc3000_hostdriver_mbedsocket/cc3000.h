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
#ifndef CC3000_H
#define CC3000_H

#include "mbed.h"
#include "cc3000_common.h"
#include "cc3000_spi.h"
#include "cc3000_simplelink.h"
#include "cc3000_netapp.h"
#include "cc3000_nvmem.h"
#include "cc3000_socket.h"

#define MAX_SOCKETS 4
// cc3000 Ethernet Interface - enabled by default
#define CC3000_ETH_COMPAT   1

/** Enable debug messages - set 1  */
// Debug - Socket interface messages
#define CC3000_DEBUG_SOCKET 0
// Debug - HCI TX messages
#define CC3000_DEBUG_HCI_TX 0
// Debug - HCI Rx messages
#define CC3000_DEBUG_HCI_RX 0
// Debug - General Debug
#define CC3000_DEBUG        0
// Add colour to the debug messages, requires a VT100 terminal like putty, comment out to remove
#define VT100_COLOUR        0

#if (CC3000_DEBUG_SOCKET == 1)
    #if (VT100_COLOUR == 1)
        #define DBG_SOCKET(x, ...) std::printf("\x1b[2;32;40m[CC3000 : SOCKET] "x"\x1b[0;37;40m\r\n", ##__VA_ARGS__);
    #else
        #define DBG_SOCKET(x, ...) std::printf("[CC3000 : SOCKET] "x"\r\n", ##__VA_ARGS__);
    #endif
#else
    #define DBG_SOCKET(x, ...)
#endif

#if (CC3000_DEBUG_HCI_TX == 1)
    #if (VT100_COLOUR == 1)
        #define DBG_HCI(x, ...) std::printf("\x1b[2;35;40m[CC3000 : HCI RX] "x"\x1b[0;37;40m\r\n", ##__VA_ARGS__);
    #else
        #define DBG_HCI(x, ...) std::printf("[CC3000 : HCI RX] "x"\r\n", ##__VA_ARGS__);
    #endif
#else
    #define DBG_HCI(x, ...)
#endif

#if (CC3000_DEBUG_HCI_RX == 1)
    #if (VT100_COLOUR == 1)
        #define DBG_HCI_CMD(x, ...) std::printf("\x1b[2;36;40m[CC3000 : HCI TX] "x"\x1b[0;37;40m\r\n", ##__VA_ARGS__);
    #else
        #define DBG_HCI_CMD(x, ...) std::printf("[CC3000 : HCI TX] "x"\r\n", ##__VA_ARGS__);
    #endif
#else
    #define DBG_HCI_CMD(x, ...)
#endif

#if (CC3000_DEBUG == 1)
    #if (VT100_COLOUR == 1)
        #define DBG_CC(x, ...) std::printf("\x1b[2;32;40m[CC3000] "x"\x1b[0;37;40m\r\n", ##__VA_ARGS__);
    #else
        #define DBG_CC(x, ...) std::printf("[CC3000] "x"\r\n", ##__VA_ARGS__);
    #endif
#else
    #define DBG_CC(x, ...)
#endif

namespace mbed_cc3000 {

/** User info structure
 */
typedef struct {
    uint8_t FTC;           // First time config performed
    uint8_t PP_version[2]; // Patch Programmer version
    uint8_t SERV_PACK[2];  // Service Pack Version
    uint8_t DRV_VER[3];    // Driver Version
    uint8_t FW_VER[3];     // Firmware Version
    uint8_t validCIK;      // CIK[] is valid (Client Interface Key)
    uint8_t CIK[40];
} tUserFS;

/** Function pointers which are not yet implemented
 */
enum FunctionNumber {
    FW_PATCHES          = 0,
    DRIVER_PATCHES      = 1,
    BOOTLOADER_PATCHES  = 2,
};

/** AP security
 */
enum Security {
    NONE = 0,
    WEP  = 1,
    WPA  = 2,
    WPA2 = 3
};

/** CC3000 Simple Link class which contains status of cc3000.
 */
class cc3000_simple_link {
public:
    /**
     *  \brief ctor - sets magic number in the buffers (overflow mark).
     *  \param none
     *  \return none
     */
    cc3000_simple_link();
    /**
     *  \brief dtor
     *  \param none
     *  \return none
     */
    ~cc3000_simple_link();
    /**
     *  \brief Returns data received flag.
     *  \return Data received flag.
     */
    uint8_t get_data_received_flag();
    /**
     *  \brief Set data received flag.
     *  \param value The value to be set.
     */
    void set_data_received_flag(uint8_t value);
    /** Returns if tx was completed.
     *  \return
     *    true if tx was completed,
     *    false otherwise.
     */
    bool get_tx_complete_signal();
    /**
     *  \brief Sets flag that tx was completed.
     *  \param value Value to be set
     *  \return none
     */
    void set_tx_complete_signal(bool value);
    /**
     *  \brief Get receive buffer.
     *  \param none
     *  \return Pointer to the receive buffer.
     */
    uint8_t *get_received_buffer();
    /**
     *  \brief Get transmit buffer.
     *  \param none
     *  \return Pointer to the transmit buffer.
     */
    uint8_t *get_transmit_buffer();
    /**
     *  \brief Get number of free buffers.
     *  \param none
     *  \return
     *      Number of free buffers.
     */
    uint16_t get_number_free_buffers();
    /**
     *  \brief Set number of free buffers.
     *  \param value Number of free buffers.
     *  \return none
     */
    void set_number_free_buffers(uint16_t value);
    /**
     *  \brief Retrieve buffer length.
     *  \param none
     *  \return Buffer length
     */
    uint16_t get_buffer_length();
    /**
     *  \brief Set buffer length
     *  \param value The length
     *  \return none
     */
    void set_buffer_length(uint16_t value);
    /**
     *  \brief Retrieve pending data flag.
     *  \param none
     *  \return Pending data flag
     */
    uint16_t get_pending_data();
    /**
     *  \brief Set pending data flag.
     *  \param value Pending data value.
     *  \return none
     */
    void set_pending_data(uint16_t value);
    /**
     *  \brief Retreive op code.
     *  \param none
     *  \return Op code
     */
    uint16_t get_op_code();
    /**
     *  \brief Set op code.
     *  \param code op code.
     *  \return none
     */
    void set_op_code(uint16_t code);
    /**
     *  \brief Get number of released packets.
     *  \param none
     *  \return Number of released packets.
     */
    uint16_t get_released_packets();
    /**
     *  \brief Set number of released packets.
     *  \param value Number of released packets.
     *  \return none
     */
    void set_number_of_released_packets(uint16_t value);
    /**
     *  \brief Get number of sent packats
     *  \param none
     *  \return Number of sent packets.
     */
    uint16_t get_sent_packets();
    /**
     *  \brief Set number of sent packets
     *  \param value Number of sent packets.
     *  \return none
     */
    void set_sent_packets(uint16_t value);
    /**
     *  \brief Retrieve transmit error
     *  \param none
     *  \return Transmit error
     */
    int32_t get_transmit_error();
    /**
     *  \brief Set transmit error.
     *  \param value Error to be set.
     *  \return none
     */
    void set_transmit_error(int32_t value);
    /**
     *  \brief Get buffer size.
     *  \param none
     *  \return Size of buffer.
     */
    uint16_t get_buffer_size();
    /**
     *  \brief Set buffer size.
     *  \param value Buffer size.
     *  \return none
     */
    void set_buffer_size(uint16_t value);
    /**
     *  \brief Not used currently.
     *  \param function Number of desired function.
     *  \return void pointer to the function (need to recast).
     */
    void *get_func_pointer(FunctionNumber function);
    /**
     *  \brief Retreive pointer to the received data.
     *  \param none
     *  \return Pointer to the received data buffer.
     */
    uint8_t *get_received_data();
    /**
     *  \brief Set received data pointer.
     *  \param pointer Pointer to the buffer.
     *  \return none
     */
    void set_received_data(uint8_t *pointer);
private:
    uint8_t  _data_received_flag;
    bool     _tx_complete_signal;
    uint16_t _rx_event_opcode;
    uint16_t _free_buffers;
    uint16_t _buffer_length;
    uint16_t _buffer_size;
    uint16_t _rx_data_pending;
    uint16_t _sent_packets;
    uint16_t _released_packets;
    int32_t  _transmit_data_error;
    uint8_t  *_received_data;
    uint8_t  _rx_buffer[CC3000_RX_BUFFER_SIZE];
    uint8_t  _tx_buffer[CC3000_TX_BUFFER_SIZE];
private:
    /* Not used currently */
    int8_t *(* _fFWPatches)(uint32_t *length);
    int8_t *(* _fDriverPatches)(uint32_t *length);
    int8_t *(* _fBootLoaderPatches)(uint32_t *length);
};

/** Forward declaration classes
 */
class cc3000_hci;
class cc3000_nvmem;
class cc3000_spi;
class cc3000;

/** Event layer
 */
class cc3000_event {
public:
    /**
     *  \brief Ctor
     *  \param simplelink Reference to simple link object.
     *  \param hci        Reference to hci object.
     *  \param spi        Reference to spi object.
     *  \param cc3000     Reference to cc3000 object.
     *  \return none
     */
    cc3000_event(cc3000_simple_link &simplelink, cc3000_hci &hci, cc3000_spi &spi, cc3000 &cc3000);
    /**
     *  \brief Dtor
     *  \param none
     *  \return none
     */
     ~cc3000_event();
    /**
     *  \brief Handle unsolicited event from type patch request.
     *  \param  event_hdr  event header
     *  \return none
     */
    void hci_unsol_handle_patch_request(uint8_t *event_hdr);
    /**
    *  \brief  Parse the incoming event packets and issue corresponding event handler from global array of handlers pointers.
    *  \param  ret_param      incoming data buffer
    *  \param  from           from information (in case of data received)
    *  \param  fromlen        from information length (in case of data received)
    *  \return                none
    */
    uint8_t* hci_event_handler(void *ret_param, uint8_t *from, uint8_t *fromlen);
    /**
    *  \brief  Handle unsolicited events.
    *  \param  event_hdr Event header
    *  \return           1 if event supported and handled
    *  \return           0 if event is not supported
    */
    int32_t hci_unsol_event_handler(uint8_t *event_hdr);
    /**
    *  \brief   Parse the incoming unsolicited event packets and start corresponding event handler.
    *  \param   None
    *  \return  ESUCCESS if successful, EFAIL if an error occurred.
    */
    int32_t hci_unsolicited_event_handler(void);
    /**
    *  \brief  Get the socket status.
    *  \param  Sd Socket IS
    *  \return Current status of the socket.
    */
    int32_t get_socket_active_status(int32_t sd);
    /**
    *  \brief Check if the socket ID and status are valid and set the global socket status accordingly.
    *  \param Sd Sock descr
    *  \param Status status to be set
    *  \return  none
    */
    void set_socket_active_status(int32_t sd, int32_t status);
    /**
    *  \brief Keep track on the number of packets transmitted and update the number of free buffer in the SL device.
    *  \brief Called when unsolicited event = HCI_EVNT_DATA_UNSOL_FREE_BUFF has received.
    *  \param event pointer to the string contains parameters for IPERF.
    *  \return ESUCCESS if successful, EFAIL if an error occurred.
    */
    int32_t hci_event_unsol_flowcontrol_handler(uint8_t *event);
    /**
    *  \brief Update the socket status.
    *  \param resp_params Socket IS
    *  \return Current status of the socket.
    */
    void update_socket_active_status(uint8_t *resp_params);
    /**
     *  \brief  Wait for event, pass it to the hci_event_handler and update the event opcode in a global variable.
     *  \param  op_code   Command operation code
     *  \param  ret_param Command return parameters
     *  \return none
     */
    void simplelink_wait_event(uint16_t op_code, void *ret_param);
    /**
     *  \brief  Wait for data, pass it to the hci_event_handler and set the data available flag.
     *  \param  buffer  Data buffer
     *  \param  from    From information
     *  \param  fromlen From information length
     *  \return none
     */
    void simplelink_wait_data(uint8_t *buffer, uint8_t *from, uint8_t *fromlen);
    /**
     *  \brief Trigger Received event/data processing - called from the SPI library to receive the data
     *  \param buffer pointer to the received data buffer\n
     *                The function triggers Received event/data processing\n
     *  \return none
     */
    void received_handler(uint8_t *buffer);
private:
    uint32_t            socket_active_status;
    cc3000_simple_link  &_simple_link;
    cc3000_hci          &_hci;
    cc3000_spi          &_spi;
    cc3000              &_cc3000;
};

/** Netapp layer
 */
class cc3000_netapp {
public:
    /**
     *  \brief Ctor
     *  \param simple_link Reference to the simple link object.
     *  \param nvmem       Reference to the nvmem object.
     *  \param hci         Reference to the hci object.
     *  \param event       Reference to the event object.
     *  \return none
     */
    cc3000_netapp(cc3000_simple_link &simple_link, cc3000_nvmem &nvmem, cc3000_hci &hci, cc3000_event &event);
    /**
     *  \brief Dtor
     *  \param none
     *  \return none
     */
    ~cc3000_netapp();
    /**
     *  \brief Configure device MAC address and store it in NVMEM.
     *         The value of the MAC address configured through the API will be\n
     *         stored in CC3000 non volatile memory, thus preserved over resets.\n
     *  \param  mac   device mac address, 6 bytes. Saved: yes
     *  \return       return on success 0, otherwise error.
     */
    int32_t config_mac_adrress(uint8_t *mac);
    /**
     *  \brief Configure the network interface, static or dynamic (DHCP).
     *         In order to activate DHCP mode, ip, subnet_mask, default_gateway must be 0.\n
     *         The default mode of CC3000 is DHCP mode. The configuration is saved in non volatile memory\n
     *         and thus preserved over resets.\n
     *  \param  ip                device mac address, 6 bytes. Saved: yes
     *  \param  subnet_mask       device mac address, 6 bytes. Saved: yes
     *  \param  default_gateway   device mac address, 6 bytes. Saved: yes
     *  \param  dns_server        device mac address, 6 bytes. Saved: yes
     *  \return 0 on success, otherwise error.
     *  \note   If the mode is altered, a reset of CC3000 device is required to apply the changes.\n
     *          Also note that an asynchronous event of type 'DHCP_EVENT' is generated only when\n
     *          a connection to the AP was established. This event is generated when an IP address\n
     *          is allocated either by the DHCP server or by static allocation.\n
     */
    int32_t dhcp(uint32_t *ip, uint32_t *subnet_mask,uint32_t *default_gateway, uint32_t *dns_server);
#ifndef CC3000_TINY_DRIVER
    /**
     *  \brief Get the CC3000 Network interface information.
     *        This information is only available after establishing a WLAN connection.\n
     *        Undefined values are returned when this function is called before association.\n
     *  \param  ipconfig  pointer to a tNetappIpconfigRetArgs structure for storing the network interface configuration.\n
     *          tNetappIpconfigRetArgs: aucIP             - ip address,\n
     *                                  aucSubnetMask     - mask
     *                                  aucDefaultGateway - default gateway address\n
     *                                  aucDHCPServer     - dhcp server address\n
     *                                  aucDNSServer      - dns server address\n
     *                                  uaMacAddr         - mac address\n
     *                                  uaSSID            - connected AP ssid\n
     *  \return  none
     *  \note    This function is useful for figuring out the IP Configuration of\n
     *           the device when DHCP is used and for figuring out the SSID of\n
     *           the Wireless network the device is associated with.\n
     */
    void ipconfig(tNetappIpconfigRetArgs *ipconfig);
    /**
     *  \brief Set new timeout values for DHCP lease timeout, ARP  refresh timeout, keepalive event timeout and socket inactivity timeout
     *  \param  dhcp       DHCP lease time request, also impact\n
     *                     the DHCP renew timeout.\n
     *                     Range:               [0-0xffffffff] seconds,\n
     *                                          0 or 0xffffffff = infinite lease timeout.\n
     *                     Resolution:          10 seconds.\n
     *                     Influence:           only after reconnecting to the AP. \n
     *                     Minimal bound value: MIN_TIMER_VAL_SECONDS - 20 seconds.\n
     *                     The parameter is saved into the CC3000 NVMEM.\n
     *                     The default value on CC3000 is 14400 seconds.\n
     *
     *  \param  arp        ARP refresh timeout, if ARP entry is not updated by\n
     *                     incoming packet, the ARP entry will be  deleted by\n
     *                     the end of the timeout. \n
     *                     Range:               [0-0xffffffff] seconds, 0 = infinite ARP timeout\n
     *                     Resolution:          10 seconds.\n
     *                     Influence:           at runtime.\n
     *                     Minimal bound value: MIN_TIMER_VAL_SECONDS - 20 seconds\n
     *                     The parameter is saved into the CC3000 NVMEM.\n
     *                     The default value on CC3000 is 3600 seconds.\n
     *
     *  \param  keep_alive     Keepalive event sent by the end of keepalive timeout\n
     *                         Range:               [0-0xffffffff] seconds, 0 == infinite timeout\n
     *                         Resolution:          10 seconds.\n
     *                         Influence:           at runtime.\n
     *                         Minimal bound value: MIN_TIMER_VAL_SECONDS - 20 sec\n
     *                         The parameter is saved into the CC3000 NVMEM. \n
     *                         The default value on CC3000 is 10 seconds.\n
     *
     *  \param  inactivity      Socket inactivity timeout, socket timeout is\n
     *                          refreshed by incoming or outgoing packet, by the\n
     *                          end of the socket timeout the socket will be closed\n
     *                          Range:               [0-0xffffffff] sec, 0 == infinite timeout.\n
     *                          Resolution:          10 seconds.\n
     *                          Influence:           at runtime.\n
     *                          Minimal bound value: MIN_TIMER_VAL_SECONDS - 20 sec\n
     *                          The parameter is saved into the CC3000 NVMEM.\n
     *                          The default value on CC3000 is 60 seconds.\n
     *
     *  \return 0 on success,otherwise error.
     *
     *  \note   A parameter set to a non zero value less than 20s automatically changes to 20s.
     */
    int32_t timeout_values(uint32_t *dhcp, uint32_t *arp,uint32_t *keep_alive, uint32_t *inactivity);
    /**
     *  \brief send ICMP ECHO_REQUEST to network hosts
     *  \param  ip              destination IP address
     *  \param  ping_attempts   number of echo requests to send
     *  \param  ping_size       send buffer size which may be up to 1400 bytes
     *  \param  ping_timeout    Time to wait for a response,in milliseconds.
     *  \return 0 on success, otherwise error.
     *
     *  \note     A succesful operation will generate an asynchronous ping report event.\n
     *            The report structure is defined by structure netapp_pingreport_args_t.\n
     *  \warning  Calling this function while a Ping Request is in progress will kill the ping request in progress.
     */
    int32_t ping_send(uint32_t *ip, uint32_t ping_attempts, uint32_t ping_size, uint32_t ping_timeout);
    /**
     *  \brief Ping status request.
     *         This API triggers the CC3000 to send asynchronous events: HCI_EVNT_WLAN_ASYNC_PING_REPORT.\n
     *         This event will create the report structure in netapp_pingreport_args_t.\n
     *         This structure is filled with ping results until the API is triggered.\n
     *         netapp_pingreport_args_t: packets_sent     - echo sent\n
     *                                   packets_received - echo reply\n
     *                                   min_round_time   - minimum round time\n
     *                                   max_round_time   - max round time\n
     *                                   avg_round_time   - average round time\n
     *
     *  \param   none
     *  \return  none
     *  \note    When a ping operation is not active, the returned structure fields are 0.
     */
    void ping_report();
    /**
     *  \brief Stop any ping request.
     *  \param none
     *  \return 0 on success
     *         -1 on error
     */
    int32_t ping_stop();
    /**
     *  \brief Flush ARP table
     *  \param none
     *  \return none
     */
    int32_t arp_flush();
#endif
private:
    cc3000_simple_link  &_simple_link;
    cc3000_nvmem        &_nvmem;
    cc3000_hci          &_hci;
    cc3000_event        &_event;
};

#ifndef CC3000_UNENCRYPTED_SMART_CONFIG
/** Security class used only if encrypted smart config is set
 */
class cc3000_security {
public:
    /**
     *  \brief Expand a 16 bytes key for AES128 implementation.
     *  \param expanded_key expanded AES128 key
     *  \param key AES128 key - 16 bytes
     *  \return none
     */
    void expandKey(uint8_t *expanded_key, uint8_t *key);
    /**
     *  \brief multiply by 2 in the galois field.
     *  \param value Argument to multiply
     *  \return multiplied argument
     */
    uint8_t galois_mul2(uint8_t value);
    /**
     *  \brief internal implementation of AES128 encryption.
     *      straight forward aes encryption implementation\n
     *      first the group of operations
     *      - addRoundKey
     *      - subbytes
     *      - shiftrows
     *      - mixcolums\n
     *
     *      is executed 9 times, after this addroundkey to finish the 9th\n
     *      round, after that the 10th round without mixcolums\n
     *      no further subfunctions to save cycles for function calls\n
     *      no structuring with "for (....)" to save cycles.\n
     *  \param[in]     expanded_key expanded AES128 key
     *  \param[in/out] state 16 bytes of plain text and cipher text
     *  \return  none
     */
    void aes_encr(uint8_t *state, uint8_t *expanded_key);
    /**
     *  \brief internal implementation of AES128 decryption.
     *      straightforward aes decryption implementation\n
     *      the order of substeps is the exact reverse of decryption\n
     *      inverse functions:
     *      - addRoundKey is its own inverse
     *      - rsbox is inverse of sbox
     *      - rightshift instead of leftshift
     *      - invMixColumns = barreto + mixColumns\n
     *
     *      no further subfunctions to save cycles for function calls\n
     *      no structuring with "for (....)" to save cycles\n
     *  \param[in]     expanded_key expanded AES128 key
     *  \param[in\out] state 16 bytes of cipher text and plain text
     *  \return  none
     */
    void aes_decr(uint8_t *state, uint8_t *expanded_key);
    /**
     *  \brief AES128 encryption.
     *      Given AES128 key and 16 bytes plain text, cipher text of 16 bytes is computed.\n
     *      The AES implementation is in mode ECB (Electronic Code Book).\n
     *  \param[in]  key   AES128 key of size 16 bytes
     *  \param[in\out] state   16 bytes of plain text and cipher text
     *  \return  none
     */
    void aes_encrypt(uint8_t *state, uint8_t *key);
    /**
     *  \brief AES128 decryption.
     *      Given AES128 key and  16 bytes cipher text, plain text of 16 bytes is computed.\n
     *      The AES implementation is in mode ECB (Electronic Code Book).\n
     *  \param[in]  key   AES128 key of size 16 bytes
     *  \param[in\out] state   16 bytes of cipher text and plain text
     *  \return  none
     */
    void aes_decrypt(uint8_t *state, uint8_t *key);
    /**
     *  \brief Read the AES128 key from fileID #12 in EEPROM.
     *  \param[out]  key   AES128 key of size 16 bytes
     *  \return  0 on success, error otherwise.
     */
    int32_t aes_read_key(uint8_t *key);
    /**
     *  \brief Write the AES128 key to fileID #12 in EEPROM.
     *  \param[out]  key   AES128 key of size 16 bytes
     *  \return  on success 0, error otherwise.
     */
    int32_t aes_write_key(uint8_t *key);
private:
    uint8_t _expanded_key[176];
};
#endif

/** Socket layer
 */
class cc3000_socket {
public:
    /**
     *  \brief Ctor
     *  \param simplelink Reference to simple link object.
     *  \param hci        Reference to hci object.
     *  \param event      Reference to event object.
     *  \return none
     */
    cc3000_socket(cc3000_simple_link &simplelink, cc3000_hci &hci, cc3000_event &event);
    /**
     *  \brief Dtor
     *  \param
     *  \return none
     */
    ~cc3000_socket();
    /**
     *  \brief create an endpoint for communication.
     *      The socket function creates a socket that is bound to a specific transport service provider.\n
     *      This function is called by the application layer to obtain a socket handle.\n
     *
     *  \param   domain    selects the protocol family which will be used for\n
     *                     communication. On this version only AF_INET is supported\n
     *  \param   type      specifies the communication semantics. On this version\n
     *                     only SOCK_STREAM, SOCK_DGRAM, SOCK_RAW are supported\n
     *  \param   protocol  specifies a particular protocol to be used with the\n
     *                     socket IPPROTO_TCP, IPPROTO_UDP or IPPROTO_RAW are supported.\n
     *  \return  On success, socket handle that is used for consequent socket operations\n
     *           On error, -1 is returned.\n
     */
    int32_t socket(int32_t domain, int32_t type, int32_t protocol);
    /**
     *  \brief accept a connection on a socket.
     *      This function is used with connection-based socket types\n
     *      (SOCK_STREAM). It extracts the first connection request on the\n
     *      queue of pending connections, creates a new connected socket, and\n
     *      returns a new file descriptor referring to that socket.\n
     *      The newly created socket is not in the listening state.\n
     *      The original socket sd is unaffected by this call.\n
     *      The argument sd is a socket that has been created with socket(),\n
     *      bound to a local address with bind(), and is  listening for \n
     *      connections after a listen(). The argument addr is a pointer \n
     *      to a sockaddr structure. This structure is filled in with the \n
     *      address of the peer socket, as known to the communications layer.\n
     *      The exact format of the address returned addr is determined by the \n
     *      socket's address family. The addrlen argument is a value-result\n
     *      argument: it should initially contain the size of the structure\n
     *      pointed to by addr, on return it will contain the actual\n
     *      length (in bytes) of the address returned.\n
     *
     *  \param[in]   sd      socket descriptor (handle)\n
     *  \param[out]  addr    the argument addr is a pointer to a sockaddr structure\n
     *                       This structure is filled in with the address of the \n
     *                       peer socket, as known to the communications layer.  \n
     *                       determined. The exact format of the address returned \n
     *                       addr is by the socket's address sockaddr. \n
     *                       On this version only AF_INET is supported.\n
     *                       This argument returns in network order.\n
     *  \param[out] addrlen  the addrlen argument is a value-result argument: \n
     *                       it should initially contain the size of the structure\n
     *                       pointed to by addr.\n
     *  \return  For socket in blocking mode:\n
     *            - On success, socket handle. on failure negative\n
     *           For socket in non-blocking mode:\n
     *            - On connection establishment, socket handle\n
     *            - On connection pending, SOC_IN_PROGRESS (-2)\n
     *            - On failure, SOC_ERROR    (-1)\n
     *  \sa     socket ; bind ; listen
     */
    int32_t accept(int32_t sd, sockaddr *addr, socklen_t *addrlen);
    /**
     *  \brief assign a name to a socket.
     *      This function gives the socket the local address addr.\n
     *      addr is addrlen bytes long. Traditionally, this is called when a \n
     *      socket is created with socket, it exists in a name space (address \n
     *      family) but has no name assigned.\n
     *      It is necessary to assign a local address before a SOCK_STREAM\n
     *      socket may receive connections.\n
     *
     *  \param[in]   sd      socket descriptor (handle)
     *  \param[out]  addr    specifies the destination address. On this version\n
     *                       only AF_INET is supported.\n
     *  \param[out] addrlen  contains the size of the structure pointed to by addr.\n
     *  \return      On success, zero is returned.\n
     *               On error, -1 is returned.\n
     *  \sa          socket ; accept ; listen
     */
    int32_t bind(int32_t sd, const sockaddr *addr, int32_t addrlen);
    /**
     *  \brief HostFlowControlConsumeBuff.
     *      if SEND_NON_BLOCKING is not defined - block until a free buffer is available,\n
     *      otherwise return the status of the available buffers.\n
     *
     *  \param  sd  socket descriptor
     *  \return  0 in case there are buffers available, \n
     *          -1 in case of bad socket\n
     *          -2 if there are no free buffers present (only when SEND_NON_BLOCKING is enabled)\n
     */
    int32_t HostFlowControlConsumeBuff(int32_t sd);
    /**
     *  \brief The socket function closes a created socket.
     *  \param   sd    socket handle.
     *  \return  On success, zero is returned. On error, -1 is returned.
     */
    int32_t closesocket(int32_t sd);
    /**
     *  \brief listen for connections on a socket.
     *      The willingness to accept incoming connections and a queue\n
     *      limit for incoming connections are specified with listen(),\n
     *      and then the connections are accepted with accept.\n
     *      The listen() call applies only to sockets of type SOCK_STREAM\n
     *      The backlog parameter defines the maximum length the queue of\n
     *      pending connections may grow to. \n
     *
     *  \param[in]  sd       socket descriptor (handle)
     *  \param[in]  backlog  specifies the listen queue depth. On this version\n
     *                       backlog is not supported.\n
     *  \return     On success, zero is returned.\n
     *              On error, -1 is returned.\n
     *  \sa         socket ; accept ; bind
     *  \note       On this version, backlog is not supported
     */
    int32_t listen(int32_t sd, int32_t backlog);
    /**
     *  \brief initiate a connection on a socket.
     *      Function connects the socket referred to by the socket descriptor\n
     *      sd, to the address specified by addr. The addrlen argument \n
     *      specifies the size of addr. The format of the address in addr is \n
     *      determined by the address space of the socket. If it is of type \n
     *      SOCK_DGRAM, this call specifies the peer with which the socket is \n
     *      to be associated; this address is that to which datagrams are to be\n
     *      sent, and the only address from which datagrams are to be received. \n
     *      If the socket is of type SOCK_STREAM, this call attempts to make a \n
     *      connection to another socket. The other socket is specified  by \n
     *      address, which is an address in the communications space of the\n
     *      socket. Note that the function implements only blocking behavior \n
     *      thus the caller will be waiting either for the connection \n
     *      establishment or for the connection establishment failure.\n
     *
     *  \param[in]   sd       socket descriptor (handle)
     *  \param[in]   addr     specifies the destination addr. On this version\n
     *                        only AF_INET is supported.\n
     *  \param[out]  addrlen  contains the size of the structure pointed to by addr
     *  \return      On success, zero is returned.\n
                   On error, -1 is returned\n
     *  \sa socket
     */
    int32_t connect(int32_t sd, const sockaddr *addr, int32_t addrlen);
    /**
     *  \brief Monitor socket activity.
     *      Select allow a program to monitor multiple file descriptors,\n
     *      waiting until one or more of the file descriptors become \n
     *      "ready" for some class of I/O operation \n
     *
     *  \param[in]    nfds       the highest-numbered file descriptor in any of the\n
     *                           three sets, plus 1.  \n
     *  \param[out]   readsds    socket descriptors list for read monitoring\n
     *  \param[out]   writesds   socket descriptors list for write monitoring\n
     *  \param[out]   exceptsds  socket descriptors list for exception monitoring\n
     *  \param[in]    timeout    is an upper bound on the amount of time elapsed\n
     *                           before select() returns. Null means infinity \n
     *                           timeout. The minimum timeout is 5 milliseconds,\n
     *                          less than 5 milliseconds will be set\n
     *                           automatically to 5 milliseconds.\n
     *  \return    On success, select() returns the number of file descriptors\n
     *             contained in the three returned descriptor sets (that is, the\n
     *             total number of bits that are set in readfds, writefds,\n
     *             exceptfds) which may be zero if the timeout expires before\n
     *             anything interesting  happens.\n
     *             On error, -1 is returned.\n
     *                   *readsds - return the sockets on which Read request will\n
     *                              return without delay with valid data.\n
     *                   *writesds - return the sockets on which Write request \n
     *                                 will return without delay.\n
     *                   *exceptsds - return the sockets which closed recently.\n
     *  \Note   If the timeout value set to less than 5ms it will automatically\n
     *          change to 5ms to prevent overload of the system\n
     *  \sa socket
     */
    int32_t select(int32_t nfds, fd_set *readsds, fd_set *writesds, fd_set *exceptsds, struct timeval *timeout);
    /**
     *  \brief get socket options.
     *      This function manipulate the options associated with a socket.\n
     *      Options may exist at multiple protocol levels; they are always\n
     *      present at the uppermost socket level.\n
     *      When manipulating socket options the level at which the option \n
     *      resides and the name of the option must be specified.  \n
     *      To manipulate options at the socket level, level is specified as \n
     *      SOL_SOCKET. To manipulate options at any other level the protocol \n
     *      number of the appropriate protocol controlling the option is \n
     *      supplied. For example, to indicate that an option is to be \n
     *      interpreted by the TCP protocol, level should be set to the \n
     *      protocol number of TCP; \n
     *      The parameters optval and optlen are used to access optval -\n
     *      use for setsockopt(). For getsockopt() they identify a buffer\n
     *      in which the value for the requested option(s) are to \n
     *      be returned. For getsockopt(), optlen is a value-result \n
     *      parameter, initially containing the size of the buffer \n
     *      pointed to by option_value, and modified on return to \n
     *      indicate the actual size of the value returned. If no option \n
     *      value is to be supplied or returned, option_value may be NULL.\n
     *
     *  \param[in]   sd          socket handle
     *  \param[in]   level       defines the protocol level for this option
     *  \param[in]   optname     defines the option name to Interrogate
     *  \param[out]  optval      specifies a value for the option
     *  \param[out]  optlen      specifies the length of the option value
     *  \return      On success, zero is returned. On error, -1 is returned
     *
     *  \Note   On this version the following two socket options are enabled:\n
     *          The only protocol level supported in this version is SOL_SOCKET (level).\n
     *               1. SOCKOPT_RECV_TIMEOUT (optname)\n
     *                  SOCKOPT_RECV_TIMEOUT configures recv and recvfrom timeout in milliseconds.\n
     *                  In that case optval should be pointer to unsigned long.\n
     *               2. SOCKOPT_NONBLOCK (optname). sets the socket non-blocking mode on or off.\n
     *                  In that case optval should be SOCK_ON or SOCK_OFF (optval).\n
     *  \sa setsockopt
     */
    int32_t getsockopt (int32_t sd, int32_t level, int32_t optname, void *optval, socklen_t *optlen);
    /**
     *  \brief Read data from socket (simple_link_recv).
     *      Return the length of the message on successful completion.\n
     *      If a message is too long to fit in the supplied buffer, excess bytes may\n
     *      be discarded depending on the type of socket the message is received from.\n
     *
     *  \param sd       socket handle
     *  \param buf      read buffer
     *  \param len      buffer length
     *  \param flags    indicates blocking or non-blocking operation
     *  \param from     pointer to an address structure indicating source address
     *  \param fromlen  source address structure size
     *  \return         Return the number of bytes received, or -1 if an error occurred
     */
    int32_t simple_link_recv(int32_t sd, void *buf, int32_t len, int32_t flags, sockaddr *from, socklen_t *fromlen, int32_t opcode);
    /**
     *  \brief Transmit a message to another socket (simple_link_send).
     *  \param sd       socket handle
     *  \param buf      write buffer
     *  \param len      buffer length
     *  \param flags    On this version, this parameter is not supported
     *  \param to       pointer to an address structure indicating destination address
     *  \param tolen    destination address structure size
     *  \return         Return the number of bytes transmitted, or -1 if an error\n
     *                  occurred, or -2 in case there are no free buffers available\n
     *                  (only when SEND_NON_BLOCKING is enabled)\n
     */
    int32_t simple_link_send(int32_t sd, const void *buf, int32_t len, int32_t flags, const sockaddr *to, int32_t tolen, int32_t opcode);
    /**
     *  \brief Receive a message from a connection-mode socket.
     *  \param[in]  sd     socket handle
     *  \param[out] buf    Points to the buffer where the message should be stored
     *  \param[in]  len    Specifies the length in bytes of the buffer pointed to \n
     *                     by the buffer argument.\n
     *  \param[in] flags   Specifies the type of message reception. \n
     *                     On this version, this parameter is not supported.\n
     *  \return         Return the number of bytes received, or -1 if an error occurred
     *  \sa recvfrom
     *  \Note On this version, only blocking mode is supported.
     */
    int32_t recv(int32_t sd, void *buf, int32_t len, int32_t flags);
    /**
     *  \brief read data from socket (recvfrom).
     *      Receives a message from a connection-mode or connectionless-mode socket.\n
     *      Note that raw sockets are not supported.\n
     *
     *  \param[in]  sd       socket handle
     *  \param[out] buf      Points to the buffer where the message should be stored
     *  \param[in]  len      Specifies the length in bytes of the buffer pointed to \n
     *                       by the buffer argument.\n
     *  \param[in] flags     Specifies the type of message reception.\n
     *                       On this version, this parameter is not supported.\n
     *  \param[in] from      pointer to an address structure indicating the source\n
     *                       address: sockaddr. On this version only AF_INET is\n
     *                       supported.\n
     *  \param[in] fromlen   source address structure size
     *  \return              Return the number of bytes received, or -1 if an error occurred
     *  \sa recv
     *  \Note On this version, only blocking mode is supported.
     */
    int32_t recvfrom(int32_t sd, void *buf, int32_t len, int32_t flags, sockaddr *from, socklen_t *fromlen);
    /**
     *  \brief Transmit a message to another socket (send).
     *  \param sd       socket handle
     *  \param buf      Points to a buffer containing the message to be sent
     *  \param len      message size in bytes
     *  \param flags    On this version, this parameter is not supported
     *  \return         Return the number of bytes transmitted, or -1 if an\n
     *                  error occurred\n
     *  \Note           On this version, only blocking mode is supported.
     *  \sa             sendto
     */
    int32_t send(int32_t sd, const void *buf, int32_t len, int32_t flags);
    /**
     *  \brief Transmit a message to another socket (sendto).
     *  \param sd       socket handle
     *  \param buf      Points to a buffer containing the message to be sent
     *  \param len      message size in bytes
     *  \param flags    On this version, this parameter is not supported
     *  \param to       pointer to an address structure indicating the destination\n
     *                  address: sockaddr. On this version only AF_INET is\n
     *                  supported.\n
     *  \param tolen    destination address structure size
     *  \return         Return the number of bytes transmitted, or -1 if an error occurred
     *  \Note           On this version, only blocking mode is supported.
     *  \sa             send
     */
    int32_t sendto(int32_t sd, const void *buf, int32_t len, int32_t flags, const sockaddr *to, socklen_t tolen);
    /**
     *  \brief Set CC3000 in mDNS advertiser mode in order to advertise itself.
     *  \param[in] mdns_enabled                 flag to enable/disable the mDNS feature
     *  \param[in] device_service_name          Service name as part of the published\n
     *                                          canonical domain name\n
     *  \param[in] device_service_name_length   Length of the service name
     *  \return   On success, zero is returned,\n
     *            return SOC_ERROR if socket was not opened successfully, or if an error occurred.\n
     */
    int32_t mdns_advertiser(uint16_t mdns_enabled, uint8_t * device_service_name, uint16_t device_service_name_length);
    /**
     *  \brief
     *  \param[in] s_addr in host format ( little endian )
     *  \param[in] *buf     buffer to write too
     *  \param[in] buflen   length of supplied buffer
     *  \return    pointer to buf \n
     */
    char * inet_ntoa_r(uint32_t s_addr, char *buf, int buflen);
#ifndef CC3000_TINY_DRIVER
    /**
     *  \brief Get host IP by name.\n
     *      Obtain the IP Address of machine on network\n
     *
     *  \param[in]   hostname     host name
     *  \param[in]   name_length  name length
     *  \param[out]  out_ip_addr  This parameter is filled in with host IP address.\n
     *                            In case that host name is not resolved, \n
     *                            out_ip_addr is zero.\n
     *  \return      On success, positive is returned.\n
     *               On error, negative is returned by its name.\n
     *  \note  On this version, only blocking mode is supported. Also note that\n
     *         The function requires DNS server to be configured prior to its usage.\n
     */
    int32_t gethostbyname(uint8_t *hostname, uint16_t name_length, uint32_t *out_ip_addr);
    /**
     *  \brief set socket options.
     *      This function manipulate the options associated with a socket.\n
     *      Options may exist at multiple protocol levels; they are always\n
     *      present at the uppermost socket level.\n
     *      When manipulating socket options the level at which the option \n
     *      resides and the name of the option must be specified.\n
     *      To manipulate options at the socket level, level is specified as\n
     *      SOL_SOCKET. To manipulate options at any other level the protocol \n
     *      number of the appropriate protocol controlling the option is \n
     *      supplied. For example, to indicate that an option is to be \n
     *      interpreted by the TCP protocol, level should be set to the \n
     *      protocol number of TCP; \n
     *      The parameters optval and optlen are used to access optval - \n
     *      use for setsockopt(). For getsockopt() they identify a buffer\n
     *      in which the value for the requested option(s) are to \n
     *      be returned. For getsockopt(), optlen is a value-result \n
     *      parameter, initially containing the size of the buffer \n
     *      pointed to by option_value, and modified on return to \n
     *      indicate the actual size of the value returned. If no option \n
     *      value is to be supplied or returned, option_value may be NULL.\n
     *
     *  \param[in]   sd          socket handle
     *  \param[in]   level       defines the protocol level for this option
     *  \param[in]   optname     defines the option name to Interrogate
     *  \param[in]   optval      specifies a value for the option
     *  \param[in]   optlen      specifies the length of the option value
     *  \return      On success, zero is returned.\n
     *               On error, -1 is returned\n
     *
     *  \Note   On this version the following two socket options are enabled:\n
     *          The only protocol level supported in this version is SOL_SOCKET (level).\n
     *               1. SOCKOPT_RECV_TIMEOUT (optname)\n
     *                  SOCKOPT_RECV_TIMEOUT configures recv and recvfrom timeout in milliseconds.\n
     *                  In that case optval should be pointer to unsigned long.\n
     *               2. SOCKOPT_NONBLOCK (optname). sets the socket non-blocking mode on or off.\n
     *                  In that case optval should be SOCK_ON or SOCK_OFF (optval).\n
     *  \sa getsockopt
     */
    int32_t setsockopt(int32_t sd, int32_t level, int32_t optname, const void *optval, socklen_t optlen);
#endif
private:
    cc3000_simple_link  &_simple_link;
    cc3000_hci          &_hci;
    cc3000_event        &_event;
};

/** SPI communication layer
 */
class cc3000_spi {
public:
    /**
     *  \brief Ctor
     *  \param irq         IRQ pin
     *  \param cc3000_en   Enable pin
     *  \param cc3000_cs   Chip select pin
     *  \param cc3000_spi  SPI object
     *  \param irq_port    Port for IRQ pin (needed for enable/disable interrupts)
     *  \param event       Reference to the event object.
     *  \param simple_link Reference to the simple link object.
     *  \return none
     */
     cc3000_spi(PinName cc3000_irq, PinName cc3000_en, PinName cc3000_cs, SPI cc3000_spi, cc3000_event &event, cc3000_simple_link &simple_link);
    /**
     *  \brief Dtor
     *  \param none
     *  \return none
     */
     ~cc3000_spi();
     /**
      *  \brief Close SPI - disables IRQ and set received buffer to 0
      *  \param none
      *  \return none
     */
     void close();
    /**
     *  \brief Open the SPI interface
     *  \param  none
     *  \return none
     */
     void open();
    /**
     *  \brief First SPI write after powerup (delay needed between SPI header and body)
     *  \param  buffer pointer to write buffer
     *  \param  length buffer length
     *  \return 0
     */
     uint32_t first_write(uint8_t *buffer, uint16_t length);
    /**
     *  \brief SPI Write function
     *  \param  buffer pointer to write buffer
     *  \param  length buffer length
     *  \return 0
     */
     uint32_t write(uint8_t *buffer, uint16_t length);
    /**
     *  \brief Low level SPI write
     *  \param  data pointer to data buffer
     *  \param  size number of bytes
     *  \return none
     */
     void write_synchronous(uint8_t *data, uint16_t size);
    /**
     *  \brief Low level SPI read
     *  \param  data pointer to data buffer
     *  \param  size number of bytes
     *  \return none
     */
     void read_synchronous(uint8_t *data, uint16_t size);
    /**
     *  \brief Process the received SPI Header and in accordance with it - continue reading the packet
     *  \param  None
     *  \return 0
     */
     uint32_t read_data_cont();
     /**
     *  \brief Enable WLAN interrutp
     *  \param  None
     *  \return None
     */
    void wlan_irq_enable();
     /**
     *  \brief Disable WLAN interrutp
     *  \param  None
     *  \return None
     */
    void wlan_irq_disable();
    /**
     *  \brief Get WLAN interrupt status
     *  \param   None
     *  \return  0 : No interrupt occured
     *           1 : Interrupt occured
     */
    uint32_t wlan_irq_read();
    /**
     *  \brief SPI interrupt Handler.
     *      The external WLAN device asserts the IRQ line when data is ready.\n
     *      The host CPU needs to acknowledges the IRQ by asserting CS.\n
     *
     *  \param  none
     *  \return none
     */
    void WLAN_IRQHandler();
    /**
     *  \brief Enable/Disable the WLAN module
     *  \param  value 1 : Enable
     *                0 : Disable
     *  \return None
     */
    void set_wlan_en(uint8_t value);
private:
    tSpiInfo            _spi_info;
    InterruptIn         _wlan_irq;
    DigitalOut          _wlan_en;
    DigitalOut          _wlan_cs;
    SPI                 _wlan_spi;
    cc3000_event        &_event;
    cc3000_simple_link  &_simple_link;
    bool                _process_irq;
};

/** HCI layer
 */
class cc3000_hci {
public:
    /**
     *  \brief Ctor
     *  \param spi Reference to the spi object.
     *  \return none
     */
    cc3000_hci(cc3000_spi &spi);
    /**
     *  \brief Dtor
     *  \param none
     *  \return none
     */
    ~cc3000_hci();
    /**
     *  \brief Initiate an HCI command.
     *  \param op_code command operation code
     *  \param buffer  pointer to the command's arguments buffer
     *  \param length  length of the arguments
     *  \return 0
     */
    uint16_t command_send(uint16_t op_code, uint8_t *buffer, uint8_t length);
    /**
     *  \brief Initiate an HCI data write operation
     *  \param op_code     command operation code
     *  \param args        pointer to the command's arguments buffer
     *  \param arg_length  length of the arguments
     *  \param data_length length od data
     *  \param tail        pointer to the data buffer
     *  \param tail_length buffer length
     *  \return ESUCCESS
     */
    uint32_t data_send(uint8_t op_code, uint8_t *args, uint16_t arg_length,
                        uint16_t data_length, const uint8_t *tail, uint16_t tail_length);
    /**
     *  \brief Prepare HCI header and initiate an HCI data write operation.
     *  \param op_code     command operation code
     *  \param buffer      pointer to the data buffer
     *  \param arg_length  arguments length
     *  \param data_length data length
     *  \return none
     */
    void data_command_send(uint16_t op_code, uint8_t *buffer, uint8_t arg_length,
                            uint16_t data_length);
    /**
     *  \brief Prepare HCI header and initiate an HCI patch write operation.
     *  \param op_code     command operation code
     *  \param buffer      pointer to the command's arguments buffer
     *  \param patch       pointer to patch content buffer
     *  \param data_length data length
     *  \return none
     */
    void patch_send(uint8_t op_code, uint8_t *buffer, uint8_t *patch, uint16_t data_length);
private:
    cc3000_spi &_spi;
};

/** NVMEM layer
 */
class cc3000_nvmem {
public:
    /**
     *  \brief Ctor
     *  \param hci         Reference to the hci object.
     *  \param event       Reference to the event object.
     *  \param simple_link Reference to the simple link object.
     *  \return none
     */
    cc3000_nvmem(cc3000_hci &hci, cc3000_event &event, cc3000_simple_link &simple_link);
    /**
     *  \brief Dtor
     *  \param none
     *  \return none
     */
    ~cc3000_nvmem();
    /**
     *  \brief Reads data from the file referred by the file_id parameter.
     *      Reads data from file offset till length. Err if the file can't be used,
     *      is invalid, or if the read is out of bounds.
     *  \param file_id nvmem file id.
     *  \param length  number of bytes to read.
     *  \param offset  offset in file from where to read.
     *  \param buff    output buffer pointer.
     *  \return
     *      Number of bytes read, otherwise error.
     */
    int32_t read(uint32_t file_id, uint32_t length, uint32_t offset, uint8_t *buff);
    /**
     *  \brief Write data to nvmem.
     *  \param file_id      Nvmem file id
     *  \param length       number of bytes to write
     *  \param entry_offset offset in file to start write operation from
     *  \param buff         data to write
     *  \return
     *      On success 0, error otherwise.
     */
    int32_t write(uint32_t file_id, uint32_t length, uint32_t entry_offset, uint8_t *buff);
    /**
     *  \brief Write MAC address to EEPROM.
     *  \param mac Mac address to be set
     *  \return
     *      On success 0, error otherwise.
     */
    uint8_t set_mac_address(uint8_t *mac);
    /**
     *  \brief Read MAC address from EEPROM.
     *  \param mac Mac address
     *  \return
     *      On success 0, error otherwise.
     */
    uint8_t get_mac_address(uint8_t *mac);
    /**
     *  \brief Program a patch to a specific file ID. The SP data is assumed to be organized in 2-dimensional.
     *      Each line is SP_PORTION_SIZE bytes long.
     *  \param file_id nvmem file id/
     *  \param length  number of bytes to write
     *  \param data    SP data to write
     *  \return
     *      On success 0, error otherwise.
     */
    uint8_t write_patch(uint32_t file_id, uint32_t length, const uint8_t *data);
    /**
     *  \brief Create new file entry and allocate space on the NVMEM. Applies only to user files.
     *  \param file_id nvmem file Id
     *  \param new_len entry ulLength
     *  \return
     */
    int32_t create_entry(uint32_t file_id, uint32_t new_len);
#ifndef CC3000_TINY_DRIVER
    /**
     *  \brief Read patch version. read package version (WiFi FW patch, river-supplicant-NS patch,
     *      bootloader patch)
     *  \param patch_ver First number indicates package ID and the second number indicates
     *      package build number
     *  \return
     *      On success 0, error otherwise.
     */
    uint8_t read_sp_version(uint8_t* patch_ver);
#endif
private:
    cc3000_hci          &_hci;
    cc3000_event        &_event;
    cc3000_simple_link  &_simple_link;
};

/** WLAN layer
 */
class cc3000_wlan {
public:
    /**
     *  \brief Ctor
     *  \param simple_link Reference to the simple link object.
     *  \param event       Reference to the event object.
     *  \param spi         Reference to the spi object.
     *  \param hci         Reference to the hci object.
     *  \return none
     */
    cc3000_wlan(cc3000_simple_link &simple_link, cc3000_event &event, cc3000_spi &spi, cc3000_hci &hci);
    /**
     *  \brief Dtor
     *  \param none
     *  \return none
     */
    ~cc3000_wlan();
    /**
     *  \brief Send SIMPLE LINK START to cc3000.
     *  \param patches_available_host Flag to indicate if patches are available.
     *  \return none
     */
    void simpleLink_init_start(uint16_t patches_available_host);
    /**
     *  \brief Start wlan device. Blocking call until init is completed.
     *  \param patches_available_host Flag to indicate if patches are available.
     *  \return none
     */
    void start(uint16_t patches_available_host);
    /**
     *  \brief Stop wlan device
     *  \param none
     *  \return none
     */
    void stop(void);
#ifndef CC3000_TINY_DRIVER
    /**
     *  \brief Connect to AP.
     *  \param sec_type    Security option.
     *  \param ssid        up to 32 bytes, ASCII SSID
     *  \param ssid_length length of SSID
     *  \param b_ssid      6 bytes specified the AP bssid
     *  \param key         up to 16 bytes specified the AP security key
     *  \param key_len     key length
     *  \return
     *      On success, zero is returned. On error, negative is returned.
     */
    int32_t connect(uint32_t sec_type, const uint8_t *ssid, int32_t ssid_length, uint8_t *b_ssid, uint8_t *key, int32_t key_len);
    /**
     *  \brief Add profile. Up to 7 profiles are supported.
     *  \param sec_type                      Security option.
     *  \param ssid                          Up to 32 bytes, ASCII SSID
     *  \param ssid_length                   Length of SSID
     *  \param b_ssid                        6 bytes specified the AP bssid
     *  \param priority                      Up to 16 bytes specified the AP security key
     *  \param pairwise_cipher_or_tx_key_len Key length
     *  \param group_cipher_tx_key_index     Key length for WEP security
     *  \param key_mgmt                      KEY management
     *  \param pf_or_key                     Security key
     *  \param pass_phrase_length            Security key length for WPA\WPA2
     *  \return
     *      On success, zero is returned. On error, negative is returned.
     */
    int32_t add_profile(uint32_t sec_type, uint8_t* ssid, uint32_t ssid_length, uint8_t *b_ssid, uint32_t priority, uint32_t pairwise_cipher_or_tx_key_len, uint32_t group_cipher_tx_key_index,
                          uint32_t key_mgmt, uint8_t* pf_or_key, uint32_t pass_phrase_length);
    /**
     *  \brief Gets entry from scan result table. The scan results are returned
     *      one by one, and each entry represents a single AP found in the area.
     *  \param scan_timeout Not supported yet
     *  \param results      Scan result
     *  \return
     *      On success, zero is returned. On error, -1 is returned
     */
    int32_t ioctl_get_scan_results(uint32_t scan_timeout, uint8_t *results);
    /**
     *  \brief Start and stop scan procedure. Set scan parameters.
     *  \param enable             Start/stop application scan
     *  \param min_dwell_time     Minimum dwell time value to be used for each channel, in ms. (Default: 20)
     *  \param max_dwell_time     Maximum dwell time value to be used for each channel, in ms. (Default: 30)
     *  \param num_probe_requests Max probe request between dwell time. (Default:2)
     *  \param channel_mask       Bitwise, up to 13 channels (0x1fff).
     *  \param rssi_threshold     RSSI threshold. Saved: yes (Default: -80)
     *  \param snr_threshold      NSR threshold. Saved: yes (Default: 0)
     *  \param default_tx_power   probe Tx power. Saved: yes (Default: 205)
     *  \param interval_list      Pointer to array with 16 entries (16 channels)
     *  \return
     *      On success, zero is returned. On error, -1 is returned.
     */
    int32_t ioctl_set_scan_params(uint32_t enable, uint32_t min_dwell_time, uint32_t max_dwell_time, uint32_t num_probe_requests,
                                uint32_t channel_mask, int32_t rssi_threshold, uint32_t snr_threshold, uint32_t default_tx_power, uint32_t *interval_list);
    /**
     *  \brief Get wlan status: disconnected, scanning, connecting or connected
     *  \param none
     *  \return
     *      WLAN_STATUS_DISCONNECTED, WLAN_STATUS_SCANING, STATUS_CONNECTING or WLAN_STATUS_CONNECTED
     */
    int32_t ioctl_statusget(void);
#else
    /**
     *  \brief Connect to AP
     *  \param ssid        Up to 32 bytes and is ASCII SSID of the AP
     *  \param ssid_length Length of the SSID
     *  \return
     *      On success, zero is returned. On error, negative is returned.
     */
    int32_t connect(const uint8_t *ssid, int32_t ssid_length);
    /**
     *  \brief When auto start is enabled, the device connects to station from the profiles table.
     *      If several profiles configured the device choose the highest priority profile.
     *  \param sec_type                      WLAN_SEC_UNSEC,WLAN_SEC_WEP,WLAN_SEC_WPA,WLAN_SEC_WPA2
     *  \param ssid                          SSID up to 32 bytes
     *  \param ssid_length                   SSID length
     *  \param b_ssid                        bssid 6 bytes
     *  \param priority                      Profile priority. Lowest priority:0.
     *  \param pairwise_cipher_or_tx_key_len Key length for WEP security
     *  \param group_cipher_tx_key_index     Key index
     *  \param key_mgmt                      KEY management
     *  \param pf_or_key                     Security key
     *  \param pass_phrase_length            Security key length for WPA\WPA2
     *  \return
     *      On success, zero is returned. On error, -1 is returned
     */
    int32_t add_profile(uint32_t sec_type, uint8_t *ssid, uint32_t ssid_length, uint8_t *b_ssid, uint32_t priority,
                      uint32_t pairwise_cipher_or_tx_key_len, uint32_t group_cipher_tx_key_index, uint32_t key_mgmt,
                      uint8_t* pf_or_key, uint32_t pass_phrase_length);
#endif
#ifndef CC3000_UNENCRYPTED_SMART_CONFIG
    /**
     *  \brief Process the acquired data and store it as a profile.
     *  \param none
     *  \return
     *      On success, zero is returned. On error, -1 is returned.
     */
    int32_t smart_config_process(void);
#endif
    /**
     *  \brief Disconnect connection from AP.
     *  \param none
     *  \return
     *      0 if disconnected done, other CC3000 already disconnected.
     */
    int32_t disconnect();
    /**
     *  \brief When auto is enabled, the device tries to connect according the following policy:
     *      1) If fast connect is enabled and last connection is valid, the device will try to
     *      connect to it without the scanning procedure (fast). The last connection will be
     *      marked as invalid, due to adding/removing profile.
     *      2) If profile exists, the device will try to connect it (Up to seven profiles).
     *      3) If fast and profiles are not found, and open mode is enabled, the device
     *      will try to connect to any AP.
     *      Note that the policy settings are stored in the CC3000 NVMEM.
     *  \param should_connect_to_open_ap Enable(1), disable(0) connect to any available AP.
     *  \param use_fast_connect          Enable(1), disable(0). if enabled, tries to
     *                                   connect to the last connected AP.
     *  \param use_profiles              Enable(1), disable(0) auto connect after reset.
     *                                   and periodically reconnect if needed.
     *  \return
     *      On success, zero is returned. On error, -1 is returned
     */
    int32_t ioctl_set_connection_policy(uint32_t should_connect_to_open_ap, uint32_t use_fast_connect, uint32_t use_profiles);
    /**
     *  \brief Delete WLAN profile
     *  \param index Number of profile to delete
     *  \return
     *      On success, zero is returned. On error, -1 is returned
     */
    int32_t ioctl_del_profile(uint32_t index);
    /**
     *  \brief Mask event according to bit mask. In case that event is
     *      masked (1), the device will not send the masked event to host.
     *  \param mask event mask
     *  \return
     *      On success, zero is returned. On error, -1 is returned
     */
    int32_t set_event_mask(uint32_t mask);
    /**
     *  \brief Start to acquire device profile. The device acquire its own
     *      profile, if profile message is found.
     *  \param encrypted_flag Indicates whether the information is encrypted
     *  \return
     *      On success, zero is returned. On error, -1 is returned.
     */
    int32_t smart_config_start(uint32_t encrypted_flag);
    /**
     *  \brief Stop the acquire profile procedure.
     *  \param none
     *  \return
     *      On success, zero is returned. On error, -1 is returned
     */
    int32_t smart_config_stop(void);
    /**
     *  \brief Configure station ssid prefix.
     *  \param new_prefix 3 bytes identify the SSID prefix for the Smart Config.
     *  \return
     *      On success, zero is returned. On error, -1 is returned.
     */
    int32_t smart_config_set_prefix(uint8_t *new_prefix);
private:
    cc3000_simple_link  &_simple_link;
    cc3000_event        &_event;
    cc3000_spi          &_spi;
    cc3000_hci          &_hci;
};

/** The main object of cc3000 implementation
 */
class cc3000 {
public:
    /** status structure */
    typedef struct {
        uint8_t socket;
        bool    dhcp;
        bool    connected;
        bool    smart_config_complete;
        bool    stop_smart_config;
        bool    dhcp_configured;
        bool    ok_to_shut_down;
        bool    enabled;
    } tStatus;
    /**
     *  \brief Ctor.
     *  \param cc3000_irq IRQ pin
     *  \param cc3000_en  Enable pin
     *  \param cc3000_cs  Chip select pin
     *  \param cc3000_spi SPI interface
     */
    cc3000(PinName cc3000_irq, PinName cc3000_en, PinName cc3000_cs, SPI cc3000_spi);
    /**
     *  \brief Dtor.
     */
    ~cc3000();
    /**
     *  \brief Initiate cc3000. It starts the wlan communication.
     *  \param patch Patch
     */
    void start(uint8_t patch);
    /**
     *  \brief Stops the wlan communication.
     */
    void stop();
    /**
     *  \brief Restarts the wlan communication.
     */
    void restart(uint8_t patch);
    /**
     *  \brief Callback which is called from the event class. This updates status of cc3000.
     *  \param event_type Type of the event
     *  \param data       Pointer to data
     *  \param length     Length of data
     *  \return none
     */
    void usync_callback(int32_t event_type, uint8_t *data, uint8_t length);
    /**
     *  \brief Start connection to SSID (open/secured) non-blocking
     *  \param ssid          SSID name
     *  \param key           Security key (if key = 0, open connection)
     *  \param security_mode Security mode
     *  \return true if connection was established, false otherwise.
     */
    bool connect_non_blocking(const uint8_t *ssid, const uint8_t *key, int32_t security_mode);
    /**
     *  \brief Connect to SSID (open/secured) with timeout (10s).
     *  \param ssid          SSID name
     *  \param key           Security key (if key = 0, open connection)
     *  \param security_mode Security mode
     *  \return true if connection was established, false otherwise.
     */
    bool connect_to_AP(const uint8_t *ssid, const uint8_t *key, int32_t security_mode);
    /**
     *  \brief Connect to SSID which is secured
     *  \param ssid          SSID name
     *  \param key           Security key
     *  \param security_mode Security mode
     *  \return true if connection was established, false otherwise.
     */
    bool connect_secure(const uint8_t *ssid, const uint8_t *key, int32_t security_mode);
    /**
     *  \brief Connect to SSID which is open (no security)
     *  \param ssid          SSID name
     *  \return true if connection was established, false otherwise.
     */
    bool connect_open(const uint8_t *ssid);
    /**
     *  \brief Status of the cc3000 module.
     *  \return true if it's enabled, false otherwise.
     */
    bool is_enabled();
    /**
     *  \brief Status of the cc3000 connection.
     *  \return true if it's connected, false otherwise.
     */
    bool is_connected();
    /**
     *  \brief Status of DHCP.
     *  \param none
     *  \return true if DCHP is configured, false otherwise.
     */
    bool is_dhcp_configured();
    /**
     *  \brief Status of smart confing completation.
     *  \param none
     *  \return smart config was set, false otherwise.
     */
    bool is_smart_confing_completed();
    /**
     *  \brief Return the cc3000's mac address.
     *  \param address Retreived mac address.
     *  \return
     */
    uint8_t get_mac_address(uint8_t address[6]);
    /**
     *  \brief Set the cc3000's mac address.
     *  \param address Mac address to be set.
     *  \return
     */
    uint8_t set_mac_address(uint8_t address[6]);
    /**
     *  \brief Get user file info.
     *  \param  info_file Pointer where info will be stored.
     *  \param  size      Available size.
     *  \return none
     */
    void get_user_file_info(uint8_t *info_file, size_t size);
    /**
     *  \brief Set user filo info.
     *  \param info_file Pointer to user's info.
     *  \return none
     */
    void set_user_file_info(uint8_t *info_file, size_t size);
    /**
     *  \brief Start smart config.
     *  \param smart_config_key Pointer to smart config key.
     *  \return none
     */
    void start_smart_config(const uint8_t *smart_config_key);  /* TODO enable AES ? */
#ifndef CC3000_TINY_DRIVER
    /**
     *  \brief Return ip configuration.
     *  \param ip_config Pointer to ipconfig data.
     *  \return true if it's connected and info was retrieved, false otherwise.
     */
    bool get_ip_config(tNetappIpconfigRetArgs *ip_config);
#endif
    /**
     *  \brief Delete all stored profiles.
     *  \param none
     *  \return none
     */
    void delete_profiles(void);
    /**
     *  \brief Ping an ip address.
     *  \param ip       Destination IP address
     *  \param attempts Number of attempts
     *  \param timeout  Time to wait for a response,in milliseconds.
     *  \param size     Send buffer size which may be up to 1400 bytes
     */
    uint32_t ping(uint32_t ip, uint8_t attempts, uint16_t timeout, uint8_t size);
    /**
     *  \brief Returns cc3000 instance. Used in Socket interface.
     *  \param none
     *  \return Pointer to cc3000 object
     */
    static cc3000* get_instance() {
        return _inst;
    }
#if (CC3000_ETH_COMPAT == 1)
    /**
     *  \brief Ctor for EthernetInterface
     *  \param cc3000_irq   IRQ pin
     *  \param cc3000_en    Enable pin
     *  \param cc3000_cs    Chip select pin
     *  \param cc3000_spi   SPI interface
     *  \param ssid         SSID
     *  \param phrase       Password
     *  \param sec          Security of the AP
     *  \param smart_config Smart config selection
     */
    cc3000(PinName cc3000_irq, PinName cc3000_en, PinName cc3000_cs, SPI cc3000_spi, const char *ssid, const char *phrase, Security sec, bool smart_config);
    /**
     *  \brief Disconnect wlan device.
     *  \param none
     *  \return 0 if successful, -1 otherwise.
     */
    int disconnect();
    /**
     *  \brief Initialize the interface with DHCP.
     *  \param none
     *  \return none
     */
    void init();
    /**
     *  \brief Initialize the interface with a static IP address.
     *  \param ip      the IP address to use.
     *  \param mask    the IP address mask
     *  \param gateway the gateway to use
     *  \return none
     */
    void init(const char *ip, const char *mask, const char *gateway);
    /**
     *  \brief Connect Bring the interface up.
     *  \param timeout_ms timeout in ms
     *  \return 0 if successful, -1 otherwise.
     */
    int connect(unsigned int timeout_ms = 20000);
    /**
     *  \brief Get the MAC address of your Ethernet interface.
     *  \param none
     *  \return
     *      Pointer to a string containing the MAC address.
     */
    char* getMACAddress();
     /**
     *  \brief Get the IP address of your Ethernet interface.
     *  \param none
     *  \return
     *      Pointer to a string containing the IP address.
     */
    char* getIPAddress();
     /**
     *  \brief Get the Gateway address of your Ethernet interface
     *  \param none
     *  \return
     *      Pointer to a string containing the Gateway address
     */
    char* getGateway();
     /**
     *  \brief Get the Network mask of your Ethernet interface
     *  \param none
     *  \return
     *      Pointer to a string containing the Network mask
     */
    char* getNetworkMask();
#endif
public:
    cc3000_simple_link  _simple_link;
    cc3000_event        _event;
    cc3000_socket       _socket;
    cc3000_spi          _spi;
    cc3000_hci          _hci;
    cc3000_nvmem        _nvmem;
    cc3000_netapp       _netapp;
    cc3000_wlan         _wlan;
#ifndef CC3000_UNENCRYPTED_SMART_CONFIG
    cc3000_security     _security;
#endif
protected:
    static cc3000       *_inst;
private:
    tStatus                  _status;
    netapp_pingreport_args_t _ping_report;
    bool                     _closed_sockets[MAX_SOCKETS];
#if (CC3000_ETH_COMPAT == 1)
    uint8_t                  _phrase[30];
    uint8_t                  _ssid[30];
    Security                 _sec;
    bool                     _smart_config;
#endif
};

/**
 * Copy 32 bit to stream while converting to little endian format.
 * @param  p       pointer to the new stream
 * @param  u32     pointer to the 32 bit
 * @return         pointer to the new stream
 */
uint8_t *UINT32_TO_STREAM_f (uint8_t *p, uint32_t u32);

/**
 * Copy 16 bit to stream while converting to little endian format.
 * @param  p       pointer to the new stream
 * @param  u32     pointer to the 16 bit
 * @return         pointer to the new stream
 */
uint8_t *UINT16_TO_STREAM_f (uint8_t *p, uint16_t u16);

/**
 * Copy received stream to 16 bit in little endian format.
 * @param  p          pointer to the stream
 * @param  offset     offset in the stream
 * @return            pointer to the new 16 bit
 */
uint16_t STREAM_TO_UINT16_f(uint8_t* p, uint16_t offset);

/**
 * Copy received stream to 32 bit in little endian format.
 * @param  p          pointer to the stream
 * @param  offset     offset in the stream
 * @return            pointer to the new 32 bit
 */
uint32_t STREAM_TO_UINT32_f(uint8_t* p, uint16_t offset);

} /* end of mbed_cc3000 namespace */


#endif
