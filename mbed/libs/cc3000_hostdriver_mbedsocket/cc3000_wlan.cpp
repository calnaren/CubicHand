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

namespace mbed_cc3000 {

cc3000_wlan::cc3000_wlan(cc3000_simple_link &simple_link, cc3000_event &event, cc3000_spi &spi, cc3000_hci &hci) :
    _simple_link(simple_link), _event(event), _spi(spi), _hci(hci) {

}

cc3000_wlan::~cc3000_wlan() {

}

void cc3000_wlan::simpleLink_init_start(uint16_t patches_available_host) {
    uint8_t *ptr;
    uint8_t *args;

    ptr = _simple_link.get_transmit_buffer();
    args = (uint8_t *)(ptr + HEADERS_SIZE_CMD);

    UINT8_TO_STREAM(args, ((patches_available_host) ? SL_PATCHES_REQUEST_FORCE_HOST : SL_PATCHES_REQUEST_DEFAULT));

    // IRQ Line asserted - send HCI_CMND_SIMPLE_LINK_START to CC3000
    _hci.command_send(HCI_CMND_SIMPLE_LINK_START, ptr, WLAN_SL_INIT_START_PARAMS_LEN);
    _event.simplelink_wait_event(HCI_CMND_SIMPLE_LINK_START, 0);
}

void cc3000_wlan::start(uint16_t patches_available_host) {
    uint32_t spi_irq_state;

    _simple_link.set_sent_packets(0);
    _simple_link.set_number_of_released_packets(0);
    _simple_link.set_op_code(0);
    _simple_link.set_number_free_buffers(0);
    _simple_link.set_buffer_length(0);
    _simple_link.set_buffer_size(0);
    _simple_link.set_pending_data(0);
    _simple_link.set_transmit_error(0);
    _simple_link.set_data_received_flag(0);
    _simple_link.set_buffer_size(0);

    // init spi
    _spi.open();
    // Check the IRQ line
    spi_irq_state = _spi.wlan_irq_read();
    // ASIC 1273 chip enable: toggle WLAN EN line
    _spi.set_wlan_en(WLAN_ENABLE);

    if (spi_irq_state) {
        // wait till the IRQ line goes low
        while(_spi.wlan_irq_read() != 0);
    } else {
        // wait till the IRQ line goes high and then low
        while(_spi.wlan_irq_read() == 0);
        while(_spi.wlan_irq_read() != 0);
    }
    simpleLink_init_start(patches_available_host);

    // Read Buffer's size and finish
    _hci.command_send(HCI_CMND_READ_BUFFER_SIZE, _simple_link.get_transmit_buffer(), 0);
    _event.simplelink_wait_event(HCI_CMND_READ_BUFFER_SIZE, 0);
}


void cc3000_wlan::stop() {
    // ASIC 1273 chip disable
    _spi.set_wlan_en(WLAN_DISABLE);

    // Wait till IRQ line goes high
    while(_spi.wlan_irq_read() == 0);

    _spi.close();
}


int32_t cc3000_wlan::disconnect() {
    int32_t ret;
    uint8_t *ptr;

    ret = EFAIL;
    ptr = _simple_link.get_transmit_buffer();

    _hci.command_send(HCI_CMND_WLAN_DISCONNECT, ptr, 0);

    // Wait for command complete event
    _event.simplelink_wait_event(HCI_CMND_WLAN_DISCONNECT, &ret);
    errno = ret;

    return ret;
}


int32_t cc3000_wlan::ioctl_set_connection_policy(uint32_t should_connect_to_open_ap,
                                      uint32_t use_fast_connect,
                                      uint32_t use_profiles) {
    int32_t ret;
    uint8_t *ptr;
    uint8_t *args;

    ret = EFAIL;
    ptr = _simple_link.get_transmit_buffer();
    args = (uint8_t *)(ptr + HEADERS_SIZE_CMD);

    // Fill in HCI packet structure
    args = UINT32_TO_STREAM(args, should_connect_to_open_ap);
    args = UINT32_TO_STREAM(args, use_fast_connect);
    args = UINT32_TO_STREAM(args, use_profiles);

    // Initiate a HCI command
    _hci.command_send(HCI_CMND_WLAN_IOCTL_SET_CONNECTION_POLICY, ptr, WLAN_SET_CONNECTION_POLICY_PARAMS_LEN);

    // Wait for command complete event
    _event.simplelink_wait_event(HCI_CMND_WLAN_IOCTL_SET_CONNECTION_POLICY, &ret);

    return ret;
}


int32_t cc3000_wlan::ioctl_del_profile(uint32_t index) {
    int32_t ret;
    uint8_t *ptr;
    uint8_t *args;

    ptr = _simple_link.get_transmit_buffer();
    args = (uint8_t *)(ptr + HEADERS_SIZE_CMD);

    // Fill in HCI packet structure
    args = UINT32_TO_STREAM(args, index);
    ret = EFAIL;

    // Initiate a HCI command
    _hci.command_send(HCI_CMND_WLAN_IOCTL_DEL_PROFILE, ptr, WLAN_DEL_PROFILE_PARAMS_LEN);

    // Wait for command complete event
    _event.simplelink_wait_event(HCI_CMND_WLAN_IOCTL_DEL_PROFILE, &ret);

    return ret;
}

int32_t cc3000_wlan::set_event_mask(uint32_t mask) {
    int32_t ret;
    uint8_t *ptr;
    uint8_t *args;


    if ((mask & HCI_EVNT_WLAN_TX_COMPLETE) == HCI_EVNT_WLAN_TX_COMPLETE) {
        _simple_link.set_tx_complete_signal(0);

        // Since an event is a virtual event - i.e. it is not coming from CC3000
        // there is no need to send anything to the device if it was an only event
        if (mask == HCI_EVNT_WLAN_TX_COMPLETE) {
            return 0;
        }

        mask &= ~HCI_EVNT_WLAN_TX_COMPLETE;
        mask |= HCI_EVNT_WLAN_UNSOL_BASE;
    } else {
        _simple_link.set_tx_complete_signal(1);
    }

    ret = EFAIL;
    ptr = _simple_link.get_transmit_buffer();
    args = (uint8_t *)(ptr + HEADERS_SIZE_CMD);

    // Fill in HCI packet structure
    args = UINT32_TO_STREAM(args, mask);

    // Initiate a HCI command
    _hci.command_send(HCI_CMND_EVENT_MASK, ptr, WLAN_SET_MASK_PARAMS_LEN);

    // Wait for command complete event
    _event.simplelink_wait_event(HCI_CMND_EVENT_MASK, &ret);

    return ret;
}


int32_t cc3000_wlan::smart_config_start(uint32_t encrypted_flag) {
    int32_t ret;
    uint8_t *ptr;
    uint8_t *args;

    ret = EFAIL;
    ptr = _simple_link.get_transmit_buffer();
    args = (uint8_t *)(ptr + HEADERS_SIZE_CMD);

    // Fill in HCI packet structure
    args = UINT32_TO_STREAM(args, encrypted_flag);
    ret = EFAIL;

    _hci.command_send(HCI_CMND_WLAN_IOCTL_SIMPLE_CONFIG_START, ptr, WLAN_SMART_CONFIG_START_PARAMS_LEN);

    // Wait for command complete event
    _event.simplelink_wait_event(HCI_CMND_WLAN_IOCTL_SIMPLE_CONFIG_START, &ret);

    return ret;
}


int32_t cc3000_wlan::smart_config_stop(void) {
    int32_t ret;
    uint8_t *ptr;

    ret = EFAIL;
    ptr = _simple_link.get_transmit_buffer();

    _hci.command_send(HCI_CMND_WLAN_IOCTL_SIMPLE_CONFIG_STOP, ptr, 0);

    // Wait for command complete event
    _event.simplelink_wait_event(HCI_CMND_WLAN_IOCTL_SIMPLE_CONFIG_STOP, &ret);

    return ret;
}

int32_t cc3000_wlan::smart_config_set_prefix(uint8_t *new_prefix) {
    int32_t ret;
    uint8_t *ptr;
    uint8_t *args;

    ret = EFAIL;
    ptr = _simple_link.get_transmit_buffer();
    args = (ptr + HEADERS_SIZE_CMD);

    if (new_prefix == NULL) {
        return ret;
    } else {
        // with the new Smart Config, prefix must be TTT
        *new_prefix = 'T';
        *(new_prefix + 1) = 'T';
        *(new_prefix + 2) = 'T';
    }

    ARRAY_TO_STREAM(args, new_prefix, SL_SIMPLE_CONFIG_PREFIX_LENGTH);

    _hci.command_send(HCI_CMND_WLAN_IOCTL_SIMPLE_CONFIG_SET_PREFIX, ptr, SL_SIMPLE_CONFIG_PREFIX_LENGTH);

    // Wait for command complete event
    _event.simplelink_wait_event(HCI_CMND_WLAN_IOCTL_SIMPLE_CONFIG_SET_PREFIX, &ret);

    return ret;
}

#ifndef CC3000_TINY_DRIVER
int32_t cc3000_wlan::connect(uint32_t sec_type, const uint8_t *ssid, int32_t ssid_len, uint8_t *bssid,
              uint8_t *key, int32_t key_len) {
    int32_t ret;
    uint8_t *ptr;
    uint8_t *args;
    uint8_t bssid_zero[] = {0, 0, 0, 0, 0, 0};

    ret      = EFAIL;
    ptr      = _simple_link.get_transmit_buffer();
    args     = (ptr + HEADERS_SIZE_CMD);

    // Fill in command buffer
    args = UINT32_TO_STREAM(args, 0x0000001c);
    args = UINT32_TO_STREAM(args, ssid_len);
    args = UINT32_TO_STREAM(args, sec_type);
    args = UINT32_TO_STREAM(args, 0x00000010 + ssid_len);
    args = UINT32_TO_STREAM(args, key_len);
    args = UINT16_TO_STREAM(args, 0);

    // padding shall be zeroed
    if (bssid) {
        ARRAY_TO_STREAM(args, bssid, ETH_ALEN);
    } else {
        ARRAY_TO_STREAM(args, bssid_zero, ETH_ALEN);
    }

    ARRAY_TO_STREAM(args, ssid, ssid_len);

    if (key_len && key) {
        ARRAY_TO_STREAM(args, key, key_len);
    }

    // Initiate a HCI command
    _hci.command_send(HCI_CMND_WLAN_CONNECT, ptr, WLAN_CONNECT_PARAM_LEN + ssid_len + key_len - 1);

    // Wait for command complete event
    _event.simplelink_wait_event(HCI_CMND_WLAN_CONNECT, &ret);
    errno = ret;

    return ret;
}

int32_t cc3000_wlan::add_profile(uint32_t sec_type,
                      uint8_t* ssid,
                      uint32_t ssid_length,
                      uint8_t *b_ssid,
                      uint32_t priority,
                      uint32_t pairwise_cipher_or_tx_key_len,
                      uint32_t group_cipher_tx_key_index,
                      uint32_t key_mgmt,
                      uint8_t* pf_or_key,
                      uint32_t pass_phrase_len) {
    uint16_t arg_len = 0x00;
    int32_t ret;
    uint8_t *ptr;
    int32_t i = 0;
    uint8_t *args;
    uint8_t bssid_zero[] = {0, 0, 0, 0, 0, 0};

    ptr = _simple_link.get_transmit_buffer();
    args = (ptr + HEADERS_SIZE_CMD);

    args = UINT32_TO_STREAM(args, sec_type);

    // Setup arguments in accordance with the security type
    switch (sec_type)
    {
        //OPEN
        case WLAN_SEC_UNSEC:
        {
            args = UINT32_TO_STREAM(args, 0x00000014);
            args = UINT32_TO_STREAM(args, ssid_length);
            args = UINT16_TO_STREAM(args, 0);
            if(b_ssid) {
                ARRAY_TO_STREAM(args, b_ssid, ETH_ALEN);
            } else {
                ARRAY_TO_STREAM(args, bssid_zero, ETH_ALEN);
            }
            args = UINT32_TO_STREAM(args, priority);
            ARRAY_TO_STREAM(args, ssid, ssid_length);

            arg_len = WLAN_ADD_PROFILE_NOSEC_PARAM_LEN + ssid_length;
        }
        break;

        //WEP
        case WLAN_SEC_WEP:
        {
            args = UINT32_TO_STREAM(args, 0x00000020);
            args = UINT32_TO_STREAM(args, ssid_length);
            args = UINT16_TO_STREAM(args, 0);
            if (b_ssid) {
                ARRAY_TO_STREAM(args, b_ssid, ETH_ALEN);
            } else {
                ARRAY_TO_STREAM(args, bssid_zero, ETH_ALEN);
            }
            args = UINT32_TO_STREAM(args, priority);
            args = UINT32_TO_STREAM(args, 0x0000000C + ssid_length);
            args = UINT32_TO_STREAM(args, pairwise_cipher_or_tx_key_len);
            args = UINT32_TO_STREAM(args, group_cipher_tx_key_index);
            ARRAY_TO_STREAM(args, ssid, ssid_length);

            for(i = 0; i < 4; i++) {
                uint8_t *p = &pf_or_key[i * pairwise_cipher_or_tx_key_len];

                ARRAY_TO_STREAM(args, p, pairwise_cipher_or_tx_key_len);
            }

            arg_len = WLAN_ADD_PROFILE_WEP_PARAM_LEN + ssid_length +
                pairwise_cipher_or_tx_key_len * 4;

        }
        break;

        //WPA
        //WPA2
        case WLAN_SEC_WPA:
        case WLAN_SEC_WPA2:
        {
            args = UINT32_TO_STREAM(args, 0x00000028);
            args = UINT32_TO_STREAM(args, ssid_length);
            args = UINT16_TO_STREAM(args, 0);
            if (b_ssid) {
                ARRAY_TO_STREAM(args, b_ssid, ETH_ALEN);
            } else {
                ARRAY_TO_STREAM(args, bssid_zero, ETH_ALEN);
            }
            args = UINT32_TO_STREAM(args, priority);
            args = UINT32_TO_STREAM(args, pairwise_cipher_or_tx_key_len);
            args = UINT32_TO_STREAM(args, group_cipher_tx_key_index);
            args = UINT32_TO_STREAM(args, key_mgmt);
            args = UINT32_TO_STREAM(args, 0x00000008 + ssid_length);
            args = UINT32_TO_STREAM(args, pass_phrase_len);
            ARRAY_TO_STREAM(args, ssid, ssid_length);
            ARRAY_TO_STREAM(args, pf_or_key, pass_phrase_len);

            arg_len = WLAN_ADD_PROFILE_WPA_PARAM_LEN + ssid_length + pass_phrase_len;
        }

        break;
    }

    // Initiate a HCI command
    _hci.command_send(HCI_CMND_WLAN_IOCTL_ADD_PROFILE, ptr, arg_len);

    // Wait for command complete event
    _event.simplelink_wait_event(HCI_CMND_WLAN_IOCTL_ADD_PROFILE, &ret);

    return ret;
}

int32_t cc3000_wlan::ioctl_get_scan_results(uint32_t scan_timeout, uint8_t *results) {
    uint8_t *ptr;
    uint8_t *args;

    ptr = _simple_link.get_transmit_buffer();
    args = (ptr + HEADERS_SIZE_CMD);

    // Fill in temporary command buffer
    args = UINT32_TO_STREAM(args, scan_timeout);

    // Initiate a HCI command
    _hci.command_send(HCI_CMND_WLAN_IOCTL_GET_SCAN_RESULTS, ptr, WLAN_GET_SCAN_RESULTS_PARAMS_LEN);

    // Wait for command complete event
    _event.simplelink_wait_event(HCI_CMND_WLAN_IOCTL_GET_SCAN_RESULTS, results);

    return 0;
}

int32_t cc3000_wlan::ioctl_set_scan_params(uint32_t enable,
                                uint32_t min_dwell_time,
                                uint32_t max_dwell_time,
                                uint32_t num_probe_requests,
                                uint32_t channel_mask,
                                int32_t rssi_threshold,
                                uint32_t snr_threshold,
                                uint32_t default_tx_power,
                                uint32_t *interval_list) {
    uint32_t  uiRes;
    uint8_t *ptr;
    uint8_t *args;

    ptr = _simple_link.get_transmit_buffer();
    args = (ptr + HEADERS_SIZE_CMD);

    // Fill in temporary command buffer
    args = UINT32_TO_STREAM(args, 36);
    args = UINT32_TO_STREAM(args, enable);
    args = UINT32_TO_STREAM(args, min_dwell_time);
    args = UINT32_TO_STREAM(args, max_dwell_time);
    args = UINT32_TO_STREAM(args, num_probe_requests);
    args = UINT32_TO_STREAM(args, channel_mask);
    args = UINT32_TO_STREAM(args, rssi_threshold);
    args = UINT32_TO_STREAM(args, snr_threshold);
    args = UINT32_TO_STREAM(args, default_tx_power);
    ARRAY_TO_STREAM(args, interval_list, sizeof(uint32_t) * SL_SET_SCAN_PARAMS_INTERVAL_LIST_SIZE);

    // Initiate a HCI command
    _hci.command_send(HCI_CMND_WLAN_IOCTL_SET_SCANPARAM, ptr, WLAN_SET_SCAN_PARAMS_LEN);

    // Wait for command complete event
    _event.simplelink_wait_event(HCI_CMND_WLAN_IOCTL_SET_SCANPARAM, &uiRes);

    return(uiRes);
}

int32_t cc3000_wlan::ioctl_statusget(void) {
    int32_t ret;
    uint8_t *ptr;

    ret = EFAIL;
    ptr = _simple_link.get_transmit_buffer();

    _hci.command_send(HCI_CMND_WLAN_IOCTL_STATUSGET,ptr, 0);

    // Wait for command complete event
    _event.simplelink_wait_event(HCI_CMND_WLAN_IOCTL_STATUSGET, &ret);

    return ret;
}

#else
int32_t cc3000_wlan::add_profile(uint32_t sec_type,
                      uint8_t *ssid,
                      uint32_t ssid_length,
                      uint8_t *b_ssid,
                      uint32_t priority,
                      uint32_t pairwise_cipher_or_tx_key_len,
                      uint32_t group_cipher_tx_key_index,
                      uint32_t key_mgmt,
                      uint8_t* pf_or_key,
                      uint32_t pass_phrase_length)
{
    return -1;
}

int32_t cc3000_wlan::connect(const uint8_t *ssid, int32_t ssid_len) {
    int32_t ret;
    uint8_t *ptr;
    uint8_t *args;
    uint8_t bssid_zero[] = {0, 0, 0, 0, 0, 0};

    ret      = EFAIL;
    ptr      = _simple_link.get_transmit_buffer();
    args     = (ptr + HEADERS_SIZE_CMD);

    // Fill in command buffer
    args = UINT32_TO_STREAM(args, 0x0000001c);
    args = UINT32_TO_STREAM(args, ssid_len);
    args = UINT32_TO_STREAM(args, 0);
    args = UINT32_TO_STREAM(args, 0x00000010 + ssid_len);
    args = UINT32_TO_STREAM(args, 0);
    args = UINT16_TO_STREAM(args, 0);

    // padding shall be zeroed
    ARRAY_TO_STREAM(args, bssid_zero, ETH_ALEN);
    ARRAY_TO_STREAM(args, ssid, ssid_len);

    // Initiate a HCI command
    _hci.command_send(HCI_CMND_WLAN_CONNECT, ptr, WLAN_CONNECT_PARAM_LEN + ssid_len  - 1);

    // Wait for command complete event
    _event.simplelink_wait_event(HCI_CMND_WLAN_CONNECT, &ret);
    errno = ret;

    return ret;
}
#endif



#ifndef CC3000_UNENCRYPTED_SMART_CONFIG
int32_t cc3000_wlan::smart_config_process(void) {
    int32_t  returnValue;
    uint32_t ssidLen, keyLen;
    uint8_t *decKeyPtr;
    uint8_t *ssidPtr;

    // read the key from EEPROM - fileID 12
    returnValue = aes_read_key(key);

    if (returnValue != 0)
        return returnValue;

    // read the received data from fileID #13 and parse it according to the followings:
    // 1) SSID LEN - not encrypted
    // 2) SSID - not encrypted
    // 3) KEY LEN - not encrypted. always 32 bytes long
    // 4) Security type - not encrypted
    // 5) KEY - encrypted together with true key length as the first byte in KEY
    //     to elaborate, there are two corner cases:
    //        1) the KEY is 32 bytes long. In this case, the first byte does not represent KEY length
    //        2) the KEY is 31 bytes long. In this case, the first byte represent KEY length and equals 31
    returnValue = nvmem_read(NVMEM_SHARED_MEM_FILEID, SMART_CONFIG_PROFILE_SIZE, 0, profileArray);

    if (returnValue != 0)
        return returnValue;

    ssidPtr = &profileArray[1];

    ssidLen = profileArray[0];

    decKeyPtr = &profileArray[profileArray[0] + 3];

    aes_decrypt(decKeyPtr, key);
    if (profileArray[profileArray[0] + 1] > 16)
        aes_decrypt((uint8_t *)(decKeyPtr + 16), key);

    if (*(uint8_t *)(decKeyPtr +31) != 0) {
        if (*decKeyPtr == 31) {
            keyLen = 31;
            decKeyPtr++;
        } else {
            keyLen = 32;
        }
    } else {
        keyLen = *decKeyPtr;
        decKeyPtr++;
    }

    // add a profile
    switch (profileArray[profileArray[0] + 2])
    {
    case WLAN_SEC_UNSEC://None
         {
            returnValue = wlan_add_profile(profileArray[profileArray[0] + 2],     // security type
                                           ssidPtr,                               // SSID
                                           ssidLen,                               // SSID length
                                           NULL,                                  // BSSID
                                           1,                                     // Priority
                                           0, 0, 0, 0, 0);

            break;
         }

    case WLAN_SEC_WEP://WEP
        {
            returnValue = wlan_add_profile(profileArray[profileArray[0] + 2],     // security type
                                           ssidPtr,                               // SSID
                                           ssidLen,                               // SSID length
                                           NULL,                                  // BSSID
                                           1,                                     // Priority
                                           keyLen,                                // KEY length
                                           0,                                     // KEY index
                                           0,
                                           decKeyPtr,                             // KEY
                                           0);

            break;
        }

    case WLAN_SEC_WPA:  //WPA
    case WLAN_SEC_WPA2: //WPA2
        {
            returnValue = wlan_add_profile(WLAN_SEC_WPA2,     // security type
                                           ssidPtr,
                                           ssidLen,
                                           NULL,              // BSSID
                                           1,                 // Priority
                                           0x18,              // PairwiseCipher
                                           0x1e,              // GroupCipher
                                           2,                 // KEY management
                                           decKeyPtr,         // KEY
                                           keyLen);           // KEY length

            break;
        }
    }

    return returnValue;
}
#endif

}
