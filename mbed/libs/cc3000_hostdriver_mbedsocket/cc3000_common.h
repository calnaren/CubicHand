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
#ifndef CC3000_COMMON_H
#define CC3000_COMMON_H

#include <errno.h>

//#define CC3000_TINY_DRIVER                // Driver for small memory model CPUs

#define ESUCCESS        0
#define EFAIL          -1
#define EERROR          EFAIL

#define CC3000_UNENCRYPTED_SMART_CONFIG   // No encryption

#define ERROR_SOCKET_INACTIVE   -57

#define HCI_CC_PAYLOAD_LEN      5

#define WLAN_ENABLE            (1)
#define WLAN_DISABLE           (0)

#define MAC_ADDR_LEN           (6)


/*Defines for minimal and maximal RX buffer size. This size includes the spi
  header and hci header.
  maximal buffer size: MTU + HCI header + SPI header + sendto() args size
  minimum buffer size: HCI header + SPI header + max args size

  This buffer is used for receiving events and data.
  The packet can not be longer than MTU size and CC3000 does not support
  fragmentation. Note that the same buffer is used for reception of the data
  and events from CC3000. That is why the minimum is defined.
  The calculation for the actual size of buffer for reception is:
  Given the maximal data size MAX_DATA that is expected to be received by
  application, the required buffer Using recv() or recvfrom():

    max(CC3000_MINIMAL_RX_SIZE, MAX_DATA + HEADERS_SIZE_DATA + fromlen + ucArgsize + 1)

  Using gethostbyname() with minimal buffer size will limit the host name returned to 99 bytes.
  The 1 is used for the overrun detection
*/

#define CC3000_MINIMAL_RX_SIZE      (118 + 1)
#define CC3000_MAXIMAL_RX_SIZE      (511 + 1)

/*Defines for minimal and maximal TX buffer size.
  This buffer is used for sending events and data.
  The packet can not be longer than MTU size and CC3000 does not support
  fragmentation. Note that the same buffer is used for transmission of the data
  and commands. That is why the minimum is defined.
  The calculation for the actual size of buffer for transmission is:
  Given the maximal data size MAX_DATA, the required buffer is:
  Using Sendto():

   max(CC3000_MINIMAL_TX_SIZE, MAX_DATA + SPI_HEADER_SIZE
   + SOCKET_SENDTO_PARAMS_LEN + SIMPLE_LINK_HCI_DATA_HEADER_SIZE + 1)

  Using Send():

   max(CC3000_MINIMAL_TX_SIZE, MAX_DATA + SPI_HEADER_SIZE
   + HCI_CMND_SEND_ARG_LENGTH + SIMPLE_LINK_HCI_DATA_HEADER_SIZE + 1)

  The 1 is used for the overrun detection */

#define CC3000_MINIMAL_TX_SIZE      (118 + 1)
#define CC3000_MAXIMAL_TX_SIZE      (1519 + 1)

//TX and RX buffer size - allow to receive and transmit maximum data at lengh 8.
#ifdef CC3000_TINY_DRIVER
#define TINY_CC3000_MAXIMAL_RX_SIZE 44
#define TINY_CC3000_MAXIMAL_TX_SIZE 59
#endif

/*In order to determine your preferred buffer size,
  change CC3000_MAXIMAL_RX_SIZE and CC3000_MAXIMAL_TX_SIZE to a value between
  the minimal and maximal specified above.
  Note that the buffers are allocated by SPI.
*/

#ifndef CC3000_TINY_DRIVER

    #define CC3000_RX_BUFFER_SIZE   (CC3000_MAXIMAL_RX_SIZE)
    #define CC3000_TX_BUFFER_SIZE   (CC3000_MAXIMAL_TX_SIZE)
    #define SP_PORTION_SIZE         512

//TINY DRIVER: We use smaller rx and tx buffers in order to minimize RAM consumption
#else
    #define CC3000_RX_BUFFER_SIZE   (TINY_CC3000_MAXIMAL_RX_SIZE)
    #define CC3000_TX_BUFFER_SIZE   (TINY_CC3000_MAXIMAL_TX_SIZE)
    #define SP_PORTION_SIZE         32
#endif


//Copy 8 bit to stream while converting to little endian format.
#define UINT8_TO_STREAM(_p, _val)    {*(_p)++ = (_val);}
//Copy 16 bit to stream while converting to little endian format.
#define UINT16_TO_STREAM(_p, _u16)    (UINT16_TO_STREAM_f(_p, _u16))
//Copy 32 bit to stream while converting to little endian format.
#define UINT32_TO_STREAM(_p, _u32)    (UINT32_TO_STREAM_f(_p, _u32))
//Copy a specified value length bits (l) to stream while converting to little endian format.
#define ARRAY_TO_STREAM(p, a, l)     {uint32_t _i; for (_i = 0; _i < l; _i++) *(p)++ = ((uint8_t *) a)[_i];}
//Copy received stream to 8 bit in little endian format.
#define STREAM_TO_UINT8(_p, _offset, _u8)    {_u8 = (uint8_t)(*(_p + _offset));}
//Copy received stream to 16 bit in little endian format.
#define STREAM_TO_UINT16(_p, _offset, _u16)    {_u16 = STREAM_TO_UINT16_f(_p, _offset);}
//Copy received stream to 32 bit in little endian format.
#define STREAM_TO_UINT32(_p, _offset, _u32)    {_u32 = STREAM_TO_UINT32_f(_p, _offset);}
#define STREAM_TO_STREAM(p, a, l)     {uint32_t _i; for (_i = 0; _i < l; _i++) *(a)++= ((uint8_t *) p)[_i];}

typedef struct _sockaddr_t
{
    uint16_t  family;
    uint8_t   data[14];
} sockaddr;

struct timeval
{
    int32_t tv_sec;       /* seconds */
    int32_t tv_usec;      /* microseconds */
};

#define SMART_CONFIG_PROFILE_SIZE        67        // 67 = 32 (max ssid) + 32 (max key) + 1 (SSID length) + 1 (security type) + 1 (key length)

/* patches type */
#define PATCHES_HOST_TYPE_WLAN_DRIVER   0x01
#define PATCHES_HOST_TYPE_WLAN_FW       0x02
#define PATCHES_HOST_TYPE_BOOTLOADER    0x03

#define SL_SET_SCAN_PARAMS_INTERVAL_LIST_SIZE    (16)
#define SL_SIMPLE_CONFIG_PREFIX_LENGTH           (3)
#define ETH_ALEN                                 (6)
#define MAXIMAL_SSID_LENGTH                      (32)

#define SL_PATCHES_REQUEST_DEFAULT               (0)
#define SL_PATCHES_REQUEST_FORCE_HOST            (1)
#define SL_PATCHES_REQUEST_FORCE_NONE            (2)


#define      WLAN_SEC_UNSEC  (0)
#define      WLAN_SEC_WEP    (1)
#define      WLAN_SEC_WPA    (2)
#define      WLAN_SEC_WPA2   (3)


#define WLAN_SL_INIT_START_PARAMS_LEN           (1)
#define WLAN_PATCH_PARAMS_LENGTH                (8)
#define WLAN_SET_CONNECTION_POLICY_PARAMS_LEN   (12)
#define WLAN_DEL_PROFILE_PARAMS_LEN             (4)
#define WLAN_SET_MASK_PARAMS_LEN                (4)
#define WLAN_SET_SCAN_PARAMS_LEN                (100)
#define WLAN_GET_SCAN_RESULTS_PARAMS_LEN        (4)
#define WLAN_ADD_PROFILE_NOSEC_PARAM_LEN        (24)
#define WLAN_ADD_PROFILE_WEP_PARAM_LEN          (36)
#define WLAN_ADD_PROFILE_WPA_PARAM_LEN          (44)
#define WLAN_CONNECT_PARAM_LEN                  (29)
#define WLAN_SMART_CONFIG_START_PARAMS_LEN      (4)

#endif
