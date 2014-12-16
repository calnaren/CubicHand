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
#include "cc3000_nvmem.h"
#include "cc3000_common.h"

namespace mbed_cc3000 {

cc3000_nvmem::cc3000_nvmem(cc3000_hci &hci, cc3000_event &event, cc3000_simple_link &simple_link)
        : _hci(hci), _event(event), _simple_link(simple_link) {

}

cc3000_nvmem::~cc3000_nvmem() {

}

int32_t  cc3000_nvmem::read(uint32_t file_id, uint32_t length, uint32_t offset, uint8_t *buff) {
    uint8_t ucStatus = 0xFF;
    uint8_t *ptr;
    uint8_t *args;

    ptr = _simple_link.get_transmit_buffer();
    args = (ptr + HEADERS_SIZE_CMD);
    // Fill in HCI packet structure
    args = UINT32_TO_STREAM(args, file_id);
    args = UINT32_TO_STREAM(args, length);
    args = UINT32_TO_STREAM(args, offset);

    // Initiate HCI command
    _hci.command_send(HCI_CMND_NVMEM_READ, ptr, NVMEM_READ_PARAMS_LEN);
    _event.simplelink_wait_event(HCI_CMND_NVMEM_READ, &ucStatus);

    // If data is present, read it even when an error is returned.
    // Note: It is the users responsibility to ignore the data when an error is returned.
    // Wait for the data in a synchronous way.
    //  We assume the buffer is large enough to also store nvmem parameters.
    _event.simplelink_wait_data(buff, 0, 0);

    return(ucStatus);
}

int32_t  cc3000_nvmem::write(uint32_t file_id, uint32_t length, uint32_t entry_offset, uint8_t *buff) {
    int32_t iRes;
    uint8_t *ptr;
    uint8_t *args;

    iRes = EFAIL;

    ptr = _simple_link.get_transmit_buffer();
    args = (ptr + SPI_HEADER_SIZE + HCI_DATA_CMD_HEADER_SIZE);

    // Fill in HCI packet structure
    args = UINT32_TO_STREAM(args, file_id);
    args = UINT32_TO_STREAM(args, 12);
    args = UINT32_TO_STREAM(args, length);
    args = UINT32_TO_STREAM(args, entry_offset);

    memcpy((ptr + SPI_HEADER_SIZE + HCI_DATA_CMD_HEADER_SIZE +
                    NVMEM_WRITE_PARAMS_LEN),buff,length);

    // Initiate a HCI command on the data channel
    _hci.data_command_send(HCI_CMND_NVMEM_WRITE, ptr, NVMEM_WRITE_PARAMS_LEN, length);

    _event.simplelink_wait_event(HCI_EVNT_NVMEM_WRITE, &iRes);

    return(iRes);
}

uint8_t  cc3000_nvmem::set_mac_address(uint8_t *mac) {
    return  write(NVMEM_MAC_FILEID, MAC_ADDR_LEN, 0, mac);
}

uint8_t  cc3000_nvmem::get_mac_address(uint8_t *mac) {
    return  read(NVMEM_MAC_FILEID, MAC_ADDR_LEN, 0, mac);
}

uint8_t  cc3000_nvmem::write_patch(uint32_t file_id, uint32_t length, const uint8_t *data) {
    uint8_t     status = 0;
    uint16_t    offset = 0;
    uint8_t*      spDataPtr = (uint8_t*)data;

    while ((status == 0) && (length >= SP_PORTION_SIZE)) {
        status = write(file_id, SP_PORTION_SIZE, offset, spDataPtr);
        offset += SP_PORTION_SIZE;
        length -= SP_PORTION_SIZE;
        spDataPtr += SP_PORTION_SIZE;
    }

    if (status !=0) {
        // NVMEM error occurred
        return status;
    }

    if (length != 0) {
        // If length MOD 512 is nonzero, write the remaining bytes.
        status = write(file_id, length, offset, spDataPtr);
    }

    return status;
}

int32_t  cc3000_nvmem::create_entry(uint32_t file_id, uint32_t new_len) {
    uint8_t *ptr;
    uint8_t *args;
    uint16_t retval;

    ptr = _simple_link.get_transmit_buffer();
    args = (ptr + HEADERS_SIZE_CMD);

    // Fill in HCI packet structure
    args = UINT32_TO_STREAM(args, file_id);
    args = UINT32_TO_STREAM(args, new_len);

    // Initiate a HCI command
    _hci.command_send(HCI_CMND_NVMEM_CREATE_ENTRY,ptr, NVMEM_CREATE_PARAMS_LEN);

    _event.simplelink_wait_event(HCI_CMND_NVMEM_CREATE_ENTRY, &retval);

    return(retval);
}

#ifndef CC3000_TINY_DRIVER
uint8_t cc3000_nvmem::read_sp_version(uint8_t* patch_ver) {
    uint8_t *ptr;
    // 1st byte is the status and the rest is the SP version
    uint8_t retBuf[5];

    ptr = _simple_link.get_transmit_buffer();

   // Initiate a HCI command, no args are required
    _hci.command_send(HCI_CMND_READ_SP_VERSION, ptr, 0);
    _event.simplelink_wait_event(HCI_CMND_READ_SP_VERSION, retBuf);

    // package ID
    *patch_ver = retBuf[3];
    // package build number
    *(patch_ver+1) = retBuf[4];

    return(retBuf[0]);
}
#endif

} // mbed_cc3000 namespace
