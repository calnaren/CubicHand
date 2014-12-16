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

cc3000_hci::cc3000_hci(cc3000_spi &spi) : _spi(spi) {

}

cc3000_hci::~cc3000_hci() {

}

uint16_t  cc3000_hci::command_send(uint16_t op_code, uint8_t *buffer, uint8_t length) {
    uint8_t *stream;

    DBG_HCI_CMD("Command Sent : 0x%04X", op_code);

    stream = (buffer + SPI_HEADER_SIZE);

    UINT8_TO_STREAM(stream, HCI_TYPE_CMND);
    stream = UINT16_TO_STREAM(stream, op_code);
    UINT8_TO_STREAM(stream, length);
    //Update the opcode of the event we will be waiting for
    _spi.write(buffer, length + SIMPLE_LINK_HCI_CMND_HEADER_SIZE);
    return(0);
}

uint32_t  cc3000_hci::data_send(uint8_t op_code, uint8_t *args, uint16_t arg_length,
                    uint16_t data_length, const uint8_t *tail, uint16_t tail_length) {
    uint8_t *stream;

    stream = ((args) + SPI_HEADER_SIZE);

    UINT8_TO_STREAM(stream, HCI_TYPE_DATA);
    UINT8_TO_STREAM(stream, op_code);
    UINT8_TO_STREAM(stream, arg_length);
    stream = UINT16_TO_STREAM(stream, arg_length + data_length + tail_length);

    // Send the packet
    _spi.write(args, SIMPLE_LINK_HCI_DATA_HEADER_SIZE + arg_length + data_length + tail_length);

    return 0;
}

void  cc3000_hci::data_command_send(uint16_t op_code, uint8_t *buffer, uint8_t arg_length, uint16_t data_length) {
    uint8_t *stream = (buffer + SPI_HEADER_SIZE);

    UINT8_TO_STREAM(stream, HCI_TYPE_DATA);
    UINT8_TO_STREAM(stream, op_code);
    UINT8_TO_STREAM(stream, arg_length);
    stream = UINT16_TO_STREAM(stream, arg_length + data_length);

    // Send the command
    _spi.write(buffer, arg_length + data_length + SIMPLE_LINK_HCI_DATA_CMND_HEADER_SIZE);

    return;
}

void  cc3000_hci::patch_send(uint8_t op_code, uint8_t *buffer, uint8_t *patch, uint16_t data_length) {
    uint16_t usTransLength;
    uint8_t *stream = (buffer + SPI_HEADER_SIZE);
    UINT8_TO_STREAM(stream, HCI_TYPE_PATCH);
    UINT8_TO_STREAM(stream, op_code);
    stream = UINT16_TO_STREAM(stream, data_length + SIMPLE_LINK_HCI_PATCH_HEADER_SIZE);
    if (data_length <= SL_PATCH_PORTION_SIZE) {
        UINT16_TO_STREAM(stream, data_length);
        stream = UINT16_TO_STREAM(stream, data_length);
        memcpy((buffer + SPI_HEADER_SIZE) + HCI_PATCH_HEADER_SIZE, patch, data_length);
        // Update the opcode of the event we will be waiting for
        _spi.write(buffer, data_length + HCI_PATCH_HEADER_SIZE);
    } else {

        usTransLength = (data_length/SL_PATCH_PORTION_SIZE);
        UINT16_TO_STREAM(stream, data_length + SIMPLE_LINK_HCI_PATCH_HEADER_SIZE + usTransLength*SIMPLE_LINK_HCI_PATCH_HEADER_SIZE);
        stream = UINT16_TO_STREAM(stream, SL_PATCH_PORTION_SIZE);
        memcpy(buffer + SPI_HEADER_SIZE + HCI_PATCH_HEADER_SIZE, patch, SL_PATCH_PORTION_SIZE);
        data_length -= SL_PATCH_PORTION_SIZE;
        patch += SL_PATCH_PORTION_SIZE;

        // Update the opcode of the event we will be waiting for
        _spi.write(buffer, SL_PATCH_PORTION_SIZE + HCI_PATCH_HEADER_SIZE);

        stream = (buffer + SPI_HEADER_SIZE);
        while (data_length) {
            if (data_length <= SL_PATCH_PORTION_SIZE) {
                usTransLength = data_length;
                data_length = 0;
            } else {
                usTransLength = SL_PATCH_PORTION_SIZE;
                data_length -= usTransLength;
            }

            *(uint16_t *)stream = usTransLength;
            memcpy(stream + SIMPLE_LINK_HCI_PATCH_HEADER_SIZE, patch, usTransLength);
            patch += usTransLength;

            // Update the opcode of the event we will be waiting for
            _spi.write((unsigned char *)stream, usTransLength + sizeof(usTransLength));
        }
    }
}

} // mbed_cc3000 namespace
