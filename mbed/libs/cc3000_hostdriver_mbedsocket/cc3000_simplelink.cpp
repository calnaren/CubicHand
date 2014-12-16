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
#include "cc3000_common.h"

namespace mbed_cc3000 {

cc3000_simple_link::cc3000_simple_link() {
    _rx_buffer[CC3000_RX_BUFFER_SIZE - 1] = CC3000_BUFFER_MAGIC_NUMBER;
    _tx_buffer[CC3000_TX_BUFFER_SIZE - 1] = CC3000_BUFFER_MAGIC_NUMBER;
}

cc3000_simple_link::~cc3000_simple_link() {
}

uint8_t cc3000_simple_link::get_data_received_flag() {
    return _data_received_flag;
}

void *cc3000_simple_link::get_func_pointer(FunctionNumber function){
    void *result;
    /* casting to void *, will be casted back once used */
    switch(function) {
        case FW_PATCHES:
            result = (void *)_fFWPatches;
            break;
        case DRIVER_PATCHES:
            result = (void *)_fDriverPatches;
            break;
        case BOOTLOADER_PATCHES:
            result = (void *)_fBootLoaderPatches;
            break;
        default:
            result = 0;
         }
         return result;
}

uint8_t* cc3000_simple_link::get_transmit_buffer() {
    return _tx_buffer;
}

uint8_t* cc3000_simple_link::get_received_buffer() {
    return _rx_buffer;
}

void cc3000_simple_link::set_op_code(uint16_t code) {
    _rx_event_opcode = code;
}

void cc3000_simple_link::set_pending_data(uint16_t value) {
    _rx_data_pending = value;
}

uint16_t cc3000_simple_link::get_pending_data() {
    return _rx_data_pending;
}

void cc3000_simple_link::set_number_free_buffers(uint16_t value) {
    _free_buffers = value;
}

void cc3000_simple_link::set_number_of_released_packets(uint16_t value) {
    _released_packets = value;
}


void cc3000_simple_link::set_tx_complete_signal(bool value) {
    _tx_complete_signal = value;
}

bool cc3000_simple_link::get_tx_complete_signal() {
    return _tx_complete_signal;
}

void cc3000_simple_link::set_data_received_flag(uint8_t value) {
    _data_received_flag = value;
}

uint16_t cc3000_simple_link::get_number_free_buffers() {
    return _free_buffers;
}

uint16_t cc3000_simple_link::get_buffer_length() {
    return _buffer_length;
}

void cc3000_simple_link::set_buffer_length(uint16_t value) {
    _buffer_length = value;
}

uint16_t cc3000_simple_link::get_op_code() {
    return _rx_event_opcode;
}

uint16_t cc3000_simple_link::get_released_packets() {
    return _released_packets;
}

uint16_t cc3000_simple_link::get_sent_packets() {
    return _sent_packets;
}

void cc3000_simple_link::set_sent_packets(uint16_t value) {
    _sent_packets = value;
}

void cc3000_simple_link::set_transmit_error(int32_t value){
    _transmit_data_error = value;
}

int32_t cc3000_simple_link::get_transmit_error(){
    return _transmit_data_error;
}

void cc3000_simple_link::set_buffer_size(uint16_t value) {
    _buffer_size = value;
}

uint16_t cc3000_simple_link::get_buffer_size(void) {
    return _buffer_size;
}

uint8_t *cc3000_simple_link::get_received_data(void) {
    return _received_data;
}

void cc3000_simple_link::set_received_data(uint8_t *pointer) {
    _received_data = pointer;
}

} // mbed_cc3000 namespace
