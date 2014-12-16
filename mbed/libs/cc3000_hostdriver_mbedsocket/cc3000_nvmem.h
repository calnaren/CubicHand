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
#ifndef CC3000_NVMEM_H
#define CC3000_NVMEM_H

#define NVMEM_READ_PARAMS_LEN       (12)
#define NVMEM_CREATE_PARAMS_LEN     (8)
#define NVMEM_WRITE_PARAMS_LEN      (16)


/****************************************************************************
**
**    Definitions for File IDs
**
****************************************************************************/
/* --------------------------------------------------------- EEPROM FAT table ---------------------------------------------------------

 File ID                            Offset      File Size   Used Size   Parameter
 #  ID                              address     (bytes)     (bytes)
 --------------------------------------------------------------------------------------------------------------------------------------
 0  NVMEM_NVS_FILEID                0x50        0x1A0       0x1A        RF Calibration results table(generated automatically by TX Bip)
 1  NVMEM_NVS_SHADOW_FILEID         0x1F0       0x1A0       0x1A        NVS Shadow
 2  NVMEM_WLAN_CONFIG_FILEID        0x390       0x1000      0x64        WLAN configuration
 3  NVMEM_WLAN_CONFIG_SHADOW_FILEID 0x1390      0x1000      0x64        WLAN configuration shadow
 4  NVMEM_WLAN_DRIVER_SP_FILEID     0x2390      0x2000      variable    WLAN Driver ROM Patches
 5  NVMEM_WLAN_FW_SP_FILEID         0x4390      0x2000      variable    WLAN FW Patches
 6  NVMEM_MAC_FILEID                0x6390      0x10        0x10        6 bytes of MAC address
 7  NVMEM_FRONTEND_VARS_FILEID      0x63A0      0x10        0x10        Frontend Vars
 8  NVMEM_IP_CONFIG_FILEID          0x63B0      0x40        0x40        IP configuration
 9  NVMEM_IP_CONFIG_SHADOW_FILEID   0x63F0      0x40        0x40        IP configuration shadow
10  NVMEM_BOOTLOADER_SP_FILEID      0x6430      0x400       variable    Bootloader Patches
11  NVMEM_RM_FILEID                 0x6830      0x200       0x7F        Radio parameters
12  NVMEM_AES128_KEY_FILEID         0x6A30      0x10        0x10        AES128 key file
13  NVMEM_SHARED_MEM_FILEID         0x6A40      0x50        0x44        Host-CC3000 shared memory file
14  NVMEM_USER_FILE_1_FILEID        0x6A90      variable    variable    1st user file
15  NVMEM_USER_FILE_2_FILEID        variable    variable    variable    2nd user file
*/
/* NVMEM file ID - system files*/
#define NVMEM_NVS_FILEID                             (0)
#define NVMEM_NVS_SHADOW_FILEID                      (1)
#define NVMEM_WLAN_CONFIG_FILEID                     (2)
#define NVMEM_WLAN_CONFIG_SHADOW_FILEID              (3)
#define NVMEM_WLAN_DRIVER_SP_FILEID                  (4)
#define NVMEM_WLAN_FW_SP_FILEID                      (5)
#define NVMEM_MAC_FILEID                             (6)
#define NVMEM_FRONTEND_VARS_FILEID                   (7)
#define NVMEM_IP_CONFIG_FILEID                       (8)
#define NVMEM_IP_CONFIG_SHADOW_FILEID                (9)
#define NVMEM_BOOTLOADER_SP_FILEID                   (10)
#define NVMEM_RM_FILEID                              (11)

/* NVMEM file ID - user files*/
#define NVMEM_AES128_KEY_FILEID                      (12)
#define NVMEM_SHARED_MEM_FILEID                      (13)
#define NVMEM_USER_FILE_1_FILEID                     (14)
#define NVMEM_USER_FILE_2_FILEID                     (15)

/*  max entry in order to invalid nvmem              */
#define NVMEM_MAX_ENTRY                              (16)


#endif
