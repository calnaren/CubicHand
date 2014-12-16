/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "main.h"
#include "mbed.h"

#if (MY_BOARD == WIGO)

#include "NVIC_set_all_priorities/NVIC_set_all_priorities.h"

/**
 *  \brief Wi-Go initialization
 *  \param none
 *  \return none
 */
void init() {
    DigitalOut PWR_EN1(PTB2);
    DigitalOut PWR_EN2(PTB3);

    // Wi-Go set current to 500mA since we're turning on the Wi-Fi
    PWR_EN1 = 0;
    PWR_EN2 = 1;

    NVIC_set_all_irq_priorities(0x3);
    NVIC_SetPriority(SPI0_IRQn, 0x0);     // Wi-Fi SPI interrupt must be higher priority than SysTick
    NVIC_SetPriority(PORTA_IRQn, 0x1);
    NVIC_SetPriority(SysTick_IRQn, 0x2);  // SysTick set to lower priority than Wi-Fi SPI bus interrupt
    PORTA->PCR[16] |= PORT_PCR_ISF_MASK;
    PORTA->ISFR |= (1 << 16);
}

#elif (MY_BOARD == WIFI_DIPCORTEX)

/**
 *  \brief Wifi DipCortex initialization
 *  \param none
 *  \return none
 */
void init() {
    NVIC_SetPriority(SSP1_IRQn, 0x0);
    NVIC_SetPriority(PIN_INT0_IRQn, 0x1);

    // SysTick set to lower priority than Wi-Fi SPI bus interrupt
    NVIC_SetPriority(SysTick_IRQn, 0x2);
}

#else

/**
 *  \brief Place here init routine for your board
 *  \param none
 *  \return none
 */
void init() {

}

#endif
