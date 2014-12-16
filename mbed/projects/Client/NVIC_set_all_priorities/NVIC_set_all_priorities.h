#ifndef SET_ALL_PRIO
#define SET_ALL_PRIO

#include "mbed.h"

enum FIRST_LAST_IRQ {
#ifdef TARGET_KL05Z
     first_IRQ_number = SysTick_IRQn,
     last_IRQ_number  = PORTB_IRQn,
#elif defined TARGET_KL25Z
     first_IRQ_number = SysTick_IRQn,
     last_IRQ_number  = PORTD_IRQn,
#elif defined LPC11CXX
     first_IRQ_number = SysTick_IRQn,
     last_IRQ_number  = EINT0_IRQn,
#elif defined LPC11UXX
     first_IRQ_number = SysTick_IRQn,
     last_IRQ_number  = Reserved6_IRQn,
#elif defined LPC11XX
     first_IRQ_number = SysTick_IRQn,
     last_IRQ_number  = EINT0_IRQn,
#elif defined TARGET_LPC13XX
     first_IRQ_number = SysTick_IRQn,
     last_IRQ_number  = Reserved5_IRQn,
#elif defined LPC23XX
     first_IRQ_number = WDT_IRQn,
     last_IRQ_number  = I2S_IRQn,
#elif defined LPC43XX
     first_IRQ_number = SysTick_IRQn,
     last_IRQ_number  = QEI_IRQn,
#elif defined LPC81X
     first_IRQ_number = SysTick_IRQn,
     last_IRQ_number  = PININT7_IRQn,
#elif defined TARGET_LPC176X
     first_IRQ_number = SysTick_IRQn,
     last_IRQ_number  = CANActivity_IRQn,
#elif defined LPC408X
     first_IRQ_number = SysTick_IRQn,
     last_IRQ_number  = CMP1_IRQn,
#elif defined STM32F4XX
     first_IRQ_number = SysTick_IRQn,
     last_IRQ_number  = HASH_RNG_IRQn,
#elif defined STM32F40XX
     first_IRQ_number = SysTick_IRQn,
     last_IRQ_number  = FPU_IRQn,
#elif defined STM32F427X
     first_IRQ_number = SysTick_IRQn,
     last_IRQ_number  = SPI6_IRQn
#endif
};

void NVIC_set_all_irq_priorities(int priority);

#endif // SET_ALL_PRIO
