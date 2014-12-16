/**************************************************************************************************
 *****                                                                                        *****
 *****  Name: NVIC_set_all_priorities.cpp                                                     *****
 *****  Date: 18/09/2013                                                                      *****
 *****  Auth: Frank Vannieuwkerke                                                             *****
 *****  Func: library for changing all IRQ priorities at once                                 *****
 *****        Supported targets : see enum declaration in .h file                             *****
 *****                                                                                        *****
 **************************************************************************************************/
 
#include "NVIC_set_all_priorities.h"

void NVIC_set_all_irq_priorities(int priority)
{
    int irqnum;
    for(irqnum = first_IRQ_number ; irqnum < last_IRQ_number + 1 ; irqnum++)
        NVIC_SetPriority((IRQn_Type)irqnum, priority);
}
