/*
 * File Name     : irq.c
 * Purpose       :
 * Creation Date : 03-06-2015
 * Last Modified : Wed 03 Jun 2015 11:10:45 PM CEST
 * Created By    :
 */

#define _XOPEN_SOURCE 9001
#include <openbeacon.h>
#include "LPC13xx.h"
#include "arc.h"
#include "libnfc.h"

uint8_t main_menu;
uint8_t mode;
uint32_t clock_1s;

/* Set and Handle Interrupts */
void SysTick_Handler (void)
{
    static uint8_t ticks = 0;
    if (ticks++ >= 99) {
        ticks = 0;
        clock_1s += 1;
	    debug_printf("Clk %02d:%02d:%02d\n", clock_1s / 3600, (clock_1s / 60) % 60, clock_1s % 60);

	    if ((CLONE == main_menu) && (WRITE == mode)) {
            GPIOSetValue(0, 7, LED_ON);
            GPIOSetValue(1, 10, LED_OFF);
        } else if ((CLONE == main_menu) && (READ == mode)) {
            GPIOSetValue(1, 10, LED_ON);
            GPIOSetValue(0, 7, LED_OFF);
        } else {
            GPIOSetValue(0, 7, LED_ON);
            GPIOSetValue(1, 10, LED_ON);
        }
    }
}

void WAKEUP_IRQHandlerPIO2_0(void)
{
	debug_printf("READ (Pressed 2_0)\n");
	/* Clear pending IRQ */
	LPC_SYSCON->STARTRSRP0CLR = STARTxPRP0_PIO2_0;
	main_menu = CLONE;
    mode = READ;
}

void WAKEUP_IRQHandlerPIO0_1(void)
{
    debug_printf("DUMP (Pressed 0_1)\n");
    LPC_SYSCON->STARTRSRP0CLR = STARTxPRP0_PIO0_1;
    dump_mifare_card();
}

void WAKEUP_IRQHandlerPIO1_0(void)
{
    debug_printf("WRITE (Pressed 1_0)\n");
    LPC_SYSCON->STARTRSRP0CLR = STARTxPRP0_PIO1_0;
	main_menu = CLONE;
    mode = WRITE;
}

