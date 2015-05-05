#include <openbeacon.h>
#include "iap.h"
#include "rfid.h"
#include "usbserial.h"
#include "libnfc.h"
#include "arc.h"

uint8_t main_menu = LIBNFC;
uint8_t submenu = READ;
uint8_t status = 0; /* 1: finished */

/* Set and Handle Interrupts */
void WAKEUP_IRQHandlerPIO2_0(void)
{
	debug_printf("Menu (Pressed 2_0)\n");
	/* Clear pending IRQ */
	LPC_SYSCON->STARTRSRP0CLR = STARTxPRP0_PIO2_0;
	main_menu = CLONE;
	if (0x01 == status) {
	    submenu = WRITE;
    }
}

void WAKEUP_IRQHandlerPIO0_1(void)
{
    debug_printf("Profile (Pressed 0_1)\n");
    LPC_SYSCON->STARTRSRP0CLR = STARTxPRP0_PIO0_1;
    dump_mifare_card();
}

void WAKEUP_IRQHandlerPIO1_0(void)
{
    debug_printf("OK (Pressed 1_0)\n");
    LPC_SYSCON->STARTRSRP0CLR = STARTxPRP0_PIO1_0;
	main_menu = LIBNFC;
}

void ButtonInit(void)
{
	NVIC_EnableIRQ(WAKEUP_PIO2_0_IRQn);

	LPC_SYSCON->STARTAPRP0 = (LPC_SYSCON->STARTAPRP0 & ~STARTxPRP0_PIO2_0);
	LPC_SYSCON->STARTRSRP0CLR = STARTxPRP0_PIO2_0;
	LPC_SYSCON->STARTERP0 |= STARTxPRP0_PIO2_0;

	NVIC_EnableIRQ(WAKEUP_PIO1_0_IRQn);

	LPC_SYSCON->STARTAPRP0 = (LPC_SYSCON->STARTAPRP0 & ~STARTxPRP0_PIO1_0);
	LPC_SYSCON->STARTRSRP0CLR = STARTxPRP0_PIO1_0;
	LPC_SYSCON->STARTERP0 |= STARTxPRP0_PIO1_0;

	NVIC_EnableIRQ(WAKEUP_PIO0_1_IRQn);

	LPC_SYSCON->STARTAPRP0 = (LPC_SYSCON->STARTAPRP0 & ~STARTxPRP0_PIO0_1);
	LPC_SYSCON->STARTRSRP0CLR = STARTxPRP0_PIO0_1;
	LPC_SYSCON->STARTERP0 |= STARTxPRP0_PIO0_1;
}

void LEDInit(void)
{
	GPIOSetDir(LED_PORT, LED_BIT, 1);
	GPIOSetValue(LED_PORT, LED_BIT, LED_OFF);

	GPIOSetDir(0, 7, 1);
	GPIOSetValue(0, 7, LED_OFF);

	GPIOSetDir(1, 10, 1);
	GPIOSetValue(1, 10, LED_OFF);

	GPIOSetDir(1, 1, 1);
	GPIOSetValue(1, 1, LED_OFF);
}

void LED_finished(void)
{
    for (uint8_t i = 0; i < 10; i++) {
        pmu_wait_ms(50);
        GPIOSetValue(0, 7, LED_OFF);
        GPIOSetValue(1, 10, LED_OFF);
        GPIOSetValue(1, 1, LED_OFF);
        pmu_wait_ms(50);
        GPIOSetValue(0, 7, LED_ON);
        GPIOSetValue(1, 10, LED_ON);
        GPIOSetValue(1, 1, LED_ON);
    }
    GPIOSetValue(0, 7, LED_OFF);
    GPIOSetValue(1, 10, LED_OFF);
    GPIOSetValue(1, 1, LED_OFF);
}

int main(void)
{

	/* Initialize GPIO (sets up clock) */
	GPIOInit();

	ButtonInit();

    LEDInit();

	/* UART setup */
	UARTInit(115200, 0);

	/* CDC USB Initialization */
	usb_init();

	/* Init Power Management Routines */
	pmu_init();

	/* Init RFID SPI interface */
	rfid_init();

	debug_printf("OpenPCD2 vTROOPERS15\n");

	/* show LED to signal initialization */
	GPIOSetValue(LED_PORT, LED_BIT, LED_ON);
	pmu_wait_ms(500);

    /* UID */
    TDeviceUID uid;
    debug_printf ("UID:");
    iap_read_uid (&uid);
    rfid_hexdump(&uid, DEVICE_UID_MEMBERS*4);

	debug_printf ("You have passed the Test\n");
	debug_printf ("What Test?\n");
	debug_printf ("... the Debuginterfacetest\n");

    while (1) {
        switch (main_menu) {
            case LIBNFC:
                loop_libnfc_rfid();
                break;
            case CLONE:
                loop_clone_rfid(LED_finished);
                break;
        }
    }
    return 0;
}
