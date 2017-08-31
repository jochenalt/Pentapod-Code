
#include <Arduino.h>
void watchdogReset() {
	/*	noInterrupts();
	WDOG_REFRESH = 0xA602;
	WDOG_REFRESH = 0xB480;
	interrupts();
	*/
}

/* comment out original ResetHandler in C:\Program Files\Arduino\hardware\teensy\cores\teensy3\mk20dx128.c */

void setWatchdogTimeout(int milliseconds) {
	/*
	WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
	WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
	delayMicroseconds(1); // Need to wait a bit..
	WDOG_STCTRLH = 0x0001; // Enable WDG
	WDOG_PRESC = 0; // This sets prescale clock so that the watchdog timer ticks at 1kHZ instead of the default 1kHZ/4 = 200 HZ

	WDOG_TOVALL = (milliseconds/5); // watchdog checks with 200Hz
	WDOG_TOVALH = 0;
	*/
}
