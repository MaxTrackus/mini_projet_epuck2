#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <leds.h>

void clear_led_with_int(unsigned int led_int_number) {
	
	switch (led_int_number) {
		case 1: 
			set_led(LED1, 0);
			break;
		case 2:
			set_rgb_led(LED2, 0, 0, 0);
			break;
		case 3:
			set_led(LED3, 0);
			break;
		case 4:
			set_rgb_led(LED4, 0, 0, 0);
			break; ///
		case 5: 
			set_led(LED5, 0);
			break;
		case 6:
			set_rgb_led(LED6, 0, 0, 0);
			break;
		case 7:
			set_led(LED7, 0);
			break;
		case 8:
			set_rgb_led(LED8, 0, 0, 0);
			break; ///
		default:
			break;
	}

}

void set_led_with_int(unsigned int led_int_number) {
	
	switch (led_int_number) {
		case 1: 
			set_led(LED1, 1);
			break;
		case 2:
			set_rgb_led(LED2, 10, 0, 0);
			break;
		case 3:
			set_led(LED3, 1);
			break;
		case 4:
			set_rgb_led(LED4, 10, 0, 0);
			break; ///
		case 5: 
			set_led(LED5, 1);
			break;
		case 6:
			set_rgb_led(LED6, 10, 0, 0);
			break;
		case 7:
			set_led(LED7, 1);
			break;
		case 8:
			set_rgb_led(LED8, 10, 0, 0);
			break; ///
		default:
			break;
	}

}