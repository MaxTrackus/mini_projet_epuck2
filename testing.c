#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <sensors/proximity.h>
#include <leds.h>
#include <motors.h>
#include <epuck1x/utility/utility.h>
#include <selector.h>

#include <sensors.h>


#define NUM_LEDS						8

#define MAX_PROX_VALUE					4.095

#define MAX_SPEED						400

#define MOTOR_STEP_TO_DEGREES			2.7

#define PROX_DETECTION_THRESHOLD		100

// GPIO_C id IR sensor
#define PROX_FRONT_RIGHT_17 			0 // IR 1
#define PROX_FRONT_LEFT_17				7 // IR 8
#define PROX_FRONT_RIGHT_49 			1 // IR 2
#define PROX_FRONT_LEFT_49				6 // IR 7
#define PROX_RIGHT						2 // IR 3
#define PROX_BACK_RIGHT 				3 // IR 4


// GPIO_B id IR sensor
#define PROX_LEFT						5 // IR 6
#define PROX_BACK_LEFT					4 // IR 5


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

void test_prox_with_leds(unsigned int sensor_number) {

	volatile unsigned int prox_right_value = ((float)get_calibrated_prox(sensor_number)/(MAX_PROX_VALUE))*0.008; // Normalize proximity value to int of max value <= 9

	set_led_with_int(prox_right_value);

	wait(84000);

	clear_led_with_int(prox_right_value);

	// wait(84000);
}
