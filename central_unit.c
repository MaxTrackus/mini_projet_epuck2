// implicit includes
#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

// from libraries
#include <leds.h>
#include <selector.h>

// from other files
#include <central_unit.h>
#include <move.h>
#include <process_image.h>

static uint8_t currentMode = STOP; // to add

static THD_WORKING_AREA(waCentralUnit, 256);
static THD_FUNCTION(CentralUnit, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1){
        time = chVTGetSystemTime();

		//analyseMode
		if(currentMode == ANALYSE) {
			set_body_led(1);
		}
		else {
			set_body_led(0);
		}

		//alignementMode
		if(currentMode == ALIGN) {
			set_front_led(1);
		}
		else {
			set_front_led(0);
		}

		//from idle to analyseMode
		if((get_selector() == 1) && (currentMode == STOP)) {
			currentMode = ANALYSE;
		}
		//from analyseMode to alignementMode
		if((currentMode == ANALYSE) && get_staticFoundLine()) {
			currentMode = ALIGN;
		}
		//from alignementMode to analyseMode
		if((currentMode == ALIGN) && (!(get_staticFoundLine()))) {
			currentMode = ANALYSE;
		}
		//stop and idle
		if((get_selector() == 15)) {
			currentMode = STOP;
		}

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

task_mode get_current_mode(void) {
	return currentMode;
}

void central_unit_start(void){
	chThdCreateStatic(waCentralUnit, sizeof(waCentralUnit), NORMALPRIO, CentralUnit, NULL);
}
