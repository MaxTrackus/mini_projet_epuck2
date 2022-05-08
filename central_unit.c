#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <leds.h>
#include <selector.h>

#include <central_unit.h>
#include <process_image.h>
#include <move.h>
#include <pi_regulator.h>

static uint8_t currentMode = STOP;
static uint8_t lostLineCounter = 0;

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

		//pursuitMode
		if(currentMode == PURSUIT) {
			set_led(LED3, 1);
			if(get_staticFoundLine() == false) {
				++lostLineCounter;
			} else {
				lostLineCounter = 0;
			}
			if(lostLineCounter == 100) {
				currentMode = STOP;
			}
			if(get_lineWidth() > (uint16_t)(350)) {
				set_led(LED5, 1);
				currentMode = STOP;
			} else {
				set_led(LED5, 0);
			}
		}
		else {
			set_led(LED3, 0);
			set_led(LED5, 0);
		}

		//from idle to analyseMode
		if((get_selector() == 1) && !(currentMode == ALIGN) && !(currentMode == PURSUIT) && !(currentMode == WAIT_MOVING)) {
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
		//from alignementMode to pursuit
		if((currentMode == ALIGN) && (get_regulationCompleted())) {
			currentMode = PURSUIT;
		}
		//from idle to avoid
		if((get_selector() == 8)) {
			currentMode = AVOID;
		}
		//stop and idle
		if((get_selector() == 15)) {
			currentMode = STOP;
		}

		update_currentModeInMove(currentMode);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void central_unit_start(void){
	chThdCreateStatic(waCentralUnit, sizeof(waCentralUnit), NORMALPRIO, CentralUnit, NULL);
}
