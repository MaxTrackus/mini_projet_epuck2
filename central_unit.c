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

//static uint8_t currentMode = STOP;// to add
static bool analyseMode = false;    //to remove
static bool alignementMode = false; // to remove


static THD_WORKING_AREA(waCentralUnit, 256);
static THD_FUNCTION(CentralUnit, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1){
        time = chVTGetSystemTime();

		//analyseMode
		if(analyseMode) {
			set_body_led(1);
		}
		else {
			set_body_led(0);
		}

		//alignementMode
		if(alignementMode) {
			set_front_led(1);
		}
		else {
			set_front_led(0);
		}

		//from idle to analyseMode
		if((get_selector() == 1) && (!analyseMode) && (!alignementMode)) {
			analyseMode = true;
			spin_angle_degree(360);
		}
		//from analyseMode to alignementMode
		if(analyseMode && get_staticFoundLine()) {
			analyseMode = false;
			alignementMode = true;
			stopMove();
		}
		//from alignementMode to analyseMode
		if(alignementMode && (!(get_staticFoundLine()))) {
			analyseMode = true;
			alignementMode = false;
			stopMove();
			spin_angle_degree(360);
		}
		//stop and idle
		if((get_selector() == 15)) {
			analyseMode = false;
			alignementMode = false;
			stopMove();
		}

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

//////////////////////////////////////////////////////////to remove
bool get_alignementMode(void) {
	return alignementMode;
}
//////////////////////////////////////////////////////////to remove

//////////////////////////////////////////////////////////to add
//task_mode get_current_mode(void) {
//
//}
//////////////////////////////////////////////////////////to add

void central_unit_start(void){
	chThdCreateStatic(waCentralUnit, sizeof(waCentralUnit), NORMALPRIO, CentralUnit, NULL);
}
