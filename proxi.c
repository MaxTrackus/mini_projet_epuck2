#include "ch.h"
#include "hal.h"
#include <sensors/proximity.h>

#include <proxi.h>
#include <constants.h>



static uint16_t prox_value[NB_PROX_SENSORS];

/****************************PUBLIC FUNCTIONS*************************************/

bool* get_prox_activation_status(uint16_t prox_detection_threshold) {

	static bool prox_activation_status[NB_PROX_SENSORS];

	for (uint8_t i = 0; i < NB_PROX_SENSORS; ++i) {
		if (prox_value[i] > prox_detection_threshold) {
			prox_activation_status[i] = true;
		} else {
			prox_activation_status[i] = false;
		}
	}
	return prox_activation_status;
}

uint16_t* get_prox_value(void) {
	return prox_value;
}

static THD_WORKING_AREA(waReadProx, 256);
static THD_FUNCTION(ReadProx, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1){

    	time = chVTGetSystemTime();

    	for (uint8_t i = 0; i < NB_PROX_SENSORS; ++i) {
			prox_value[i] = get_calibrated_prox(i);
		}

    	//100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void read_prox_start(void) {
	chThdCreateStatic(waReadProx, sizeof(waReadProx), NORMALPRIO, ReadProx, NULL);
}

/**************************END PUBLIC FUNCTIONS***********************************/
