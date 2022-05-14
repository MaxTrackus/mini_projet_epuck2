// #include "ch.h" //usage ?
// #include "hal.h" //usage ?
// #include <chprintf.h> //usage ?
// #include <usbcfg.h> //usage ?
#include <sensors/proximity.h>

#include <main.h>
#include <proxi.h>

#define NB_PROX_SENSORS		8

static int prox_value[NB_PROX_SENSORS];

bool* get_prox_activation_status(int prox_detection_threshold) {

	static bool prox_activation_status[NB_PROX_SENSORS];

	for (int i = 0; i < NB_PROX_SENSORS; ++i) {
		if (prox_value[i] > prox_detection_threshold) {
			prox_activation_status[i] = true;
		} else {
			prox_activation_status[i] = false;
		}
	}
	return prox_activation_status;
}

int* get_prox_value(void) {
	return prox_value;
}

static THD_WORKING_AREA(waReadProx, 256); //???? How to know the size to allocate ?
static THD_FUNCTION(ReadProx, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1){

    	time = chVTGetSystemTime();

    	for (int i = 0; i < NB_PROX_SENSORS; ++i) {
			prox_value[i] = get_calibrated_prox(i);
		}

    	//100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void read_prox_start(void) {
	chThdCreateStatic(waReadProx, sizeof(waReadProx), NORMALPRIO, ReadProx, NULL);
}
