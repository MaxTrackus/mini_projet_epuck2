#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <leds.h>
#include <selector.h>
#include <sensors/VL53L0X/VL53L0X.h>

#include <central_unit.h>
#include <process_image.h>
#include <move.h>

#define	OBJECT_DIAMETER					30 // [mm]	

static uint8_t currentMode = STOP;

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
		//from idle to avoid
		if((get_selector() == 8)) {
			currentMode = AVOID;
		}
		//stop and idle
		if((get_selector() == 15)) {
			currentMode = STOP;
		}
		if((get_selector() == 12)) {
			currentMode = SPIN;
		}
		if((get_selector() == 13)) {
			currentMode = WAIT_MOVING;
		}
		if((get_selector() == 10)) {
			currentMode = MAINTAIN_DISTANCE;
		}

		update_currentModeInMove(currentMode);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void central_unit_start(void){
	chThdCreateStatic(waCentralUnit, sizeof(waCentralUnit), NORMALPRIO, CentralUnit, NULL);
}


int calculate_distance_from_wall(void) {

	// if (stored_time == 0) {
		systime_t current_time = chVTGetSystemTime();
	// }

	int distance_to_object = VL53L0X_get_dist_mm();

	do {
		rotate_right(SLOW_SPEED);
	} while (VL53L0X_get_dist_mm() < (distance_to_object+OBJECT_DIAMETER));
	
	motor_stop();
	
	systime_t rotation_duration = chVTGetSystemTime() - current_time;

	volatile int distance_from_wall = VL53L0X_get_dist_mm();

	current_time = chVTGetSystemTime();

	do {
		rotate_left(SLOW_SPEED);
	} while (chVTGetSystemTime() < (current_time + rotation_duration));

	motor_stop();

	return distance_from_wall;

	
	// int half_angle = degrees_between_points/2;

	// int default_speed = 200;

	// rotate_right_in_degrees(default_speed, half_angle);

	// float tof_right_value = VL53L0X_get_dist_mm();

	// rotate_left_in_degrees(default_speed, degrees_between_points);

	// float tof_left_value = VL53L0X_get_dist_mm();

	// float distance_from_wall = tof_right_value + tof_left_value; //some magic calculation

	// return distance_from_wall;

	// ------- OR, we just look at the measurement right on the side of the object...
}