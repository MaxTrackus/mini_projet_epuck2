#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <sensors/proximity.h>
#include <leds.h>
#include <motors.h>
#include <epuck1x/utility/utility.h>

#include <main.h>

#include <proxi.h>
#include <testing.h>


#define MAX_PROX_VALUE					4.095

#define MAX_SPEED						200

#define MOTOR_STEP_TO_DEGREES			2.7

#define PROX_DETECTION_THRESHOLD		100

#define NB_PROX_SENSORS					8

bool* get_prox_activation_status(int prox_detection_threshold) {

	static bool prox_activation_status[NB_PROX_SENSORS];

	for (int i = 0; i < NB_PROX_SENSORS; ++i) {
		if (get_calibrated_prox(i) > prox_detection_threshold) {
			prox_activation_status[i] = true;
		} else {
			prox_activation_status[i] = false;
		}
	}
}

int* get_prox_value(void) {

	static bool prox_value[NB_PROX_SENSORS];

	for (int i = 0; i < NB_PROX_SENSORS; ++i) {
		prox_value[i] = get_calibrated_prox(i);
	}
}

void obstacles_avoidance_algorithm(void) {

	if ((get_calibrated_prox(PROX_FRONT_RIGHT_49) > PROX_DETECTION_THRESHOLD) && (get_calibrated_prox(PROX_FRONT_LEFT_49) > PROX_DETECTION_THRESHOLD)) {
		motor_stop();
//		rotate_left(MAX_SPEED);
	} else if ((get_calibrated_prox(PROX_FRONT_LEFT_49) > PROX_DETECTION_THRESHOLD) || (get_calibrated_prox(PROX_FRONT_LEFT_17) > PROX_DETECTION_THRESHOLD)) {
		motor_stop();
		rotate_right(MAX_SPEED);
	} else if ((get_calibrated_prox(PROX_FRONT_RIGHT_49) > PROX_DETECTION_THRESHOLD) || (get_calibrated_prox(PROX_FRONT_RIGHT_17) > PROX_DETECTION_THRESHOLD)) {
		motor_stop();
		rotate_left(MAX_SPEED);
	} else {
		motor_stop();
		right_motor_set_speed(MAX_SPEED);
    	left_motor_set_speed(MAX_SPEED);
	}

}

void motor_stop(void) {
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

void rotate_left(int speed) {
	left_motor_set_speed(-speed);
	right_motor_set_speed(speed);
}

void rotate_right(int speed) {
	left_motor_set_speed(speed);
	right_motor_set_speed(-speed);
}

void rotate_right_in_degrees(int speed, float degrees) {

	float duration = abs(degrees) / MOTOR_STEP_TO_DEGREES;
	float start_time = chVTGetSystemTime();
	do {
		rotate_right(speed);
	} while (chVTGetSystemTime() < start_time + MS2ST(duration));

	motor_stop();
}

void obstacles_follow_algorithm(void) {

	bool follow_mode = false; 
	bool exit_found = false; 

	if (!exit_found) {
		obstacles_avoidance_algorithm();
	} else {
		//rotate following external center
		//go straight 
		//go back to center
	}
	motor_stop();
}

// static float calculate_rotation_time(float degrees)
// {
// 	return abs(degrees) / ROBOT_ANGULAR_SPEED_IN_DEGREES;
// }

// void motor_rotate_left_in_degrees(float degrees) {
// 	motor_rotate_left();
	
// 	float duration = calculate_rotation_time(degrees);
// 	float start_time = wb_robot_get_time();
// 	do
// 	{
// 		wb_robot_step(TIME_STEP);
// 	} while (wb_robot_get_time() < start_time + duration);
	
// 	motor_stop();
// }


static THD_WORKING_AREA(waReadIR, 256); //???? How to know the size to allocate ?
static THD_FUNCTION(ReadIR, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;

    while(1){

    	time = chVTGetSystemTime();

    	obstacles_avoidance_algorithm();

    	// if (get_calibrated_prox(PROX_FRONT_RIGHT_17) > PROX_DETECTION_THRESHOLD) {
    	// 	speed = 0;
    	// } else {
    	// 	speed = MAX_SPEED;
    	// }

    	// right_motor_set_speed(speed);
    	// left_motor_set_speed(speed);

    	// test_prox_with_leds(PROX_LEFT);

    	//100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void test_prox_with_leds(unsigned int sensor_number) {

	volatile unsigned int prox_right_value = ((float)get_calibrated_prox(sensor_number)/(MAX_PROX_VALUE))*0.008; // Normalize proximity value to int of max value <= 9

	set_led_with_int(prox_right_value);

	wait(84000);

	clear_led_with_int(prox_right_value);

	// wait(84000);
}

void read_IR_start(void) {
	chThdCreateStatic(waReadIR, sizeof(waReadIR), NORMALPRIO, ReadIR, NULL);
}