#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <move.h>
#include <leds.h>

#define MAX_SPIN_ANGLE				360

#define MOTOR_STEP_TO_DEGREES		2.7

static int32_t goalLeftMotorPos = 0;

static bool enableChangeOfLeftMotorPos = true;

static THD_WORKING_AREA(waStepTracker, 256);
static THD_FUNCTION(StepTracker, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1){
        time = chVTGetSystemTime();

        if(!enableChangeOfLeftMotorPos) {
    		if(left_motor_get_pos() >= goalLeftMotorPos) {
    			goalLeftMotorPos = 0;
    			left_motor_set_speed(0);
    			right_motor_set_speed(0);
    			enableChangeOfLeftMotorPos = true;
    		}
        }

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void spin_angle_degree(uint16_t angle_in_degree) {
	uint16_t angle = angle_in_degree*1.95;
	if(angle_in_degree > MAX_SPIN_ANGLE) {
		angle = 360;
	}

	// unit of positions are steps
	if(enableChangeOfLeftMotorPos) {
		left_motor_set_pos(0);

		goalLeftMotorPos = (int32_t)((25/9)*angle);
		enableChangeOfLeftMotorPos = false;

		left_motor_set_speed(150);
		right_motor_set_speed(-150);

	}
}

void move_start(void){
	chThdCreateStatic(waStepTracker, sizeof(waStepTracker), NORMALPRIO, StepTracker, NULL);
}

bool toggle_boolean(bool x) {
	if(x) {
		return false;
	}
	else {
		return true;
	}
}

void stopMove(void) {
	left_motor_set_speed(0);
	right_motor_set_speed(0);
	enableChangeOfLeftMotorPos = true; /////DISCUSS WITH MAX
}

// BEGIN -- Added by j.Suchet on 04.05.22

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

// END -- Added by j.Suchet on 04.05.22