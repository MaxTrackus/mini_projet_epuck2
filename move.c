#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>

#include <move.h>
#include <pi_regulator.h>
#include <proxi.h>

#define MAX_SPIN_ANGLE		360
#define MAX_MOTOR_SPEED		1100 // [steps/s]

// global use
static bool enableCallsOfFunctionThatUseStepTracker = true;
static move_mode currentModeOfMove = STOP_MOVE;
static int movingSpeed = 0; // devrait ?re signed ?

// rotation mapping
static bool rotationMappingIsOn = false;
static int rotationMappingValue = 0;

// follow mode
static int16_t leftMotorCorrectionSpeed = 0;


static THD_WORKING_AREA(waStepTracker, 256);
static THD_FUNCTION(StepTracker, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1){
        time = chVTGetSystemTime();

        switch (currentModeOfMove) {
			case STOP_MOVE:
				stopMove();
				break;

			case SPIN_RIGHT:
				rotate_right(movingSpeed);
				break;

			case SPIN_LEFT:
				rotate_left(movingSpeed);
				break;

			case MOVE_STRAIGHT:
				move_straight(movingSpeed);
				break;

			case SPIN_ALIGNEMENT:
				set_currentRegulatorMode(ALIGN_ROTATION);
				break;

			case MOVE_STRAIGHT_CORRECT_ALIGNEMENT:
				set_currentRegulatorMode(PURSUIT_CORRECTION);
				break;

			case MOVE_STRAIGHT_WITH_CORRECTION:
				left_motor_set_speed(movingSpeed + leftMotorCorrectionSpeed);
				right_motor_set_speed(movingSpeed - leftMotorCorrectionSpeed);
				break;

			default:
			 stopMove();
        }

        //disable PI regulator when the mode doesn't need it
        if((!(currentModeOfMove == SPIN_ALIGNEMENT)) && (!(currentModeOfMove == MOVE_STRAIGHT_CORRECT_ALIGNEMENT))) {
        	set_currentRegulatorMode(NOTHING);
        }

        if(rotationMappingIsOn) {
        	rotationMappingValue = left_motor_get_pos();
        }

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void follow_left_wall_with_speed_correction(int16_t leftSpeedCorrection) {
	currentModeOfMove = MOVE_STRAIGHT_WITH_CORRECTION;
	leftMotorCorrectionSpeed = leftSpeedCorrection;
}

void set_rotationMappingIsOn(bool status) {
	if(rotationMappingIsOn && !status) {
		enableCallsOfFunctionThatUseStepTracker = true;
		left_motor_set_pos(0);
	}
	if(!rotationMappingIsOn && status) {
		enableCallsOfFunctionThatUseStepTracker = false;
		left_motor_set_pos(0);
	}
	rotationMappingIsOn = status;
}

int get_rotationMappingValue(void) {
	return rotationMappingValue;
}

void set_movingSpeed(int speed) {
	movingSpeed = motor_speed_protection(speed);
}

void reset_motor_pos(void) {
	left_motor_set_pos(0);
	right_motor_set_pos(0);
}

int32_t get_right_motor_pos(void) {
	return right_motor_get_pos();
}

int32_t get_left_motor_pos(void) {
	return left_motor_get_pos();
}

void move_straight(int speed) {
	left_motor_set_speed(speed);
	right_motor_set_speed(speed);
}

int motor_speed_protection(int speed) {
	if (speed > MAX_MOTOR_SPEED) {
		speed = MAX_MOTOR_SPEED;
	} else if (speed < -MAX_MOTOR_SPEED) {
		speed = -MAX_MOTOR_SPEED;
	}
	return speed;
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
	enableCallsOfFunctionThatUseStepTracker = true;
	set_currentRegulatorMode(NOTHING);
}

void update_currentModeOfMove(move_mode mode) {
	currentModeOfMove = mode;
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

void avoid_obstacles(int speed, int prox_detection_threshold) {

	bool *prox_status_table = get_prox_activation_status(prox_detection_threshold);

	if (prox_status_table[PROX_FRONT_LEFT_49] == true) {
		rotate_right(speed);
	} else if (prox_status_table[PROX_FRONT_RIGHT_49] == true) {
		rotate_left(speed);
	} else {
		right_motor_set_speed(speed);
		left_motor_set_speed(speed);
	}
}
