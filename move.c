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
#define MAX_MOTOR_SPEED					1100 // [steps/s]

#define TRACK_WIDTH						51			// [mm]
#define TWENTY_DEGREES					20 			// [degree]
#define	DEG2RAD							(M_PI)/180
#define NSTEP_ONE_TURN      			1000 // number of step for 1 turn of the motor
#define WHEEL_PERIMETER     			130 // [mm]


static bool enableCallsOfFunctionThatUseStepTracker = true;
static move_mode currentModeOfMove = STOP_MOVE;

static bool rotationMappingIsOn = false;
static int rotationMappingValue = 0;

static uint16_t movingSpeed = 0; // devrait ?re signed ?
static uint32_t	right_motor_pos_target = 0;
static uint32_t	left_motor_pos_target = 0;
static bool robotMoving = false; 

//////////////////////////////////////////////////////////////////////// test_max_1205
static int16_t leftMotorCorrectionSpeed = 0;
//////////////////////////////////////////////////////////////////////// test_max_1205


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

			//////////////////////////////////////////////////////////////////////// test_max_1205
			case MOVE_STRAIGHT_WITH_LEFT_MOTOR_CORRECTION:
				left_motor_set_speed(movingSpeed + leftMotorCorrectionSpeed);
				right_motor_set_speed(movingSpeed - leftMotorCorrectionSpeed);
				break;
			//////////////////////////////////////////////////////////////////////// test_max_1205

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
//        chprintf((BaseSequentialStream *)&SD3, "v=%d", rotationMappingValue);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

//////////////////////////////////////////////////////////////////////// test_max_1205
void follow_left_wall_with_speed_correction(int16_t leftSpeedCorrection) {
	currentModeOfMove = MOVE_STRAIGHT_WITH_LEFT_MOTOR_CORRECTION;
	leftMotorCorrectionSpeed = leftSpeedCorrection;
}
//////////////////////////////////////////////////////////////////////// test_max_1205

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

uint32_t get_right_motor_pos(void) {
	return right_motor_get_pos();
}

uint32_t get_left_motor_pos(void) {
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

//degrees>0 -> rotate right; degrees<0 -> rotate left;
//how to know when movement finished.. _
void rotate_in_degrees(int speed, int degrees) {
	if (!robotMoving) {
		reset_motor_pos();
        movingSpeed = motor_speed_protection(speed); //check with issue about moving speed uint...
        if (degrees > 0) {
        	left_motor_pos_target = (uint32_t)(((DEG2RAD) * degrees * TRACK_WIDTH * NSTEP_ONE_TURN)/(2 * WHEEL_PERIMETER));
        	currentModeOfMove = SPIN_RIGHT;
        } else {
        	right_motor_pos_target = (uint32_t)(((DEG2RAD) * abs(degrees) * TRACK_WIDTH * NSTEP_ONE_TURN)/(2 * WHEEL_PERIMETER));
        	currentModeOfMove = SPIN_LEFT;
        }
        robotMoving = true;
	} else { 
		if ((degrees > 0) && (get_left_motor_pos() >= left_motor_pos_target)) {
			currentModeOfMove = STOP_MOVE;
			reset_moving_static();
		} else if ((degrees < 0) && (get_right_motor_pos() >= right_motor_pos_target)) {
			currentModeOfMove = STOP_MOVE;
			reset_moving_static();		
		}
	}
}

void reset_moving_static(void) {
	movingSpeed = 0; // devrait ?re signed ?
	right_motor_pos_target = 0;
	left_motor_pos_target = 0;
	robotMoving = false; 
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

// void obstacles_avoidance_algorithm(void) {

// 	if ((get_calibrated_prox(PROX_FRONT_RIGHT_49) > PROX_DETECTION_THRESHOLD) && (get_calibrated_prox(PROX_FRONT_LEFT_49) > PROX_DETECTION_THRESHOLD)) {
// 		motor_stop();
// //		rotate_left(MAX_SPEED);
// 	} else if ((get_calibrated_prox(PROX_FRONT_LEFT_49) > PROX_DETECTION_THRESHOLD) || (get_calibrated_prox(PROX_FRONT_LEFT_17) > PROX_DETECTION_THRESHOLD)) {
// 		motor_stop();
// 		rotate_right(MAX_SPEED);
// 	} else if ((get_calibrated_prox(PROX_FRONT_RIGHT_49) > PROX_DETECTION_THRESHOLD) || (get_calibrated_prox(PROX_FRONT_RIGHT_17) > PROX_DETECTION_THRESHOLD)) {
// 		motor_stop();
// 		rotate_left(MAX_SPEED);
// 	} else {
// 		motor_stop();
// 		right_motor_set_speed(MAX_SPEED);
//     	left_motor_set_speed(MAX_SPEED);
// 	}

// }

// END -- Added by j.Suchet on 04.05.22
