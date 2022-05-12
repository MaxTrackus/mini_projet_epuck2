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


static int32_t goalLeftMotorPos = 0;
static bool enableCallsOfFunctionThatUseStepTracker = true;
static uint8_t currentModeInMove = STOP;

//static bool rotationMappingIsOn = false;
//static int rotationMappingValue = 0;

static bool currentlySpinning = false;

static uint16_t movingSpeed = 0;

static THD_WORKING_AREA(waStepTracker, 256);
static THD_FUNCTION(StepTracker, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1){
        time = chVTGetSystemTime();

        switch (currentModeInMove)
        {
             case STOP:
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
             default:
            	 stopMove();
        }

//         if((!(currentModeInMove == ALIGN)) && (!(currentModeInMove == PURSUIT))) {
//         	set_currentRegulatorMode(NOTHING);
//         }

//         // rotationMapping
//         if(((currentModeInMove == ANALYSE) || (currentModeInMove == ALIGN)) && !rotationMappingIsOn) {
//         	rotationMappingIsOn = true;
//         	enableCallsOfFunctionThatUseStepTracker = false;
//         	left_motor_set_pos(0);
//         }
//         if(!(currentModeInMove == ANALYSE) && !(currentModeInMove == ALIGN) && rotationMappingIsOn) {
//         	rotationMappingIsOn = false;
//         	enableCallsOfFunctionThatUseStepTracker = true;
//         	rotationMappingValue = rotationMappingValue + left_motor_get_pos();
//         }

// //        chprintf((BaseSequentialStream *)&SD3, "v=%d", rotationMappingValue);

//         // stepTracker for spinning
//         if(currentlySpinning) {
//     		if(left_motor_get_pos() >= goalLeftMotorPos) {
//     			goalLeftMotorPos = 0;
//     			left_motor_set_speed(0);
//     			right_motor_set_speed(0);
//     			enableCallsOfFunctionThatUseStepTracker = true;
//     			currentlySpinning = false;
//     		}
//         }

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
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

void set_movingSpeed(int speed) {
	movingSpeed = motor_speed_protection(speed);
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

void spin_angle_degree(uint16_t angle_in_degree) {
	uint16_t angle = angle_in_degree*1.95;
	if(angle_in_degree > MAX_SPIN_ANGLE) {
		angle = 360;
	}

	// unit of positions are steps
	if(enableCallsOfFunctionThatUseStepTracker) {
		left_motor_set_pos(0);

		goalLeftMotorPos = (int32_t)((25/9)*angle);
		enableCallsOfFunctionThatUseStepTracker = false;
		currentlySpinning = true;

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
	enableCallsOfFunctionThatUseStepTracker = true;
	set_enablePiRegulator(false);
}

void update_currentModeInMove(uint8_t moving_mode) {
	currentModeInMove = moving_mode;
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

// void rotate_right_in_degrees(int speed, float degrees) {

// //	float duration = abs(degrees) / MOTOR_STEP_TO_DEGREES;
// //	float start_time = chVTGetSystemTime();
// //	do {
// //		rotate_right(speed);
// //	} while (chVTGetSystemTime() < start_time + MS2ST(duration));
// //
// //	motor_stop();
// }

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
