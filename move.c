#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>

#include <move.h>
#include <pi_regulator.h>
#include <proxi.h>



// constantes.h please
#define MAX_MOTOR_SPEED		1100 // [steps/s]

#define MAX_SPIN_ANGLE		360



static move_mode currentModeOfMove = STOP_MOVE;
static int movingSpeed = 0;

// follow mode
static int16_t leftMotorCorrectionSpeed = 0;

/***************************INTERNAL FUNCTIONS************************************/

/**
 * @brief   Stop motors in move.c and in pi_regulator
 */
void stopMove(void) {
	left_motor_set_speed(0);
	right_motor_set_speed(0);
	set_currentRegulatorMode(NOTHING);
}

/*************************END INTERNAL FUNCTIONS**********************************/

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
				left_motor_set_speed(movingSpeed);
				right_motor_set_speed(-movingSpeed);
				break;

			case SPIN_LEFT:
				left_motor_set_speed(-movingSpeed);
				right_motor_set_speed(movingSpeed);
				break;

			case MOVE_STRAIGHT:
				left_motor_set_speed(movingSpeed);
				right_motor_set_speed(movingSpeed);
				break;

			/**
			* @brief   Made for the align mode of central unit
			*/
			case SPIN_ALIGNEMENT:
				set_currentRegulatorMode(ALIGN_ROTATION);
				break;

			/**
			* @brief   Made for the pursuit mode of central unit
			*/
			case MOVE_STRAIGHT_CORRECT_ALIGNEMENT:
				set_currentRegulatorMode(PURSUIT_CORRECTION);
				break;

			/**
			* @brief   Made for the follow mode of central unit
			*/
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

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

/****************************PUBLIC FUNCTIONS*************************************/

void move_start(void){
	chThdCreateStatic(waStepTracker, sizeof(waStepTracker), NORMALPRIO, StepTracker, NULL);
}

void set_movingSpeed(int speed) {
	if (speed > MAX_MOTOR_SPEED) {
		movingSpeed = MAX_MOTOR_SPEED;
	} else if (speed < -MAX_MOTOR_SPEED) {
		movingSpeed = -MAX_MOTOR_SPEED;
	} else {
		movingSpeed = speed;
	}
}

void update_currentModeOfMove(move_mode mode) {
	currentModeOfMove = mode;
}

void follow_wall_with_speed_correction(int16_t leftSpeedCorrection) {
	currentModeOfMove = MOVE_STRAIGHT_WITH_CORRECTION;
	leftMotorCorrectionSpeed = leftSpeedCorrection;
}

/**************************END PUBLIC FUNCTIONS***********************************/
