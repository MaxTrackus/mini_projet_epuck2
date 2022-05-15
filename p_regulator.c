#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <p_regulator.h>
#include <main.h> // pour les defines mais c'est tout pas censé normalement
#include <process_image.h>

static regulation_mode currentRegulatorMode = NOTHING;

static uint8_t regulationCompletedCounter = 0;
static bool regulationCompleted = false;

/***************************INTERNAL FUNCTIONS************************************/

/**
* @brief   simple P regulator implementation
*
* @param mesuredValue		mesured value
*
* @param goal				desired value
*
* @return			output command of the controller
*/
int16_t p_regulator(float mesuredValue, float goal){

	float error = 0;
	float speed = 0;

//	static volatile float sum_error = 0;

	error = mesuredValue - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move due to the noise of the camera
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

//	sum_error += error;
//
//	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
//	if(sum_error > MAX_SUM_ERROR){
//		sum_error = MAX_SUM_ERROR;
//	}else if(sum_error < -MAX_SUM_ERROR){
//		sum_error = -MAX_SUM_ERROR;
//	}

	speed = KP * error /*+ KI * sum_error*/;

    return (int16_t)speed;
}

/*************************END INTERNAL FUNCTIONS**********************************/

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;

    while(1){
        time = chVTGetSystemTime();

        switch (currentRegulatorMode) {
        	case NOTHING:
        		regulationCompletedCounter = 0;
        		regulationCompleted = false;
        		break;
			case ALIGN_ROTATION:
				speed = p_regulator((float)get_line_position(), (IMAGE_BUFFER_SIZE/2));
				right_motor_set_speed(-speed);
				left_motor_set_speed(speed);

				if(speed < 5) {
					++regulationCompletedCounter;
				}
				else {
					regulationCompletedCounter = 0;
				}

				if(regulationCompletedCounter == 200) {
					regulationCompleted = true;
				}
				break;
			case PURSUIT_CORRECTION:
				// must do something
				//computes a correction factor to let the robot rotate to be in front of the line
				speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));

				//if the line is nearly in front of the camera, don't rotate
				if(abs(speed_correction) < ROTATION_THRESHOLD){
					speed_correction = 0;
				}

				//applies the speed from the PI regulator and the correction for the rotation
				right_motor_set_speed(200 - ROTATION_COEFF * speed_correction);
				left_motor_set_speed(200 + ROTATION_COEFF * speed_correction);
				break;
			default:
				regulationCompletedCounter = 0;
				regulationCompleted = false;
        }

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

/****************************PUBLIC FUNCTIONS*************************************/

bool get_regulationCompleted(void) {
	return regulationCompleted;
}

void set_currentRegulatorMode(regulation_mode mode) {
	currentRegulatorMode = mode;
}

void p_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}

/**************************END PUBLIC FUNCTIONS***********************************/


