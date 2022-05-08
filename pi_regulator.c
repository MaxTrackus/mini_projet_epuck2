#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <pi_regulator.h>
#include <main.h> // pour les defines mais c'est tout pas censé normalement
#include <process_image.h> // besoin pour avoir la line position mais pas censé on devrait changer

static bool enablePiRegulator = false; // to be removed

static regulation_mode currentRegulatorMode = NOTHING;

static uint8_t regulationCompletedCounter = 0;
static bool regulationCompleted = false;

//simple PI regulator implementation
int16_t pi_regulator(float distance, float goal){

	float error = 0;
	float speed = 0;

	static volatile float sum_error = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error /*+ KI * sum_error*/;

	if(speed < 30) {
		speed = 0;
	}

    return (int16_t)speed;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
//    int16_t speed_correction = 0;

    while(1){
        time = chVTGetSystemTime();

        switch (currentRegulatorMode) {
        	case NOTHING:
        		regulationCompletedCounter = 0;
        		regulationCompleted = false;
        		break;
			case ALIGN_ROTATION:
				speed = pi_regulator((float)get_line_position(), (IMAGE_BUFFER_SIZE/2));
				right_motor_set_speed(-speed);
				left_motor_set_speed(speed);

				if(speed < 50) {
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
				break;
			default:
				regulationCompletedCounter = 0;
				regulationCompleted = false;
        }

//        if(enablePiRegulator) {
//        	speed = pi_regulator((float)get_line_position(), (IMAGE_BUFFER_SIZE/2));
//        	right_motor_set_speed(-speed);
//        	left_motor_set_speed(speed);
//
//        	if(speed < 50) {
//        		++regulationCompletedCounter;
//        	}
//        	else {
//        		regulationCompletedCounter = 0;
//        	}
//
//        	if(regulationCompletedCounter == 200) {
//        		regulationCompleted = true;
//        	}
//        }
//        else {
//        	regulationCompletedCounter = 0;
//        	regulationCompleted = false;
//        }

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

bool get_regulationCompleted(void) {
	return regulationCompleted;
}

void set_currentRegulatorMode(regulation_mode mode) {
	currentRegulatorMode = mode;
}

///////////////////////////////////////////////////////////////////to be removed
void set_enablePiRegulator(bool status) {
	enablePiRegulator = status;
}
///////////////////////////////////////////////////////////////////to be removed

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}



