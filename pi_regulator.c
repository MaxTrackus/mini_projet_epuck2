#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <pi_regulator.h>
#include <main.h> // pour les defines mais c'est tout pas censé normalement
#include <process_image.h> // besoin pour avoir la line position mais pas censé on devrait changer

static bool enablePiRegulator = false;

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

        if(enablePiRegulator) {
        	speed = pi_regulator((float)get_line_position(), (IMAGE_BUFFER_SIZE/2));
        	right_motor_set_speed(-speed);
        	left_motor_set_speed(speed);
        }

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void set_enablePiRegulator(bool status) {
	enablePiRegulator = status;
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}



