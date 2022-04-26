#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
#include <selector.h>

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

    uint16_t waitBeginningAlignementMode = 0;

    while(1){
        time = chVTGetSystemTime();

        if((get_staticAlignementMode() == 1) && (get_selector() == 1)) {
			//computes the speed to give to the motors
			//distance_cm is modified by the image processing thread
//			speed = pi_regulator(get_distance_cm(), GOAL_DISTANCE);
        	speed = pi_regulator((float)get_line_position(), (IMAGE_BUFFER_SIZE/2)); //added
			//computes a correction factor to let the robot rotate to be in front of the line
//			speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));

			//if the line is nearly in front of the camera, don't rotated
//			if(abs(speed_correction) < ROTATION_THRESHOLD){
//				speed_correction = 0;
//			}

			//applies the speed from the PI regulator and the correction for the rotation
//			right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
//			left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);

        	//wait 5 iterations before starting moving for alignement
        	if(waitBeginningAlignementMode < 200) {
        		++waitBeginningAlignementMode;
//        		chprintf((BaseSequentialStream *)&SD3, "pos=%d speed=%d", get_line_position(), speed);
        		chprintf((BaseSequentialStream *)&SD3, "test");
        	}
        	else {
        		right_motor_set_speed(-speed);
        		left_motor_set_speed(speed);
        	}
        }
        else if(get_selector() != 1) {
        	right_motor_set_speed(0);
        	left_motor_set_speed(0);
        	waitBeginningAlignementMode = 0;
        }

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}



void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
