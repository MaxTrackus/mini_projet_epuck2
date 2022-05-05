#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <move.h>
#include <leds.h>
#include <pi_regulator.h>
#include <central_unit.h>

#define MAX_SPIN_ANGLE		360

static int32_t goalLeftMotorPos = 0;

static bool enableChangeOfLeftMotorPos = true;

static THD_WORKING_AREA(waStepTracker, 256);
static THD_FUNCTION(StepTracker, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1){
        time = chVTGetSystemTime();

        ////////////////////////////////////////////////try adding a switch
        switch (get_current_mode())
        {
             case STOP:
            	 stopMove();
            	 break;
             case ANALYSE:
            	 spin_angle_degree(360);
            	 break;
             case ALIGN:
            	 set_enablePiRegulator(true);
            	 break;
             default:
            	 stopMove();
        }

        ////////////////////////////////////////////////try adding a switch

//        if(!enableChangeOfLeftMotorPos) {
//    		if(left_motor_get_pos() >= goalLeftMotorPos) {
//    			goalLeftMotorPos = 0;
//    			left_motor_set_speed(0);
//    			right_motor_set_speed(0);
//    			enableChangeOfLeftMotorPos = true;
//    		}
//        }

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
	enableChangeOfLeftMotorPos = true;
	set_enablePiRegulator(false);
}
