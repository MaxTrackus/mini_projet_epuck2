#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include "sensors/VL53L0X/VL53L0X.h"

#include <motors.h>

#include <move.h>
#include <pi_regulator.h>
#include <proxi.h>

#define MOTOR_STEP_TO_DEGREES			360 //find other name maybe

//List of the different mode, i.e the different tasks that the robot must perform for our application
typedef enum {
	STOP,
	WAIT_MOVING,
	ANALYSE,
	ALIGN,
	AVOID,
	SPIN,
	MAINTAIN_DISTANCE,
} task_mode;

#define MAX_SPIN_ANGLE		360

static int32_t goalLeftMotorPos = 0;
static bool enableCallsOfFunctionThatUseStepTracker = true;
static uint8_t currentModeInMove = STOP;
//static bool rotationMappingIsOn = false;
//static int rotationMappingValue = 0;
static bool currentlySpinning = false;

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
             case WAIT_MOVING:
				 // do nothing
            	 // we wait for the stepTracker to stop the motors
				 break;
             case ANALYSE:
            	 stopMove();
            	 spin_angle_degree(360);
            	 break;
             case ALIGN:
            	 stopMove();
            	 set_enablePiRegulator(true);
            	 break;
        	 case AVOID:
	        	 stopMove();
	        	 avoid_obstacles(200, 100);
	        	 break;
        	 case SPIN:
				 stopMove();
				 spin_angle_degree(180);
				 break;
			 case MAINTAIN_DISTANCE:
		 		 stopMove();
		 		 maintain_distance(40, 200); //maintains the robot at 40mm from target with TOF, 200 step/s speed
             default:
            	 stopMove();
        }

        // rotationMapping
//        if(((currentModeInMove == ANALYSE) || (currentModeInMove == ALIGN)) && !rotationMappingIsOn) {
//        	rotationMappingIsOn = true;
//        	enableCallsOfFunctionThatUseStepTracker = false;
//        	left_motor_set_pos(rotationMappingValue);
//        }
//        if(!(currentModeInMove == ANALYSE) && !(currentModeInMove == ALIGN) && rotationMappingIsOn) {
//        	rotationMappingIsOn = false;
//        	enableCallsOfFunctionThatUseStepTracker = true;
//        	rotationMappingValue = left_motor_get_pos();
//        }

        // stepTracker for spinning
        if(currentlySpinning) {
    		if(left_motor_get_pos() >= goalLeftMotorPos) {
    			goalLeftMotorPos = 0;
    			left_motor_set_speed(0);
    			right_motor_set_speed(0);
    			enableCallsOfFunctionThatUseStepTracker = true;
    			currentlySpinning = false;
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

void update_currentModeInMove(uint8_t mode) {
	currentModeInMove = mode;
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

	float duration = (degrees) * (speed/MOTOR_STEP_TO_DEGREES);
	float start_time = chVTGetSystemTime();
	do {
		rotate_right(speed);
	} while (chVTGetSystemTime() < start_time + MS2ST(duration));

	motor_stop();
}

void rotate_left_in_degrees(int speed, float degrees) {

	float duration = (degrees) * (speed/MOTOR_STEP_TO_DEGREES);
	float start_time = chVTGetSystemTime();
	do {
		rotate_left(speed);
	} while (chVTGetSystemTime() < start_time + MS2ST(duration));

	motor_stop();
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

void maintain_distance(int distance, int speed) {
	if (VL53L0X_get_dist_mm() > distance) {
		right_motor_set_speed(-speed);
		left_motor_set_speed(-speed);
	} else if (VL53L0X_get_dist_mm() > distance) {
		right_motor_set_speed(speed);
		left_motor_set_speed(speed);
	} else {
		motor_stop();
	}
}

float calculate_distance_from_wall(float degrees_between_points) {
	
	int half_angle = degrees_between_points/2;

	rotate_right_in_degrees(half_angle);

	float tof_right_value = VL53L0X_get_dist_mm();

	rotate_left_in_degrees(degrees_between_points);

	float tof_left_value = VL53L0X_get_dist_mm();

	float distance_from_wall; //some magic calculation

	// ------- OR, we just look at the measurement right on the side of the object...
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
