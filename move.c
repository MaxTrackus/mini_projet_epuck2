#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <sensors/VL53L0X/VL53L0X.h>

#include <motors.h>

#include <move.h>
#include <pi_regulator.h>
#include <proxi.h>

#define MOTOR_STEP_TO_DEGREES			360 //find other name maybe


#define MAX_MOTOR_SPEED					1100 // [steps/s]

#define DEFAULT_SPEED					200 // [steps/s]
#define SEC2MSEC						1000

// #define POSITION_NOT_REACHED			0
// #define POSITION_REACHED       			1	
// #define MOTOR_SPEED_LIMIT   13 // [cm/s]
#define NSTEP_ONE_TURN      			100 // number of step for 1 turn of the motor
// #define NSTEP_ONE_EL_TURN   4  //number of steps to do 1 electrical turn
// #define NB_OF_PHASES        4  //number of phases of the motors
#define WHEEL_PERIMETER     			130 // [mm]

// static int16_t position_to_reach_right = 0;	    // in [step]
// static int16_t position_to_reach_left = 0;	    // in [step]
// static uint8_t position_right_reached = 0;
// static uint8_t position_left_reached = 0;
// static uint8_t state_motor = 0;		

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
static bool blockMovingRessources = false;
static bool currentlyMoving = false;
static int32_t actionDuration = 0;
// static int distanceToTravel = 0;

//static systime_t stored_time = 0;

static THD_WORKING_AREA(waStepTracker, 1024);
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
		 		 // if (distanceToTravel == 0) {
		 		 // 	distanceToTravel = calculate_distance_from_wall();
		 		 // }
		 		 // //maintain_distance(40, 200);//maintains the robot at 40mm from target with TOF, 200 step/s speed
		 		 // move_straight(DEFAULT_SPEED,distanceToTravel);
		 		 break;
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

        // moving straight
        if(currentlyMoving) {
    		if(chVTGetSystemTime() >= actionDuration) {
    			actionDuration = 0;
    			motor_stop();
    			blockMovingRessources = false;
    			currentlyMoving = false;
    			update_currentModeInMove(STOP);
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

void move_straight(int speed, int distance_in_mm) {

	speed = motor_speed_protection(speed);

	// volatile int duration = ((distance_in_mm*SEC2MSEC)/((speed/NSTEP_ONE_TURN)*WHEEL_PERIMETER)); // SEC2MSEC convert sec. -> msec.

	// volatile systime_t start_time = chVTGetSystemTime();

	// do {
	// 	right_motor_set_speed(speed);
	// 	left_motor_set_speed(speed);
	// } while (chVTGetSystemTime() < (start_time + MS2ST(duration)));

	// motor_stop();
	// update_currentModeInMove(STOP);

	if(!blockMovingRessources) {
		actionDuration = ((distance_in_mm*SEC2MSEC)/((speed/NSTEP_ONE_TURN)*WHEEL_PERIMETER))+chVTGetSystemTime(); // SEC2MSEC convert sec. -> msec.

		blockMovingRessources = true;
		currentlyMoving = true;

		left_motor_set_speed(speed);
		right_motor_set_speed(speed);

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


int motor_speed_protection(int speed) {
	if (speed > MAX_MOTOR_SPEED) {
		speed = MAX_MOTOR_SPEED;
	} else if (speed < -MAX_MOTOR_SPEED) {
		speed = -MAX_MOTOR_SPEED;
	}
	return speed;
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
	if (VL53L0X_get_dist_mm() < distance) {
		right_motor_set_speed(-speed);
		left_motor_set_speed(-speed);
	} else if (VL53L0X_get_dist_mm() > distance) {
		right_motor_set_speed(speed);
		left_motor_set_speed(speed);
	} else {
		motor_stop();
	}
}



// uint8_t motor_position_reached(void)
// {
//     if(state_motor == POSITION_CONTROL && position_right_reached && position_left_reached){
//         return POSITION_REACHED;
//     }else{
//         return POSITION_NOT_REACHED;
//     }
// }

// void motor_set_position(float position_r, float position_l, float speed_r, float speed_l)
// {
// 	//reinit global variable
// 	counter_step_left = 0;
// 	counter_step_right = 0;

//     position_right_reached = 0;
//     position_left_reached = 0;

// 	//Set global variable with position to reach in step
// 	position_to_reach_left = position_l * NSTEP_ONE_TURN / WHEEL_PERIMETER;
// 	position_to_reach_right = -position_r * NSTEP_ONE_TURN / WHEEL_PERIMETER;

// 	motor_set_speed(speed_r, speed_l);

// 	//flag for position control, will erase flag for speed control only
// 	state_motor = POSITION_CONTROL;
// }

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
