#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <leds.h>
#include <selector.h>
#include <sensors/VL53L0X/VL53L0X.h>

#include <central_unit.h>
#include <process_image.h>
#include <move.h>
#include <pi_regulator.h>
#include <proxi.h>

#define DEFAULT_SPEED					200 		// [steps/s]
#define SLOW_SPEED						50 			// [steps/s]
#define	OBJECT_DIAMETER					30 			// [mm]	
#define ERROR_MARGIN					75 			// [mm]
#define WALL_CLEARANCE					0 			// [mm]
#define SEC2MSEC						1000 		//1000
#define MAX_MOTOR_SPEED					1100 		// [steps/s]
#define NSTEP_ONE_TURN      			100 		// number of step for 1 turn of the motor
#define WHEEL_PERIMETER     			130 		// [mm]

#define QUARTER_TURN					90
#define MOTOR_STEP_TO_DEGREES			360 		//find other name maybe
#define	PROX_DETECTION_THRESHOLD		150 

#define TRACK_WIDTH						51			// [mm]
#define TWENTY_DEGREES					20 			// [degree]
#define	DEG2RAD							(M_PI)/180

typedef enum {
	IDLE,
	ANALYSE,
	ALIGN,
	AVOID,
	PURSUIT,
	MEASURE,
	PUSH,
	FOLLOW,
	EXIT,
	RECENTER
} program_mode;

static volatile uint8_t currentProgramMode = MEASURE;
static volatile systime_t currentTime = 0;
static volatile uint32_t actionTime = 0;
static volatile uint16_t distanceToTravel = 0;
static volatile bool wallFound = false;
static bool optimizedExitOnLeft = true; //Jeremy's version of Max's flag given after rotation mapping
static uint8_t exitProx = PROX_RIGHT;

static volatile uint32_t motor_right_pos_target = 0;
static volatile uint32_t motor_left_pos_target = 0;



//static uint8_t lostLineCounter = 0;

static THD_WORKING_AREA(waCentralUnit, 256);
static THD_FUNCTION(CentralUnit, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    volatile systime_t time;

    while(1){
        time = chVTGetSystemTime();

        //used for testing purposes 
        //set_mode_with_selector();

        switch (currentProgramMode) {
        	case IDLE:
        		break;
        	case MEASURE:
        		if (distanceToTravel == 0) {
        			distanceToTravel = VL53L0X_get_dist_mm(); //This saves the value to the object here
        			set_movingSpeed(SLOW_SPEED);
        			reset_motor_pos();
        			set_right_rotation_in_degrees(TWENTY_DEGREES);
        			update_currentModeInMove(SPIN_RIGHT);
        		}
        		
        		volatile uint32_t motor_pos = get_left_motor_pos();

        		if ((motor_pos >= motor_left_pos_target) && (wallFound == false)) {
        			distanceToTravel = VL53L0X_get_dist_mm() - distanceToTravel - WALL_CLEARANCE - OBJECT_DIAMETER;
        			reset_motor_pos();
        			set_left_rotation_in_degrees(TWENTY_DEGREES);
        			update_currentModeInMove(SPIN_LEFT);
        			wallFound = true;
        		}

        		//gets back to original position
        		if ((get_right_motor_pos() >= motor_right_pos_target) && (wallFound == true)) {
        			update_currentModeInMove(STOP);
        			currentProgramMode = IDLE;
        			wallFound = false;
        			currentTime = 0;
        			actionTime = 0;
        			//VL53L0X_stop();
        		}
        		break;
        	case PUSH:
        		if ((actionTime == 0) && (distanceToTravel != 0)) {
        			actionTime = ((distanceToTravel*SEC2MSEC)/(((DEFAULT_SPEED)/NSTEP_ONE_TURN)*WHEEL_PERIMETER)); // 1000 convert sec. -> msec.
        			currentTime = chVTGetSystemTime();
        			set_movingSpeed(DEFAULT_SPEED);
        			update_currentModeInMove(MOVE_STRAIGHT);
        		}

        		if (time >= (currentTime + MS2ST(actionTime))) {
        			update_currentModeInMove(STOP);
        			currentProgramMode = FOLLOW;
        			distanceToTravel = 0;
        			actionTime = 0;
        			currentTime = 0;

        		}
        		break;
        	case FOLLOW:
        		if (wallFound == false) {
	        		if (actionTime == 0) {
	        			//0.6 motor turn for 360 degree turn
	        			actionTime = 3020*1.1;//(QUARTER_TURN * DEFAULT_SPEED * SEC2MSEC)/(MOTOR_STEP_TO_DEGREES);
	        			set_movingSpeed(DEFAULT_SPEED);
	        			currentTime = chVTGetSystemTime();
	        			if (optimizedExitOnLeft) {
	        				update_currentModeInMove(SPIN_LEFT); //depends on the flag given by MAX
	        				exitProx = PROX_RIGHT;
	        			} else {
	        				update_currentModeInMove(SPIN_RIGHT); //depends on the flag given by MAX
	        				exitProx = PROX_LEFT;

	        			}
	        			
	        		}

	        		if (chVTGetSystemTime() >= (currentTime + MS2ST(actionTime))) {
	        			update_currentModeInMove(STOP);
	        			//currentProgramMode = IDLE;
	        			actionTime = 0;
	        			currentTime = 0;
	        			wallFound = true;
	        		}
	        	} else {
	        		bool *prox_status_table = get_prox_activation_status(PROX_DETECTION_THRESHOLD);
	        		int *prox_values = get_prox_value();

	        		set_movingSpeed(DEFAULT_SPEED);
	        		if (prox_status_table[PROX_FRONT_LEFT_49] == true) {
						update_currentModeInMove(SPIN_RIGHT);
					} else if (prox_status_table[PROX_FRONT_RIGHT_49] == true) {
						update_currentModeInMove(SPIN_LEFT);
					}
					else if (prox_values[exitProx] <= 10) {
						update_currentModeInMove(STOP);
						currentProgramMode = IDLE;
					}
					else {
						update_currentModeInMove(MOVE_STRAIGHT);
					}

	        	}

        		break;
        	case EXIT:
        		break;
        	case RECENTER:
        		break;
        	default:
        		currentProgramMode = IDLE;
        		break;
        }


		//100Hz
        chThdSleepUntil(MS2ST(10));

// 		//analyseMode
// 		if(currentMode == ANALYSE) {
// 			set_body_led(1);
// 		}
// 		else {
// 			set_body_led(0);
// 		}

// //		//alignementMode
// //		if(currentMode == ALIGN) {
// //			set_front_led(1);
// //		}
// //		else {
// //			set_front_led(0);
// //		}

// 		//pursuitMode
// 		if(currentMode == PURSUIT) {
// 			set_led(LED3, 1);
// 			if(get_staticFoundLine() == false) {
// 				++lostLineCounter;
// 			} else {
// 				lostLineCounter = 0;
// 			}
// 			if(lostLineCounter == 100) {
// 				currentMode = STOP;
// 			}
// 			if(get_lineWidth() > (uint16_t)(400)) {
// 				set_led(LED5, 1);
// 				currentMode = STOP;
// 			} else {
// 				set_led(LED5, 0);
// 			}
// 		}
// 		else {
// 			set_led(LED3, 0);
// 			set_led(LED5, 0);
// 		}

// 		//from idle to analyseMode
// 		if((get_selector() == 1) && !(currentMode == ALIGN) && !(currentMode == PURSUIT) && !(currentMode == WAIT_MOVING)) {
// 			currentMode = ANALYSE;
// 		}
// 		//from analyseMode to alignementMode
// 		if((currentMode == ANALYSE) && get_staticFoundLine()) {
// 			currentMode = ALIGN;
// 		}
// 		//from alignementMode to analyseMode
// 		if((currentMode == ALIGN) && (!(get_staticFoundLine()))) {
// 			currentMode = ANALYSE;
// 		}
// 		//from alignementMode to pursuit
// 		if((currentMode == ALIGN) && (get_regulationCompleted())) {
// 			currentMode = PURSUIT;
// 		}
// 		//from idle to avoid
// 		if((get_selector() == 8)) {
// 			currentMode = AVOID;
// 		}
// 		//stop and idle
// 		if((get_selector() == 15)) {
// 			currentMode = STOP;
// 		}

// 		update_currentModeInMove(currentMode);

        
    }
}

//This is a function used for testing -> to remove for final 
void set_mode_with_selector(void) {
	switch (get_selector()) {
		case 0:
			currentProgramMode = IDLE;
			break;
		case 1:
			currentProgramMode = ANALYSE;
			break;
		case 2:
			currentProgramMode = ALIGN;
			break;
		case 3:
			currentProgramMode = PURSUIT;
			break;
		case 4:
			currentProgramMode = MEASURE;
			break;
		case 5:
			currentProgramMode = PUSH;
			break;
		case 6:
			currentProgramMode = FOLLOW;
			break;
		case 7:
			currentProgramMode = EXIT;
			break;
		case 8:
			currentProgramMode = RECENTER;
			break;
		default:
			currentProgramMode = IDLE;
			break;
	}
}

void set_right_rotation_in_degrees(uint16_t angle_in_degree) {
//	motor_right_pos_target = -((DEG2RAD) * angle_in_degree * TRACK_WIDTH * NSTEP_ONE_TURN)/(2 * WHEEL_PERIMETER);
	motor_left_pos_target = (uint32_t)(((DEG2RAD) * angle_in_degree * TRACK_WIDTH * NSTEP_ONE_TURN)/(2 * WHEEL_PERIMETER));
}

void set_left_rotation_in_degrees(uint16_t angle_in_degree) {
	motor_right_pos_target = ((DEG2RAD) * angle_in_degree * TRACK_WIDTH * NSTEP_ONE_TURN)/(2 * WHEEL_PERIMETER);
//	motor_left_pos_target = -((DEG2RAD) * angle_in_degree * TRACK_WIDTH * NSTEP_ONE_TURN)/(2 * WHEEL_PERIMETER);
}

void set_straight_move_in_mm(uint32_t distance_in_mm) {
	motor_right_pos_target = (distance_in_mm)/(WHEEL_PERIMETER/NSTEP_ONE_TURN);
	motor_left_pos_target = (distance_in_mm)/(WHEEL_PERIMETER/NSTEP_ONE_TURN);
}

void central_unit_start(void){
	chThdCreateStatic(waCentralUnit, sizeof(waCentralUnit), NORMALPRIO, CentralUnit, NULL);
}
