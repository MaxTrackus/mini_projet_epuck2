#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <leds.h>
#include <selector.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <epuck1x/utility/utility.h> //used for wait function

#include <central_unit.h>
#include <constants.h>
#include <process_image.h>
#include <move.h>
#include <proxi.h>
#include <move_tracker.h>
#include <p_regulator.h>



// general purpose
static task_mode currentMode = STOP; // laisse à STOP stp.       MaxTrackus
static bool optimizedExitOnLeft = true;

// pursuit mode
static uint8_t lostLineCounter = 0;

// measure modes
static uint16_t measuredValue = 0;
static uint16_t distanceToObject = 0;

// follow mode
static bool foundWall = false;



/***************************INTERNAL FUNCTIONS************************************/

// if spin left, angle must be negative. If spin right, angle must be positive
void rotate_degree_and_update_mode(int speed, int16_t angle, task_mode nextMode) {
	if(!get_trackerIsUsed()) {
		set_movingSpeed(speed);
		if(angle >= 0) {
			update_currentModeOfMove(SPIN_RIGHT);
		}
		else {
			update_currentModeOfMove(SPIN_LEFT);
		}
		trackRotationOfDegree((int16_t)(angle + TRACKING_ERROR * angle));
	}
	if(get_trackerIsUsed()) {
		if(get_trackingFinished()) {
			currentMode = nextMode;
		}
	}
}

// if move forward, speed and distance_mm must be positives, if move backwards speed and distance must be negatives
void move_straight_mm_and_update_mode(int speed, int16_t distance_mm, task_mode nextMode) {
	if(!get_trackerIsUsed()) {
		if(distance_mm >= 0) {
			set_movingSpeed(speed);
		}
		else {
			set_movingSpeed(-speed);
		}
		update_currentModeOfMove(MOVE_STRAIGHT);
		trackStraightAdvance((int16_t)(distance_mm - TRACKING_ERROR * distance_mm));
	}
	if(get_trackerIsUsed()) {
		if(get_trackingFinished()) {
			currentMode = nextMode;
		}
	}
}

/*************************END INTERNAL FUNCTIONS**********************************/

/**
* @brief   Thread which gather the sensors measurements and updates the modes of move.c accordingly
*/
static THD_WORKING_AREA(waCentralUnit, 1024);
static THD_FUNCTION(CentralUnit, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    uint8_t prox_for_follow;
    uint8_t prox_for_exit;

    while(1){
        time = chVTGetSystemTime();

        //update the LED to the current mode
        currentMode == ANALYSE ? set_body_led(1) : set_body_led(0);
        currentMode == ALIGN ? set_led(LED1, 1) : set_led(LED1, 0);
        currentMode == PURSUIT ? set_led(LED3, 1) : set_led(LED3, 0);
        if((currentMode == MEASURE_TOF) && (currentMode == MEASURE_SPIN_RIGHT) && (currentMode == MEASURE_SPIN_LEFT)) {
        	set_led(LED5, 1);
        }
        else {
        	set_led(LED5, 0);
        }
        currentMode == PUSH ? set_led(LED7, 1) : set_led(LED7, 0);
        currentMode == ROTATE_BEFORE_FOLLOW ? set_led(LED1, 1) : set_led(LED1, 0);
        currentMode == FOLLOW ? set_led(LED3, 1) : set_led(LED3, 0);
		currentMode == EXIT ? set_led(LED5, 1) : set_led(LED5, 0);
		currentMode == PUSH_OUT ? set_led(LED7, 1) : set_led(LED7, 0);
		currentMode == RECENTER ? set_led(LED1, 1) : set_led(LED1, 0);

        switch(currentMode) {
        	case IDLE:
        		break;

        	case STOP:
        		set_movingSpeed(DEFAULT_SPEED);
        		update_currentModeOfMove(STOP_MOVE);
        		optimizedExitOnLeft = true;
        		stop_tracker();
        		//from stop to analyseMode
        		if(get_selector() == 1) {
        			currentMode = ANALYSE;
        		}
        		break;

			/**
			* @brief   Look for the object by spinning infinitely to the right
			*/
        	case ANALYSE:
        		set_rotationMappingIsOn(true);
        		set_movingSpeed(DEFAULT_SPEED);
        		update_currentModeOfMove(SPIN_RIGHT);
        		//from analyseMode to alignementMode
        		if(get_staticFoundLine()) {
					currentMode = ALIGN;
				}
        		break;

			/**
			* @brief   Align the robot to the object using the camera and a P regulator
			*/
        	case ALIGN:
        		set_movingSpeed(DEFAULT_SPEED);
        		update_currentModeOfMove(SPIN_ALIGNEMENT);
        		// if the robot made a complete turn, stop
        		if(get_rotationMappingValue() >= (int)(((DEG2RAD) * COMPLETE_TURN * TRACK_WIDTH * NSTEP_ONE_TURN)/(2 * WHEEL_PERIMETER))) {
        			currentMode = STOP;
        		}
        		//from alignementMode to analyseMode
        		if(!get_staticFoundLine()) {
                	set_rotationMappingIsOn(true);
        			currentMode = ANALYSE;
        		}
        		//from alignementMode to pursuit
				if(get_regulationCompleted()) {
					// determine if it is shorter to follow the wall counterclockwise (true) or clockwise (false)
					if(get_rotationMappingValue() >= THRESHOLD_STEPS_FOR_OPTIMIZED_EXIT) {
						optimizedExitOnLeft = false;
					} else {
						optimizedExitOnLeft = true;
					}
					currentMode = PURSUIT;
				}
        		break;

			/**
			* @brief   Move towards the object and correct its trajectory if the object moves.
			* 		   Stops when the object image is large enough
			*/
        	case PURSUIT:
        		set_movingSpeed(DEFAULT_SPEED);
        		update_currentModeOfMove(MOVE_STRAIGHT_CORRECT_ALIGNEMENT);
    			if(get_staticFoundLine() == false) {
    				++lostLineCounter;
    			} else {
    				lostLineCounter = 0;
    			}
    			if(lostLineCounter == NB_INCREMENTS_LOST_OBJECT) {
    				currentMode = STOP;
    			}
    			if(get_lineWidth() > (uint16_t)(LARGE_WIDTH_STOP)) {
    				update_currentModeOfMove(STOP_MOVE);
    				distanceToObject = 0;
    				currentMode = MEASURE_TOF;
    			}
        		break;

			/**
			* @brief   Measure the TOF distance twenty times and make a mean. This mode is made two times
			*/
        	case MEASURE_TOF:;
        		update_currentModeOfMove(STOP_MOVE);
        		uint8_t counter = 0;
        		do {
        			measuredValue += VL53L0X_get_dist_mm() - TOF_OFFSET; //TOF_OFFSET determined by experimentation
					counter += 1;
					chThdSleepMilliseconds(100);
				} while (counter < NB_TOF_MESURE_MEAN);
        		measuredValue = (uint16_t)(measuredValue/NB_TOF_MESURE_MEAN); //VL53L0X_get_dist_mm(); //gets distance to object
        		if(distanceToObject == 0) {
        			distanceToObject = measuredValue;
        			stop_tracker();
        			currentMode = MEASURE_SPIN_RIGHT;
        		}
        		else {
        			stop_tracker();
        			currentMode = MEASURE_SPIN_LEFT;
        		}
        		break;

			/**
			* @brief   Spin right of 20 degrees and run again the MEASURE_TOF mode
			*/
        	case MEASURE_SPIN_RIGHT:
        		rotate_degree_and_update_mode(DEFAULT_SPEED, TWENTY_DEGREES, MEASURE_TOF);
				break;

			/**
			* @brief   Spin back of 20 degrees to be in front of the object again
			*/
        	case MEASURE_SPIN_LEFT:
				rotate_degree_and_update_mode(DEFAULT_SPEED, -TWENTY_DEGREES, PUSH);
        		break;

			/**
			* @brief   Push the object against the wall
			*/
        	case PUSH: ;
        		//measuredVale is the distance to the wall
        		int distance_travel = measuredValue - distanceToObject - OBJECT_DIAMETER;
        		distanceToObject = 0;
        		move_straight_mm_and_update_mode(DEFAULT_SPEED, distance_travel, ROTATE_BEFORE_FOLLOW);
        		break;

			/**
			* @brief   Rotate of 90 degrees to expose the poxi sensors to the wall
			*/
        	case ROTATE_BEFORE_FOLLOW:
        		if(optimizedExitOnLeft) {
        			rotate_degree_and_update_mode(DEFAULT_SPEED, -QUARTER_TURN, FOLLOW);
        		}
        		else {
					rotate_degree_and_update_mode(DEFAULT_SPEED, QUARTER_TURN, FOLLOW);
        		}
        		break;

			/**
			* @brief   Follow the wall using proxi sensors, until reaching the exit.
			*/
        	case FOLLOW: ;
        		set_movingSpeed(FAST_SPEED);
        		uint16_t *prox_values = get_prox_value();

        		if (optimizedExitOnLeft) {
        			prox_for_follow = PROX_FRONT_RIGHT_49;
        			prox_for_exit = PROX_RIGHT;
        		} else {
        			prox_for_follow = PROX_FRONT_LEFT_49;
        			prox_for_exit = PROX_LEFT;
        		}

        		int16_t speedCorrection = (int16_t)(SPEED_CORRECTION_SENSIBILITY_OVER_PROXI * prox_values[prox_for_follow]) - (GOAL_PROXI_VALUE * SPEED_CORRECTION_SENSIBILITY_OVER_PROXI);
        		if((prox_values[prox_for_follow] <= GOAL_PROXI_VALUE)) {
        			speedCorrection = 0;
        		} else {
        			foundWall = true;
        		}

        		(optimizedExitOnLeft) ? follow_wall_with_speed_correction(-speedCorrection) : follow_wall_with_speed_correction(speedCorrection);

        		bool *prox_status_table = get_prox_activation_status(PROX_DETECTION_THRESHOLD);
				if ((prox_status_table[prox_for_follow] == false) && foundWall) {
					update_currentModeOfMove(MOVE_STRAIGHT);
					if(prox_status_table[prox_for_exit] == false) {
						foundWall = false;
						update_currentModeOfMove(STOP_MOVE);
						currentMode = EXIT;
					}
				}
        		break;

			/**
			* @brief   Rotate in front of the exit
			*/
        	case EXIT:
        		if(!optimizedExitOnLeft) {
					rotate_degree_and_update_mode(DEFAULT_SPEED, -SIXTY_DEGREES, PUSH_OUT);
        		}
        		else {
					rotate_degree_and_update_mode(DEFAULT_SPEED, SIXTY_DEGREES, PUSH_OUT);
        		}
        		break;

			/**
			* @brief   Push the object out of the arena
			*/
        	case PUSH_OUT:
				move_straight_mm_and_update_mode(DEFAULT_SPEED, EXIT_DISTANCE, HIDE_OBJECT_TURN);
        		break;

			/**
			* @brief   Turn behind the arena to hide the object
			*/
        	case HIDE_OBJECT_TURN:
        		if(!optimizedExitOnLeft) {
					rotate_degree_and_update_mode(DEFAULT_SPEED, -QUARTER_TURN, HIDE_OBJECT_PUSH);
				}
				else {
					rotate_degree_and_update_mode(DEFAULT_SPEED, QUARTER_TURN, HIDE_OBJECT_PUSH);
				}
        		break;

			/**
			* @brief   Push the object behind arena
			*/
			case HIDE_OBJECT_PUSH:
				move_straight_mm_and_update_mode(DEFAULT_SPEED, EXIT_DISTANCE, RETREAT_BACK);
        		break;

			/**
			* @brief   Move backward towards the exit
			*/
			case RETREAT_BACK:
				move_straight_mm_and_update_mode(DEFAULT_SPEED, -EXIT_DISTANCE, RETREAT_TURN);
        		break;

			/**
			* @brief   Turn again in the opposite direction to be align to the exit
			*/
			case RETREAT_TURN:
        		if(!optimizedExitOnLeft) {
					rotate_degree_and_update_mode(DEFAULT_SPEED, QUARTER_TURN, RECENTER);
				}
				else {
					rotate_degree_and_update_mode(DEFAULT_SPEED, -QUARTER_TURN, RECENTER);
				}
        		break;

			/**
			* @brief   Retreat back to the center of the arena
			*/
        	case RECENTER:
				move_straight_mm_and_update_mode(FAST_SPEED, -(ARENA_RADIUS+EXIT_DISTANCE), ANALYSE);
        		break;

        	default:
        		currentMode = IDLE;
        		break;
        }

		// force stop mode
		if((get_selector() == FIFTEENTH_POS_SELECTOR)) {
			currentMode = STOP;
		}

        //enable rotationMapping only in analyse and align modes
        if((currentMode == ANALYSE) || (currentMode == ALIGN)) {
        	set_rotationMappingIsOn(true);
        } else {
        	set_rotationMappingIsOn(false);
        }

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

/****************************PUBLIC FUNCTIONS*************************************/

void central_unit_start(void){
	chThdCreateStatic(waCentralUnit, sizeof(waCentralUnit), NORMALPRIO, CentralUnit, NULL);
}

/**************************END PUBLIC FUNCTIONS***********************************/

