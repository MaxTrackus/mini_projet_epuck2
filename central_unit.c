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
#include <process_image.h>
#include <move.h>
#include <pi_regulator.h>
#include <proxi.h>
#include <move_tracker.h>

#define DEFAULT_SPEED					200 	// [steps/s]
#define SLOW_SPEED						50 		// [steps/s]
#define FAST_SPEED						400 	// [steps/s]

#define	OBJECT_DIAMETER					30 		// [mm]	
#define ERROR_MARGIN					75 		// [mm]
#define WALL_CLEARANCE					10 		// [mm]
#define SEC2MSEC						1000
#define MAX_MOTOR_SPEED					1100	// [steps/s]
#define NSTEP_ONE_TURN      			1000	// number of step for 1 turn of the motor
#define WHEEL_PERIMETER     			130		// [mm]

#define ARENA_RADIUS					250 	// [mm]
#define EXIT_DISTANCE					100		// [mm]

#define QUARTER_TURN					90
#define MOTOR_STEP_TO_DEGREES			360 //find other name maybe
#define	PROX_DETECTION_THRESHOLD		10

// follow mode
#define	SPEED_CORRECTION_SENSIBILITY_OVER_PROXI		2
#define	GOAL_PROXI_VALUE							150

// rotate_before_follow mode
#define DEG2RAD										M_PI/180
#define TRACK_WIDTH									51 //distance between the wheels [mm]
#define THRESHOLD_ANGLE_FOR_OPTIMIZED_EXIT			180 // [deg]
#define THRESHOLD_STEPS_FOR_OPTIMIZED_EXIT			620 // [steps]       (uint16_t)(THRESHOLD_ANGLE_FOR_OPTIMIZED_EXIT * DEG2RAD * TRACK_WIDTH * NSTEP_ONE_TURN) / (2*WHEEL_PERIMETER)   ?

// move_tracker
#define TRACKING_ERROR 					0.05

// general purpose
static /*volatile */task_mode currentMode = STOP; // laisse à STOP stp.       MaxTrackus
static bool optimizedExitOnLeft = true;

// pursuit mode
static uint8_t lostLineCounter = 0;

// measure modes
static uint16_t measuredValue = 0;
static uint16_t distanceToObject = 0;

// follow mode
static bool foundWall = false;

// INTERNAL FUNCTIONS BEGINS
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
// INTERNAL FUNCTIONS ENDS

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
        		chprintf((BaseSequentialStream *)&SD3, "IDLE");
        		break;

        	case STOP:
        		chprintf((BaseSequentialStream *)&SD3, "STOP");
        		set_movingSpeed(DEFAULT_SPEED);
        		update_currentModeOfMove(STOP_MOVE);
        		optimizedExitOnLeft = true;
        		stop_tracker();
        		break;

        	case ANALYSE:
        		chprintf((BaseSequentialStream *)&SD3, "ANALYSE");
        		set_rotationMappingIsOn(true);
        		set_movingSpeed(DEFAULT_SPEED);
        		update_currentModeOfMove(SPIN_RIGHT);
        		break;

        	case ALIGN:
        		chprintf((BaseSequentialStream *)&SD3, "ALIGN");
        		set_movingSpeed(DEFAULT_SPEED);
        		update_currentModeOfMove(SPIN_ALIGNEMENT);
        		break;

        	case PURSUIT:
        		chprintf((BaseSequentialStream *)&SD3, "PURSUIT");
        		set_movingSpeed(DEFAULT_SPEED);
        		update_currentModeOfMove(MOVE_STRAIGHT_CORRECT_ALIGNEMENT);
    			if(get_staticFoundLine() == false) {
    				++lostLineCounter;
    			} else {
    				lostLineCounter = 0;
    			}
    			if(lostLineCounter == 100) {
    				currentMode = STOP;
    			}
    			if(get_lineWidth() > (uint16_t)(400)) {
    				update_currentModeOfMove(STOP_MOVE);
    				distanceToObject = 0;
    				currentMode = MEASURE_TOF;
    			}
        		break;

        	// this mode is made two times
        	case MEASURE_TOF:;
        		chprintf((BaseSequentialStream *)&SD3, "MEASURE_TOF");
        		update_currentModeOfMove(STOP_MOVE);
        		uint8_t counter = 0;
        		do {
        			measuredValue += VL53L0X_get_dist_mm();
					counter += 1;
					chThdSleepMilliseconds(100);
				} while (counter < 20);
        		measuredValue = (uint16_t)(measuredValue/20); //VL53L0X_get_dist_mm(); //gets distance to object
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

        	case MEASURE_SPIN_RIGHT:
        		rotate_degree_and_update_mode(DEFAULT_SPEED, 20, MEASURE_TOF);
				break;

        	case MEASURE_SPIN_LEFT:
				rotate_degree_and_update_mode(DEFAULT_SPEED, -20, PUSH);
        		break;

        	case PUSH: ;
        		//measuredVale is the distance to the wall
        		/*volatile */int distance_travel = measuredValue - distanceToObject - OBJECT_DIAMETER;
        		distanceToObject = 0;
        		///////////////////////////////////////////////////////////////////////////////////// advance_mm function
        		if(!get_trackerIsUsed()) {
					set_movingSpeed(DEFAULT_SPEED);
					update_currentModeOfMove(MOVE_STRAIGHT);
					trackStraightAdvance((int16_t)(distance_travel - TRACKING_ERROR * distance_travel));
				}
        		if(get_trackerIsUsed()) {
					if(get_trackingFinished()) {
						currentMode = ROTATE_BEFORE_FOLLOW;
					}
				}
				///////////////////////////////////////////////////////////////////////////////////// advance_mm function
        		break;

        	case ROTATE_BEFORE_FOLLOW:
        		if(optimizedExitOnLeft) {
        			rotate_degree_and_update_mode(DEFAULT_SPEED, -90, FOLLOW);
        		}
        		else {
					rotate_degree_and_update_mode(DEFAULT_SPEED, 90, FOLLOW);
        		}
        		break;

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

        		(optimizedExitOnLeft) ? follow_left_wall_with_speed_correction(-speedCorrection) : follow_left_wall_with_speed_correction(speedCorrection);

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

        	case EXIT:
        		if(!optimizedExitOnLeft) {
					rotate_degree_and_update_mode(DEFAULT_SPEED, -60, PUSH_OUT);
        		}
        		else {
					rotate_degree_and_update_mode(DEFAULT_SPEED, 60, PUSH_OUT);
        		}
        		break;

        	case PUSH_OUT:
        		///////////////////////////////////////////////////////////////////////////////////// advance_mm function
        		if(!get_trackerIsUsed()) {
					set_movingSpeed(DEFAULT_SPEED);
					update_currentModeOfMove(MOVE_STRAIGHT);
					trackStraightAdvance((int16_t)(EXIT_DISTANCE + TRACKING_ERROR * EXIT_DISTANCE));
				}
				if(get_trackerIsUsed()) {
					if(get_trackingFinished()) {
						currentMode = RECENTER;
					}
				}
        		///////////////////////////////////////////////////////////////////////////////////// advance_mm function
        		break;

        	case RECENTER:
        		///////////////////////////////////////////////////////////////////////////////////// advance_mm function
				if(!get_trackerIsUsed()) {
					set_movingSpeed(-FAST_SPEED);
					update_currentModeOfMove(MOVE_STRAIGHT);
					trackStraightAdvance((int16_t)(-(ARENA_RADIUS+EXIT_DISTANCE) - TRACKING_ERROR * (ARENA_RADIUS+EXIT_DISTANCE)));
				}
				if(get_trackerIsUsed()) {
					if(get_trackingFinished()) {
						currentMode = ANALYSE;
					}
				}
				///////////////////////////////////////////////////////////////////////////////////// advance_mm function

        		break;

        		// if spin left, angle degree must be negative. if spin right angle degree must be positiv
        	case ROTATE_TRACKER_TEST:
				if(!get_trackerIsUsed()) {
					set_movingSpeed(DEFAULT_SPEED);
					update_currentModeOfMove(SPIN_RIGHT);
					trackRotationOfDegree((int16_t)(360 + TRACKING_ERROR * 360));
				}
				if(get_trackerIsUsed() && get_trackingFinished()) {
					currentMode = STOP;
				}
				break;

				// if move forward, speed and mm must be positives, if move backwards speed and distance must be negatives
			case STRAIGHT_TRACKER_TEST:
				if(!get_trackerIsUsed()) {
					set_movingSpeed(-DEFAULT_SPEED);
					update_currentModeOfMove(MOVE_STRAIGHT);
					trackStraightAdvance((int16_t)(-70 - TRACKING_ERROR * 70));
				}
				if(get_trackerIsUsed() && get_trackingFinished()) {
					currentMode = STOP;
				}
				break;

			case TEST_ROTATION_MAPPING:
				set_rotationMappingIsOn(true);
				update_currentModeOfMove(SPIN_RIGHT);
				//determine if it is shorter to follow the wall counterclockwise (true) or clockwise (false)
				if(get_rotationMappingValue() >= THRESHOLD_STEPS_FOR_OPTIMIZED_EXIT) {
					optimizedExitOnLeft = false;
				} else {
					optimizedExitOnLeft = true;
				}
				break;

        	default:
        		currentMode = IDLE;
        		break;
        }

		//from idle to analyseMode
		if((get_selector() == 1) && !(currentMode == ALIGN) && !(currentMode == PURSUIT)) {
			currentMode = ANALYSE;
		}
		//from analyseMode to alignementMode
		if((currentMode == ANALYSE) && get_staticFoundLine()) {
			currentMode = ALIGN;
		}
		//from alignementMode to analyseMode
		if((currentMode == ALIGN) && (!(get_staticFoundLine()))) {
        	set_rotationMappingIsOn(true);
			currentMode = ANALYSE;
		}
		//from alignementMode to pursuit
		if((currentMode == ALIGN) && (get_regulationCompleted())) {
			// determine if it is shorter to follow the wall counterclockwise (true) or clockwise (false)
			if(get_rotationMappingValue() >= THRESHOLD_STEPS_FOR_OPTIMIZED_EXIT) { // must be calibrated, maybe 700 is not the good parameter. must test with the rotation of a certain angle when avalaible
				optimizedExitOnLeft = false;
			} else {
				optimizedExitOnLeft = true;
			}
			currentMode = PURSUIT;
		}
		//stop and idle
		if((get_selector() == 15)) {
			currentMode = STOP;
		}

		//////////////////////////////////////////////////////////////testing purposes
//		if(get_selector() == 1) {
//			currentMode = MEASURE_TOF;
//		}
		//////////////////////////////////////////////////////////////testing purposes

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

void central_unit_start(void){
	chThdCreateStatic(waCentralUnit, sizeof(waCentralUnit), NORMALPRIO, CentralUnit, NULL);
}

