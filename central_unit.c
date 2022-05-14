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

#define DEFAULT_SPEED					200 // [steps/s]
#define SLOW_SPEED						50 	// [steps/s]
#define	OBJECT_DIAMETER					30 	// [mm]	
#define ERROR_MARGIN					75 	// [mm]
#define WALL_CLEARANCE					10 	// [mm]
#define SEC2MSEC						1000  //1000
#define MAX_MOTOR_SPEED					1100 // [steps/s]
#define NSTEP_ONE_TURN      			1000 // number of step for 1 turn of the motor
#define WHEEL_PERIMETER     			130 // [mm]

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

// follow mode
static bool foundWall = false;
static bool usingStepCounters = false;

static volatile task_mode currentMode = STOP;
static volatile uint16_t distanceToTravel = 0;
static volatile bool wallFound = false;
static bool wallMeasured = false;
static bool optimizedExitOnLeft = true;

static bool moving = false;

//static uint32_t	right_motor_pos_target = 0;
//static uint32_t	left_motor_pos_target = 0;

static uint8_t lostLineCounter = 0;

static uint16_t measurement_average = 0;
static uint8_t counter = 0;

static THD_WORKING_AREA(waCentralUnit, 1024);
static THD_FUNCTION(CentralUnit, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    volatile systime_t time;

    while(1){
        time = chVTGetSystemTime();

        //update the LED to the current mode
        currentMode == ANALYSE ? set_body_led(1) : set_body_led(0);
        currentMode == ALIGN ? set_led(LED1, 1) : set_led(LED1, 0);
        currentMode == PURSUIT ? set_led(LED3, 1) : set_led(LED3, 0);
        currentMode == MEASURE ? set_led(LED5, 1) : set_led(LED5, 0);
        currentMode == PUSH ? set_led(LED7, 1) : set_led(LED7, 0);
        currentMode == ROTATE_BEFORE_FOLLOW ? set_led(LED1, 1) : set_led(LED1, 0);
        currentMode == FOLLOW ? set_led(LED3, 1) : set_led(LED3, 0);
		currentMode == EXIT ? set_led(LED5, 1) : set_led(LED5, 0);

        switch(currentMode) {
        	case IDLE:
        		break;

        	case STOP:
        		set_movingSpeed(DEFAULT_SPEED);
        		update_currentModeOfMove(STOP_MOVE);
        		optimizedExitOnLeft = true;
        		stop_tracker();
        		break;

//        	case ANALYSE:
//        		set_movingSpeed(DEFAULT_SPEED);
//        		update_currentModeOfMove(SPIN_RIGHT);
//        		break;
//
//        	case ALIGN:
//        		set_movingSpeed(DEFAULT_SPEED);
//        		update_currentModeOfMove(SPIN_ALIGNEMENT);
//        		break;
//
//        	case PURSUIT:
//        		set_movingSpeed(DEFAULT_SPEED);
//        		update_currentModeOfMove(MOVE_STRAIGHT_CORRECT_ALIGNEMENT);
//    			if(get_staticFoundLine() == false) {
//    				++lostLineCounter;
//    			} else {
//    				lostLineCounter = 0;
//    			}
//    			if(lostLineCounter == 100) {
//    				currentMode = STOP;
//    			}
//    			if(get_lineWidth() > (uint16_t)(400)) {
//    				update_currentModeOfMove(STOP_MOVE);
//    				currentMode = MEASURE;
//    			}
//        		break;
//
//        	case MEASURE:
//        		if (distanceToTravel == 0) {
//        			set_front_led(1);
//        			//faire un nouveau mode pour faire uniquement la mesure ? :thinking
//        			do {
//        				measurement_average += VL53L0X_get_dist_mm();
//        				counter += 1;
//        				chThdSleepMilliseconds(100);
//        			} while (counter < 20);
//        			distanceToTravel = measurement_average/20; //VL53L0X_get_dist_mm(); //gets distance to object
//        			measurement_average = 0;
//        			counter = 0;
//        			left_motor_pos_target = 72;
//        			reset_motor_pos();
//        			set_movingSpeed(SLOW_SPEED);
//        			update_currentModeOfMove(SPIN_RIGHT);
//        		}
//
//        		if ((get_left_motor_pos() >= left_motor_pos_target) && (wallFound == false)) {
//        			update_currentModeOfMove(STOP_MOVE);
//        			do {
//						measurement_average += VL53L0X_get_dist_mm();
//						counter += 1;
//        				chThdSleepMilliseconds(100);
//					} while (counter < 20);
//        			wallFound = true;
//        			distanceToTravel = (measurement_average/20) - distanceToTravel - OBJECT_DIAMETER/* - WALL_CLEARANCE*/;
//        			measurement_average = 0;
//					counter = 0;
//        		} else if ((wallFound == true) && (wallMeasured == false)) {
//        			set_front_led(0);
//        			reset_motor_pos();
//        			right_motor_pos_target = 72;
//        			wallMeasured = true;
//					set_movingSpeed(SLOW_SPEED);
//					update_currentModeOfMove(SPIN_LEFT);
//        		}
//
//        		if ((get_right_motor_pos() >= right_motor_pos_target) && (wallMeasured == true)) {
//        			update_currentModeOfMove(STOP);
//        			left_motor_pos_target = 0;
//        			right_motor_pos_target = 0;
//        			wallFound = false;
//        			wallMeasured = false;
//        			currentMode = PUSH;
//        		}
//        		break;
//
//        	case PUSH:
//        		if ((distanceToTravel != 0) && (moving == false)) {
//        			set_straight_move_in_mm(distanceToTravel);
//       				distanceToTravel = 0;
//
//        			moving = true;
//        			reset_motor_pos();
//        			set_movingSpeed(DEFAULT_SPEED);
//        			update_currentModeOfMove(MOVE_STRAIGHT);
//        		}
//        		volatile uint32_t current_motor_pos = get_right_motor_pos();
//        		if ((current_motor_pos >= right_motor_pos_target)) {
//        			moving = false;
//        			left_motor_pos_target = 0;
//        			right_motor_pos_target = 0;
//        			update_currentModeOfMove(STOP_MOVE);
//        			currentMode = ROTATE_BEFORE_FOLLOW;
//        		}
//        		break;
//
//        	case ROTATE_BEFORE_FOLLOW:
//        		if(optimizedExitOnLeft) {
//        			/////////////////////////////////////////////////////////////////////////////function rotate of a certain angle begin
//        			if (!usingStepCounters) {
//        				right_motor_pos_target = 308; // to do a 90 degrees right rotation
//						reset_motor_pos();
//						set_movingSpeed(DEFAULT_SPEED);
//						update_currentModeOfMove(SPIN_LEFT);
//						usingStepCounters = true;
//					}
//					if ((get_right_motor_pos() >= right_motor_pos_target) && usingStepCounters) {
//						usingStepCounters = false;
//						reset_motor_pos();
////						currentMode = FOLLOW;
//						currentMode = STOP;
//					}
//					/////////////////////////////////////////////////////////////////////////////function rotate of a certain angle end
//        		}
//        		else {
//        			/////////////////////////////////////////////////////////////////////////////function rotate of a certain angle begin
//        			if (!usingStepCounters) {
//						left_motor_pos_target = 308; // to do a 90 degrees right rotation
//						reset_motor_pos();
//						set_movingSpeed(DEFAULT_SPEED);
//						update_currentModeOfMove(SPIN_RIGHT);
//						usingStepCounters = true;
//					}
//					if ((get_left_motor_pos() >= left_motor_pos_target) && usingStepCounters) {
//						usingStepCounters = false;
//						reset_motor_pos();
////						currentMode = FOLLOW;
//						currentMode = STOP;
//					}
//					/////////////////////////////////////////////////////////////////////////////function rotate of a certain angle end
//        		}
//        		break;
//
//        	case FOLLOW: ;
//        		set_movingSpeed(400);
//        		int *prox_values = get_prox_value();
//        		int16_t speedCorrection = (int16_t)(SPEED_CORRECTION_SENSIBILITY_OVER_PROXI * prox_values[PROX_FRONT_LEFT_49]) - (GOAL_PROXI_VALUE * SPEED_CORRECTION_SENSIBILITY_OVER_PROXI);
//        		if((prox_values[PROX_FRONT_LEFT_49] <= GOAL_PROXI_VALUE)) {
//        			speedCorrection = 0;
//        		} else {
//        			foundWall = true;
//        		}
//        		follow_left_wall_with_speed_correction(speedCorrection);
//
//        		bool *prox_status_table = get_prox_activation_status(PROX_DETECTION_THRESHOLD);
//				if ((prox_status_table[PROX_FRONT_LEFT_49] == false) && foundWall) {
//					update_currentModeOfMove(MOVE_STRAIGHT);
//					if(prox_status_table[PROX_LEFT] == false) {
//						foundWall = false;
//						update_currentModeOfMove(STOP_MOVE);
//						currentMode = EXIT;
//					}
//				}
//        		break;
//
//        	case EXIT:
//        		break;
//
//        	case RECENTER:
//        		break;

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

        	case STRAIGHT_TRACKER_TEST:
        		if(!get_trackerIsUsed()) {
					set_movingSpeed(DEFAULT_SPEED);
					update_currentModeOfMove(MOVE_STRAIGHT);
					trackStraightAdvance((int16_t)(70 + TRACKING_ERROR * 70));
				}
				if(get_trackerIsUsed() && get_trackingFinished()) {
					currentMode = STOP;
				}
				break;

        	default:
        		currentMode = IDLE;
        		break;
        }

//		//from idle to analyseMode
//		if((get_selector() == 1) && !(currentMode == ALIGN) && !(currentMode == PURSUIT)) {
//			currentMode = ANALYSE;
//		}
//		//from analyseMode to alignementMode
//		if((currentMode == ANALYSE) && get_staticFoundLine()) {
//			currentMode = ALIGN;
//		}
//		//from alignementMode to analyseMode
//		if((currentMode == ALIGN) && (!(get_staticFoundLine()))) {
//			currentMode = ANALYSE;
//		}
//		//from alignementMode to pursuit
//		if((currentMode == ALIGN) && (get_regulationCompleted())) {
//			// determine if it is shorter to follow the wall counterclockwise (true) or clockwise (false)
//			if(get_rotationMappingValue() >= THRESHOLD_STEPS_FOR_OPTIMIZED_EXIT) { // must be calibrated, maybe 700 is not the good parameter. must test with the rotation of a certain angle when avalaible
//				optimizedExitOnLeft = false;
//			} else {
//				optimizedExitOnLeft = true;
//			}
//			currentMode = PURSUIT;
//		}
		//stop and idle
		if((get_selector() == 15)) {
			currentMode = STOP;
		}

		//////////////////////////////////////////////////////////////testing purposes
		if(get_selector() == 1) {
			currentMode = STRAIGHT_TRACKER_TEST;
		}
		//////////////////////////////////////////////////////////////testing purposes

//        //enable rotationMapping only in analyse and align modes
//        if((currentMode == ANALYSE) || (currentMode == ALIGN)) {
//        	set_rotationMappingIsOn(true);
//        } else {
//        	set_rotationMappingIsOn(false);
//        }

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

////This is a function used for testing -> to remove for final
//void set_mode_with_selector(void) {
//	switch (get_selector()) {
//		case 0:
//			currentMode = IDLE;
//			break;
//		case 1:
//			currentMode = ANALYSE;
//			break;
//		case 2:
//			currentMode = ALIGN;
//			break;
//		case 3:
//			currentMode = PURSUIT;
//			break;
//		case 4:
//			currentMode = MEASURE;
//			break;
//		case 5:
//			currentMode = PUSH;
//			break;
//		case 6:
//			currentMode = FOLLOW;
//			break;
//		case 7:
//			currentMode = EXIT;
//			break;
//		case 8:
//			currentMode = RECENTER;
//			break;
//		default:
//			currentMode = IDLE;
//			break;
//	}
//}

void central_unit_start(void){
	chThdCreateStatic(waCentralUnit, sizeof(waCentralUnit), NORMALPRIO, CentralUnit, NULL);
}

//void set_straight_move_in_mm(uint32_t distance_in_mm) {
//	right_motor_pos_target = (distance_in_mm*NSTEP_ONE_TURN)/(WHEEL_PERIMETER);
//	left_motor_pos_target = (distance_in_mm*NSTEP_ONE_TURN)/(WHEEL_PERIMETER);
//}

