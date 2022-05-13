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

//////////////////////////////////////////////////////////////////////// test_max_1205
#define	SPEED_CORRECTION_SENSIBILITY_OVER_PROXI		2
#define	GOAL_PROXI_VALUE		100

static bool foundWall = false;
static bool usingStepCounters = false;
//////////////////////////////////////////////////////////////////////// test_max_1205

static volatile task_mode currentMode = IDLE;
//static volatile systime_t currentTime = 0;
//static volatile uint32_t actionTime = 0;
static volatile uint16_t distanceToTravel = 0;
static volatile bool wallFound = false;
static bool wallMeasured = false;
static bool optimizedExitOnLeft = true;
//static uint8_t exitProx = PROX_RIGHT;

static bool moving = false;

static uint32_t	right_motor_pos_target = 0;
static uint32_t	left_motor_pos_target = 0;

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
        		break;

        	case ANALYSE:
        		set_movingSpeed(DEFAULT_SPEED);
        		update_currentModeOfMove(SPIN_RIGHT);
        		break;

        	case ALIGN:
        		set_movingSpeed(DEFAULT_SPEED);
        		update_currentModeOfMove(SPIN_ALIGNEMENT);
        		break;

        	case PURSUIT:
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
    				currentMode = MEASURE;
    			}
        		break;

        	case MEASURE:
        		if (distanceToTravel == 0) {
        			set_front_led(1);
//        			chThdSleepMilliseconds(2000);
        			//faire un nouveau mode pour faire uniquement la mesure ? :thinking
        			do {
        				measurement_average += VL53L0X_get_dist_mm();
        				counter += 1;
        				chThdSleepMilliseconds(100);
        			} while (counter < 20);
        			distanceToTravel = measurement_average/20; //VL53L0X_get_dist_mm(); //gets distance to object
        			measurement_average = 0;
        			counter = 0;
        			left_motor_pos_target = 72;//1295;
        			reset_motor_pos();
        			set_movingSpeed(SLOW_SPEED);
        			update_currentModeOfMove(SPIN_RIGHT);
        		}

        		//volatile uint16_t tof = VL53L0X_get_dist_mm();

        		if ((get_left_motor_pos() >= left_motor_pos_target) && (wallFound == false)) {
        			update_currentModeOfMove(STOP_MOVE);
        			do {
						measurement_average += VL53L0X_get_dist_mm();
						counter += 1;
        				chThdSleepMilliseconds(100);
					} while (counter < 20);
        			wallFound = true;
        			distanceToTravel = (measurement_average/20) - distanceToTravel - OBJECT_DIAMETER/* - WALL_CLEARANCE*/;
        			measurement_average = 0;
					counter = 0;
        		} else if ((wallFound == true) && (wallMeasured == false)) {
        			set_front_led(0);
        			reset_motor_pos();
        			right_motor_pos_target = 72;//1295;
        			wallMeasured = true;
					set_movingSpeed(SLOW_SPEED);
					update_currentModeOfMove(SPIN_LEFT);
        		}

        		if ((get_right_motor_pos() >= right_motor_pos_target) && (wallMeasured == true)) {
        			update_currentModeOfMove(STOP);
        			left_motor_pos_target = 0;
        			right_motor_pos_target = 0;
        			currentMode = PUSH;
        		}
        		break;

        	case PUSH:
        		if ((distanceToTravel != 0) && (moving == false)) {
        			set_straight_move_in_mm(distanceToTravel);
       				distanceToTravel = 0;

        			moving = true;
        			reset_motor_pos();
        			set_movingSpeed(DEFAULT_SPEED);
        			update_currentModeOfMove(MOVE_STRAIGHT);
        		}
        		volatile uint32_t current_motor_pos = get_right_motor_pos();
        		if ((current_motor_pos >= right_motor_pos_target)) {
        			//////////////////////////////////////////////////////////////////////// test_max_1205 Commented
//        			update_currentModeOfMove(FOLLOW);
//        			currentMode = IDLE;
        			//////////////////////////////////////////////////////////////////////// test_max_1205 Commented

        			//////////////////////////////////////////////////////////////////////// test_max_1205
        			moving = false;
        			left_motor_pos_target = 0;
        			right_motor_pos_target = 0;
        			update_currentModeOfMove(STOP_MOVE);
        			currentMode = ROTATE_BEFORE_FOLLOW;
        			//////////////////////////////////////////////////////////////////////// test_max_1205
        		}
        		break;

        	//////////////////////////////////////////////////////////////////////// test_max_1205
        	case ROTATE_BEFORE_FOLLOW:
        		if (!usingStepCounters) {
					left_motor_pos_target = 308; // to do a 90 degrees right rotation
					reset_motor_pos();
					set_movingSpeed(DEFAULT_SPEED);
					update_currentModeOfMove(SPIN_RIGHT);
					usingStepCounters = true;
				}
				if ((get_left_motor_pos() >= left_motor_pos_target) && usingStepCounters) {
					usingStepCounters = false;
					reset_motor_pos();
					currentMode = FOLLOW;
				}

        		break;
        	//////////////////////////////////////////////////////////////////////// test_max_1205

        	case FOLLOW: ;
//        		if (wallFound == false) {
//	        		if (actionTime == 0) {
//	        			//0.6 motor turn for 360 degree turn
//	        			actionTime = 3020*1.1;//(QUARTER_TURN * DEFAULT_SPEED * SEC2MSEC)/(MOTOR_STEP_TO_DEGREES);
//	        			set_movingSpeed(DEFAULT_SPEED);
//	        			currentTime = chVTGetSystemTime();
//	        			if (optimizedExitOnLeft) {
//	        				update_currentModeOfMove(SPIN_LEFT); //depends on the flag given by MAX
//	        				exitProx = PROX_RIGHT;
//	        			} else {
//	        				update_currentModeOfMove(SPIN_RIGHT); //depends on the flag given by MAX
//	        				exitProx = PROX_LEFT;
//	        			}
//	        		}
//
//	        		if (chVTGetSystemTime() >= (currentTime + MS2ST(actionTime))) {
//	        			update_currentModeOfMove(STOP);
//	        			//currentProgramMode = IDLE;
//	        			actionTime = 0;
//	        			currentTime = 0;
//	        			wallFound = true;
//	        		}
//	        	} else {
//	        		bool *prox_status_table = get_prox_activation_status(PROX_DETECTION_THRESHOLD);
//	        		int *prox_values = get_prox_value();
//
//	        		set_movingSpeed(DEFAULT_SPEED);
//	        		if (prox_status_table[PROX_FRONT_LEFT_49] == true) {
//	        			update_currentModeOfMove(SPIN_RIGHT);
//					} else if (prox_status_table[PROX_FRONT_RIGHT_49] == true) {
//						update_currentModeOfMove(SPIN_LEFT);
//					}
//					else if (prox_values[exitProx] <= 10) {
//						update_currentModeOfMove(STOP);
//						currentMode = IDLE;
//					}
//					else {
//						update_currentModeOfMove(MOVE_STRAIGHT);
//					}
//
//	        	}

        		//////////////////////////////////////////////////////////////////////// test_max_1205
        		set_movingSpeed(400);
        		int *prox_values = get_prox_value();
        		int16_t speedCorrection = (int16_t)(SPEED_CORRECTION_SENSIBILITY_OVER_PROXI * prox_values[PROX_FRONT_LEFT_49]) - (GOAL_PROXI_VALUE * SPEED_CORRECTION_SENSIBILITY_OVER_PROXI);
        		if((!foundWall) && (prox_values[PROX_FRONT_LEFT_49] <= GOAL_PROXI_VALUE)) {
        			speedCorrection = 0;
        		} else {
        			foundWall = true;
        		}
        		follow_left_wall_with_speed_correction(speedCorrection);

        		bool *prox_status_table = get_prox_activation_status(PROX_DETECTION_THRESHOLD);
				if ((prox_status_table[PROX_FRONT_LEFT_49] == false) && foundWall) {
					update_currentModeOfMove(MOVE_STRAIGHT);
					if(prox_status_table[PROX_LEFT] == false) {
						foundWall = false;
						update_currentModeOfMove(STOP_MOVE);
						currentMode = EXIT;
					}
				}

//        		chprintf((BaseSequentialStream *)&SD3, "v=%d", prox_status_table[PROX_LEFT]);
        		//////////////////////////////////////////////////////////////////////// test_max_1205
        		break;

        	case EXIT:
        		break;

        	case RECENTER:
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
			currentMode = ANALYSE;
		}
		//from alignementMode to pursuit
		if((currentMode == ALIGN) && (get_regulationCompleted())) {
			if(get_rotationMappingValue() >= 700) { // must be calibrated, maybe 700 is not the good parameter. must test with the rotation of a certain angle when avalaible
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

		//////////////////////////////////////////////////////////////////////// test_max_1205
//		if(get_selector() == 1) {
//			currentMode = FOLLOW;
//		}
		//////////////////////////////////////////////////////////////////////// test_max_1205

//		chprintf((BaseSequentialStream *)&SD3, "v=%d", optimizedExitOnLeft);

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

//This is a function used for testing -> to remove for final 
void set_mode_with_selector(void) {
	switch (get_selector()) {
		case 0:
			currentMode = IDLE;
			break;
		case 1:
			currentMode = ANALYSE;
			break;
		case 2:
			currentMode = ALIGN;
			break;
		case 3:
			currentMode = PURSUIT;
			break;
		case 4:
			currentMode = MEASURE;
			break;
		case 5:
			currentMode = PUSH;
			break;
		case 6:
			currentMode = FOLLOW;
			break;
		case 7:
			currentMode = EXIT;
			break;
		case 8:
			currentMode = RECENTER;
			break;
		default:
			currentMode = IDLE;
			break;
	}
}

void central_unit_start(void){
	chThdCreateStatic(waCentralUnit, sizeof(waCentralUnit), NORMALPRIO, CentralUnit, NULL);
}

void set_straight_move_in_mm(uint32_t distance_in_mm) {
	right_motor_pos_target = (distance_in_mm*NSTEP_ONE_TURN)/(WHEEL_PERIMETER);
	left_motor_pos_target = (distance_in_mm*NSTEP_ONE_TURN)/(WHEEL_PERIMETER);
}

