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

#define DEFAULT_SPEED					200 // [steps/s]
#define SLOW_SPEED						50 	// [steps/s]
#define	OBJECT_DIAMETER					30 	// [mm]	
#define ERROR_MARGIN					2 	// [mm]
#define WALL_CLEARANCE					20 	// [mm]
#define SEC2MSEC						1000
#define MAX_MOTOR_SPEED					1100 // [steps/s]
#define NSTEP_ONE_TURN      			100 // number of step for 1 turn of the motor
#define WHEEL_PERIMETER     			130 // [mm]




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
        			currentTime = chVTGetSystemTime(); 
        			update_currentModeInMove(SPIN_RIGHT);
        		}
        		
        		uint16_t tof_value = VL53L0X_get_dist_mm();

        		if (tof_value > (distanceToTravel+OBJECT_DIAMETER+ERROR_MARGIN)) {
        			actionTime = chVTGetSystemTime() - currentTime;
        			distanceToTravel = tof_value - distanceToTravel - WALL_CLEARANCE - OBJECT_DIAMETER;
        			currentTime = chVTGetSystemTime();
        			update_currentModeInMove(SPIN_LEFT);
        			wallFound = true;
        		}

        		//gets back to original position
        		if ((chVTGetSystemTime() > (currentTime + actionTime)) && (wallFound == true)) {
        			update_currentModeInMove(STOP);
        			currentProgramMode = PUSH;
        			wallFound = false;
        			currentTime = 0;
        			actionTime = 0;
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
        			currentProgramMode = IDLE;
        			distanceToTravel = 0;
        			actionTime = 0;
        			currentTime = 0;

        		}
        		break;
        	case FOLLOW:
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
        chThdSleepUntilWindowed(time, time + MS2ST(10));

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

void central_unit_start(void){
	chThdCreateStatic(waCentralUnit, sizeof(waCentralUnit), NORMALPRIO, CentralUnit, NULL);
}
