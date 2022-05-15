#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>

#include <move_tracker.h>

//constantes.h pleases
#define DEG2RAD						M_PI/180
#define TRACK_WIDTH					51 //distance between the wheels [mm]
#define NSTEP_ONE_TURN				1000 // [steps]
#define WHEEL_PERIMETER				130 // [mm]

static bool trackingFinished = true;
static move_tracker_mode currentModeOfTracker = TRACK_NOTHING;
static int16_t right_motor_pos_targetTRACK = 0;
static bool trackerIsUsed = false;
static int rotationMappingValue = 0;

/***************************INTERNAL FUNCTIONS************************************/
/**
* @brief   Check if the wanted number of step is reached
*/
void check_position(void) {
	if((right_motor_pos_targetTRACK < 0) && ((int16_t)right_motor_get_pos() < right_motor_pos_targetTRACK)) {
		currentModeOfTracker = TRACK_NOTHING;
	}

	if((right_motor_pos_targetTRACK > 0) && ((int16_t)right_motor_get_pos() > right_motor_pos_targetTRACK)) {
		currentModeOfTracker = TRACK_NOTHING;
	}
}

/**
* @brief   Reset the counter of step provided by the library in motor.c
*/
void reset_motor_pos_TRACKER(void) {
	left_motor_set_pos(0);
	right_motor_set_pos(0);
}
/*************************END INTERNAL FUNCTIONS**********************************/

static THD_WORKING_AREA(waMoveTracker, 256);
static THD_FUNCTION(MoveTracker, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1){
        time = chVTGetSystemTime();

        switch(currentModeOfTracker) {
			case TRACK_NOTHING:
				//do nothing
				trackingFinished = true;
				reset_motor_pos_TRACKER();
				right_motor_pos_targetTRACK = 0;
				break;

			case TRACK_SPIN:
				check_position();
				break;
			case TRACK_STRAIGHT:
				check_position();
				break;
			case ROTATION_MAPPING:
				rotationMappingValue = left_motor_get_pos();
				break;
        }

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

/****************************PUBLIC FUNCTIONS*************************************/

void move_tracker_start(void) {
	chThdCreateStatic(waMoveTracker, sizeof(waMoveTracker), NORMALPRIO, MoveTracker, NULL);
}

void stop_tracker(void) {
	currentModeOfTracker = TRACK_NOTHING;
	trackingFinished = true;
	right_motor_pos_targetTRACK = 0;
	trackerIsUsed = false;
}

bool get_trackingFinished(void) {
	if(trackingFinished) {
		trackerIsUsed = false;
	}
	return trackingFinished;
}

bool get_trackerIsUsed(void) {
	return trackerIsUsed;
}

void set_trackerMode(move_tracker_mode mode) {
	currentModeOfTracker = mode;
}

void trackRotationOfDegree(int16_t degree) {
	if(!trackerIsUsed) {
		trackerIsUsed = true;
		trackingFinished = false;
		reset_motor_pos_TRACKER();
		right_motor_pos_targetTRACK = (int16_t)(((DEG2RAD) * (-degree) * TRACK_WIDTH * NSTEP_ONE_TURN)/(2 * WHEEL_PERIMETER));
		currentModeOfTracker = TRACK_SPIN;
	}
}

void trackStraightAdvance(int16_t milimeters) {
	if(!trackerIsUsed) {
		trackerIsUsed = true;
		trackingFinished = false;
		reset_motor_pos_TRACKER();
		right_motor_pos_targetTRACK = (int16_t)(milimeters * (NSTEP_ONE_TURN / WHEEL_PERIMETER));
		currentModeOfTracker = TRACK_STRAIGHT;
	}
}

void set_rotationMappingIsOn(bool status) {
	if((currentModeOfTracker == ROTATION_MAPPING) && !status) {
		trackerIsUsed = false;
		left_motor_set_pos(0);
		rotationMappingValue = 0;
	}
	if(!(currentModeOfTracker == ROTATION_MAPPING) && status) {
		trackerIsUsed = true;
		left_motor_set_pos(0);
		rotationMappingValue = 0;
	}
	if(status) {
		currentModeOfTracker = ROTATION_MAPPING;
	}
}

int get_rotationMappingValue(void) {
	return rotationMappingValue;
}
/**************************END PUBLIC FUNCTIONS***********************************/
