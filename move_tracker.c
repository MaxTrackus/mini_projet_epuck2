/*
 * mave_tracker.c
 *
 *  Created on: 14 mai 2022
 *      Author: m_the
 */

#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>

#include <move_tracker.h>

static bool trackingFinished = true;
static move_tracker_mode currentModeOfTracker = TRACK_NOTHING;
static int16_t right_motor_pos_target = 0;
static int16_t left_motor_pos_target = 0;

//// rotation mapping
//static bool rotationMappingIsOn = false;
//static int rotationMappingValue = 0;

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
				break;

			case TRACK_SPIN:
				if() {

				}
				break;
        }

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void move_tracker_start(void) {
	chThdCreateStatic(waMoveTracker, sizeof(waMoveTracker), NORMALPRIO, MoveTracker, NULL);
}

bool get_trackingFinished(void) {
	return trackingFinished;
}

void set_trackerMode(move_tracker_mode mode) {
	currentModeOfTracker = mode;
}

void trackRotationOfDegree(int16_t degree) {
	trackingFinished = false;
	reset_motor_pos_MOVETRACKER();
	currentModeOfTracker = TRACK_SPIN;
}

void reset_motor_pos_MOVETRACKER(void) {
	left_motor_set_pos(0);
	right_motor_set_pos(0);
}
