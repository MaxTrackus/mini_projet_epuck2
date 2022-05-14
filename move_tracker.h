/*
 * move_tracker.h
 *
 *  Created on: 14 mai 2022
 *      Author: m_the
 */

#ifndef MOVE_TRACKER_H_
#define MOVE_TRACKER_H_

//List of the different mode in the move_tracker.c file
typedef enum {
	TRACK_NOTHING,
	TRACK_SPIN,
} move_tracker_mode;

void move_tracker_start(void);
bool get_trackingFinished();
void set_trackerMode(move_tracker_mode mode);
void reset_motor_pos_MOVETRACKER(void);

#endif /* MOVE_TRACKER_H_ */
