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
void stop_tracker(void);
bool get_trackingFinished(void);
void set_trackerMode(move_tracker_mode mode);
void trackRotationOfDegree(int16_t degree);
void reset_motor_pos_TRACKER(void);
bool get_trackerIsUsed(void);

#endif /* MOVE_TRACKER_H_ */
