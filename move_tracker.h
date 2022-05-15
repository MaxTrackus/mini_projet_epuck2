#ifndef MOVE_TRACKER_H_
#define MOVE_TRACKER_H_

//List of the different mode in the move_tracker.c file
typedef enum {
	TRACK_NOTHING,
	TRACK_SPIN,
	TRACK_STRAIGHT,
	ROTATION_MAPPING
} move_tracker_mode;

/**
* @brief   Init a thread that provide a flag when the given angle or distance is reached
*/
void move_tracker_start(void);

/**
* @brief   Reinitialize variables and stop tracking
*/
void stop_tracker(void);

/**
* @brief   Return the flag storing if the intended angle or distance is reached
*
* @return			boolean flag
*/
bool get_trackingFinished(void);

/**
* @brief   Return the flag storing if the thread is busy.
* 		   This flag used in central unit to avoid the multiples calls of the functions
* 		   that request a tracking.
*
* @return			boolean flag
*/
bool get_trackerIsUsed(void);

/**
* @brief   Give control of what is done in the thread.
*
* @param mode			mode of the tracker for the switch in the thread
*/
void set_trackerMode(move_tracker_mode mode);

/**
* @brief   Initiate the tracking of rotation. Call get_finished repeatedly to be informed
* 		   when the desired angle is reached
*
* @param degree			desired degree. (degree > 0) => clockwise
* 										(degree < 0) => counterclockwise
*/
void trackRotationOfDegree(int16_t degree);

/**
* @brief   Initiate the tracking of advance forward or backward. Call get_finished
* 		   repeatedly to be informed when the desired angle is reached
*
* @param milimeters			desired distance in mm. (milimeters > 0) => forward
* 													(milimeters < 0) => backward
*/
void trackStraightAdvance(int16_t milimeters);

/**
* @brief   Switch on or off the rotation mapping
*
* @param status			boolean. true => ON
* 								 false => OFF
*/
void set_rotationMappingIsOn(bool status);

/**
* @brief   Return the value of the rotation mapping
*
* @return			number of step
*/
int get_rotationMappingValue(void);

#endif /* MOVE_TRACKER_H_ */
