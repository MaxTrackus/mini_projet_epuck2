#ifndef CENTRAL_UNIT_H
#define CENTRAL_UNIT_H

//List of the different mode, i.e the different tasks that the robot must perform for our application
typedef enum {
	IDLE,
	STOP,
	ANALYSE,
	ALIGN,
	PURSUIT,
	MEASURE_TOF,
	MEASURE_SPIN_RIGHT,
	MEASURE_SPIN_LEFT,
	PUSH,
	ROTATE_BEFORE_FOLLOW,
	FOLLOW,
	EXIT,
	PUSH_OUT,
	RECENTER,
	ROTATE_TRACKER_TEST,
	STRAIGHT_TRACKER_TEST,
	TEST_ROTATION_MAPPING
} task_mode;


/**
* @brief   Init a thread which gather the sensors measurements and updates the modes of move.c accordingly
*/
void central_unit_start(void);

#endif /* CENTRAL_UNIT_H */
