#ifndef CENTRAL_UNIT_H
#define CENTRAL_UNIT_H

//List of the different modes, i.e the different tasks that the robot must perform for our application
typedef enum {
	STOP,
	WAIT_MOVING,
	ANALYSE,
	ALIGN,
	AVOID,
	PURSUIT,
} task_mode;

//start the central unit thread
void central_unit_start(void);

#endif /* CENTRAL_UNIT_H */
