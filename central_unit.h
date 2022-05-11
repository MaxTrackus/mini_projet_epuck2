#ifndef CENTRAL_UNIT_H
#define CENTRAL_UNIT_H

//List of the different mode, i.e the different tasks that the robot must perform for our application
typedef enum {
	STOP,
	ANALYSE,
	ALIGN,
	PURSUIT,
} task_mode;

//start the central unit thread
void central_unit_start(void);

void set_mode_with_selector(void);

#endif /* CENTRAL_UNIT_H */
