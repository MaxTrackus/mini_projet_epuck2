#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

//List of the different modes, i.e the different tasks that the robot must perform for our application
typedef enum {
	NOTHING,
	ALIGN_ROTATION,
	PURSUIT_CORRECTION,
} regulation_mode;

bool get_regulationCompleted(void);

void set_currentRegulatorMode(regulation_mode mode);

//start the PI regulator thread
void pi_regulator_start(void);

#endif /* PI_REGULATOR_H */
