#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

//List of the different modes, i.e the different tasks that the robot must perform for our application
typedef enum {
	NOTHING,
	ALIGN_ROTATION,
	PURSUIT_CORRECTION,
} regulation_mode;

/**
* @brief   Return a flag that store if the speed of the motor is very small during some time
* 		   and therefore the alignment is completed
*
* @return			Address of boolean buffer with activation status for the proximity sensors
*/
bool get_regulationCompleted(void);

/**
* @brief   Set what the regulator must control.
*
* @param mode			mode of the switch in the thread
*/
void set_currentRegulatorMode(regulation_mode mode);

/**
* @brief   Init a thread which compute continuously the speed for the motors for the alignement
*/
void p_regulator_start(void);

#endif /* PI_REGULATOR_H */
