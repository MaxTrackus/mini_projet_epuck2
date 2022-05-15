#ifndef MOVE_H
#define MOVE_H

//List of the different mode in the move.c file, i.e the different configurations of the motors
typedef enum {
	STOP_MOVE,
	SPIN_RIGHT,
	SPIN_LEFT,
	MOVE_STRAIGHT,
	SPIN_ALIGNEMENT,
	MOVE_STRAIGHT_CORRECT_ALIGNEMENT,
	MOVE_STRAIGHT_WITH_CORRECTION,
} move_mode;

/**
* @brief   Init a thread which provides the moves needed by central_unit
*/
void move_start(void);

/**
* @brief   Set the speed of rotation or movement of the robot. Constrained by the define MAX_MOTOR_SPEED
*
* @param speed			speed in step/second
*/
void set_movingSpeed(int speed);

/**
* @brief   Set the correct mode of move and update the speed correction for FOLLOW mode
*
* @param leftSpeedCorrection			speed added/subtracted to correct trajectory
*/
void follow_wall_with_speed_correction(int16_t leftSpeedCorrection);

/**
* @brief   Set the mode of move. Used by central unit to control movements
*
* @param mode			mode of type move_mode
*/
void update_currentModeOfMove(move_mode mode);

#endif /* MOVE_H */
//uint16_t angle_in_degree
