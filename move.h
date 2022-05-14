#ifndef MOVE_H
#define MOVE_H

//List of the different mode in the move.c file, i.e the different configurations of the motors
typedef enum {
	STOP_MOVE,
	SPIN_RIGHT, //for the analyse mode of central unit
	SPIN_LEFT,
	MOVE_STRAIGHT,
	SPIN_ALIGNEMENT, //for the align mode of central unit
	MOVE_STRAIGHT_CORRECT_ALIGNEMENT, //for the pursuit mode of central unit
	MOVE_STRAIGHT_WITH_CORRECTION,
} move_mode;

void move_start(void);
bool toggle_boolean(bool x);
void stopMove(void);

void rotate_left(int speed);
void rotate_right(int speed);
void motor_stop(void);
void avoid_obstacles(int speed, int prox_detection_threshold);
void move_straight(int speed);
int motor_speed_protection(int speed);
void set_movingSpeed(int speed);
void update_currentModeOfMove(move_mode mode);
void reset_motor_pos(void);
int32_t get_right_motor_pos(void);
int32_t get_left_motor_pos(void);

// follow mode
void follow_left_wall_with_speed_correction(int16_t leftSpeedCorrection);

#endif /* MOVE_H */
//uint16_t angle_in_degree
