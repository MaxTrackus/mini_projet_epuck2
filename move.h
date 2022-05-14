#ifndef MOVE_H
#define MOVE_H

//List of the different mode in the move.c file, i.e the different configurations of the motors
typedef enum {
	STOP_MOVE,
	SPIN_RIGHT, //for the analyse mode of central unit
	SPIN_LEFT, //to be added!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	MOVE_STRAIGHT,
	SPIN_ALIGNEMENT, //for the align mode of central unit
	MOVE_STRAIGHT_CORRECT_ALIGNEMENT, //for the pursuit mode of central unit
	//////////////////////////////////////////////////////////////////////// test_max_1205
	MOVE_STRAIGHT_WITH_LEFT_MOTOR_CORRECTION,
	//////////////////////////////////////////////////////////////////////// test_max_1205
} move_mode;

typedef enum {
	WAITING, 
	STARTED,
	COMPLETED,
} action_status;

void move_start(void);
bool toggle_boolean(bool x);
void stopMove(void);
void update_currentModeOfMove(move_mode mode);

void rotate_left(int speed);
void rotate_right(int speed);
void rotate_in_degrees(int speed, int degrees);
void motor_stop(void);
void avoid_obstacles(int speed, int prox_detection_threshold);
void move_straight(int speed);
int motor_speed_protection(int speed);
void set_movingSpeed(int speed);
void update_currentModeOfMove(move_mode mode);
void set_rotationMappingIsOn(bool status);
int get_rotationMappingValue(void);
action_status get_movementStatus(void);
void set_movementStatus(action_status status)
void reset_motor_pos(void);
uint32_t get_right_motor_pos(void);
uint32_t get_left_motor_pos(void);

//////////////////////////////////////////////////////////////////////// test_max_1205
void follow_left_wall_with_speed_correction(int16_t leftSpeedCorrection);
//////////////////////////////////////////////////////////////////////// test_max_1205

#endif /* MOVE_H */
//uint16_t angle_in_degree
