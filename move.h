#ifndef MOVE_H
#define MOVE_H

void spin_angle_degree(uint16_t angle_in_degree);
void move_start(void);
bool toggle_boolean(bool x);
void stopMove(void);

void rotate_left(int speed);
void rotate_right(int speed);
void rotate_right_in_degrees(int speed, float degrees);
void motor_stop(void);

#endif /* MOVE_H */
//uint16_t angle_in_degree
