#ifndef SENSORS_H
#define SENSORS_H

//float get_distance_cm(void);
//uint16_t get_line_position(void);
//void process_image_start(void);
void set_led_with_int(unsigned int led_int_number);
void clear_led_with_int(unsigned int led_int_number);
void test_prox_with_leds(unsigned int sensor_number);
void read_IR_start(void);
void obstacles_avoidance_algorithm(void);
void rotate_left(int speed);
void rotate_right(int speed);
void rotate_right_in_degrees(int speed, float degrees);
void motor_stop(void);

#endif /* SENSORS_H */
