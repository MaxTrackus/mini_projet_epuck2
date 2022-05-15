#ifndef PROXI_H
#define PROXI_H

// Id proximity sensor
#define PROX_FRONT_RIGHT_17 			0 // IR 1
#define PROX_FRONT_LEFT_17				7 // IR 8
#define PROX_FRONT_RIGHT_49 			1 // IR 2
#define PROX_FRONT_LEFT_49				6 // IR 7
#define PROX_RIGHT						2 // IR 3
#define PROX_BACK_RIGHT 				3 // IR 4
#define PROX_LEFT						5 // IR 6
#define PROX_BACK_LEFT					4 // IR 5

/**
* @brief   Init a thread which uses the proximity sensors to continuously store every sensor's value inside a buffer
*/
void read_prox_start(void);

/**
* @brief   Gives an information on whether the proximity sensors's value are above a certain threshold
*
* @param prox_detection_threshold			Activation threshold value (sensor's max value is 4096)
* 
* @return			Address of boolean buffer with activation status for the proximity sensors
*/
bool* get_prox_activation_status(uint16_t prox_detection_threshold);

/**
* @brief   Returns address of buffer with the calibration value for every sensor
* 
* @return			Address of boolean buffer with activation status for the proximity sensors
*/
uint16_t* get_prox_value(void);

#endif /* PROXI_H */
