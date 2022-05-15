#ifndef CONSTANTS_H_
#define CONSTANTS_H_

// process_image
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40
#define PXTOCM					1570.0f //experimental value
#define GOAL_DISTANCE 			10.0f
#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera
#define CAPTURE_ONE_SHOT

//p_regulator
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2
#define KP						2.0f
#define KI 						0.01f	//must not be zero

//central_unit
#define DEFAULT_SPEED								200 	// [steps/s]
#define SLOW_SPEED									50 		// [steps/s]
#define FAST_SPEED									400 	// [steps/s]
#define	OBJECT_DIAMETER								30 		// [mm]
#define ARENA_RADIUS								250 	// [mm]
#define EXIT_DISTANCE								100		// [mm]
#define	PROX_DETECTION_THRESHOLD					10
#define	SPEED_CORRECTION_SENSIBILITY_OVER_PROXI		2
#define	GOAL_PROXI_VALUE							150
#define THRESHOLD_STEPS_FOR_OPTIMIZED_EXIT			620 	// [steps]
#define TRACKING_ERROR 								0.05
#define COMPLETE_TURN								360 	// degrees

//move
#define MAX_MOTOR_SPEED					1100	// [steps/s]

//move_tracker
#define NSTEP_ONE_TURN      			1000	// number of step for 1 turn of the motor
#define WHEEL_PERIMETER     			130		// [mm]
#define DEG2RAD						M_PI/180
#define TRACK_WIDTH					51 //distance between the wheels [mm]

//proxi
#define NB_PROX_SENSORS		8

//#define QUARTER_TURN					90
//#define MOTOR_STEP_TO_DEGREES			360 //find other name maybe
//#define THRESHOLD_ANGLE_FOR_OPTIMIZED_EXIT			180 // [deg]
//#define MAX_SPIN_ANGLE		360
//#define MOTOR_SPEED_LIMIT 				1100 	// [step/s]
//#define ERROR_MARGIN					75 		// [mm]
//#define WALL_CLEARANCE					10 		// [mm]
//#define SEC2MSEC						1000
//#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)

#endif /* CONSTANTS_H_ */
