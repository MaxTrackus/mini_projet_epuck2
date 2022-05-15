#ifndef CONSTANTS_H_
#define CONSTANTS_H_

//PROCESS_IMAGE.C
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40
#define PXTOCM					1570.0f // experimental value
#define GOAL_DISTANCE 			10.0f
#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD			0.1f	// [cm] because of the noise of the camera
#define CAPTURE_ONE_SHOT

//P_REGULATOR.C
#define ROTATION_THRESHOLD							10
#define ROTATION_COEFF								2
#define KP											2.0f
#define KI 											0.01f	//must not be zero
#define NB_INCREMENTS_VALIDATION_ALIGNEMENT			200

//CENTRAL_UNIT.C
#define DEFAULT_SPEED								200 	// [steps/s]
#define SLOW_SPEED									50 		// [steps/s]
#define FAST_SPEED									400 	// [steps/s]
#define	OBJECT_DIAMETER								30 		// [mm]
#define ARENA_RADIUS								250 	// [mm]
#define EXIT_DISTANCE								100		// [mm]
#define THRESHOLD_STEPS_FOR_OPTIMIZED_EXIT			620 	// [steps]
#define TRACKING_ERROR 								0.05
#define NB_INCREMENTS_LOST_OBJECT					100
#define LARGE_WIDTH_STOP							400		// [pixels]
//proxi
#define	PROX_DETECTION_THRESHOLD					10
#define	SPEED_CORRECTION_SENSIBILITY_OVER_PROXI		2
#define	GOAL_PROXI_VALUE							150
//TOF
#define NB_TOF_MESURE_MEAN							20
#define TOF_OFFSET									5 		// [mm]
// angles
#define TWENTY_DEGREES								20 		// [degrees]
#define QUARTER_TURN								90		// [degrees]
#define	SIXTY_DEGREES								60		// [degrees]
#define COMPLETE_TURN								360 	// [degrees]
//selector
#define FIFTEENTH_POS_SELECTOR						15

//MOVE.C
#define MAX_MOTOR_SPEED			1100	// [steps/s]

//MOVE_TRACKER.C
#define NSTEP_ONE_TURN      	1000	// number of step for 1 turn of the motor
#define WHEEL_PERIMETER     	130		// [mm]
#define DEG2RAD					M_PI/180
#define TRACK_WIDTH				51 		//distance between the wheels [mm]

//PROXI.C
#define NB_PROX_SENSORS			8

#endif /* CONSTANTS_H_ */
