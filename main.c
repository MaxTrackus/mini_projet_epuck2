#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>
#include <msgbus/messagebus.h>
#include <spi_comm.h>
#include <sensors/proximity.h>
#include <sensors/VL53L0X/VL53L0X.h>

#include <main.h>
#include <central_unit.h>
#include <process_image.h>
#include <move.h>
#include <proxi.h>
#include <move_tracker.h>
#include <p_regulator.h>



messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);

	//TUTO how to print in shell
//   chprintf((BaseSequentialStream *)&SD3, "pos=%d", get_line_position());
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //start the USB communication
    usb_start();
    //starts the camera
    dcmi_start();
	po8030_start();
	//inits the motors
	motors_init();
	//inits the proximity sensors
    proximity_start();
    //starts the TOF thread
    VL53L0X_start(); // a voir si on veut pas le mettre dans central unit avec thd stop...
    //starts the serial communication
    serial_start();

	//calibrate proximity sensors
    calibrate_ir();

    //start thread for proximity sensors
    read_prox_start();

	//stars the threads for the pi regulator and the processing of the image
	p_regulator_start();
	process_image_start();
	move_start();
	central_unit_start();
	move_tracker_start();

    /* Infinite loop. */
    while (1) {
    	//waits 1 second for the init to finish
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
