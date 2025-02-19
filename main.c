#include "ch.h"
#include "hal.h"
#include "memory_protection.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <stm32f4xx.h>

#include <motors.h>
#include <camera/po8030.h>
#include <audio/microphone.h>
#include <sensors/VL53L0X/VL53L0X.h>

#include <main.h>
#include <audio_processing.h>
#include <process_image.h>
#include <deplacement.h>
#include <selector.h>
#include <gpio.h>
#include <fft.h>

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

    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();

    //starts the camera
    dcmi_start();
	po8030_start();

	//inits the motors
	motors_init();

	//init le selecteur
	init_selector();

	VL53L0X_start();

	process_image_start();
	deplacement_start();

	//starts the microphones processing thread.
	//it calls the callback given in parameter when samples are ready
	mic_start(&processAudioData);

    /* Infinite loop. */
    while (1)
    {
    	if(!get_state())
		{
			left_motor_set_speed(0);
			right_motor_set_speed(0);
		}
	}
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
