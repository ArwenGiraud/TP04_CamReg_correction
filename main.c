#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <audio/microphone.h>
#include <chprintf.h>
#include <i2c_bus.h>
#include <sensors/imu.h>
#include <sensors/mpu9250.h>
//#include <sensors/VL53L0X/VL53L0X.h>
#include <sensors/proximity.h>
#include <spi_comm.h>

//#include <pi_regulator.h>
#include <audio_processing.h>
#include <process_image.h>
#include <deplacement.h>

//Pour utiliser le capteur de distances
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

//void SendUint8ToComputer(uint8_t* data, uint16_t size)
//{
//	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
//	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
//	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
//}

//uncomment to use double buffering to send the FFT to the computer
#define DOUBLE_BUFFERING

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

//static void timer12_start(void){
//    //General Purpose Timer configuration
//    //timer 12 is a 16 bit timer so we can measure time
//    //to about 65ms with a 1Mhz counter
//    static const GPTConfig gpt12cfg = {
//        1000000,        /* 1MHz timer clock in order to measure uS.*/
//        NULL,           /* Timer callback.*/
//        0,
//        0
//    };
//
//    gptStart(&GPTD12, &gpt12cfg);
//    //let the timer count to max value
//    gptStartContinuous(&GPTD12, 0xFFFF);
//}

int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();
    //starts timer 12
    //timer12_start();
    //starts the camera
    dcmi_start();
	po8030_start();
	//inits the motors
	motors_init();

	//temp tab used to store values in complex_float format
	//needed bx doFFT_c
	//static complex_float temp_tab[FFT_SIZE];
	//send_tab is used to save the state of the buffer to send (double buffering)
	//to avoid modifications of the buffer while sending it
	//static float send_tab[FFT_SIZE];

	//initialisation du capteur de distance
	messagebus_init(&bus, &bus_lock, &bus_condvar);

	proximity_start();
	calibrate_ir();

	//stars the threads for the pi regulator and the processing of the image
	//pi_regulator_start();
	//process_image_start();

	//starts the microphones processing thread.
	//it calls the callback given in parameter when samples are ready
	mic_start(&processAudioData);

	//waits 3 second
	chThdSleepMilliseconds(3000);

//	right_motor_set_speed(300);
//	left_motor_set_speed(300);

    /* Infinite loop. */
    while (1)
    {
    	if(!get_state())
		{
			left_motor_set_speed(0);
			right_motor_set_speed(0);
		}
			//waits 1 second
			//chThdSleepMilliseconds(1000);
	}
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
