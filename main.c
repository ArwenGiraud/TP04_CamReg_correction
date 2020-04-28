#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <stm32f4xx.h>
#include <system_clock_config.h>
#include <gpio.h>
#include <timer.h>
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <gpio_motor.h>
#include <camera/po8030.h>
#include <chprintf.h>
#include <i2c_bus.h>
#include <sensors/imu.h>
#include <sensors/mpu9250.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <sensors/proximity.h>
#include <spi_comm.h>

#include <pi_regulator.h>
#include <process_image.h>

//Pour utiliser le capteur de distances
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

//void SendUint8ToComputer(uint8_t* data, uint16_t size)
//{
	//chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	//chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	//chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
//}

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
	SystemClock_Config();

    halInit();
    chSysInit();
    mpu_init();

    // Enable GPIOD peripheral clock
    RCC->AHB1ENR    |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIODEN;

    //config the timer with a given duty_cycle
    timer4_PWM_start(0);

    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();
    //starts the camera
    dcmi_start();
	po8030_start();
	//inits the motors
	motors_init();
	gpio_motor_init();

	//initialisation du capteur de distance
	messagebus_init(&bus, &bus_lock, &bus_condvar);

	proximity_start();
	VL53L0X_start();
	//calibrate_ir();

	//stars the threads for the pi regulator and the processing of the image
	//pi_regulator_start();
	process_image_start();

    /* Infinite loop. */
    while (1) {

    	//waits 1 second
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
