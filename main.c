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
#include <chprintf.h>
#include <i2c_bus.h>
#include <sensors/imu.h>
#include <sensors/mpu9250.h>
//#include <sensors/VL53L0X/VL53L0X.h>
#include <sensors/proximity.h>
#include <spi_comm.h>

//#include <pi_regulator.h>
#include <process_image.h>
#include <deplacement.h>

#define MAX_IR_VALUE	200
#define NB_IR_SENSORS	8
#define IR_20_RIGHT		0
#define IR_50_RIGHT		1
#define IR_90_RIGHT		2
#define IR_160_RIGHT	3
#define IR_160_LEFT		4
#define IR_90_LEFT		5
#define IR_50_LEFT		6
#define IR_20_LEFT		7
#define PAS_D_OBSTACLE 	0

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

	//initialisation du capteur de distance
	messagebus_init(&bus, &bus_lock, &bus_condvar);

	proximity_start();
	calibrate_ir();

	//stars the threads for the pi regulator and the processing of the image
	//pi_regulator_start();
	process_image_start();

	uint16_t ir_sensor[NB_IR_SENSORS] = {0};
	uint8_t max_ir_value = 0;
	uint8_t ir_sensor_nb = 0;
	uint8_t type_obstacle = PAS_D_OBSTACLE;

	//waits 3 second
	chThdSleepMilliseconds(3000);

	right_motor_set_speed(300);
	left_motor_set_speed(300);

    /* Infinite loop. */
    while (1)
    {
    	//pour mettre les valeurs des IR dans le tableau et les transmettre à la fonction suivante
		for(uint8_t i = 0; i < NB_IR_SENSORS; i++)
		{
			ir_sensor[i] = get_prox(i);
		}

		max_ir_value = ir_sensor[IR_20_RIGHT];
		ir_sensor_nb = IR_20_RIGHT;

		//concerver la plus haute valeur
		for(uint8_t i = 1; i < NB_IR_SENSORS; i++)
		{
			if(ir_sensor[i] > max_ir_value)
			{
				max_ir_value = ir_sensor[i];
				ir_sensor_nb = i;
			}
		}

	//vérification si l'une des valeurs est trop grande
		if(max_ir_value > MAX_IR_VALUE)
		{
			lieu_obstacle(ir_sensor_nb);
			//on peut facilement définir la taille de l'obstacle car on sait à quelle distance on en est
			type_obstacle = taille_obstacle();
			contourne_obstacle(type_obstacle);
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
