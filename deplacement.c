#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <motors.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <sensors/proximity.h>
#include <deplacement.h>
#include <main.h>
#include <process_image.h>

//donne une valeur à chaque type d'ostacle: aucun, petit, moyen, mur
#define PAS_D_OBSTACLE 	0
#define PETIT_OBSTACLE	1	//diamètre 2cm
#define MOYEN_OBSTACLE	2	//diamètre 4cm
#define GRAND_OBSTACLE	3	//mur

//donne les valeur pour le sens de rotation
#define CLOCKWISE			0
#define COUNTER_CLOCKWISE	1
#define RECULE				2

//donner une valeur pour chaque senseur IR + définir leurs paramètres de fonctionnement
//20 = à 20°, 45 = à 45° et 90 = à 90°
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

//paramètres du moteur et camera
#define VITESSE_NORMALE		600
#define VITESSE_LENTE		300
#define ARRET				0
#define ESPACE				5 //cm
#define PI                  3.1415926536f
#define WHEEL_DISTANCE      5.5f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
#define NSTEP_ONE_TURN		1000

//facteur rotatif pour placement => théorie != expérience. Facteurs trouvés expérimentalement
#define ROT_R20		17.2f	//18*20 = 360
#define ROT_R50		6		//7.2*50 = 360
#define ROT_R90		3.5f	//4*90 = 360
#define ROT_R160	1.6f	//2.25*160 = 360
#define ROT_R180	1.5f

static uint8_t contournement = 0;

void lieu_obstacle(uint8_t ir_sensor_nb)//PENSER A VIRER LES MAGIC NUMBERS
{
	//je veux faire un switch case, pas sûr que ça marche bien...

	switch(ir_sensor_nb)
	{
		case IR_20_RIGHT:	face_obstacle(ROT_R20, CLOCKWISE);
							break;
		case IR_50_RIGHT:	face_obstacle(ROT_R50, CLOCKWISE);
							break;
		case IR_90_RIGHT:	face_obstacle(ROT_R90, CLOCKWISE);
							break;
		case IR_160_RIGHT:	face_obstacle(ROT_R160, CLOCKWISE);
							break;
		case IR_160_LEFT:	face_obstacle(ROT_R20, COUNTER_CLOCKWISE);
							break;
		case IR_90_LEFT:	face_obstacle(ROT_R50, COUNTER_CLOCKWISE);
							break;
		case IR_50_LEFT:	face_obstacle(ROT_R90, COUNTER_CLOCKWISE);
							break;
		case IR_20_LEFT:	face_obstacle(ROT_R160, COUNTER_CLOCKWISE);
							break;
	}
}

void face_obstacle(float facteur_rotatif, uint8_t sens)
{
	//Etant donné qu'on ne cherche pas à compter les steps dans des situation ou on recule et on avance, j'ai modifié
	//le code de motors qui incrémente tjr le compteur de steps.
	uint32_t nb_steps = 0;
	float position = 0;
	float goal = (PERIMETER_EPUCK/facteur_rotatif);

	if(sens == CLOCKWISE)
	{
		//Rotation de (360/facteur_rotatif)° dans le sens horaire
		reset_motor_count();

		right_motor_set_speed(-VITESSE_LENTE);
		left_motor_set_speed(VITESSE_LENTE);

		while(position < goal)
		{
			nb_steps = left_motor_get_pos();
			position = ((nb_steps*PERIMETER_EPUCK)/NSTEP_ONE_TURN);
		}
	}
	if(sens == COUNTER_CLOCKWISE)
	{
		//Rotation de (360/facteur_rotatif)° dans le sens anti-horaire
		reset_motor_count();

		right_motor_set_speed(VITESSE_LENTE);
		left_motor_set_speed(-VITESSE_LENTE);

		while(position < goal)
		{
			nb_steps = right_motor_get_pos();
			position = ((nb_steps*PERIMETER_EPUCK)/NSTEP_ONE_TURN);
		}
	}

	if(!contournement)
	{
		//recule de 5cm
		reset_motor_count();
		position = 0;

		right_motor_set_speed(-VITESSE_LENTE);
		left_motor_set_speed(-VITESSE_LENTE);

		while(position < ESPACE)
		{
			nb_steps = left_motor_get_pos();
			position = ((nb_steps*PERIMETER_EPUCK)/NSTEP_ONE_TURN);
		}
	}
	else if(contournement)
	{
		//avance de 5cm
		reset_motor_count();
		position = 0;

		right_motor_set_speed(VITESSE_LENTE);
		left_motor_set_speed(VITESSE_LENTE);

		while(position < ESPACE)
		{
			nb_steps = left_motor_get_pos();
			position = ((nb_steps*PERIMETER_EPUCK)/NSTEP_ONE_TURN);
		}
	}

	right_motor_set_speed(ARRET);
	left_motor_set_speed(ARRET);
}

void contourne_obstacle(uint8_t type_obstacle)
{
	contournement = 1;

	switch(type_obstacle)
	{
	case PETIT_OBSTACLE:	face_obstacle(ROT_R90, CLOCKWISE);
							contournement = 0;
							break;
	case MOYEN_OBSTACLE:	face_obstacle(ROT_R90, COUNTER_CLOCKWISE);
							contournement = 0;
							break;
	case GRAND_OBSTACLE:	face_obstacle(ROT_R180, CLOCKWISE);
							contournement = 0;
							break;
	}
}
