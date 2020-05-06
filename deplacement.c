#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <motors.h>
#include <sensors/VL53L0X/VL53L0X.h>

#include <main.h>
#include <deplacement.h>
#include <process_image.h>
#include <audio_processing.h>
#include <selector.h>

//donne une valeur à chaque type d'ostacle: aucun, petit, moyen, mur
#define ERREUR 	0
#define PETIT_OBSTACLE	1	//diamètre 2cm
#define MOYEN_OBSTACLE	2	//diamètre 4cm
#define GRAND_OBSTACLE	3	//mur

//donne les valeur pour le sens de rotation
#define CLOCKWISE			0
#define COUNTER_CLOCKWISE	1
#define RECULE				2

//paramètres du moteur et camera
#define VITESSE_LENTE		300
#define ARRET				0
#define ESPACE				4 //cm
#define PI                  3.1415926536f
#define WHEEL_DISTANCE      5.5f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
#define NSTEP_ONE_TURN		1000

//facteur rotatif pour placement => théorie != expérience. Facteurs trouvés expérimentalement
#define ROT_R90		3.5f	//4*90 = 360
#define ROT_R180	1.5f

#define NORMAL			0
#define OBSTACLE_MAN	1
#define OBSTACLE_AUTO	2

static bool guidage = false;

void recule(void)
{
	uint32_t nb_steps = 0;
	reset_motor_count();
	float position = 0;

	//recule de 5cm
	right_motor_set_speed(-VITESSE_LENTE);
	left_motor_set_speed(-VITESSE_LENTE);

	while(position < ESPACE)
	{
		nb_steps = left_motor_get_pos();
		position = ((nb_steps*PERIMETER_EPUCK)/NSTEP_ONE_TURN);
	}
}

void tourne(float facteur_rotatif, uint8_t sens)
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
	else if(sens == COUNTER_CLOCKWISE)
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

	right_motor_set_speed(ARRET);
	left_motor_set_speed(ARRET);
}

void contourne_obstacle(uint8_t type_obstacle)
{
	switch(type_obstacle)
	{
	case ERREUR:
		right_motor_set_speed(ARRET);
		left_motor_set_speed(ARRET);
		break;
	case PETIT_OBSTACLE:
		tourne(ROT_R90, CLOCKWISE);
		break;
	case MOYEN_OBSTACLE:
		tourne(ROT_R90, COUNTER_CLOCKWISE);
		break;
	case GRAND_OBSTACLE:
		tourne(ROT_R180, CLOCKWISE);
		break;
	}
}

static THD_WORKING_AREA(waDeplacement, 1024);
static THD_FUNCTION(Deplacement, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	systime_t time;
	uint16_t taille_obstacle = 0;
	uint8_t mode = NORMAL;

	// Pour initialisation plus fiable
	chThdSleepMilliseconds(300);

		while(1)
		{
			time = chVTGetSystemTime();

			uint16_t dist_mm = VL53L0X_get_dist_mm();
			uint8_t selector = get_selector_mode();

			if((dist_mm < 50) && (selector == OBSTACLE_AUTO))
			{
				mode = OBSTACLE_AUTO;
			}
			else if(((dist_mm < 50) || guidage) && (selector == OBSTACLE_MAN))
			{
				guidage = true;
				mode = OBSTACLE_MAN;
			}
			else
			{
				mode = NORMAL;
			}

			switch(mode)
			{
			case NORMAL:
				sound_remote();
				break;
			case OBSTACLE_MAN:
				guidage = sound_manuel_remote();
				break;
			case OBSTACLE_AUTO:
				recule();
				taille_obstacle = get_taille_obstacle();
				contourne_obstacle(taille_obstacle);
				break;
			}
			chThdSleepUntilWindowed(time, time + MS2ST(10));
		}
}

void deplacement_start(void)
{
	chThdCreateStatic(waDeplacement, sizeof(waDeplacement), NORMALPRIO+1, Deplacement, NULL);
}
