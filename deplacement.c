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
#include <pi_regulator.h>

//donne une valeur à chaque type d'ostacle: aucun, petit, moyen, mur
#define PAS_D_OBSTACLE 	0
#define PETIT_OBSTACLE	1
#define MOYEN_OBSTACLE	2
#define GRAND_OBSTACLE	3

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

//paramètres du moteur
#define VITESSE_NORMALE		600
#define VITESSE_LENTE		300
#define ARRET				0
#define ESPACE				3 //cm
#define PI                  3.1415926536f
#define WHEEL_DISTANCE      5.5f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
#define NSTEP_ONE_TURN		1000

//facteur rotatif pour placement => théorie != expérience. Facteurs trouvés expérimentalement
#define ROT_R20		17.2f	//18*20 = 360
#define ROT_R50		6		//7.2*50 = 360
#define ROT_R90		3.5f	//4*90 = 360
#define ROT_R160	1.6f	//2.25*160 = 360

/*
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

uint16_t ir_sensor[NB_IR_SENSORS] = {0};
uint8_t max_ir_value = 0;
uint8_t ir_sensor_nb = 0;
uint8_t type_obstacle = PAS_D_OBSTACLE;

//pour mettre les valeurs des IR dans le tableau et les transmettre à la fonction suivante
	for(uint8_t i = 0; i < NB_IR_SENSORS; i++)
	{
		ir_sensor[i] = get_prox(i);

		//concerver la plus haute valeur
		if(i=0)
		{
			max_ir_value = ir_sensor[i];
			ir_sensor_nb = i;
		}
		if(i>0)
		{
			if(ir_sensor[i] > ir_sensor[i-1])
			{
				max_ir_value = ir_sensor[i];
				ir_sensor_nb = i;
			}
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
//Si ce if est faux, le robot reste dans la thread pour suivre le son
*/

void lieu_obstacle(uint8_t ir_sensor_nb)//PENSER A VIRER LES MAGIC NUMBERS
{
	//je veux faire un switch case, pas sûr que ça marche bien...
	/*
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
	 */
	//se place face à l'ostacle et recule de 5cm pour pouvoir ensuite analyser l'ostacle
	if(ir_sensor_nb == IR_20_RIGHT)
	{
		face_obstacle(ROT_R20, CLOCKWISE);
	}
	if(ir_sensor_nb == IR_50_RIGHT)
	{
		face_obstacle(ROT_R50, CLOCKWISE);
	}
	if(ir_sensor_nb == IR_90_RIGHT)
	{
		face_obstacle(ROT_R90, CLOCKWISE);
	}
	if(ir_sensor_nb == IR_160_RIGHT)
	{
		face_obstacle(ROT_R160, CLOCKWISE);
	}
	if(ir_sensor_nb == IR_20_LEFT)
	{
		face_obstacle(ROT_R20, COUNTER_CLOCKWISE);
	}
	if(ir_sensor_nb == IR_50_LEFT)
	{
		face_obstacle(ROT_R50, COUNTER_CLOCKWISE);
	}
	if(ir_sensor_nb == IR_90_LEFT)
	{
		face_obstacle(ROT_R90, COUNTER_CLOCKWISE);
	}
	if(ir_sensor_nb == IR_160_LEFT)
	{
		face_obstacle(ROT_R160, COUNTER_CLOCKWISE);
	}
}

void face_obstacle(float facteur_rotatif, uint8_t sens)
{
	//idem switch case...
	/*
	uint32_t nb_steps = 0;
	float distance = 0;
	float goal = (PERIMETER_EPUCK/facteur_rotatif);

	switch(sens)
	{
								//Rotation de (360/facteur_rotatif)° dans le sens horaire
		case CLOCKWISE:			reset_motor_count();
								right_motor_set_speed(-VITESSE_LENTE);
								left_motor_set_speed(VITESSE_LENTE);

								while(position < goal)
								{
									nb_steps = left_motor_get_pos();
									position = ((nb_steps*PERIMETER_EPUCK)/NSTEP_ONE_TURN);
								}

								sens = RECULE;
								break;
								//Rotation de (360/facteur_rotatif)° dans le sens anti-horaire
		case COUNTER_CLOCKWISE:	reset_motor_count();
								right_motor_set_speed(VITESSE_LENTE);
								left_motor_set_speed(-VITESSE_LENTE);

								while(position < goal)
								{
									nb_steps = right_motor_get_pos();
									position = ((nb_steps*PERIMETER_EPUCK)/NSTEP_ONE_TURN);
								}

								sens = RECULE;
								break;
								//recule de 3cm
		case RECULE:			reset_motor_count();
								position = 0;

								right_motor_set_speed(-VITESSE_LENTE);
								left_motor_set_speed(-VITESSE_LENTE);

								while(position < ESPACE)
								{
									nb_steps = left_motor_get_pos();
									position = ((nb_steps*PERIMETER_EPUCK)/NSTEP_ONE_TURN);
								}
	}
	 */
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
	//recule de 3cm
	reset_motor_count();
	position = 0;

	right_motor_set_speed(-VITESSE_LENTE);
	left_motor_set_speed(-VITESSE_LENTE);

	while(position < ESPACE)
	{
		nb_steps = left_motor_get_pos();
		position = ((nb_steps*PERIMETER_EPUCK)/NSTEP_ONE_TURN);
	}

	right_motor_set_speed(ARRET);
	left_motor_set_speed(ARRET);
}

uint16_t taille_obstacle(void)
{

}
