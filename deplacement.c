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

//donner une valeur pour chaque senseur IR + définir leurs paramètres de fonctionnement
//20 = à 20°, 45 = à 45° et 90 = à 90°
#define MAX_IR_VALUE	200
#define NB_IR_SENSORS	8
#define IR_20_RIGHT		0
#define IR_50_RIGHT		1
#define IR_90_RIGHT		2
#define IR_90_LEFT		5
#define IR_50_LEFT		6
#define IR_20_LEFT		7

//paramètres du moteur
#define VITESSE				5 //cm/s
#define ESPACE				5 //cm
#define PI                  3.1415926536f
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)

//facteur rotatif pour placement
#define ROT_R20		18
#define ROT_R50		7		//en théorie 7.2 mais je toruve inutile de mettre un float pour cette petite nuance
#define ROT_R90		4

/*
#define BACK_SENSOR_RIGHT	3
#define BACK_SENSOR_LEFT	4

uint16_t ir_sensor[NB_IR_SENSORS] = {0};
uint8_t max_ir_value = 0;
uint8_t ir_sensor_nb = 0;
uint8_t type_obstacle = PAS_D_OBSTACLE;

//pour mettre les valeurs des IR dans le tableau et les transmettre à la fonction suivante
	for(uint8_t i = 0, i < NB_IR_SENSORS, i++)
	{
		//Comme on ne fait pas de marche arrière continue, on peut ignorer les capteurs arrières
		if((i == BACK_SENSOR_RIGHT) || (i == BACK_SENSOR_LEFT))
			{
			ir_sensor[i] = PAS_D_OBSTACLE;
			i++;
			}
		ir_sensor[i] = get_prox(i);

		//concerver la plus haute valeur
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

//Idée pour bouger le robot
/*
//move 5cm forward at 5cm/s
motor_set_position(5, 5, 5, 5);
while(motor_position_reached() != POSITION_REACHED);
//clockwise rotation of 180Â°
motor_set_position(PERIMETER_EPUCK/2, PERIMETER_EPUCK/2, -5, 5);
while(motor_position_reached() != POSITION_REACHED);
//move 5cm forward at 5cm/s
motor_set_position(5, 5, 5, 5);
while(motor_position_reached() != POSITION_REACHED);
//counterclockwise rotation of 180Â°
motor_set_position(PERIMETER_EPUCK/2, PERIMETER_EPUCK/2, 5, -5);
while(motor_position_reached() != POSITION_REACHED);
*/

void lieu_obstacle(uint8_t ir_sensor_nb)//PENSER A VIRER LES MAGIC NUMBERS
{
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
}

void face_obstacle(uint8_t facteur_rotatif, uint8_t sens)
{
	if(sens == CLOCKWISE)
	{
		//Rotation de (360/facteur_rotatif)° dans le sens horaire
		motor_set_position(PERIMETER_EPUCK/facteur_rotatif, PERIMETER_EPUCK/facteur_rotatif, -VITESSE, VITESSE);
	}
	if(sens == COUNTER_CLOCKWISE)
	{
		//Rotation de (360/facteur_rotatif)° dans le sens anti-horaire
		motor_set_position(PERIMETER_EPUCK/facteur_rotatif, PERIMETER_EPUCK/facteur_rotatif, VITESSE, -VITESSE);
	}
	//recule de 5cm à 5cm/s
	motor_set_position(ESPACE, ESPACE, -VITESSE, -VITESSE);
}

uint16_t taille_obstacle(void)
{

}
