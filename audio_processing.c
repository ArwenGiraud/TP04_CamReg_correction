#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
//#include <communications.h>
#include <deplacement.h>
#include <process_image.h>
#include <fft.h>
#include <arm_math.h>

//semaphore permet d'assurer l'unicité de l'accès à l'information => sémaphore binaire soit libre soit bloquée
//lorsque bloquée on voit le nombre de processus en attente. Dans une sémaphore pas de notion de priorité
//Cette sémaphore permet d'assurer qu'une seul chose soit envoyé à la fois à l'ordinateur
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
//Tableau contenant les echantillons de chaque micro
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
//Même tableau mais pour la sortie => donc sous forme de magnitude
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];

//Qu'est ce que c'est  D'où ça sort?
#define MIN_VALUE_THRESHOLD	35000 //A AUGMENTER SI BOUGE QUAND IL N'Y A PAS DE FREQUENCE A 250 Hz
#define NB_MIC				3	//on utilise pas celui du bas
#define MIN_FREQ			10	//we don't analyze before this index to not use resources for nothing
#define MAX_FREQ			30	//we don't analyze after this index to not use resources for nothing
#define NO_SOUND			4
#define REMOTE_FREQ			26 // 406 Hz
#define REMOTE_FREQ_H		(REMOTE_FREQ+3)
#define REMOTE_FREQ_L		(REMOTE_FREQ-3)

//Defines pour le mode manuel en collision
#define FREQ_FORWARD	16	//250Hz
#define FREQ_LEFT		19	//296Hz
#define FREQ_RIGHT		23	//359HZ
#define FREQ_BACKWARD	26	//406Hz

#define FREQ_FORWARD_L		(FREQ_FORWARD-1)
#define FREQ_FORWARD_H		(FREQ_FORWARD+1)
#define FREQ_LEFT_L			(FREQ_LEFT-1)
#define FREQ_LEFT_H			(FREQ_LEFT+1)
#define FREQ_RIGHT_L		(FREQ_RIGHT-1)
#define FREQ_RIGHT_H		(FREQ_RIGHT+1)
#define FREQ_BACKWARD_L		(FREQ_BACKWARD-1)
#define FREQ_BACKWARD_H		(FREQ_BACKWARD+1)

#define TOLERANCE_FRONT		10000
#define TOLERANCE_BACK		5000

#define SOUND_OFF	0
#define SOUND_ON	1

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

static uint8_t state = SOUND_OFF;
static bool analyse = false;

// Simple delay function
void delay(unsigned int n)
{
    while (n--) {
        __asm__ volatile ("nop");
    }
}

/*
*	Simple function used to detect the highest value in a buffer
*/
//Détecte la magnitude max pour le signal d'un micro
float max_magnitude_define(float* data)
{
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1;

	//Cherche la valeur de la magnitude pour environ 406 Hz
	for(uint16_t i = REMOTE_FREQ_L ; i <= REMOTE_FREQ_H ; i++) //CHERCHE POUR UNE FREQ DE 406 Hz
	{
		if(data[i] > max_norm)
		{
			max_norm = data[i];
			max_norm_index = i;
		}
		//chprintf((BaseSequentialStream *)&SD3, "frequence: %d\r\n", max_norm_index);
	}
	return max_norm;
}

//Possibilité de fusionner les fonctions avec if mode = MANUEL -> return index mais pb
//avec la fonction process audio data  (pas très beau)
/*
void sound_manuel_remote(float* data)
{
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1;

	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++)
	{
		if(data[i] > max_norm)
		{
			max_norm = data[i];
			max_norm_index = i;
		}
	}
	//go forward
	if(max_norm_index >= FREQ_FORWARD_L && max_norm_index <= FREQ_FORWARD_H){
		left_motor_set_speed(600);
		right_motor_set_speed(600);
	}
	//turn left
	else if(max_norm_index >= FREQ_LEFT_L && max_norm_index <= FREQ_LEFT_H){
		left_motor_set_speed(-600);
		right_motor_set_speed(600);
	}
	//turn right
	else if(max_norm_index >= FREQ_RIGHT_L && max_norm_index <= FREQ_RIGHT_H){
		left_motor_set_speed(600);
		right_motor_set_speed(-600);
	}
	//go backward
	else if(max_norm_index >= FREQ_BACKWARD_L && max_norm_index <= FREQ_BACKWARD_H){
		left_motor_set_speed(-600);
		right_motor_set_speed(-600);
	}
	else{
		left_motor_set_speed(0);
		right_motor_set_speed(0);
	}

}
*/
/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/

//Collecte les données pour chaque micro, détecte quel micro reçoit un signal maximum
//appelle la fonction de commande des moteurs.
void processAudioData(int16_t *data, uint16_t num_samples)//, int mode)
{
	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static uint16_t nb_samples = 0;
	static uint8_t mustSend = 0;
	static float mag_max[NB_MIC] = {MIN_VALUE_THRESHOLD};
	static float val_mag_max = 0;

	static uint8_t active_mic_first = NO_SOUND;
	static uint8_t active_mic_second = NO_SOUND;

	uint16_t ir_sensor[NB_IR_SENSORS] = {0};
	uint8_t max_ir_value = 0;
	uint8_t ir_sensor_nb = 0;
	uint8_t type_obstacle = PAS_D_OBSTACLE;

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
//		analyse = true;

		lieu_obstacle(ir_sensor_nb);
		//on peut facilement définir la taille de l'obstacle car on sait à quelle distance on en est
		type_obstacle = 3;//get_taille_obstacle();
		contourne_obstacle(type_obstacle);

		//chThdSleepSeconds(5);

//		analyse = false;

	}

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4)
	{
		//chprintf((BaseSequentialStream *)&SDU1, "premiere boucle for pour remplir les tableaux");
		//construct an array of complex numbers. Put 0 to the imaginary part
		micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];
		//micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];

		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;
		micBack_cmplx_input[nb_samples] = 0;
		//micFront_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE))
		{
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE))
	{
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function.
		*/

		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		//doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);

		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);
		//arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);

		//sends only one FFT result over 10 for 1 mic to not flood the computer
		//sends to UART3
		//if(mustSend > 8)
		//{
			//chprintf((BaseSequentialStream *)&SD3, "mustSend");
			//signals to send the result to the computer
			//chBSemSignal(&sendToComputer_sem);
			//mustSend = 0;
		//}
		nb_samples = 0;
		//mustSend++;

		/*if (mode == MANUEL & COLLISION) //DEF COLLISION QQ PART
		{
			sound_manuel_remote(micRight_output);
		}
		else
		{*/
		mag_max[MIC_RIGHT] = max_magnitude_define(micRight_output);
		mag_max[MIC_LEFT] = max_magnitude_define(micLeft_output);
		mag_max[MIC_BACK] = max_magnitude_define(micBack_output);
		//mag_max[MIC_FRONT] = max_magnitude_define(micFront_output);

		if((mag_max[MIC_RIGHT]<=MIN_VALUE_THRESHOLD) && (mag_max[MIC_LEFT]<=MIN_VALUE_THRESHOLD) && (mag_max[MIC_BACK]<=MIN_VALUE_THRESHOLD))
		{
			state = SOUND_OFF;
		}
		else {state = SOUND_ON;}

		val_mag_max = MIN_VALUE_THRESHOLD;

		for(uint16_t i = 0; i< NB_MIC ; i++)
		{
			if (mag_max[i] > val_mag_max)
			{
				active_mic_first = i;
				val_mag_max = mag_max[i];
			}
		}

		val_mag_max = MIN_VALUE_THRESHOLD;

		for(int16_t i = 0; i< NB_MIC ; i++)
		{
			if(i != active_mic_first && mag_max[i] > val_mag_max)
			{
				active_mic_second = i;
				val_mag_max = mag_max[i];
			}
		}
		sound_remote(active_mic_first, active_mic_second);
	}
}

//Attendre pour envoyer des tableaux complets
//void wait_send_to_computer(void)
//{
	//chBSemWait(&sendToComputer_sem);
//}

//Obtenir les pointeurs sur les tableaux des micros
//ENCORE UTILISEE?
float* get_audio_buffer_ptr(BUFFER_NAME_t name)
{
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}
	else{
		return NULL;
	}
}

//Décide du déplacement selon le micro dominant
void sound_remote(uint8_t active_mic_first, uint8_t active_mic_second)
{
	//go forward
	if(((active_mic_first == MIC_LEFT) && (active_mic_second == MIC_RIGHT)) || ((active_mic_first == MIC_RIGHT) && (active_mic_second == MIC_LEFT) ))
	{
		left_motor_set_speed(600); //avant à 600
		right_motor_set_speed(600);
		delay(1000);

	}
	//go backward
	else if((active_mic_first == MIC_BACK))
	{
		left_motor_set_speed(-600);
		right_motor_set_speed(-600);
		delay(1000);
	}
	//turn left
	else if((active_mic_first == MIC_LEFT) && (active_mic_second == MIC_BACK))
	{
		left_motor_set_speed(-600);
		right_motor_set_speed(600);
		delay(1000);
	}
	//turn right
	else if((active_mic_first == MIC_RIGHT) && (active_mic_second == MIC_BACK))
	{
		left_motor_set_speed(600);
		right_motor_set_speed(-600);
		delay(1000);
	}

	else
	{
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		delay(1000);
	}

}
uint8_t get_sound_state(void)
{
	return state;
}

bool get_analyse_state(void)
{
	return analyse;
}
