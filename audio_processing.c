#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>
#include <arm_math.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <sensors/VL53L0X/VL53L0X.h>

#include <main.h>
#include <deplacement.h>
#include <process_image.h>
#include <fft.h>

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
//Tableau contenant les echantillons de chaque micro
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
//M�me tableau mais pour la sortie => donc sous forme de magnitude
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];

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

#define SOUND_OFF	0
#define SOUND_ON	1

#define PAS_D_OBSTACLE	0

static uint8_t state = SOUND_OFF;

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
//D�tecte la magnitude max pour le signal d'un micro
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

//Possibilit� de fusionner les fonctions avec if mode = MANUEL -> return index mais pb
//avec la fonction process audio data  (pas tr�s beau)
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

//Collecte les donn�es pour chaque micro, d�tecte quel micro re�oit un signal maximum
//appelle la fonction de commande des moteurs.
void processAudioData(int16_t *data, uint16_t num_samples)//, int mode)
{
	static uint16_t nb_samples = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4)
	{
		//construct an array of complex numbers. Put 0 to the imaginary part
		micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];

		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;
		micBack_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE))
		{
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE))
	{
		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);

		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);

		nb_samples = 0;

		/*if (mode == MANUEL & COLLISION) //DEF COLLISION QQ PART
		{
			sound_manuel_remote(micRight_output);
		}
		else
		{*/
		//sound_remote();
	}
}

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

//D�cide du d�placement selon le micro dominant
void sound_remote(void)
{
	static float mag_max[NB_MIC] = {MIN_VALUE_THRESHOLD};
	static float val_mag_max = 0;

	static uint8_t active_mic_first = NO_SOUND;
	static uint8_t active_mic_second = NO_SOUND;

	mag_max[MIC_RIGHT] = max_magnitude_define(micRight_output);
	mag_max[MIC_LEFT] = max_magnitude_define(micLeft_output);
	mag_max[MIC_BACK] = max_magnitude_define(micBack_output);

	if((mag_max[MIC_RIGHT]<=MIN_VALUE_THRESHOLD) && (mag_max[MIC_LEFT]<=MIN_VALUE_THRESHOLD) && (mag_max[MIC_BACK]<=MIN_VALUE_THRESHOLD))
	{
		state = SOUND_OFF;
	}
	else
	{
		state = SOUND_ON;
	}

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
	//go forward
	if(((active_mic_first == MIC_LEFT) && (active_mic_second == MIC_RIGHT)) || ((active_mic_first == MIC_RIGHT) && (active_mic_second == MIC_LEFT) ))
	{
		left_motor_set_speed(600); //avant � 600
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

uint8_t get_state(void)
{
	return state;
}
