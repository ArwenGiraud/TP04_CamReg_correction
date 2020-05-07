#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>
#include <arm_math.h>

#include <motors.h>
#include <audio/microphone.h>
#include <sensors/VL53L0X/VL53L0X.h>

#include <main.h>
#include <deplacement.h>
#include <audio_processing.h>
#include <process_image.h>
#include <fft.h>
#include <selector.h>

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

#define MIN_VALUE_THRESHOLD	35000 //A AUGMENTER SI BOUGE QUAND IL N'Y A PAS DE FREQUENCE A 250 Hz
#define NB_MIC				3	//on utilise pas celui du bas
#define NO_SOUND			4	//4 car on regarde les micros 0-1-2-3
#define REMOTE_FREQ			26 // 406 Hz

#define REMOTE_FREQ_L		(REMOTE_FREQ-2)
#define REMOTE_FREQ_H		(REMOTE_FREQ+3)

//Defines pour le mode manuel en collision
#define MIN_FREQ			10	//we don't analyze before this index to not use resources for nothing
#define MAX_FREQ			30	//we don't analyze after this index to not use resources for nothing

#define MAN_FREQ_LEFT	16	//200Hz
#define MAN_FREQ_RIGHT	19	//250Hz
#define MAN_FREQ_BACK	22	//310HZ
#define MAN_FREQ_STOP	26	//406Hz

#define MAN_FREQ_LEFT_L		(MAN_FREQ_LEFT-1)
#define MAN_FREQ_LEFT_H		(MAN_FREQ_LEFT+1)
#define MAN_FREQ_RIGHT_L	(MAN_FREQ_RIGHT-1)
#define MAN_FREQ_RIGHT_H	(MAN_FREQ_RIGHT+1)
#define MAN_FREQ_BACK_L		(MAN_FREQ_BACK-1)
#define MAN_FREQ_BACK_H		(MAN_FREQ_BACK+1)
#define MAN_FREQ_STOP_L		(MAN_FREQ_STOP-2)
#define MAN_FREQ_STOP_H		(MAN_FREQ_STOP+3)

#define SOUND_OFF	0
#define SOUND_ON	1

#define VITESSE_NORMALE		600
#define ARRET				0

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
//Détecte la magnitude max pour le signal d'un micro
float max_magnitude_define(float* data, uint8_t freqMin, uint8_t freqMax)
{
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1;

	for(uint16_t i = freqMin ; i <= freqMax ; i++)
	{
		if(data[i] > max_norm)
		{
			max_norm = data[i];
			max_norm_index = i;
		}
	}
	if(freqMin == REMOTE_FREQ_L)//cas du mode normal
	{
		return max_norm;
	}
	else
	{
		return max_norm_index;
	}
}

//Décide du déplacement selon le micro dominant
void sound_remote(void)
{
	static float mag_max[NB_MIC] = {MIN_VALUE_THRESHOLD};
	static float val_mag_max = 0;

	static uint8_t active_mic_first = NO_SOUND;
	static uint8_t active_mic_second = NO_SOUND;

	mag_max[MIC_RIGHT] = max_magnitude_define(micRight_output, REMOTE_FREQ_L, REMOTE_FREQ_H);
	mag_max[MIC_LEFT] = max_magnitude_define(micLeft_output, REMOTE_FREQ_L, REMOTE_FREQ_H);
	mag_max[MIC_BACK] = max_magnitude_define(micBack_output, REMOTE_FREQ_L, REMOTE_FREQ_H);

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
		left_motor_set_speed(VITESSE_NORMALE); //avant à 600
		right_motor_set_speed(VITESSE_NORMALE);
		delay(1000);

	}
	//go backward
	else if((active_mic_first == MIC_BACK))
	{
		left_motor_set_speed(-VITESSE_NORMALE);
		right_motor_set_speed(-VITESSE_NORMALE);
		delay(1000);
	}
	//turn left
	else if((active_mic_first == MIC_LEFT) && (active_mic_second == MIC_BACK))
	{
		left_motor_set_speed(-VITESSE_NORMALE);
		right_motor_set_speed(VITESSE_NORMALE);
		delay(1000);
	}
	//turn right
	else if((active_mic_first == MIC_RIGHT) && (active_mic_second == MIC_BACK))
	{
		left_motor_set_speed(VITESSE_NORMALE);
		right_motor_set_speed(-VITESSE_NORMALE);
		delay(1000);
	}

	else
	{
		left_motor_set_speed(ARRET);
		right_motor_set_speed(ARRET);
		delay(1000);
	}

}

//Possibilité de fusionner les fonctions avec if mode = MANUEL -> return index mais pb
//avec la fonction process audio data  (pas très beau)
bool sound_manuel_remote(void)
{
	int16_t max_norm_index = max_magnitude_define(micRight_output, MIN_FREQ, MAX_FREQ);

	//tourne à gauche à environ 250Hz
	if(max_norm_index >= MAN_FREQ_LEFT_L && max_norm_index <= MAN_FREQ_LEFT_H)
	{
		left_motor_set_speed(-VITESSE_NORMALE);
		right_motor_set_speed(VITESSE_NORMALE);
		return true;
	}
	//tourne à droite à environ 300Hz
	else if(max_norm_index >= MAN_FREQ_RIGHT_L && max_norm_index <= MAN_FREQ_RIGHT_H)
	{
		left_motor_set_speed(VITESSE_NORMALE);
		right_motor_set_speed(-VITESSE_NORMALE);
		return true;
	}
	//recule à environ 350Hz
	else if(max_norm_index >= MAN_FREQ_BACK_L && max_norm_index <= MAN_FREQ_BACK_H)
	{
		left_motor_set_speed(-VITESSE_NORMALE);
		right_motor_set_speed(-VITESSE_NORMALE);
		return true;
	}
	//stop si on est dans la tranche où en mode normal il bouge
	else if(max_norm_index >= MAN_FREQ_STOP_L && max_norm_index <= MAN_FREQ_STOP_H)
	{
		left_motor_set_speed(ARRET);
		right_motor_set_speed(ARRET);
		return true;
	}
	//pas de son => s'arrête mais ne rend pas la main au mode normal
	else if(max_norm_index < MAN_FREQ_LEFT_L)
	{
		left_motor_set_speed(ARRET);
		right_motor_set_speed(ARRET);
		return true;
	}
	else
	{
		left_motor_set_speed(ARRET);
		right_motor_set_speed(ARRET);
		return false;
	}
}

//Collecte les données pour chaque micro, détecte quel micro reçoit un signal maximum
//appelle la fonction de commande des moteurs.
void processAudioData(int16_t *data, uint16_t num_samples)
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
	}
}

uint8_t get_state(void)
{
	return state;
}
