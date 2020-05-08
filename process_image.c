#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <camera/po8030.h>
#include <sensors/VL53L0X/VL53L0X.h>

#include <main.h>
#include <process_image.h>
#include <audio_processing.h>

#define LIMITE 20

#define MAX_WIDHT			500 //camera fait 640 pixels
#define DIST_CAM			774 //772.55 mais on arrondit, pas besoin de cette précision
#define ESPACE				5 //cm
#define RAPPORT1CM			(DIST_CAM/(ESPACE+1)) //129pxl
#define RAPPORT3CM			((DIST_CAM/(ESPACE+1))*3) //387pxl
#define MOYENNE				((RAPPORT1CM + RAPPORT3CM)/2)	//258pxl

#define ERREUR 			0
#define PETIT_OBSTACLE	1	//diamètre 2cm
#define MOYEN_OBSTACLE	2	//diamètre 4cm
#define GRAND_OBSTACLE	3	//mur

static uint8_t taille_obstacle = ERREUR;

static BSEMAPHORE_DECL(image_ready_sem, TRUE);
//static BSEMAPHORE_DECL(taille_definie_sem, TRUE);

//Returns the line's width extracted from the image buffer given
//Returns 0 if line not found
uint16_t extract_line_width(uint8_t *buffer){

	uint16_t i = 0, begin = 0, end = 0, width = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++)
	{
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE; //moyenne

	//si la moyenne qu'on trouve est très petite, tout est noir => on a un mur
	do
	{
		wrong_line = 0;
		//search for a begin
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{
			//En fait, si on trouve une valeur en dessous de la moyenne et qu'à une dist WIDTH_SLOPE on
			//en trouve une plus grande, on est en train de repéré notre obstacle
			if(buffer[i] < mean && buffer[i+WIDTH_SLOPE] > mean)
			{
				begin = i;
				stop = 1; //du moment où cette condition est vrai, on arrête de chercher le début => stop=1
			}
			i++;
		}
		//if a begin was found, search for an end
		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
		{
			stop = 0;

			while(stop == 0 && i < IMAGE_BUFFER_SIZE)
			{
				//si en i on est plus petit que la moyenne et en i-WIDTH_SLOPE on est plus grand alors on a
				//trouvé la fin => pour une ligne blanche
				if(buffer[i] < mean && buffer[i-WIDTH_SLOPE] > mean)
				{
					end = i;
					stop = 1;
				}
				i++;
			}
			//if an end was not found
			if (i > IMAGE_BUFFER_SIZE || !end)
			{
				line_not_found = 1;
			}
		}
		else//if no begin was found
		{
			line_not_found = 1;
		}

		//if a line too small has been detected, continues the search
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			wrong_line = 1;
		}
	}
	while(wrong_line);

	if(line_not_found)
	{
		begin = 0;
		end = 0;
		width = GRAND_OBSTACLE;
	}
	else
	{
		width = (end - begin);//la différence des i trouvés pour le début et la fin
	}

	//sets a maximum width or returns the measured width
	if(width > MAX_WIDHT){
		return ERREUR;
	}else{
		return width;
	}
}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1)
    {
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}

static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};
	uint16_t lineWidth = 0;

    while(1)
    {
    	//waits until an image has been captured
    	chBSemWait(&image_ready_sem);

		//gets the pointer to the array filled with the last image in RGB565
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts only the red pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2)
		{
			//extracts first 5bits of the first byte
			//takes nothing from the second byte => car il code pour les autres couleur #on ne garde que rouge
			image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
		}

		//search for a line in the image and gets its width in pixels
		lineWidth = extract_line_width(image);

		//converts the width into a distance between the robot and the camera
		if(!lineWidth)//ERROR
		{
			taille_obstacle = ERREUR;
		}
		else if(lineWidth == GRAND_OBSTACLE)
		{
			taille_obstacle = GRAND_OBSTACLE;
		}
		else if(lineWidth < MOYENNE)
		{
			taille_obstacle = PETIT_OBSTACLE;
		}
		else // if (largeur > MOYENNE)
		{
			taille_obstacle = MOYEN_OBSTACLE;
		}
    }
}

uint8_t get_taille_obstacle(void)
{
	return taille_obstacle;
}

void process_image_start(void){
    chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
    chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}


