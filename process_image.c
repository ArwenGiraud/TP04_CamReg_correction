#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <sensors/proximity.h>

#include <process_image.h>


#define ERROR 0
#define LIMITE 20

#define DIST_CAM			772.55f
#define ESPACE				5 //cm
#define RAPPORT2CM			((DIST_CAM/(ESPACE+1))*2) //257.52pxl
#define RAPPORT3CM			((DIST_CAM/(ESPACE+1))*3) //386.28pxl
#define MOYENNE				((RAPPORT2CM + RAPPORT3CM)/2)	//321.9pxl

#define PAS_D_OBSTACLE 	0
#define PETIT_OBSTACLE	1	//diamètre 2cm
#define MOYEN_OBSTACLE	2	//diamètre 4cm
#define GRAND_OBSTACLE	3	//mur

//Returns the line's width extracted from the image buffer given
//Returns 0 if line not found
uint16_t extract_line_width(uint8_t *buffer){

	chprintf((BaseSequentialStream *)&SD3, "extract_line_width");
	uint16_t i = 0, begin = 0, end = 0, width = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;

	static uint16_t last_width = PXTOCM/GOAL_DISTANCE; //ancienne distance est celle qu'on souhaite atteindre

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++)
	{
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE; //moyenne

	//si la moyenne qu'on trouve est très petite, tout est noir => on a un mur
	if(mean < LIMITE)
	{
		return ERROR;
	}
	else
	{
		do
		{
			wrong_line = 0;
			//search for a begin
			while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
			{
				//the slope must at least be WIDTH_SLOPE wide and is compared
				//to the mean of the image
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
			width = last_width;
		}else{
			last_width = width = (end - begin);//la différence des i trouvés pour le début et la fin
		}

		//sets a maximum width or returns the measured width
		if((PXTOCM/width) > MAX_DISTANCE){
			return PXTOCM/MAX_DISTANCE;
		}else{
			return width;
		}
	}
}

void capture_image(void)
{

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    //while(1)//!ready)
    //{
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
    //}
}

uint8_t process_image(void)
{
	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};
	uint16_t lineWidth = 0;

    //while(1)//!imready)
    //{
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
		if(!lineWidth)
		{
			return GRAND_OBSTACLE;
		}
		else
		{
			if(lineWidth < MOYENNE)
			{
				return PETIT_OBSTACLE;
			}
			else // if (largeur > MOYENNE)
			{
				return MOYEN_OBSTACLE;
			}
		}
    //}
}


