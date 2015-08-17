/*

 Copyright (c) by Emil Valkov,
 All rights reserved.

 License: http://www.opensource.org/licenses/bsd-license.php

*/

#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include "RaspiCamCV.h"
#include "Adafruit_Motor_HAT.h"



char path [128];
char * path_fmt = "%s/image_%04d.pgm";
char * path_base [128];
int p[] = {CV_IMWRITE_JPEG_QUALITY, 100, 0};


#define MAX_SPEED 80


#define NB_MOVES 4
unsigned char  robot_moves [][3] ={
{0, 0, 50},
{MAX_SPEED, MAX_SPEED, 50},
{MAX_SPEED, MAX_SPEED/4, 100},
{MAX_SPEED, MAX_SPEED, 200}
};

int main(int argc, char *argv[]){

	int nb_frames = 300 ;
	int frame_index = 0, move_index = 0;
	if(argc > 1) nb_frames = atoi(argv[1]);
	if(argc > 2){
		sprintf(path_base, "%s", argv[2]);
	}else{
       		sprintf(path_base, "./");
	}
	struct motor mot3 ;
        struct motor mot4 ;
        motor_init(&mot3, 3);
        motor_init(&mot4, 4);

        motor_run(RELEASE, &mot3);
        motor_run(RELEASE, &mot4);
	motor_set_speed(0, &mot3);
	motor_set_speed(0, &mot4);

	RASPIVID_CONFIG * config = (RASPIVID_CONFIG*)malloc(sizeof(RASPIVID_CONFIG));
	RASPIVID_PROPERTIES * properties = (RASPIVID_PROPERTIES*)malloc(sizeof(RASPIVID_PROPERTIES));
	config->width=640;
	config->height=480;
	config->bitrate=0;	// zero: leave as default
	config->framerate=30;
	config->monochrome=1;
	
	
	properties->hflip = 1 ;
	properties->vflip = 1 ;
	properties -> sharpness = 0 ;
	properties -> contrast = 0 ;
	properties -> brightness = 45 ;
	properties -> saturation = 0 ;
	properties -> exposure = SPORTS;
	properties -> shutter_speed = 0 ; // 0 is autoo
	int opt;
	int i = 0, j = 0 ;
	int init = 0 ;

	int motor_speed = 0 ;
	/*
	Could also use hard coded defaults method: raspiCamCvCreateCameraCapture(0)
	*/
	//RaspiCamCvCapture * capture = (RaspiCamCvCapture *) raspiCamCvCreateCameraCapture2(0, config); 
	
	printf("Init sensor \n");
	RaspiCamCvCapture * capture = (RaspiCamCvCapture *) raspiCamCvCreateCameraCapture3(0, config, properties, 1); 

	free(config);

	printf("Wait stable sensor \n");
	for(i = 0 ; i < 50 ; ){
		int success = 0 ;
                success = raspiCamCvGrab(capture);
                if(success){
                                IplImage* image = raspiCamCvRetrieve(capture);
                		i ++ ;
		}
	}
	printf("Start move \n");
	i = 0 ;
	while(i < nb_frames){
		int success = 0 ;
		success = raspiCamCvGrab(capture);
		motor_run(FORWARD, &mot4);
		motor_run(FORWARD, &mot3);
		if(success){
				sprintf(path, path_fmt, path_base, i);
				IplImage* image = raspiCamCvRetrieve(capture);
				cvSaveImage(path, image, NULL);
				i ++ ;
				if(i > robot_moves[move_index][2] && move_index < NB_MOVES){
					move_index ++ ;
					printf("mode index %d \n", move_index);
					motor_set_speed(robot_moves[move_index][0], &mot4);
                                	motor_set_speed(robot_moves[move_index][1], &mot3);
				}
		}
	}
	motor_close(&mot3);
        motor_close(&mot4);
	raspiCamCvReleaseCapture(&capture);
	return 0;
}
