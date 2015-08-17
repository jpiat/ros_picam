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


char path [128];
char * path_fmt = "./image_%04d.png";

int p[] = {CV_IMWRITE_JPEG_QUALITY, 100, 0};


void writePGM(const char *filename, IplImage * img)
{
    FILE *pgmFile;
    int i, j;
    int hi, lo;
    pgmFile = fopen(filename, "wb");
    if (pgmFile == NULL) {
        perror("cannot open file to write");
        exit(EXIT_FAILURE);
    }
    fprintf(pgmFile, "P5 ");
    //TODO : insert data as comments ... Maybe use Json file format
    fprintf(pgmFile, "%d %d ", img->width, img->height);
    fprintf(pgmFile, "%d ", 255);
    for (i = 0; i < img->height; ++i){
            for (j = 0; j < img->width; ++j) {
                lo = img->imageData[(i*img->width)+j];
                fputc(lo, pgmFile);
            }
    }
    fclose(pgmFile);
}


int main(int argc, char *argv[]){

	int nb_frames = 300 ;
	
	if(argc > 0) nb_frames = atoi(argv[1]);

	RASPIVID_CONFIG * config = (RASPIVID_CONFIG*)malloc(sizeof(RASPIVID_CONFIG));
	RASPIVID_PROPERTIES * properties = (RASPIVID_PROPERTIES*)malloc(sizeof(RASPIVID_PROPERTIES));
	config->width=640;
	config->height=480;
	config->bitrate=0;	// zero: leave as default
	config->framerate=30;
	config->monochrome=1;
	
	
	properties->hflip = 0 ;
	properties->vflip = 1 ;
	properties -> sharpness = 0 ;
	properties -> contrast = 0 ;
	properties -> brightness = 45 ;
	properties -> saturation = 0 ;
	properties -> exposure = SPORTS;
	int opt;
	int i = 0, j = 0 ;
	int init = 0 ;

	/*
	Could also use hard coded defaults method: raspiCamCvCreateCameraCapture(0)
	*/
	//RaspiCamCvCapture * capture = (RaspiCamCvCapture *) raspiCamCvCreateCameraCapture2(0, config); 
	RaspiCamCvCapture * capture = (RaspiCamCvCapture *) raspiCamCvCreateCameraCapture3(0, config, properties, 1); 

	free(config);
	while(i < nb_frames){
		int success = 0 ;
		success = raspiCamCvGrab(capture);
		if(success){
			sprintf(path, path_fmt, i);
			IplImage* image = raspiCamCvRetrieve(capture);
			cvSaveImage(path, image, NULL);
			i ++ ;
		}/*else{
			printf("no frame\n");
		}*/
	}
	raspiCamCvReleaseCapture(&capture);
	return 0;
}
