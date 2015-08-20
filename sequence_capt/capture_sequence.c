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
#include <time.h>
#include "RaspiCamCV.h"
#include "Adafruit_Motor_HAT.h"



char path [128];
char * path_fmt = "%s/image_%04d.pgm";
char path_base [128];
int p[] = {CV_IMWRITE_JPEG_QUALITY, 100, 0};


#define MAX_SEQ_LENGTH 300

#define MAX_SPEED 80
#define NB_MOVES 4
unsigned char  robot_moves [][3] ={
{0, 0, 50},
{MAX_SPEED, MAX_SPEED, 50},
{MAX_SPEED, MAX_SPEED/4, 100},
{MAX_SPEED, MAX_SPEED, 200}
};

struct frame_buffer{
	char * buffer ;
	unsigned int write_index ;
	unsigned int read_index ;
	unsigned int nb_frames_availables;
	unsigned int frame_size ;
	unsigned int max_frames ;
};

struct frame_buffer my_frame_buffer ;

int push_frame(IplImage * frame, struct frame_buffer * pBuf){
	if(pBuf->nb_frames_availables >= pBuf->max_frames) return -1 ;
	memcpy(&(pBuf->buffer[pBuf->write_index]), frame->imageData, pBuf->frame_size);
	pBuf->write_index +=  pBuf->frame_size ;
	if(pBuf->write_index >= (pBuf->max_frames * pBuf->frame_size)){
		pBuf->write_index = 0 ;
	}
	pBuf->nb_frames_availables += 1 ;
	return 0 ;
}

int pop_frame(IplImage * frame, struct frame_buffer * pBuf){
	if(pBuf->nb_frames_availables <= 0) return -1 ;
	memcpy(frame->imageData, &(pBuf->buffer[pBuf->read_index]), pBuf->frame_size);
	pBuf->read_index +=  pBuf->frame_size ;
	if(pBuf->read_index >= (pBuf->max_frames * pBuf->frame_size)){
		pBuf->read_index = 0 ;
	}
	pBuf->nb_frames_availables -= 1 ;
	return 0 ;

}

int init_frame_buffer(struct frame_buffer * pBuf, unsigned int nb_frames, unsigned int frame_size){
	pBuf -> max_frames = nb_frames ;
	pBuf -> frame_size = frame_size;
	pBuf -> buffer = malloc(nb_frames*(pBuf -> frame_size));
	if(pBuf -> buffer == NULL) return -1 ;
	pBuf -> write_index = 0 ;
	pBuf -> read_index = 0 ;
	pBuf -> nb_frames_availables = 0 ;
	return 0 ;
}

void free_frame_buffer(struct frame_buffer * pBuf){
	free(pBuf -> buffer);
}

void writePGM(const char *filename, IplImage * img, char *  comment)
{
    FILE *pgmFile;
    int i, j;
    int hi, lo;
    pgmFile = fopen(filename, "wb");
    if (pgmFile == NULL) {
        perror("cannot open file to write");
        exit(EXIT_FAILURE);
    }
    fprintf(pgmFile, "P5 \n");
    //TODO : insert data as comments ... Maybe use Json file format
    if(comment != NULL){
    	fprintf(pgmFile, "#%s \n", comment);
    }
    fprintf(pgmFile, "%d %d \n", img->width, img->height);
    fprintf(pgmFile, "%d \n", 255);
    fwrite(img->imageData, 1, img->height*img->width, pgmFile);
    fclose(pgmFile);
}

int main(int argc, char *argv[]){
	//TODO : multi_thread capture and save
	/*HANDLE acq_thread, save_thread;
	DWORD acq_thread_id, save_thread_id;*/
	int nb_frames = 300 ;
	int frame_index = 0, move_index = 0;
	IplImage * dummy_image ;
	struct timespec tstart={0,0}, tend={0,0};
	double compute_time = 0.0 ;
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

	if(init_frame_buffer(&my_frame_buffer,nb_frames, 640*480) < 0){
		printf("Cannot allocate %d bytes \n", 640*480*nb_frames);
		exit(-1);
	}
	

	dummy_image = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);

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
	for(i = 0 ; i < 10 ; ){
		int success = 0 ;
                success = raspiCamCvGrab(capture);
                if(success){
                                IplImage* image = raspiCamCvRetrieve(capture);
                		i ++ ;
		}
	}
	printf("Start move \n");
	i = 0 ;
	printf("Start capture \n");
	motor_run(FORWARD, &mot4);
        motor_run(FORWARD, &mot3);
	clock_gettime(CLOCK_MONOTONIC, &tstart);
	while(i < nb_frames){
		int success = 0 ;
		success = raspiCamCvGrab(capture);
		if(success){
				IplImage* image = raspiCamCvRetrieve(capture);
				if(push_frame(image, &my_frame_buffer) < 0) printf("lost frame %d ! \n", i);;
				i ++ ;
		}
	}
	clock_gettime(CLOCK_MONOTONIC, &tend);
	printf("Capture done \n");
	compute_time =  ((double)tend.tv_sec + 1.0e-9*tend.tv_nsec) - 
           ((double)tstart.tv_sec + 1.0e-9*tstart.tv_nsec);
	printf("Capture took %.5f seconds\n", compute_time);
	printf("Actual frame-rate was %f \n", nb_frames/compute_time);
	i = 0 ;
	clock_gettime(CLOCK_MONOTONIC, &tstart);
	while(pop_frame(dummy_image, &my_frame_buffer) >= 0){
                                sprintf(path, path_fmt, path_base, i);
                                writePGM(path, dummy_image, "This is a test !");
                                //cvSaveImage(path, image, NULL);
                                i ++ ;
        }
	clock_gettime(CLOCK_MONOTONIC, &tend);
        printf("Saving done \n");
        compute_time =  ((double)tend.tv_sec + 1.0e-9*tend.tv_nsec) - 
           ((double)tstart.tv_sec + 1.0e-9*tstart.tv_nsec);
        printf("Saving took %.5f seconds\n", compute_time);
	motor_close(&mot3);
        motor_close(&mot4);
	free_frame_buffer(&my_frame_buffer);
	raspiCamCvReleaseCapture(&capture);
	return 0;
}
