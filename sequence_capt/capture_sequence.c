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
#include <pthread.h>

typedef int DWORD;
typedef pthread_t HANDLE;


char path [128];
char * path_fmt = "%s/image_%04d.pgm";
char path_base [128];
int p[] = {CV_IMWRITE_JPEG_QUALITY, 100, 0};


#define MAX_SEQ_LENGTH 300
#define BUFFER_LENGTH 800
#define WRITE_DELAY_US 50


struct frame_buffer{
	char * buffer ;
	unsigned int write_index ;
	unsigned int read_index ;
	unsigned int nb_frames_availables;
	unsigned int frame_size ;
	unsigned int max_frames ;
};

struct frame_buffer my_frame_buffer ;
int thread_alive = 0 ;
unsigned int nb_frames ;
RaspiCamCvCapture * capture ;

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
    fprintf(pgmFile, "%d %d \n%d \n", img->width, img->height, 255);
    fwrite(img->imageData, 1, img->height*img->width, pgmFile);
    //fflush(pgmFile);
    fclose(pgmFile);
}

void save_thread_func(void * lpParam){
 	IplImage * dummy_image ;
	int i = 0 ;
	dummy_image = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
	printf("Start Save ! \n");
	while(thread_alive || my_frame_buffer.nb_frames_availables > 0){
		if(my_frame_buffer.nb_frames_availables > 0){
			if(pop_frame(dummy_image, &my_frame_buffer) >= 0){
				//printf("One frame \n");
				sprintf(path, path_fmt, path_base, i);
                                writePGM(path, dummy_image, "This is a test !");
                                //printf("Saving %s \n", path);
				//usleep(5000);
                                //cvSaveImage(path, dummy_image, NULL);
                                i ++ ;
        		}
		}
		usleep(WRITE_DELAY_US);
	}
	printf("End Save \n");
}

void acq_thread_func(void * lpParam){
	int i = 0 ;
    	struct timespec tstart={0,0}, tend={0,0};
	double compute_time = 0.0 ;
	printf("Start Capture !\n");
	clock_gettime(CLOCK_MONOTONIC, &tstart);
	while(i < nb_frames){
                int success = 0 ;
                success = raspiCamCvGrab(capture);
                if(success){
                                IplImage* image = raspiCamCvRetrieve(capture);
                                if(push_frame(image, &my_frame_buffer) < 0) printf("lost frame %d ! \n", i);;
                                i ++ ;
				usleep(1000);
                }
        }
        clock_gettime(CLOCK_MONOTONIC, &tend);
        printf("Capture done \n");
        compute_time =  ((double)tend.tv_sec + 1.0e-9*tend.tv_nsec) - 
           ((double)tstart.tv_sec + 1.0e-9*tstart.tv_nsec);
        printf("Capture took %.5f seconds\n", compute_time);
        printf("Actual frame-rate was %f \n", nb_frames/compute_time);
        raspiCamCvReleaseCapture(&capture);
}

int main(int argc, char *argv[]){
	HANDLE acq_thread, save_thread;
	DWORD acq_thread_id, save_thread_id;
	int frame_index = 0 ;
	if(argc > 1) nb_frames = atoi(argv[1]);
	if(argc > 2){
		sprintf(path_base, "%s", argv[2]);
	}else{
       		sprintf(path_base, "./");
	}

	RASPIVID_CONFIG * config = (RASPIVID_CONFIG*)malloc(sizeof(RASPIVID_CONFIG));
	RASPIVID_PROPERTIES * properties = (RASPIVID_PROPERTIES*)malloc(sizeof(RASPIVID_PROPERTIES));
	config->width=640;
	config->height=480;
	config->bitrate=0;	// zero: leave as default
	config->framerate=30;
	config->monochrome=1;

	if(init_frame_buffer(&my_frame_buffer, BUFFER_LENGTH, 640*480) < 0){
		printf("Cannot allocate %d bytes \n", 640*480*nb_frames);
		exit(-1);
	}
	


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
	/*
	Could also use hard coded defaults method: raspiCamCvCreateCameraCapture(0)
	*/
	//RaspiCamCvCapture * capture = (RaspiCamCvCapture *) raspiCamCvCreateCameraCapture2(0, config); 
	
	printf("Init sensor \n");
	capture = (RaspiCamCvCapture *) raspiCamCvCreateCameraCapture3(0, config, properties, 1); 

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
	thread_alive = 1 ;
	acq_thread_id = pthread_create(&acq_thread, NULL, &acq_thread_func, NULL);
	save_thread_id = pthread_create(&save_thread, NULL, &save_thread_func, NULL);
	pthread_join(acq_thread, NULL );
	thread_alive = 0 ;
	pthread_join(save_thread, NULL );
	free_frame_buffer(&my_frame_buffer);
	return 0;
}
