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
//#include "L3GD20.h"
//#include "LSM303_U.h"
#include "MPU9250.h"
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

int push_frame(IplImage * frame, float timestamp, struct frame_buffer * pBuf){
	if(pBuf->nb_frames_availables >= pBuf->max_frames) return -1 ;
	memcpy(&(pBuf->buffer[pBuf->write_index]), &timestamp, sizeof(float)); //pushing timestamp
	memcpy(&(pBuf->buffer[pBuf->write_index+sizeof(float)]), frame->imageData, (pBuf->frame_size - sizeof(float)));//pushing image
	pBuf->write_index +=  pBuf->frame_size ;
	if(pBuf->write_index >= (pBuf->max_frames * pBuf->frame_size)){
		pBuf->write_index = 0 ;
	}
	pBuf->nb_frames_availables += 1 ;
	return 0 ;
}

int pop_frame(IplImage * frame, float * timestamp, struct frame_buffer * pBuf){
	if(pBuf->nb_frames_availables <= 0) return -1 ;
	memcpy(timestamp, &(pBuf->buffer[pBuf->read_index]), sizeof(float));
	memcpy(frame->imageData, &(pBuf->buffer[pBuf->read_index+sizeof(float)]), (pBuf->frame_size - sizeof(float)));
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
	char line_buffer[128];
 	IplImage * dummy_image ;
	FILE * tsFile  ;
	int i = 0 ;
	float timestamp ;
	sprintf(line_buffer, "%s/images.time", path_base);
	tsFile = fopen(line_buffer, "w");
	if (tsFile == NULL) {
		perror("cannot open file to write");
		return ;
	}
	dummy_image = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
	printf("Start Save ! \n");
	while(thread_alive || my_frame_buffer.nb_frames_availables > 0){
		if(my_frame_buffer.nb_frames_availables > 0){
			if(pop_frame(dummy_image, &timestamp, &my_frame_buffer) >= 0){
				//printf("One frame \n");
				int string_size = sprintf(line_buffer, "%f\n", timestamp);//printing timestamp to file
				fwrite(line_buffer, 1, string_size, tsFile);
				sprintf(path, path_fmt, path_base, i);
                                writePGM(path, dummy_image, "");
                                //printf("Saving %s \n", path);
				//usleep(5000);
                                //cvSaveImage(path, dummy_image, NULL);
                                i ++ ;
        		}
		}
		usleep(WRITE_DELAY_US);
	}
	fclose(tsFile);
	printf("End Save \n");
}

void acq_image_thread_func(void * lpParam){
	int i = 0 ;
    	struct timespec tstart={0,0}, tend={0,0}, tcurr;
	double compute_time = 0.0 ;
	float compute_time_f ;
	printf("Wait stable sensor \n");
        for(i = 0 ; i < 30 ; ){
                int success = 0 ;
                success = raspiCamCvGrab(capture);
                if(success){
                                IplImage* image = raspiCamCvRetrieve(capture);
                                i ++ ;
                }
        }
	i = 0 ;
	printf("Start Capture !\n");
	clock_gettime(CLOCK_MONOTONIC, &tstart);
	while(i < nb_frames){
                int success = 0 ;
                success = raspiCamCvGrab(capture);
                if(success){
                                IplImage* image = raspiCamCvRetrieve(capture);
				clock_gettime(CLOCK_MONOTONIC, &tcurr);
				compute_time =  ((double)tcurr.tv_sec + 1.0e-9*tcurr.tv_nsec);
				compute_time_f = (float) compute_time ;
                                if(push_frame(image,compute_time_f, &my_frame_buffer) < 0) printf("lost frame %d ! \n", i);;
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


void acq_imu_thread_func(void * lpParam){
	char line_buffer[128] ;
	FILE *imuFile;
	int i = 0 ;
	int i2c_fd ;
	int gyro_read_status = 0 , acc_read_status = 0;
	float time_acc_gyro [7] ;
	struct timespec tcur={0,0};
	double compute_time = 0.0 ;
	float compute_time_f ;
	i2c_fd = open("/dev/i2c-1", O_RDWR);
	sprintf(line_buffer, "%s/IMU.log", path_base);
	imuFile = fopen(line_buffer, "w");
	if (imuFile == NULL) {
		perror("cannot open file to write");
		return ;
	}/*
	if(L3GD20_begin(i2c_fd, 0, L3GD20_ADDRESS)== 0){
		printf("cannot detect gyro at 0x%x\n", L3GD20_ADDRESS);
		return ;
	}
	if(LSM303_Acc_begin(i2c_fd, LSM303_ADDRESS_ACCEL) == 0){
		printf("Cannot detect acc at 0x%x\n", LSM303_ADDRESS_ACCEL);
		return;
	}*/
	if(MPU9250_begin(i2c_fd, MPU9250_ADDREss) == 0){
                printf("Cannot detect IMU at 0x%x\n", MPU9250_ADDRESS);
                return;
        }
	printf("Start Capture IMU !\n");
	while(thread_alive){
		gyro_read_status = 0 ;
		acc_read_status = 0 ;
		/*gyro_read_status = L3GD20_read(&(time_acc_gyro[4]));
		acc_read_status = LSM303_Acc_read(&(time_acc_gyro[1]));
		*/
		gyro_read_status = MPU9250_read(&(time_acc_gyro[1]));
		acc_read_status = gyro_read_status;
		if(acc_read_status || gyro_read_status){
			int string_size ;
			clock_gettime(CLOCK_MONOTONIC, &tcur);
			compute_time =  ((double)tcur.tv_sec + 1.0e-9*tcur.tv_nsec);
			time_acc_gyro[0] = (float) compute_time;
			string_size = sprintf(line_buffer, "[7]\n(%.9g,", time_acc_gyro[0]);
			fwrite(line_buffer, 1, string_size, imuFile); //array size and timestamp
			for(i = 1 ; i < 4 ; i ++ ){
				if(acc_read_status){
					string_size = sprintf(line_buffer, "%.9g,", time_acc_gyro[i]);
				}else{
                                        string_size = sprintf(line_buffer, "x,");
                                }
				fwrite(line_buffer, 1, string_size, imuFile);
			}
			for(i = 4 ; i < 7 ; i ++ ){
				int last_comma = (i == 6)?1:0;//not writing last comma
                                if(gyro_read_status){
                                        string_size = sprintf(line_buffer, "%.9g,", time_acc_gyro[i]);
                                }else{
					string_size = sprintf(line_buffer, "x,");
				}   
                                fwrite(line_buffer, 1, string_size - last_comma, imuFile);
                        }
			 fwrite(")\n", 1, 2, imuFile);
			//add save to file
		}
        }
        fclose(imuFile);
}

int main(int argc, char *argv[]){
	HANDLE acq_image_thread, save_thread, acq_imu_thread;
	DWORD acq_image_thread_id, save_thread_id, acq_imu_thread_id;
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
	if(init_frame_buffer(&my_frame_buffer,nb_frames, (640*480)+sizeof(float)) < 0){
		printf("Cannot allocate %d bytes \n", 640*480*nb_frames);
		exit(-1);
	}

	//dummy_image = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);

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
	thread_alive = 1 ;
	if(nb_frames > 0){
		acq_image_thread_id = pthread_create(&acq_image_thread, NULL, &acq_image_thread_func, NULL);
		save_thread_id = pthread_create(&save_thread, NULL, &save_thread_func, NULL);
	}
	acq_imu_thread_id = pthread_create(&acq_imu_thread, NULL, &acq_imu_thread_func, NULL);
	if(nb_frames > 0){
		pthread_join(acq_image_thread, NULL );
		thread_alive = 0 ;
		pthread_join(save_thread, NULL );
	}
	pthread_join(acq_imu_thread, NULL );
	free_frame_buffer(&my_frame_buffer);
	return 0;
}
