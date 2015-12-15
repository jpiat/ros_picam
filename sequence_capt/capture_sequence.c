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
#include <signal.h>
#include <sys/resource.h>
//#include "L3GD20.h"
//#include "LSM303_U.h"
#include "MPU9250.h"
typedef int DWORD;
typedef pthread_t HANDLE;


//#define SEPARATE_TIMESTAMP

char path [128];
char * path_fmt = "%s/image_%04d.pgm";
char * path_fmt_long = "%s/%010lu.pgm";
char path_base [128];
int p[] = {CV_IMWRITE_JPEG_QUALITY, 100, 0};

struct timeval tstart ; // timestamp of time origin for the programm

#define MAX_SEQ_LENGTH 300
#define BUFFER_LENGTH 1600
#define WRITE_DELAY_US 1


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

unsigned int max_imu_data ;
unsigned int nb_imu_data ;
char * imu_buffer ;
#define IMU_LINE_BYTES (sizeof(short)*6 + sizeof(unsigned long))

int timespec_subtract (struct timespec  * result, struct timespec *x, struct timespec *y)
{
  /* Perform the carry for the later subtraction by updating y. */
  if (x->tv_nsec < y->tv_nsec) {
    long nsec = (y->tv_nsec - x->tv_nsec) / 1000000000L + 1;
    y->tv_nsec -= nsec * 1000000000L;
    y->tv_sec += nsec;
  }
  if (x->tv_nsec - y->tv_nsec > 1000000000L) {
    long nsec = (x->tv_nsec - y->tv_nsec) / 1000000000L;
    y->tv_nsec += 1000000000L * nsec;
    y->tv_sec -= nsec;
  }

  /* Compute the time remaining to wait.
     tv_usec is certainly positive. */
  result->tv_sec = x->tv_sec - y->tv_sec;
  result->tv_nsec = x->tv_nsec - y->tv_nsec;

  /* Return 1 if result is negative. */
  return x->tv_sec < y->tv_sec;
}

int timeval_subtract (struct timeval  * result, struct timeval *x, struct timeval *y)
{
  /* Perform the carry for the later subtraction by updating y. */
  if (x->tv_usec < y->tv_usec) {
    long nsec = ((y->tv_usec - x->tv_usec)/1000000L)  + 1;
    y->tv_usec -= (nsec * 1000000L);
    y->tv_sec += nsec;
  }
  if (x->tv_usec - y->tv_usec > 1000000L) {
    long nsec = (x->tv_usec - y->tv_usec) / 1000000L;
    y->tv_usec += (1000000L * nsec);
    y->tv_sec -= nsec;
  }

  /* Compute the time remaining to wait.
     tv_usec is certainly positive. */
  result->tv_sec = x->tv_sec - y->tv_sec;
  result->tv_usec = x->tv_usec - y->tv_usec;

  /* Return 1 if result is negative. */
  return x->tv_sec < y->tv_sec;
}


void init_time(){
	//clock_gettime(CLOCK_MONOTONIC, &tstart);
	gettimeofday(&tstart, NULL);
}

unsigned long get_long_time(){
	struct timeval tcurr, telapsed, start_copy;
	unsigned long diff_time;
	start_copy = tstart ;
	gettimeofday(&tcurr, NULL);
	if(timeval_subtract(&telapsed, &tcurr, &start_copy) == 1){
                telapsed.tv_sec = -telapsed.tv_sec;
                telapsed.tv_usec = -telapsed.tv_usec;
                //printf("negative value \n");
        }
	diff_time = telapsed.tv_sec * 1000000L + telapsed.tv_usec ;
	//printf("time %lu \n", diff_time);
	return diff_time ;
}


int push_frame(IplImage * frame, unsigned long timestamp, struct frame_buffer * pBuf){
	if(pBuf->nb_frames_availables >= pBuf->max_frames) return -1 ;
	memcpy(&(pBuf->buffer[pBuf->write_index]), &timestamp, sizeof(unsigned long)); //pushing timestamp
	memcpy(&(pBuf->buffer[pBuf->write_index+sizeof(unsigned long)]), frame->imageData, (pBuf->frame_size - sizeof(unsigned long)));//pushing image
	pBuf->write_index +=  pBuf->frame_size ;
	if(pBuf->write_index >= (pBuf->max_frames * pBuf->frame_size)){
		pBuf->write_index = 0 ;
	}
	pBuf->nb_frames_availables += 1 ;
	return 0 ;
}

int pop_frame(IplImage * frame, unsigned long * timestamp, struct frame_buffer * pBuf){
	if(pBuf->nb_frames_availables <= 0) return -1 ;
	memcpy(timestamp, &(pBuf->buffer[pBuf->read_index]), sizeof(unsigned long));
	memcpy(frame->imageData, &(pBuf->buffer[pBuf->read_index+sizeof(unsigned long)]), (pBuf->frame_size - sizeof(unsigned long)));
	pBuf->read_index +=  pBuf->frame_size ;
	if(pBuf->read_index >= (pBuf->max_frames * pBuf->frame_size)){
		pBuf->read_index = 0 ;
	}
	pBuf->nb_frames_availables -= 1 ;
	return 0 ;

}

int init_frame_buffer(struct frame_buffer * pBuf, unsigned int nb_frames, unsigned int frame_size){
	if(nb_frames > BUFFER_LENGTH) nb_frames = BUFFER_LENGTH ;
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
   /* if(comment != NULL){
    	fprintf(pgmFile, "#%s \n", comment);
    }*/
    fprintf(pgmFile, "%d %d \n%d \n", img->width, img->height, 255);
    fwrite(img->imageData, 1, img->height*img->width, pgmFile);
    //fflush(pgmFile);
    fclose(pgmFile);
}

void save_thread_func(void * lpParam){
	char line_buffer[128];
 	IplImage * dummy_image ;
	int i = 0 ;
	unsigned long timestamp ;
	#ifdef SEPARATE_TIMESTAMP
	FILE * tsFile  ;
	sprintf(line_buffer, "%s/images.time", path_base);
	tsFile = fopen(line_buffer, "w");
	if (tsFile == NULL) {
		perror("cannot open file to write");
		return ;
	}
	#endif
	dummy_image = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
	printf("Start Save ! \n");
	while(thread_alive && my_frame_buffer.nb_frames_availables > 0){
		if(my_frame_buffer.nb_frames_availables > 0){
			if(pop_frame(dummy_image, &timestamp, &my_frame_buffer) >= 0){
				//printf("One frame \n");
				#ifdef SEPARATE_TIMESTAMP
				int string_size = sprintf(line_buffer, "%ld\n", timestamp);//printing timestamp to file
				fwrite(line_buffer, 1, string_size, tsFile);
				sprintf(path, path_fmt, path_base, i);
				#else
				sprintf(path, path_fmt_long, path_base, timestamp);
				#endif
                                writePGM(path, dummy_image, "");
                                //printf("Saving %s \n", path);
				//usleep(5000);
                                //cvSaveImage(path, dummy_image, NULL);
                                i ++ ;
        		}
		}
		usleep(WRITE_DELAY_US);
	}
	#ifdef SEPARATE_TIMESTAMP
	fclose(tsFile);
	#endif
	printf("End Save \n");
}

void acq_image_thread_func(void * lpParam){
	int i = 0 ;
    	unsigned long t_start, t_stop ;
	unsigned long t_diff ;

	RASPIVID_CONFIG * config = (RASPIVID_CONFIG*)malloc(sizeof(RASPIVID_CONFIG));
        RASPIVID_PROPERTIES * properties = (RASPIVID_PROPERTIES*)malloc(sizeof(RASPIVID_PROPERTIES));
        config->width=640;
        config->height=480;
        config->bitrate=0;      // zero: leave as default
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
	printf("Init sensor \n");
        capture = (RaspiCamCvCapture *) raspiCamCvCreateCameraCapture3(0, config, properties, 1);
	free(config);
	printf("Wait stable sensor \n");
        for(i = 0 ; (i < 30 && thread_alive) ; ){
                int success = 0 ;
                success = raspiCamCvGrab(capture);
                if(success){
                                IplImage* image = raspiCamCvRetrieve(capture);
                                i ++ ;
                }
        }
	i = 0 ;
	printf("Start Capture !\n");
	t_start = get_long_time();
	while(i < nb_frames && thread_alive){
                int success = 0 ;
                success = raspiCamCvGrab(capture);
                if(success){
                                IplImage* image = raspiCamCvRetrieve(capture);
				t_diff = get_long_time();
                                if(push_frame(image,t_diff, &my_frame_buffer) < 0) printf("lost frame %d ! \n", i);;
                                i ++ ;
				usleep(1000);
                }
        }
        t_stop = get_long_time();
        printf("Capture done \n");
        t_diff =  t_stop - t_start ;;
        printf("Capture took %lu ms\n", t_diff/1000L);
        //printf("Actual frame-rate was %f \n", nb_frames/t_diff);
        raspiCamCvReleaseCapture(&capture);
}


void export_imu_raw(FILE * fd, short * data, unsigned long time){
	char buffer [128];
	int size ;
	int i ;
	size = sprintf(buffer, "%lu ", time);
        if(fwrite(buffer, 1, size, fd) < size) printf("write failed ... \n");; //array size and timestamp
        for(i = 0 ; i < 6 ; i ++ ){
               size = sprintf(buffer, "%hd ", data[i]);
               if(fwrite(buffer, 1, size, fd) < size) printf("write failed ... \n");
         }
         fwrite("\n", 1, 1, fd);
}


void export_imu_calib(FILE * fd, float acc_range, float gyro_range){
        char buffer [128];
        int size ;
        size = sprintf(buffer, "%% Accelerometer Range = +/- %f \n", acc_range );
        fwrite(buffer, 1, size, fd);
        size = sprintf(buffer, "%% Gyroscope Range = +/- %f \n", gyro_range );
        fwrite(buffer, 1, size, fd);
	size = sprintf(buffer, "%% All IMU samples are 16-bit signed values \n", gyro_range );
        fwrite(buffer, 1, size, fd);
        size = sprintf(buffer, "%% TimeStamp AccX AccY AccZ GyroX GyroY GyroZ\n");
        fwrite(buffer, 1, size, fd);
}

void acq_imu_thread_func(void * lpParam){
	/*char line_buffer[128] ;
	FILE *imuFile;*/
	int i = 0 ;
	int i2c_fd ;
	int imu_read_status = 0 ;
	short raw_imu [6];
	struct timespec tcur={0,0};
	double compute_time = 0.0 ;
	float compute_time_f ;
	i2c_fd = open("/dev/i2c-1", O_RDWR);
	/*sprintf(line_buffer, "%s/IMU.log", path_base);
	imuFile = fopen(line_buffer, "w");
	if (imuFile == NULL) {
		perror("cannot open file to write");
		return ;
	}*/

	//export_imu_calib(imuFile, 4.0, 500.0); // AG and 500dps
	if(MPU9250_begin(i2c_fd, MPU9250_ADDRESS) == 0){
                printf("Cannot detect IMU at 0x%x\n", MPU9250_ADDRESS);
                return;
        }
	printf("Start Capture IMU !\n");
	while(thread_alive && nb_imu_data < max_imu_data){
		imu_read_status = MPU9250_read_raw(raw_imu);
		if(imu_read_status){
			int string_size ;
			unsigned long timestamp ;
			timestamp = get_long_time();
			memcpy(&(imu_buffer[nb_imu_data*IMU_LINE_BYTES]), &timestamp, sizeof(unsigned long));
			memcpy(&(imu_buffer[(nb_imu_data*IMU_LINE_BYTES)+sizeof(unsigned long)]), raw_imu, 6*sizeof(short));
			nb_imu_data ++ ;
			//export_imu_raw(imuFile, raw_imu, timestamp);
		}
        }
        //fclose(imuFile);
}

void killHandler(int sig) {
	printf("Ending Capture\n");
	thread_alive = 0;
}

int main(int argc, char *argv[]){
	HANDLE acq_image_thread, save_thread, acq_imu_thread;
	DWORD acq_image_thread_id, save_thread_id, acq_imu_thread_id;
	char line_buffer[128] ;
        FILE *imuFile;
	int frame_index = 0 ;
	if(argc == 1){
		printf("Usage : capture_sequence <nb_frames> <capture folder> \n");
		exit(0);
	}
	if(argc > 1) nb_frames = atoi(argv[1]);
	if(argc > 2){
		sprintf(path_base, "%s", argv[2]);
	}else{
       		sprintf(path_base, "./");
	}

	if(init_frame_buffer(&my_frame_buffer,nb_frames, (640*480)+sizeof(float)) < 0){
		printf("Cannot allocate %d bytes \n", 640*480*nb_frames);
		exit(-1);
	}
	max_imu_data = (nb_frames > 0)?(((nb_frames/30)+1024)*250):((3600)*250);
	nb_imu_data = 0 ;
	//buffer for enough imu data at 250Hz
	imu_buffer = malloc(max_imu_data*IMU_LINE_BYTES);
	if(imu_buffer == NULL){
		printf("Cannot allocate buffer for IMU data \n");
		return 0 ;
	}
	//dummy_image = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
	
	sprintf(line_buffer, "%s/IMU.log", path_base);
        imuFile = fopen(line_buffer, "w");
        if (imuFile == NULL) {
                perror("cannot open IMU file for write \n");
                return ;
        }
        export_imu_calib(imuFile, 4.0, 500.0); // AG and 500dps
	
	int opt;
	int i = 0, j = 0 ;
	int init = 0 ;
	
	signal(SIGINT, killHandler);
	signal(SIGKILL, killHandler);
	signal(SIGTERM, killHandler);
	signal(SIGTSTP, killHandler);
	thread_alive = 1 ;
	init_time();
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
	printf("Saving IMU data to disk %u \n", nb_imu_data);
	for(i = 0; i < nb_imu_data ; i ++){
		unsigned long timestamp ;
		short raw_imu [6];
		memcpy(&timestamp, &(imu_buffer[i*IMU_LINE_BYTES]), sizeof(unsigned long));
		memcpy(raw_imu, &(imu_buffer[(i*IMU_LINE_BYTES)+sizeof(unsigned long)]), 6*sizeof(short));
		export_imu_raw(imuFile, raw_imu, timestamp);
		usleep(1);
	}
	printf("save done \n");
	fclose(imuFile);
	free(imu_buffer);
	free_frame_buffer(&my_frame_buffer);
	return 0;
}
