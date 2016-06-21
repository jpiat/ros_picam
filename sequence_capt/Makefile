OBJS = objs

CFLAGS += -I/usr/local/include/opencv -I/usr/local/include -I../dependencies/motor_hat_clib
LDFLAGS += -L/usr/local/lib -lopencv_highgui -lopencv_core -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_imgproc -lpthread -lm
CFLAGS += -O3 -mfpu=neon
LDFLAGS += -lraspicamcv -lm -lrt -lwiringPi #-llz4

OBJS = \
	L3GD20.o \
	LSM303_U.o \
	MPU9250.o \
	i2c_wrapper.o

all: capture_sequence capture_sequence_compress

%o: %c
	gcc -c $(CFLAGS) $< -o $@

capture_sequence : $(OBJS) capture_sequence.o
	gcc $(LDFLAGS) -o $@ capture_sequence.o $(OBJS)

capture_sequence_laser : $(OBJS) capture_sequence_laser.o
	gcc $(LDFLAGS) -o $@ capture_sequence_laser.o $(OBJS)

capture_sequence_compress : $(OBJS) capture_sequence_compress.o
	gcc $(LDFLAGS) -o $@ capture_sequence_compress.o $(OBJS)
clean:
	rm -f *.o
	rm capture_sequence capture_sequence_compress
