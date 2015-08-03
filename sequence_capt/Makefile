OBJS = objs

CFLAGS += -I/usr/local/include/opencv -I/usr/local/include
LDFLAGS += -L/usr/local/lib -lopencv_highgui -lopencv_core -lopencv_legacy -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_imgproc -lpthread -lm
CFLAGS += -O3 -mfpu=neon
LDFLAGS += -lraspicamcv -lm

OBJS = \
	capture_sequence.o \

all: capture_sequence

%o: %c
	gcc -c $(CFLAGS) $< -o $@

capture_sequence : capture_sequence.o
	gcc $(LDFLAGS) -o $@ capture_sequence.o

clean:
	rm -f *.o

