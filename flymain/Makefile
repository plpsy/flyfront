CFLAGS = -g -fPIC
CPPFLAGS = -I ./ -g -fPIC
#CC = /tools/gcc-6.5.0-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc
CC = aarch64-linux-gnu-gcc-10

all : libfly.so
static : libfly.a
.PHONY : all

libfly-objects:= flyslice_cnn_api.o

libfly.so : $(libfly-objects)
	$(CC) -g $(libfly-objects) -shared -I -fPIC -lm -o $@

libfly.a : $(libfly-objects)
	ar crv libfly.a *.o
flymain : 
	$(CC) -o flymain flyslice_cnn_api.c cJSON.c flymain.c
yolo :
	$(CC) -o yolo3 cJSON.c yolo_test.c

clean:
	rm -f *.o libfly.so flymain yolo3
