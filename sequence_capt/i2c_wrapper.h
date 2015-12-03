#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <string.h>

#ifndef I2C_WRAPPER_H
#define I2C_WRAPPER_H
struct i2c_peripheral_struct{
	int fd;
	unsigned char addr;
};

void i2c_write8(struct i2c_peripheral_struct periph, unsigned char reg, unsigned char val);

void i2c_write_buffer(struct i2c_peripheral_struct periph, unsigned char reg, unsigned char * val, unsigned int nb);

void i2c_read8(struct i2c_peripheral_struct periph, unsigned char reg, unsigned char * val);

void i2c_read_buffer(struct i2c_peripheral_struct periph, unsigned char reg, unsigned char * val, unsigned int nb);

#endif
