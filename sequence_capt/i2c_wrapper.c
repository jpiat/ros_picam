#include "i2c_wrapper.h"


void i2c_write8(struct i2c_peripheral_struct periph, unsigned char reg, unsigned char val)
{
	unsigned char i2c_buffer[2];

	if (ioctl(periph.fd, I2C_SLAVE, periph.addr) < 0) {
		return ; 
	}

	i2c_buffer[0] = reg;
	i2c_buffer[1] = val;
	write(periph.fd, i2c_buffer, 2);
}

void i2c_write_buffer(struct i2c_peripheral_struct periph, unsigned char reg, unsigned char * val, unsigned int nb)
{
	unsigned char * i2c_buffer ;
	i2c_buffer = malloc((nb+1)*sizeof(char));
	
	if (ioctl(periph.fd, I2C_SLAVE, periph.addr) < 0) {
		return ; 
	}

	i2c_buffer[0] = reg;
	memcpy((void *) &(i2c_buffer[1]), (void *) val, nb*sizeof(char));
	write(periph.fd, i2c_buffer, nb+1);
	free(i2c_buffer);
}

void i2c_read8(struct i2c_peripheral_struct periph, unsigned char reg, unsigned char * val)
{
	unsigned char i2c_buffer[2];

	if (ioctl(periph.fd, I2C_SLAVE, periph.addr) < 0) {
		return ; 
	}

	i2c_buffer[0] = reg;
	write(periph.fd, i2c_buffer, 1);
	read(periph.fd, val, 1);
}


void i2c_read_buffer(struct i2c_peripheral_struct periph, unsigned char reg, unsigned char * val, unsigned int nb)
{
	unsigned char i2c_buffer[2];

	if (ioctl(periph.fd, I2C_SLAVE, periph.addr) < 0) {
		return ; 
	}

	i2c_buffer[0] = reg;
	write(periph.fd, i2c_buffer, 1);
	read(periph.fd, val, nb);
}
