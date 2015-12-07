#include "MPU9250.h"


struct i2c_peripheral_struct MPU9250_periph;


int MPU9250_begin(int fd, char addr){
	unsigned char dummy ;
	MPU9250_periph.addr = addr ;
	MPU9250_periph.fd = fd ;

	i2c_read8(MPU9250_periph, MPUREG_WHOAMI, &dummy);
	if(dummy != 0x71){
		printf("Sensor ID (%x) doe snot match MPU9250 \n", dummy);
		return -1 ;
	}
	i2c_write8(MPU9250_periph, MPUREG_PWR_MGMT_1, 0x80);
	usleep(1000);
	// Initialize MPU9250 device
	// wake up device
	// get stable time source
        i2c_write8(MPU9250_periph, MPUREG_PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyro
	usleep(100000L);
	i2c_write8(MPU9250_periph, MPUREG_PWR_MGMT_2, 0x00); // Clear sleep mode bit (6), enable all sensors 

	// Configure Gyro and Accelerometer
	// Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively; 
	// DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
	// Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
	i2c_write8(MPU9250_periph, MPUREG_CONFIG, 0x03);  

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	i2c_write8(MPU9250_periph, MPUREG_SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above

	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	i2c_read8(MPU9250_periph, MPUREG_GYRO_CONFIG, &dummy);
	i2c_write8(MPU9250_periph, MPUREG_GYRO_CONFIG, dummy & ~0xE0); // Clear self-test bits [7:5] 
	i2c_write8(MPU9250_periph, MPUREG_GYRO_CONFIG, dummy & ~0x18); // Clear AFS bits [4:3]
	i2c_write8(MPU9250_periph, MPUREG_GYRO_CONFIG, dummy | 1 << 3); // Set full scale range for the gyro

	// Set accelerometer configuration
	i2c_read8(MPU9250_periph, MPUREG_ACCEL_CONFIG, &dummy);
	i2c_write8(MPU9250_periph, MPUREG_ACCEL_CONFIG, dummy & ~0xE0); // Clear self-test bits [7:5] 
	i2c_write8(MPU9250_periph, MPUREG_ACCEL_CONFIG, dummy & ~0x18); // Clear AFS bits [4:3]
	i2c_write8(MPU9250_periph, MPUREG_ACCEL_CONFIG, dummy | 1 << 3); // Set scale to 4G 

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	i2c_read8(MPU9250_periph, MPUREG_ACCEL_CONFIG_2, &dummy);
	i2c_write8(MPU9250_periph, MPUREG_ACCEL_CONFIG_2, dummy & ~0x0F); // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
	i2c_write8(MPU9250_periph, MPUREG_ACCEL_CONFIG_2, dummy | 0x03); // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips 
	// can join the I2C bus and all can be controlled by the Arduino as master
	i2c_write8(MPU9250_periph, MPUREG_INT_PIN_CFG, 0x22);    
	i2c_write8(MPU9250_periph, MPUREG_INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
}

int MPU9250_read(float * xyz_rpy){
	char raw_buffer[12];
	short short_val[6];
	char status ;
	i2c_read8(MPU9250_periph, MPUREG_INT_STATUS, &status);
	//printf("Status is %x \n", status);
	if((status & 0x01) != 0) {
		//printf("data available \n");
		i2c_read_buffer(MPU9250_periph, MPUREG_ACCEL_XOUT_H, &raw_buffer[0], 6);
		i2c_read_buffer(MPU9250_periph, MPUREG_GYRO_XOUT_H, &raw_buffer[6], 6);
		short_val[0] = (((short)raw_buffer[0] << 8) | (short) raw_buffer[1]) ;
		short_val[1] = (((short)raw_buffer[2] << 8) | (short) raw_buffer[3]) ;
		short_val[2] = (((short)raw_buffer[4] << 8) | (short) raw_buffer[5]) ;
		short_val[3] = (((short)raw_buffer[6] << 8) | (short) raw_buffer[7]) ;
		short_val[4] = (((short)raw_buffer[8] << 8) | (short) raw_buffer[9]) ;
		short_val[5] = (((short)raw_buffer[10] << 8) | (short) raw_buffer[11]) ;

		xyz_rpy[0] = MPU9250A_4G * ((float) short_val[0]) * G_CONST ;
		xyz_rpy[1] = MPU9250A_4G * ((float) short_val[1]) * G_CONST;
		xyz_rpy[2] = MPU9250A_4G  * ((float) short_val[2]) * G_CONST;

		xyz_rpy[3] = MPU9250G_500DPS  * ((float) short_val[3]) ;
		xyz_rpy[4] = MPU9250G_500DPS  * ((float) short_val[4]) ;
		xyz_rpy[5] = MPU9250G_500DPS  * ((float) short_val[5]) ;

		return 1 ;
	}
	return 0 ;
}

int MPU9250_read_raw(short * xyz_rpy){
        char raw_buffer[12];
        char status ;
        i2c_read8(MPU9250_periph, MPUREG_INT_STATUS, &status);
        //printf("Status is %x \n", status);
        if((status & 0x01) != 0) {
                //printf("data available \n");
                i2c_read_buffer(MPU9250_periph, MPUREG_ACCEL_XOUT_H, &raw_buffer[0], 6);
                i2c_read_buffer(MPU9250_periph, MPUREG_GYRO_XOUT_H, &raw_buffer[6], 6);
                xyz_rpy[0] = (((short)raw_buffer[0] << 8) | (short) raw_buffer[1]) ;
                xyz_rpy[1] = (((short)raw_buffer[2] << 8) | (short) raw_buffer[3]) ;
                xyz_rpy[2] = (((short)raw_buffer[4] << 8) | (short) raw_buffer[5]) ;
               	xyz_rpy[3] = (((short)raw_buffer[6] << 8) | (short) raw_buffer[7]) ;
                xyz_rpy[4] = (((short)raw_buffer[8] << 8) | (short) raw_buffer[9]) ;
                xyz_rpy[5] = (((short)raw_buffer[10] << 8) | (short) raw_buffer[11]) ;
                return 1 ;
        }
        return 0 ;
}

