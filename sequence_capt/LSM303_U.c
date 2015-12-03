


#include "LSM303_U.h"

static float _lsm303Accel_MG_LSB     = 0.001F;   // 1, 2, 4 or 12 mg per lsb

struct i2c_peripheral_struct lsm303_periph;


/**************************************************************************/
/*!
    @brief  Reads the raw data from the sensor
*/
/**************************************************************************/
int LSM303_Acc_read(float * acc_data)
{
	unsigned char buffer [6];
	unsigned char status ;
	short raw_x, raw_y, raw_z ;
	i2c_read_8(lsm303_periph, LSM303_REGISTER_ACCEL_STATUS_REG_A, &status); // reading status data 
	if((status & XYZ_AVAILABLE_FLAG) == 0){
		return 0 ;	
	}

	
	i2c_read_buffer(lsm303_periph, LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80, buffer, 6); // reading raw data 

	// Shift values to create properly formed integer (low byte first)
	acc_data[0] = (float)((short)(buffer[0] | (buffer[1] << 8)) >> 4);
	acc_data[1] = (float)((short)(buffer[2] | (buffer[3] << 8)) >> 4);
	acc_data[2]= (float)((short)(buffer[4] | (buffer[5] << 8)) >> 4);

	//Scaling
	acc_data[0] *= (_lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD);
	acc_data[1] *= (_lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD);
	acc_data[2] *= (_lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD);
	return 1;
}


int LSM303_Acc_begin(int fd, unsigned char addr)
{
  unsigned char reg1_a  ;
  lsm303_periph.fd= fd ;
  lsm303_periph.addr= addr ;

  // Enable the accelerometer (100Hz)
  i2c_write8(lsm303_periph, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x57);

  // LSM303DLHC has no WHOAMI register so read CTRL_REG1_A back to check
  // if we are connected or not
  i2c_read8(lsm303_periph, LSM303_REGISTER_ACCEL_CTRL_REG1_A, &reg1_a);
  if (reg1_a != 0x57)
  {
    return 0;
  }  

  return 1;
}

