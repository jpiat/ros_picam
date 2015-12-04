/***************************************************
  This is a library for the L3GD20 and L3GD20H GYROSCOPE

  Designed specifically to work with the Adafruit L3GD20(H) Breakout 
  ----> https://www.adafruit.com/products/1032

  These sensors use I2C or SPI to communicate, 2 pins (I2C) 
  or 4 pins (SPI) are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Kevin "KTOWN" Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "L3GD20.h"
#include "i2c_wrapper.h"


struct i2c_peripheral_struct l3gd20_periph;
char l3gd20_range ;

void L3GD20_write8(unsigned char reg, unsigned char value)
{
   i2c_write8(l3gd20_periph, reg, value);
}

unsigned char L3GD20_read8(unsigned char reg)
{
  unsigned char value;
  i2c_read8(l3gd20_periph, reg, &value);
  return value;
}


int L3GD20_begin(int fd, unsigned char rng, unsigned char addr)
{
   l3gd20_periph.fd= fd ;
   l3gd20_periph.addr= addr ;
   l3gd20_range = rng;

  /* Make sure we have the correct chip ID since this checks
     for correct address and that the IC is properly connected */
  uint8_t id = L3GD20_read8(L3GD20_REGISTER_WHO_AM_I);
  //Serial.println(id, HEX);
  if ((id != L3GD20_ID) && (id != L3GD20H_ID))
  {
    return 0;
  }
  if(id == L3GD20_ID) printf("Gyro is L3GD20\n");
  if(id == L3GD20H_ID) printf("Gyro is L3GD20H\n");


  /* Set CTRL_REG1 (0x20)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
   7-6  DR1/0     Output data rate                                   00
   5-4  BW1/0     Bandwidth selection                                00
     3  PD        0 = Power-down mode, 1 = normal/sleep mode          0
     2  ZEN       Z-axis enable (0 = disabled, 1 = enabled)           1
     1  YEN       Y-axis enable (0 = disabled, 1 = enabled)           1
     0  XEN       X-axis enable (0 = disabled, 1 = enabled)           1 */

  /* Switch to normal mode and enable all three channels */
  L3GD20_write8(L3GD20_REGISTER_CTRL_REG1, 0x0F);
  /* ------------------------------------------------------------------ */

  /* Set CTRL_REG2 (0x21)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
   5-4  HPM1/0    High-pass filter mode selection                    00
   3-0  HPCF3..0  High-pass filter cutoff frequency selection      0000 */
 // L3GD20_write8(L3GD20_REGISTER_CTRL_REG2, 0x02);
  /* Nothing to do ... keep default values */
  /* ------------------------------------------------------------------ */

  /* Set CTRL_REG3 (0x22)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
     7  I1_Int1   Interrupt enable on INT1 (0=disable,1=enable)       0
     6  I1_Boot   Boot status on INT1 (0=disable,1=enable)            0
     5  H-Lactive Interrupt active config on INT1 (0=high,1=low)      0
     4  PP_OD     Push-Pull/Open-Drain (0=PP, 1=OD)                   0
     3  I2_DRDY   Data ready on DRDY/INT2 (0=disable,1=enable)        0
     2  I2_WTM    FIFO wtrmrk int on DRDY/INT2 (0=dsbl,1=enbl)        0
     1  I2_ORun   FIFO overrun int on DRDY/INT2 (0=dsbl,1=enbl)       0
     0  I2_Empty  FIFI empty int on DRDY/INT2 (0=dsbl,1=enbl)         0 */

  /* Nothing to do ... keep default values */
  /* ------------------------------------------------------------------ */

  /* Set CTRL_REG4 (0x23)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
     7  BDU       Block Data Update (0=continuous, 1=LSB/MSB)         0
     6  BLE       Big/Little-Endian (0=Data LSB, 1=Data MSB)          0
   5-4  FS1/0     Full scale selection                               00
                                  00 = 250 dps
                                  01 = 500 dps
                                  10 = 2000 dps
                                  11 = 2000 dps
     0  SIM       SPI Mode (0=4-wire, 1=3-wire)                       0 */

  /* Adjust resolution if requested */
  switch(l3gd20_range)
  {
    case L3DS20_RANGE_250DPS:
      L3GD20_write8(L3GD20_REGISTER_CTRL_REG4, 0x00);
      break;
    case L3DS20_RANGE_500DPS:
      L3GD20_write8(L3GD20_REGISTER_CTRL_REG4, 0x10);
      break;
    case L3DS20_RANGE_2000DPS:
      L3GD20_write8(L3GD20_REGISTER_CTRL_REG4, 0x20);
      break;
  }
  /* ------------------------------------------------------------------ */

  /* Set CTRL_REG5 (0x24)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
     7  BOOT      Reboot memory content (0=normal, 1=reboot)          0
     6  FIFO_EN   FIFO enable (0=FIFO disable, 1=enable)              0
     4  HPen      High-pass filter enable (0=disable,1=enable)        0
   3-2  INT1_SEL  INT1 Selection config                              00
   1-0  OUT_SEL   Out selection config                               00 */
   //L3GD20_write8(L3GD20_REGISTER_CTRL_REG5, 0x10);
  /* Nothing to do ... keep default values */
  /* ------------------------------------------------------------------ */

  return 1;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/
int L3GD20_read(float * xyz)
{ 
   unsigned char buffer [6];
   unsigned char status ;
   short raw_x, raw_y, raw_z ;
   i2c_read8(l3gd20_periph, L3GD20_REGISTER_STATUS_REG, &status); // reading status data 
   if((status & XYZ_AVAILABLE_FLAG) == 0){ 
	return 0 ;	
   }

   i2c_read_buffer( l3gd20_periph, L3GD20_REGISTER_OUT_X_L | 0x80, buffer, 6); // reading raw data


  // Shift values to create properly formed integer (low unsigned charfirst)
  raw_x = (short)(buffer[0] | (buffer[1] << 8));
  raw_y = (short)(buffer[2] | (buffer[3] << 8));
  raw_z= (short)(buffer[4] | (buffer[5] << 8));
  
  // Compensate values depending on the resolution
  switch(l3gd20_range)
  {
    case L3DS20_RANGE_250DPS:
      xyz[0] = ((float) raw_x) * L3GD20_SENSITIVITY_250DPS;
      xyz[1] = ((float) raw_y) * L3GD20_SENSITIVITY_250DPS;
      xyz[2] = ((float) raw_z) * L3GD20_SENSITIVITY_250DPS;
      break;
    case L3DS20_RANGE_500DPS:
      xyz[0] = ((float) raw_x) * L3GD20_SENSITIVITY_500DPS;
      xyz[1] = ((float) raw_y) * L3GD20_SENSITIVITY_500DPS;
      xyz[2] = ((float) raw_z) * L3GD20_SENSITIVITY_500DPS;
      break;
    case L3DS20_RANGE_2000DPS:
      xyz[0] = ((float) raw_x) * L3GD20_SENSITIVITY_2000DPS;
      xyz[1] = ((float) raw_y) * L3GD20_SENSITIVITY_2000DPS;
      xyz[2] = ((float) raw_z) * L3GD20_SENSITIVITY_2000DPS;
      break;
  }
  return 1 ;
}


void L3GD20_calib(){

}
