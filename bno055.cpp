/*
This library is for the BNO055 IMU

Revision History
================
2020/June/18  Barebone working code, non-library written by Yuchong Li
2021/Feb/10   Changing code to be a library BNO 055 to work on ESP32 Featherwing for ePoD by Kevin Nichols

*/

//Attached libraries
#include "bno055.h"

//#define _DEBUG_BYOASS_BNO

bool bno055::begin(){
#ifdef _DEBUG_BYOASS_BNO
  return true;
#endif

  uint8_t bno_id = bno055_read8(BNO055_CHIP_ID_ADDR);
  delay(100);
  Serial.print("BNO ID: ");
  Serial.println(bno_id);

  if(bno_id != BNO055_ID){
    Serial.println("bno055 id failed");
    return false;
  }

  bno055_write8(BNO055_OPR_MODE_ADDR, CONFIGMODE);  //change into configure mode
  bno055_write8(BNO055_SYS_TRIGGER_ADDR, 0x20);     //Reset system
  delay(50);                                        //Reset delay
  Serial.println("bno055 Reset");
  while (bno055_read8(BNO055_CHIP_ID_ADDR) != BNO055_ID) { //Recheck reading chip ID address
    delay(20);
  }
  Serial.println("bno055 id re-read");
  delay (50);
  bno055_write8(BNO055_PWR_MODE_ADDR, NORMAL_MODE); // set mode 
  delay(20);
  
  //Select page ID 1 to write to the right register
  bno055_write8(BNO055_PAGE_ID_ADDR, 1);
  //build the config value with bitshifts
  //see page 77
  //use all standard values except 16G parameter
  uint8_t acc_config = (uint8_t)((BNO055_ACC_PWRMODE_NORMAL << 5) 
                     | (BNO055_ACC_BW_62_5_Hz << 2)  
                     | BNO055_ACC_CONFIG_16G);
   bno055_write8(BNO055_ACC_CONFIG_ADDR, acc_config);
   delay(20);
   //Return to Select page ID 1
   bno055_write8(BNO055_PAGE_ID_ADDR, 0);
 
 

  delay(20);

//  bno055_write8(BNO055_OPR_MODE_ADDR, NDOF);//????
 
  bno055_write8(BNO055_SYS_TRIGGER_ADDR, 0x00);     //Set system to use internal oscillator
//  bno055_write8(BNO055_SYS_TRIGGER_ADDR, 0x80);     //Set system to use external oscillator
  delay(20);
  bno055_write8(BNO055_OPR_MODE_ADDR, ACCONLY);     //change into accelermeter mode only
  delay(20);
  return true; 
}

accelerometer_t bno055::update(){
  accelerometer_t Data = {
    /*  bool MperSS*/ false,
    /*  float accelX*/ 0.0,
    /*  float accelY*/ 0.0,
    /*  float accelZ*/ 0.0,
    /*  float gravX*/ 0.0,
    /*  float gravY*/ 0.0,
    /*  float gravZ*/ 0.0
  };
  
#ifdef _DEBUG_BYOASS_BNO
  return Data;
#endif
  
  // read bno055 acceleration
  float accel_x = (int16_t)bno055_read16(BNO055_ACCEL_DATA_x_LSB_ADDR, BNO055_ACCEL_DATA_x_MSB_ADDR);
  float accel_y = (int16_t)bno055_read16(BNO055_ACCEL_DATA_Y_LSB_ADDR, BNO055_ACCEL_DATA_Y_MSB_ADDR);
  float accel_z = (int16_t)bno055_read16(BNO055_ACCEL_DATA_Z_LSB_ADDR, BNO055_ACCEL_DATA_Z_MSB_ADDR);

  // read bno055 acceleration
  float gav_x = (int16_t)bno055_read16(BNO055_GRAVITY_DATA_x_LSB_ADDR, BNO055_GRAVITY_DATA_x_MSB_ADDR);
  float gav_y = (int16_t)bno055_read16(BNO055_GRAVITY_DATA_Y_LSB_ADDR, BNO055_GRAVITY_DATA_Y_MSB_ADDR);
  float gav_z = (int16_t)bno055_read16(BNO055_GRAVITY_DATA_Z_LSB_ADDR, BNO055_GRAVITY_DATA_Z_MSB_ADDR);

  if (Data.MperSS==true){
    Data.accelX = ((double)accel_x/100.0);
    Data.accelY = ((double)accel_y/100.0);
    Data.accelZ = ((double)accel_z/100.0);
    Data.gravX = ((double)gav_x/100.0);
    Data.gravY = ((double)gav_y/100.0);
    Data.gravZ = ((double)gav_z/100.0);
  }
  else{
    Data.accelX = ((double)accel_x/981.0);
    Data.accelY = ((double)accel_y/981.0);
    Data.accelZ = ((double)accel_z/981.0);
    Data.gravX = ((double)gav_x/981.0);
    Data.gravY = ((double)gav_y/981.0);
    Data.gravZ = ((double)gav_z/981.0);
  }

  return Data;
}


bool bno055::bno055_write8(uint8_t reg, uint8_t value) 
{
  Wire.beginTransmission(addr_a);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)value);
  Wire.endTransmission();

  /* ToDo: Check for error! */
  return true;
}

byte bno055::bno055_read8(uint8_t reg) 
{
  byte value = 0;
  Wire.beginTransmission(addr_a);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();
  Wire.requestFrom(addr_a, (byte)1);
  value = Wire.read();
  return value;
}

uint16_t bno055::bno055_read16(uint8_t lsb, uint8_t msb)
{
    uint8_t e_msb, e_lsb;
    e_msb = bno055_read8(msb);
    e_lsb = bno055_read8(lsb);
    
    return (e_msb << 8 | e_lsb);
}
