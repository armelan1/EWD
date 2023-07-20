/*
This header is for the BNO055 IMU

Revision History
================
2020/June/18  Barebone working code, non-library written by Yuchong Li
2021/Feb/10   Changing code to be a library BNO 055 to work on ESP32 Featherwing for ePoD by Kevin Nichols

*/

#ifndef bno055_H
#define bno055_H

//Attached libraries
#include <Arduino.h>
#include <Wire.h>

// bno055 parameters
#define BNO055_ADDRESS_A    (0x28)
#define BNO055_ADDRESS_B    (0x29)
#define BNO055_ID           (0xA0)
#define BNO055_CHIP_ID_ADDR (0x00)

#define addr_a (BNO055_ADDRESS_A)
#define addr_b (BNO055_ADDRESS_B)


//values taken from section 3.5.2 page 27
#define BNO055_ACC_CONFIG_2G (0) //0B00 
#define BNO055_ACC_CONFIG_4G (1) //0B01
#define BNO055_ACC_CONFIG_8G (2)  //0B10
#define BNO055_ACC_CONFIG_16G (3)  //0B11

//bandwidth definitions
#define BNO055_ACC_BW_7_81_Hz (0)  
#define BNO055_ACC_BW_15_63_Hz (1)  
#define BNO055_ACC_BW_31_25_Hz (2)   
#define BNO055_ACC_BW_62_5_Hz (3)  
#define BNO055_ACC_BW_125_Hz (4)  
#define BNO055_ACC_BW_250_Hz (5)  
#define BNO055_ACC_BW_500_Hz (6)  
#define BNO055_ACC_BW_1000_Hz (7) 

//powermode definitions 
#define BNO055_ACC_PWRMODE_NORMAL (0)  
#define BNO055_ACC_PWRMODE_SUSPEND (1) 
#define BNO055_ACC_PWRMODE_LP1 (2) 
#define BNO055_ACC_PWRMODE_STANDBY (3)  
#define BNO055_ACC_PWRMODE_LP2 (4) 
#define BNO055_ACC_PWRMODE_DEEP_SUSPEND (5) 

#define BNO055_ACC_CONFIG_ADDR 0x08 

struct accelerometer_t {
  bool MperSS;
  float accelX;
  float accelY;
  float accelZ;
  float gravX;
  float gravY;
  float gravZ;
};

typedef enum
{
      /* Page id register definition */
      BNO055_PAGE_ID_ADDR                                     = 0x07,

      /* PAGE0 REGISTER DEFINITION START*/
      
      BNO055_ACCEL_REV_ID_ADDR                                = 0x01,
      BNO055_MAG_REV_ID_ADDR                                  = 0x02,
      BNO055_GYRO_REV_ID_ADDR                                 = 0x03,
      BNO055_SW_REV_ID_LSB_ADDR                               = 0x04,
      BNO055_SW_REV_ID_MSB_ADDR                               = 0x05,
      BNO055_BL_REV_ID_ADDR                                   = 0x06,

      /* Accel data register */
      BNO055_ACCEL_DATA_x_LSB_ADDR                            = 0x08,
      BNO055_ACCEL_DATA_x_MSB_ADDR                            = 0x09,
      BNO055_ACCEL_DATA_Y_LSB_ADDR                            = 0x0A,
      BNO055_ACCEL_DATA_Y_MSB_ADDR                            = 0x0B,
      BNO055_ACCEL_DATA_Z_LSB_ADDR                            = 0x0C,
      BNO055_ACCEL_DATA_Z_MSB_ADDR                            = 0x0D,

      /* Mag data register */
      BNO055_MAG_DATA_x_LSB_ADDR                              = 0x0E,
      BNO055_MAG_DATA_x_MSB_ADDR                              = 0x0F,
      BNO055_MAG_DATA_Y_LSB_ADDR                              = 0x10,
      BNO055_MAG_DATA_Y_MSB_ADDR                              = 0x11,
      BNO055_MAG_DATA_Z_LSB_ADDR                              = 0x12,
      BNO055_MAG_DATA_Z_MSB_ADDR                              = 0x13,

      /* Gyro data registers */
      BNO055_GYRO_DATA_x_LSB_ADDR                             = 0x14,
      BNO055_GYRO_DATA_x_MSB_ADDR                             = 0x15,
      BNO055_GYRO_DATA_Y_LSB_ADDR                             = 0x16,
      BNO055_GYRO_DATA_Y_MSB_ADDR                             = 0x17,
      BNO055_GYRO_DATA_Z_LSB_ADDR                             = 0x18,
      BNO055_GYRO_DATA_Z_MSB_ADDR                             = 0x19,

      /* Euler data registers */
      BNO055_EULER_H_LSB_ADDR                                 = 0x1A,
      BNO055_EULER_H_MSB_ADDR                                 = 0x1B,
      BNO055_EULER_R_LSB_ADDR                                 = 0x1C,
      BNO055_EULER_R_MSB_ADDR                                 = 0x1D,
      BNO055_EULER_P_LSB_ADDR                                 = 0x1E,
      BNO055_EULER_P_MSB_ADDR                                 = 0x1F,

      /* Quaternion data registers */
      BNO055_QUATERNION_DATA_W_LSB_ADDR                       = 0x20,
      BNO055_QUATERNION_DATA_W_MSB_ADDR                       = 0x21,
      BNO055_QUATERNION_DATA_X_LSB_ADDR                       = 0x22,
      BNO055_QUATERNION_DATA_X_MSB_ADDR                       = 0x23,
      BNO055_QUATERNION_DATA_Y_LSB_ADDR                       = 0x24,
      BNO055_QUATERNION_DATA_Y_MSB_ADDR                       = 0x25,
      BNO055_QUATERNION_DATA_Z_LSB_ADDR                       = 0x26,
      BNO055_QUATERNION_DATA_Z_MSB_ADDR                       = 0x27,

      /* Linear acceleration data registers */
      BNO055_LINEAR_ACCEL_DATA_x_LSB_ADDR                     = 0x28,
      BNO055_LINEAR_ACCEL_DATA_x_MSB_ADDR                     = 0x29,
      BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR                     = 0x2A,
      BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR                     = 0x2B,
      BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR                     = 0x2C,
      BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR                     = 0x2D,

      /* Gravity data registers */
      BNO055_GRAVITY_DATA_x_LSB_ADDR                          = 0x2E,
      BNO055_GRAVITY_DATA_x_MSB_ADDR                          = 0x2F,
      BNO055_GRAVITY_DATA_Y_LSB_ADDR                          = 0x30,
      BNO055_GRAVITY_DATA_Y_MSB_ADDR                          = 0x31,
      BNO055_GRAVITY_DATA_Z_LSB_ADDR                          = 0x32,
      BNO055_GRAVITY_DATA_Z_MSB_ADDR                          = 0x33,

      /* Temperature data register */
      BNO055_TEMP_ADDR                                        = 0x34,

      /* Status registers */
      BNO055_CALIB_STAT_ADDR                                  = 0x35,
      BNO055_SELFTEST_RESULT_ADDR                             = 0x36,
      BNO055_INTR_STAT_ADDR                                   = 0x37,

      BNO055_SYS_CLK_STAT_ADDR                                = 0x38,
      BNO055_SYS_STAT_ADDR                                    = 0x39,
      BNO055_SYS_ERR_ADDR                                     = 0x3A,

      /* Unit selection register */
      BNO055_UNIT_SEL_ADDR                                    = 0x3B,
      BNO055_DATA_SELECT_ADDR                                 = 0x3C,

      /* Mode registers */
      BNO055_OPR_MODE_ADDR                                    = 0x3D,
      BNO055_PWR_MODE_ADDR                                    = 0x3E,

      BNO055_SYS_TRIGGER_ADDR                                 = 0x3F,
      BNO055_TEMP_SOURCE_ADDR                                 = 0x40,

      /* Axis remap registers */
      BNO055_AxIS_MAP_CONFIG_ADDR                             = 0x41,
      BNO055_AxIS_MAP_SIGN_ADDR                               = 0x42,

      /* SIC registers */
      BNO055_SIC_MATRIx_0_LSB_ADDR                            = 0x43,
      BNO055_SIC_MATRIx_0_MSB_ADDR                            = 0x44,
      BNO055_SIC_MATRIx_1_LSB_ADDR                            = 0x45,
      BNO055_SIC_MATRIx_1_MSB_ADDR                            = 0x46,
      BNO055_SIC_MATRIx_2_LSB_ADDR                            = 0x47,
      BNO055_SIC_MATRIx_2_MSB_ADDR                            = 0x48,
      BNO055_SIC_MATRIx_3_LSB_ADDR                            = 0x49,
      BNO055_SIC_MATRIx_3_MSB_ADDR                            = 0x4A,
      BNO055_SIC_MATRIx_4_LSB_ADDR                            = 0x4B,
      BNO055_SIC_MATRIx_4_MSB_ADDR                            = 0x4C,
      BNO055_SIC_MATRIx_5_LSB_ADDR                            = 0x4D,
      BNO055_SIC_MATRIx_5_MSB_ADDR                            = 0x4E,
      BNO055_SIC_MATRIx_6_LSB_ADDR                            = 0x4F,
      BNO055_SIC_MATRIx_6_MSB_ADDR                            = 0x50,
      BNO055_SIC_MATRIx_7_LSB_ADDR                            = 0x51,
      BNO055_SIC_MATRIx_7_MSB_ADDR                            = 0x52,
      BNO055_SIC_MATRIx_8_LSB_ADDR                            = 0x53,
      BNO055_SIC_MATRIx_8_MSB_ADDR                            = 0x54,

      /* Accelerometer Offset registers */
      ACCEL_OFFSET_X_LSB_ADDR                                 = 0x55,
      ACCEL_OFFSET_X_MSB_ADDR                                 = 0x56,
      ACCEL_OFFSET_Y_LSB_ADDR                                 = 0x57,
      ACCEL_OFFSET_Y_MSB_ADDR                                 = 0x58,
      ACCEL_OFFSET_Z_LSB_ADDR                                 = 0x59,
      ACCEL_OFFSET_Z_MSB_ADDR                                 = 0x5A,

      /* Magnetometer Offset registers */
      MAG_OFFSET_X_LSB_ADDR                                   = 0x5B,
      MAG_OFFSET_X_MSB_ADDR                                   = 0x5C,
      MAG_OFFSET_Y_LSB_ADDR                                   = 0x5D,
      MAG_OFFSET_Y_MSB_ADDR                                   = 0x5E,
      MAG_OFFSET_Z_LSB_ADDR                                   = 0x5F,
      MAG_OFFSET_Z_MSB_ADDR                                   = 0x60,

      /* Gyroscope Offset register s*/
      GYRO_OFFSET_X_LSB_ADDR                                  = 0x61,
      GYRO_OFFSET_X_MSB_ADDR                                  = 0x62,
      GYRO_OFFSET_Y_LSB_ADDR                                  = 0x63,
      GYRO_OFFSET_Y_MSB_ADDR                                  = 0x64,
      GYRO_OFFSET_Z_LSB_ADDR                                  = 0x65,
      GYRO_OFFSET_Z_MSB_ADDR                                  = 0x66,

      /* Radius registers */
      ACCEL_RADIUS_LSB_ADDR                                   = 0x67,
      ACCEL_RADIUS_MSB_ADDR                                   = 0x68,
      MAG_RADIUS_LSB_ADDR                                     = 0x69,
      MAG_RADIUS_MSB_ADDR                                     = 0x6A,
} bno055_reg_t;

typedef enum
{
      NORMAL_MODE                                             = 0x00,
      LOW_POWER_MODE                                          = 0x01,
      SUSPEND_MODE                                            = 0x02,
} bno055_power_mode;

typedef enum
{
      CONFIGMODE                                              = 0x00,
      ACCONLY                                                 = 0x01,
      MAGONLY                                                 = 0x02,
      GYROONLY                                                = 0x03,
      ACCMAG                                                  = 0x04,
      ACCGYRO                                                 = 0x05,
      MAGGYRO                                                 = 0x06,
      AMG                                                     = 0x07,
      IMU                                                     = 0x08,
      COMPASS                                                 = 0x09,
      M4G                                                     = 0x0a,
      NDOF_FMC_OFF                                            = 0x0b,
      NDOF                                                    = 0x0c,
} bno055_mode;


  
//
//byte bno055_read8(uint8_t reg);
//
//uint16_t bno055_read16(uint8_t lsb, uint8_t msb);

class bno055
{
  public:
    
    void accelArray();
    bool begin();
    void setExtCrystalUse(boolean usextal);
    accelerometer_t update();

  private:
    bool bno055_write8(uint8_t reg, uint8_t value);
    byte bno055_read8(uint8_t reg);
    uint16_t bno055_read16(uint8_t lsb, uint8_t msb);
  
};

#endif
