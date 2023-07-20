/*
This header is for the MAXM86161 Pulse Oximeter

Creation Date: 02/10/2021

Revision History
================
2020/June/18  Barebone working code, non-library written by Yuchong Li
2021/Feb/10   Changing code to be a library MAXM86161 to work on ESP32 Featherwing for ePoD by Kevin Nichols
2021/March/18 LED light intensity adjustment

*/
#ifndef MAXM86161_H
#define MAXM86161_H

#include <Arduino.h>
#include <Wire.h>

extern float Instant_SpO2;

/* Filter parameters */
#define SAMPLING_AVERAGE SMP_AVG_1    //controls the sample averaging before exporting data
#define SAMPLING_RATE PPG_SR_99_902     //controls the sampling clock frequency rate
#define RED_LED_CURRENT_ADJUSTMENT_MS     750 //number of milliseconds between balanace intensites adjustments
#define MIN_ACCEPTABLE_INTENSITY_DIFF_RED   0 //99
#define MAX_ACCEPTABLE_INTENSITY_DIFF_RED   10 //140
#define STARTING_RED_LED_CURRENT MAXM86161_LED_CURRENT_13_13MA //16_05MA
#define STARTING_IR_LED_CURRENT MAXM86161_LED_CURRENT_21_88MA //_24_80MA//
#define STARTING_GREEN_LED_CURRENT MAXM86161_LED_CURRENT_0MA
#define LED_SETLNG LED_SETTLING_RATE_6us //_8us
#define DIG_FILT_SEL FILTER_TYPE_CDM
#define BURST_RATE BURST_MODE_RATE_8Hz
#define BURST_EN BURST_TURNED_OFF
#define ALC_DISABLE ALC_TURNED_ON
#define ADD_OFFSET OFFSET_IS_NOT_ADDED
#define PPG1_ADC_RGE ADC_RANGE_32768
#define PPG_TINT SET_TINT_123
#define I2C_ERROR_OK 0

// Picket fence registers
#define PF_ENABLE PF_TURNED_ON
#define PF_ORDER ORDER_NUMB_4
#define IIR_TC SET_IIR_BANDWIDHT_16th
#define IIR_INIT_VALUE SET_INIT_TO_48
#define THRESHOLD_SIGMA_MULT SIGMA_GAIN_16
 
 struct MAXM86161SampleData
{
    uint32_t GreenLED = 0; // Raw 19-bit Sample Data from LED1
    uint32_t IRLED = 0;    // Raw 19-bit Sample Data from LED2
    uint32_t RedLED = 0;   // Raw 19-bit Sample Data from LED3
    uint32_t Ambient = 0;  // Raw 19-bit Sample Data from Ambient Light Level
    uint8_t ValidData = 0; // Bit-Field indicating validity of individual Samples

    bool GreenLEDValid() { return !!(ValidData & 1); } // True if Green LED Sample is valid
    bool IRLEDValid() { return !!(ValidData & 2); }    // True if IR LED Sample is valid
    bool RedLEDValid() { return !!(ValidData & 4); }   // True if Red LED Sample is valid
    bool AmbientValid() { return !!(ValidData & 8); }  // True if Ambient Light Sample is valid
    bool AllValid() { return (ValidData == 15); }      // True if All Samples are valid
};

/* Enums, data structures and typdefs. DO NOT EDIT */
struct pulseoximeter_t {
  float ir_BPM;           // Calculated BMP
  float ir_PPG;           // Cleanup IR value for PPG graphing
  float red_PPG;          // Cleanup Red value for PPG graphing
  float green_PPG;        // Cleanup Green value for PPG graphing
  float red_SpO2;         // Calculated red-IR SpO2 percentage
  float ratioRMSred;      // red-IR RMS ratio value - sent out for data collection
  float green_SpO2;       // Calculated green-IR SpO2 percentage
  float ratioRMSgreen;    // green-IR RMS ratio value - sent out for data collection
  float green_BPM;
  float red_BPM;
  float IR_amp;           // amplitude of IR signal
  float red_amp;          // amplitude of IR signal
  float green_amp;        // amplitude of IR signal
  float beat_time;        // time between reported valid heartbeat
  float SpO2_red_time;    // time between reported valid red SpO2
  float SpO2_green_time;  // time between reported valid green SpO2
  float temperature;   
  //Control indicators on Pulse Oximeter status
  bool fingerPresent;     //Indicator if tissue is near
  bool bpm_Updated;
  bool red_SpO2_Updated;
  bool green_SpO2_Updated;
  int avg_fps;
    
};

static const uint8_t I2C_Address = 0x62;
static const uint8_t PartIDValue = 0x36;

static bool Initialized = false;
static bool SamplingEnabled = false;
static uint8_t LEDSeqCount = 4; // (LED1 > LED2 > LED3 > Ambient > Loop)

enum class RegAddr : uint8_t
{
    InterruptStatus1 = 0x00,
    InterruptStatus2 = 0x01,
    InterruptEnable1 = 0x02,
    InterruptEnable2 = 0x03,
    
    FIFOWritePointer = 0x04,
    FIFOReadPointer = 0x05,
    OverFlowPointer = 0x06,
    FIFODataCounter = 0x07,
    FIFODataRegister = 0x08,
    FIFOConfiguration1 = 0x09,
    FIFOConfiguration2 = 0x0A,
    
    SystemControl = 0x0D,
    
    PPGSyncControl = 0x10,
    PPGConfiguration1 = 0x11,
    PPGConfiguration2 = 0x12,
    PPGConfiguration3 = 0x13,
    ProxInterruptThreshold = 0x14,
    PhotoDiodeBias = 0x15,

    PicketFence = 0x16,

    LEDSequenceRegister1 = 0x20,
    LEDSequenceRegister2 = 0x21,
    LEDSequenceRegister3 = 0x22,

    LED1PA = 0x23,
    LED2PA = 0x24,
    LED3PA = 0x25,
    LEDPilotPA = 0x29,
    LEDRange1 = 0x2A,

    S1_HIRES_DAC1 = 0x2C,
    S2_HIRES_DAC1 = 0x2D,
    S3_HIRES_DAC1 = 0x2E,
    S4_HIRES_DAC1 = 0x2F,
    S5_HIRES_DAC1 = 0x30,
    S6_HIRES_DAC1 = 0x31,

    DieTemperatureConfiguration = 0x40,
    DieTemperatureInteger = 0x41,
    DieTemperatureFraction = 0x42,

    DACCalibrationEnable = 0x50,

    SHACommand = 0xF0,
    SHAConfiguration = 0xF1,

    MemoryControl = 0xF2,
    MemoryIndex = 0xF3,
    MemoryData = 0xF4,

    RevisionID = 0xFE,
    PartID = 0xFF
};

enum class FIFOTag : uint8_t
{
    PPG1_LEDC1_DATA = 0b00001,
    PPG1_LEDC2_DATA = 0b00010,
    PPG1_LEDC3_DATA = 0b00011,
    PPG1_LEDC4_DATA = 0b00100,
    PPG1_LEDC5_DATA = 0b00101,
    PPG1_LEDC6_DATA = 0b00110,

    PPF1_LEDC1_DATA = 0b01101,
    PPF1_LEDC2_DATA = 0b01110,
    PPF1_LEDC3_DATA = 0b01111,

    PROX1_DATA = 0b11001,

    SUB_DAC_UPDATE = 0b11101,
    INVALID_DATA = 0b11110,
    TIME_STAMP = 0b11111
};


/* Sample Average
 *  To reduce the amount of data throughput, adjacent samples (in each individual channel) 
 *  can be averaged and decimated on the chip by setting this register.*/
typedef enum SMP_AVE {
  SMP_AVG_1     = 000,
  SMP_AVG_2     = 001,
  SMP_AVG_4     = 010,
  SMP_AVG_8     = 011,
  SMP_AVG_16    = 100,
  SMP_AVG_32    = 101,
  SMP_AVG_64    = 110,
  SMP_AVG_128   = 111
} SMP_AVE;



/* LED_SETLNG
 *  Delay from rising edge of LED to start of ADC integration. This allow for the 
 *  LED current to settle before the start of ADC integration.*/
typedef enum  LED_SETTLING_RATE {
  LED_SETTLING_RATE_4us   = 00,
  LED_SETTLING_RATE_6us   = 01, //default value
  LED_SETTLING_RATE_8us   = 10,
  LED_SETTLING_RATE_12us  = 11
}  LED_SETTLING_RATE;



/* DIG_FILT_SEL
 *  Select Digital Filter Type.*/
typedef enum  FILTER_TYPE {
  FILTER_TYPE_CDM   = 0,
  FILTER_TYPE_FDM   = 1
}  FILTER_TYPE;



/* BURST_RATE */
typedef enum  BURST_MODE_RATE {
  BURST_MODE_RATE_8Hz     = 00,
  BURST_MODE_RATE_32Hz    = 01,
  BURST_MODE_RATE_84Hz    = 10,
  BURST_MODE_RATE_256Hz   = 11,
}  BURST_MODE_RATE;


/* BURST_EN
 *  Turn on/off burst mode.*/
typedef enum  BURST_TURNED {
  BURST_TURNED_OFF   = 0,
  BURST_TURNED_ON    = 1
}  BURST_TURNED;



/* ALC_DISABLE
 *  Turn on/off Abbient Light Cancellation.*/
typedef enum  ALC_TURNED {
  ALC_TURNED_OFF   = 0,
  ALC_TURNED_ON    = 1
}  ALC_TURNED;



/* ADD_OFFSET
 *  Option for dark current measurement.*/
typedef enum  OFFSET_IS {
  OFFSET_IS_NOT_ADDED   = 0,
  OFFSET_IS_ADDED    = 1 //If PPG_SR set for single pulse mode - offset 8192 ++++ if set for dual pulse mode - offset 4096
}  OFFSET_IS;



/* PPG1_ADC_RGE
 *  Set the ADC range of the SpO2 sensor.*/
typedef enum  ADC_RANGE {
  ADC_RANGE_4096     = 00,
  ADC_RANGE_8192     = 01,
  ADC_RANGE_16384    = 10,
  ADC_RANGE_32768    = 11,
}  ADC_RANGE;



/* PPG_TINT
 *  Set the pulse width of the LED driver and intgration time of the PPG ADC range of the SpO2 sensor.
  tPW = tTINT + tLED_SETLING + 0.5us*/
typedef enum  SET_TINT {
  SET_TINT_21     = 00, //tPW = 21.3 us & tINT = 14.8 us
  SET_TINT_36     = 01, //tPW = 35.9 us & tINT = 29.4 us
  SET_TINT_65     = 10, //tPW = 65.2 us & tINT = 58.7 us
  SET_TINT_123    = 11, //tPW = 123.8 us & tINT = 117.3 us
}  SET_TINT;


/* PF_ENABLE
 *  Turn on/off picket-fence detection and replace method.*/
typedef enum  PF_TURNED {
  PF_TURNED_OFF   = 0, // (default)
  PF_TURNED_ON    = 1
}  PF_TURNED;


/* PF_ORDER
 *  Determines which prediction method is used*/
typedef enum  ORDER_NUMB {
  ORDER_NUMB_1   = 0, //Last Sample (1 point)
  ORDER_NUMB_4   = 1  //Fit 4 points to a line for prediction (default)
}  ORDER_NUMB;


/* IIR_TC
 *  Determines the IIR filter bandwidth where the lowest setting has the narrowest of a first-order filter*/
typedef enum  SET_IIR_BANDWIDHT {
  SET_IIR_BANDWIDHT_64th     = 00, // 1/64 & 146 samples to 90%
  SET_IIR_BANDWIDHT_32nd     = 01, // 1/32 & 72 samples to 90%
  SET_IIR_BANDWIDHT_16th     = 10, // 1/16 & 35 samples to 90%
  SET_IIR_BANDWIDHT_8th      = 11, //  1/8 & 17 samples to 90%
}  SET_IIR_BANDWIDHT;


/* IIR_INIT_VALUE
 *  Controls the initial values for the IIR low-pass filter when algorthm is initialized*/
typedef enum  SET_INIT_TO {
  SET_INIT_TO_64     = 00,
  SET_INIT_TO_48     = 01,
  SET_INIT_TO_32     = 10,
  SET_INIT_TO_24     = 11,
}  SET_INIT_TO;


/* THREEHOLD_SIGMA_MULT
 *  GAIN resulting from the SIGMA_MULT setting determines the number of standard deviations
    of the delta between the actual and the predicted smaple beyond which a picket-fence event is trigggerd*/
typedef enum  SIGMA_GAIN {
  SIGMA_GAIN_4     = 00,
  SIGMA_GAIN_8     = 01,
  SIGMA_GAIN_16    = 10,
  SIGMA_GAIN_32    = 11,
}  SIGMA_GAIN;



/* Sampling Rate for PPG sensor
 *  Set the effective sampling rate of the PPG sensor with
 *  the on-chip sampling clock frequency is 32768Hz.*/
typedef enum PPG_SR {
  PPG_SR_24_995       = 0x00,
  PPG_SR_50_027       = 0x01,
  PPG_SR_84_021       = 0x02,
  PPG_SR_99_902       = 0x03,
  PPG_SR_199_805      = 0x04,
  PPG_SR_399_610      = 0x05,
  PPG_SR_24_995_N2    = 0x06,
  PPG_SR_50_027_N2    = 0x07,
  PPG_SR_84_021_N2    = 0x08,
  PPG_SR_99_902_N2    = 0x09,
  PPG_SR_8_00         = 0x0A,
  PPG_SR_16_00        = 0x0B,
  PPG_SR_32_00        = 0x0C,
  PPG_SR_64_00        = 0x0D,
  PPG_SR_128_00       = 0x0E,
  PPG_SR_256_00       = 0x0F,
  PPG_SR_512_00       = 0x10,
  PPG_SR_1024_00      = 0x11,
  PPG_SR_2048_00      = 0x12,
  PPG_SR_4069_00      = 0x13
} PPG_SR;

typedef enum LEDCurrent {
  MAXM86161_LED_CURRENT_0MA             = 0x00,
  MAXM86161_LED_CURRENT_0_48MA          = 0x01,
  MAXM86161_LED_CURRENT_0_97MA          = 0x02,
  MAXM86161_LED_CURRENT_1_46MA          = 0x03, 
  MAXM86161_LED_CURRENT_1_95MA          = 0x04,
  MAXM86161_LED_CURRENT_2_43MA          = 0x05,
  MAXM86161_LED_CURRENT_2_92MA          = 0x06,
  MAXM86161_LED_CURRENT_3_40MA          = 0x07,
  MAXM86161_LED_CURRENT_3_89MA          = 0x08,
  MAXM86161_LED_CURRENT_4_38MA          = 0x09,
  MAXM86161_LED_CURRENT_4_86MA          = 0x0A,
  MAXM86161_LED_CURRENT_5_35MA          = 0x0B,
  MAXM86161_LED_CURRENT_5_84MA          = 0x0C,
  MAXM86161_LED_CURRENT_6_32MA          = 0x0D,
  MAXM86161_LED_CURRENT_6_81MA          = 0x0E,
  MAXM86161_LED_CURRENT_7_29MA          = 0x0F,
  MAXM86161_LED_CURRENT_7_78MA          = 0x10,
  MAXM86161_LED_CURRENT_8_27MA          = 0x11,
  MAXM86161_LED_CURRENT_8_75MA          = 0x12,
  MAXM86161_LED_CURRENT_9_24MA          = 0x13,
  MAXM86161_LED_CURRENT_9_73MA          = 0x14,
  MAXM86161_LED_CURRENT_10_21MA         = 0x15,
  MAXM86161_LED_CURRENT_10_70MA         = 0x16,
  MAXM86161_LED_CURRENT_11_18MA         = 0x17,
  MAXM86161_LED_CURRENT_11_67MA         = 0x18,
  MAXM86161_LED_CURRENT_12_16MA         = 0x19,
  MAXM86161_LED_CURRENT_12_64MA         = 0x1A,
  MAXM86161_LED_CURRENT_13_13MA         = 0x1B,
  MAXM86161_LED_CURRENT_13_62MA         = 0x1C,
  MAXM86161_LED_CURRENT_14_10MA         = 0x1D,
  MAXM86161_LED_CURRENT_14_59MA         = 0x1E,
  MAXM86161_LED_CURRENT_15_07MA         = 0x1F,
  MAXM86161_LED_CURRENT_15_56MA         = 0x20,
  MAXM86161_LED_CURRENT_16_05MA         = 0x21,
  MAXM86161_LED_CURRENT_16_53MA         = 0x22,
  MAXM86161_LED_CURRENT_17_02MA         = 0x23,
  MAXM86161_LED_CURRENT_17_51MA         = 0x24,
  MAXM86161_LED_CURRENT_17_99MA         = 0x25,
  MAXM86161_LED_CURRENT_18_48MA         = 0x26,
  MAXM86161_LED_CURRENT_18_96MA         = 0x27,
  MAXM86161_LED_CURRENT_19_45MA         = 0x28,
  MAXM86161_LED_CURRENT_19_94MA         = 0x29,
  MAXM86161_LED_CURRENT_20_42MA         = 0x2A,
  MAXM86161_LED_CURRENT_20_91MA         = 0x2B,
  MAXM86161_LED_CURRENT_21_40MA         = 0x2C,
  MAXM86161_LED_CURRENT_21_88MA         = 0x2D,
  MAXM86161_LED_CURRENT_22_37MA         = 0x2E,
  MAXM86161_LED_CURRENT_22_85MA         = 0x2F,
  MAXM86161_LED_CURRENT_23_34MA         = 0x30,
  MAXM86161_LED_CURRENT_23_83MA         = 0x31,
  MAXM86161_LED_CURRENT_24_31MA         = 0x32,
  MAXM86161_LED_CURRENT_24_80MA         = 0x33,
  MAXM86161_LED_CURRENT_25_29MA         = 0x34,
  MAXM86161_LED_CURRENT_25_77MA         = 0x35,
  MAXM86161_LED_CURRENT_26_26MA         = 0x36,
  MAXM86161_LED_CURRENT_26_75MA         = 0x37,
  MAXM86161_LED_CURRENT_27_23MA         = 0x38,
  MAXM86161_LED_CURRENT_27_72MA         = 0x39,
  MAXM86161_LED_CURRENT_28_20MA         = 0x3A,
  MAXM86161_LED_CURRENT_28_69MA         = 0x3B,
  MAXM86161_LED_CURRENT_29_18MA         = 0x3C,
  MAXM86161_LED_CURRENT_29_66MA         = 0x3D,
  MAXM86161_LED_CURRENT_30_15MA         = 0x3E,
  MAXM86161_LED_CURRENT_30_64MA         = 0x3F,
  MAXM86161_LED_CURRENT_31_12MA         = 0x40,
  MAXM86161_LED_CURRENT_31_61MA         = 0x41,
  MAXM86161_LED_CURRENT_32_09MA         = 0x42,
  MAXM86161_LED_CURRENT_32_58MA         = 0x43,
  MAXM86161_LED_CURRENT_33_07MA         = 0x44,
  MAXM86161_LED_CURRENT_33_55MA         = 0x45,
  MAXM86161_LED_CURRENT_34_04MA         = 0x46,
  MAXM86161_LED_CURRENT_34_53MA         = 0x47,
  MAXM86161_LED_CURRENT_35_01MA         = 0x48,
  MAXM86161_LED_CURRENT_35_50MA         = 0x49,
  MAXM86161_LED_CURRENT_35_98MA         = 0x4A,
  MAXM86161_LED_CURRENT_36_47MA         = 0x4B,
  MAXM86161_LED_CURRENT_36_96MA         = 0x4C,
  MAXM86161_LED_CURRENT_37_44MA         = 0x4D,
  MAXM86161_LED_CURRENT_37_93MA         = 0x4E,
  MAXM86161_LED_CURRENT_38_42MA         = 0x4F,
  MAXM86161_LED_CURRENT_38_90MA         = 0x50,
  MAXM86161_LED_CURRENT_39_39MA         = 0x51,
  MAXM86161_LED_CURRENT_39_87MA         = 0x52,
  MAXM86161_LED_CURRENT_40_36MA         = 0x53,
  MAXM86161_LED_CURRENT_40_85MA         = 0x54,
  MAXM86161_LED_CURRENT_41_33MA         = 0x55,
  MAXM86161_LED_CURRENT_41_82MA         = 0x56,
  MAXM86161_LED_CURRENT_42_31MA         = 0x57,
  MAXM86161_LED_CURRENT_42_79MA         = 0x58,
  MAXM86161_LED_CURRENT_43_28MA         = 0x59,
  MAXM86161_LED_CURRENT_43_76MA         = 0x5A,
  MAXM86161_LED_CURRENT_44_25MA         = 0x5B,
  MAXM86161_LED_CURRENT_44_74MA         = 0x5C,
  MAXM86161_LED_CURRENT_45_22MA         = 0x5D,
  MAXM86161_LED_CURRENT_45_71MA         = 0x5E,
  MAXM86161_LED_CURRENT_46_20MA         = 0x5F,
  MAXM86161_LED_CURRENT_46_68MA         = 0x60,
  MAXM86161_LED_CURRENT_47_17MA         = 0x61,
  MAXM86161_LED_CURRENT_47_65MA         = 0x62,
  MAXM86161_LED_CURRENT_48_14MA         = 0x63,
  MAXM86161_LED_CURRENT_48_63MA         = 0x64,
  MAXM86161_LED_CURRENT_49_11MA         = 0x65,
  MAXM86161_LED_CURRENT_49_60MA         = 0x66,
  MAXM86161_LED_CURRENT_50_09MA         = 0x67,
  MAXM86161_LED_CURRENT_50_57MA         = 0x68,
  MAXM86161_LED_CURRENT_51_06MA         = 0x69,
  MAXM86161_LED_CURRENT_51_55MA         = 0x6A,
  MAXM86161_LED_CURRENT_52_03MA         = 0x6B,
  MAXM86161_LED_CURRENT_52_52MA         = 0x6C,
  MAXM86161_LED_CURRENT_53_00MA         = 0x6D,
  MAXM86161_LED_CURRENT_53_49MA         = 0x6E,
  MAXM86161_LED_CURRENT_53_98MA         = 0x6F,
  MAXM86161_LED_CURRENT_54_46MA         = 0x70,
  MAXM86161_LED_CURRENT_54_95MA         = 0x71,
  MAXM86161_LED_CURRENT_55_44MA         = 0x72,
  MAXM86161_LED_CURRENT_55_92MA         = 0x73,
  MAXM86161_LED_CURRENT_56_41MA         = 0x74,
  MAXM86161_LED_CURRENT_56_89MA         = 0x75,
  MAXM86161_LED_CURRENT_57_38MA         = 0x76,
  MAXM86161_LED_CURRENT_57_87MA         = 0x77,
  MAXM86161_LED_CURRENT_58_35MA         = 0x78,
  MAXM86161_LED_CURRENT_58_84MA         = 0x79,
  MAXM86161_LED_CURRENT_59_33MA         = 0x7A,
  MAXM86161_LED_CURRENT_59_81MA         = 0x7B,
  MAXM86161_LED_CURRENT_60_30MA         = 0x7C,
  MAXM86161_LED_CURRENT_60_78MA         = 0x7D,
  MAXM86161_LED_CURRENT_61_27MA         = 0x7E,
  MAXM86161_LED_CURRENT_61_76MA         = 0x7F,
  MAXM86161_LED_CURRENT_62_24MA         = 0x80,
  MAXM86161_LED_CURRENT_62_73MA         = 0x81,
  MAXM86161_LED_CURRENT_63_22MA         = 0x82,
  MAXM86161_LED_CURRENT_63_70MA         = 0x83,
  MAXM86161_LED_CURRENT_64_19MA         = 0x84,
  MAXM86161_LED_CURRENT_64_67MA         = 0x85,
  MAXM86161_LED_CURRENT_65_16MA         = 0x86,
  MAXM86161_LED_CURRENT_65_65MA         = 0x87,
  MAXM86161_LED_CURRENT_66_13MA         = 0x88,
  MAXM86161_LED_CURRENT_66_62MA         = 0x89,
  MAXM86161_LED_CURRENT_67_11MA         = 0x8A,
  MAXM86161_LED_CURRENT_67_59MA         = 0x8B,
  MAXM86161_LED_CURRENT_68_08MA         = 0x8C,
  MAXM86161_LED_CURRENT_68_56MA         = 0x8D,
  MAXM86161_LED_CURRENT_69_05MA         = 0x8E,
  MAXM86161_LED_CURRENT_69_54MA         = 0x8F,
  MAXM86161_LED_CURRENT_70_02MA         = 0x90,
  MAXM86161_LED_CURRENT_70_51MA         = 0x91,
  MAXM86161_LED_CURRENT_71_00MA         = 0x92,
  MAXM86161_LED_CURRENT_71_48MA         = 0x93,
  MAXM86161_LED_CURRENT_71_97MA         = 0x94,
  MAXM86161_LED_CURRENT_72_45MA         = 0x95,
  MAXM86161_LED_CURRENT_72_94MA         = 0x96,
  MAXM86161_LED_CURRENT_73_43MA         = 0x97,
  MAXM86161_LED_CURRENT_73_91MA         = 0x98,
  MAXM86161_LED_CURRENT_74_40MA         = 0x99,
  MAXM86161_LED_CURRENT_74_89MA         = 0x9A,
  MAXM86161_LED_CURRENT_75_37MA         = 0x9B,
  MAXM86161_LED_CURRENT_75_86MA         = 0x9C,
  MAXM86161_LED_CURRENT_76_35MA         = 0x9D,
  MAXM86161_LED_CURRENT_76_83MA         = 0x9E,
  MAXM86161_LED_CURRENT_77_32MA         = 0x9F,
  MAXM86161_LED_CURRENT_77_80MA         = 0xA0,
  MAXM86161_LED_CURRENT_78_29MA         = 0xA1,
  MAXM86161_LED_CURRENT_78_78MA         = 0xA2,
  MAXM86161_LED_CURRENT_79_26MA         = 0xA3,
  MAXM86161_LED_CURRENT_79_75MA         = 0xA4,
  MAXM86161_LED_CURRENT_80_24MA         = 0xA5,
  MAXM86161_LED_CURRENT_80_72MA         = 0xA6,
  MAXM86161_LED_CURRENT_81_21MA         = 0xA7,
  MAXM86161_LED_CURRENT_81_69MA         = 0xA8,
  MAXM86161_LED_CURRENT_82_18MA         = 0xA9,
  MAXM86161_LED_CURRENT_82_67MA         = 0xAA,
  MAXM86161_LED_CURRENT_83_15MA         = 0xAB,
  MAXM86161_LED_CURRENT_83_64MA         = 0xAC,
  MAXM86161_LED_CURRENT_84_13MA         = 0xAD,
  MAXM86161_LED_CURRENT_84_61MA         = 0xAE,
  MAXM86161_LED_CURRENT_85_10MA         = 0xAF,
  MAXM86161_LED_CURRENT_85_58MA         = 0xB0,
  MAXM86161_LED_CURRENT_86_07MA         = 0xB1,
  MAXM86161_LED_CURRENT_86_56MA         = 0xB2,
  MAXM86161_LED_CURRENT_87_04MA         = 0xB3,
  MAXM86161_LED_CURRENT_87_53MA         = 0xB4,
  MAXM86161_LED_CURRENT_88_02MA         = 0xB5,
  MAXM86161_LED_CURRENT_88_50MA         = 0xB6,
  MAXM86161_LED_CURRENT_88_99MA         = 0xB7,
  MAXM86161_LED_CURRENT_89_47MA         = 0xB8,
  MAXM86161_LED_CURRENT_89_96MA         = 0xB9,
  MAXM86161_LED_CURRENT_90_45MA         = 0xBA,
  MAXM86161_LED_CURRENT_90_93MA         = 0xBB,
  MAXM86161_LED_CURRENT_91_42MA         = 0xBC,
  MAXM86161_LED_CURRENT_91_91MA         = 0xBD,
  MAXM86161_LED_CURRENT_92_39MA         = 0xBE,
  MAXM86161_LED_CURRENT_92_88MA         = 0xBF,
  MAXM86161_LED_CURRENT_93_36MA         = 0xC0,
  MAXM86161_LED_CURRENT_93_85MA         = 0xC1,
  MAXM86161_LED_CURRENT_94_34MA         = 0xC2,
  MAXM86161_LED_CURRENT_94_82MA         = 0xC3,
  MAXM86161_LED_CURRENT_95_31MA         = 0xC4,
  MAXM86161_LED_CURRENT_95_80MA         = 0xC5,
  MAXM86161_LED_CURRENT_96_28MA         = 0xC6,
  MAXM86161_LED_CURRENT_96_77MA         = 0xC7,
  MAXM86161_LED_CURRENT_97_25MA         = 0xC8,
  MAXM86161_LED_CURRENT_97_74MA         = 0xC9,
  MAXM86161_LED_CURRENT_98_23MA         = 0xCA,
  MAXM86161_LED_CURRENT_98_71MA         = 0xCB,
  MAXM86161_LED_CURRENT_99_20MA         = 0xCC,
  MAXM86161_LED_CURRENT_99_69MA         = 0xCD,
  MAXM86161_LED_CURRENT_100_2MA         = 0xCE,
  MAXM86161_LED_CURRENT_100_7MA         = 0xCF,
  MAXM86161_LED_CURRENT_101_1MA         = 0xD0,
  MAXM86161_LED_CURRENT_101_6MA         = 0xD1,                                                                 
  MAXM86161_LED_CURRENT_102_1MA         = 0xD2,
  MAXM86161_LED_CURRENT_102_6MA         = 0xD3,
  MAXM86161_LED_CURRENT_103_1MA         = 0xD4,
  MAXM86161_LED_CURRENT_103_6MA         = 0xD5,
  MAXM86161_LED_CURRENT_104_1MA         = 0xD6,
  MAXM86161_LED_CURRENT_104_5MA         = 0xD7,
  MAXM86161_LED_CURRENT_105_0MA         = 0xD8,
  MAXM86161_LED_CURRENT_105_5MA         = 0xD9,
  MAXM86161_LED_CURRENT_106_0MA         = 0xDA,
  MAXM86161_LED_CURRENT_106_5MA         = 0xDB,
  MAXM86161_LED_CURRENT_107_0MA         = 0xDC,
  MAXM86161_LED_CURRENT_107_5MA         = 0xDD,
  MAXM86161_LED_CURRENT_108_0MA         = 0xDE,
  MAXM86161_LED_CURRENT_108_4MA         = 0xDF,
  MAXM86161_LED_CURRENT_108_9MA         = 0xE0,
  MAXM86161_LED_CURRENT_109_4MA         = 0xE1,
  MAXM86161_LED_CURRENT_109_9MA         = 0xE2,
  MAXM86161_LED_CURRENT_110_4MA         = 0xE3,
  MAXM86161_LED_CURRENT_110_9MA         = 0xE4,
  MAXM86161_LED_CURRENT_111_4MA         = 0xE5,
  MAXM86161_LED_CURRENT_111_8MA         = 0xE6,
  MAXM86161_LED_CURRENT_112_3MA         = 0xE7,
  MAXM86161_LED_CURRENT_112_8MA         = 0xE8,
  MAXM86161_LED_CURRENT_113_3MA         = 0xE9,
  MAXM86161_LED_CURRENT_113_8MA         = 0xEA,
  MAXM86161_LED_CURRENT_114_3MA         = 0xEB,
  MAXM86161_LED_CURRENT_114_8MA         = 0xEC,
  MAXM86161_LED_CURRENT_115_2MA         = 0xED,
  MAXM86161_LED_CURRENT_115_7MA         = 0xEE,
  MAXM86161_LED_CURRENT_116_2MA         = 0xEF,
  MAXM86161_LED_CURRENT_116_78MA        = 0xF0,
  MAXM86161_LED_CURRENT_117_2MA         = 0xF1,
  MAXM86161_LED_CURRENT_117_7MA         = 0xF2,
  MAXM86161_LED_CURRENT_118_2MA         = 0xF3,
  MAXM86161_LED_CURRENT_118_7MA         = 0xF4,
  MAXM86161_LED_CURRENT_119_1MA         = 0xF5,
  MAXM86161_LED_CURRENT_119_6MA         = 0xF6,
  MAXM86161_LED_CURRENT_120_1MA         = 0xF7,
  MAXM86161_LED_CURRENT_120_6MA         = 0xF8,
  MAXM86161_LED_CURRENT_121_1MA         = 0xF9,
  MAXM86161_LED_CURRENT_121_6MA         = 0xFA,
  MAXM86161_LED_CURRENT_122_1MA         = 0xFB,
  MAXM86161_LED_CURRENT_122_5MA         = 0xFC,
  MAXM86161_LED_CURRENT_123_0MA         = 0xFD,
  MAXM86161_LED_CURRENT_123_5MA         = 0xFE,
  MAXM86161_LED_CURRENT_124MA           = 0xFF    
} LEDCurrent;


class MAXM86161
{
  public:
    /* MAXM86161_Init:
          Connect to the MAXM86161 via i2c (Wire Library) and initialize it.
          Configures the MAXM86161 to sample all three LEDs plus Ambient at 25Hz.
          MAXM86161 will remain in an idle (shutdown) state after initialization.
          Returns true if successful, false if an error occured.*/
    bool MAXM86161_Init();

    MAXM86161();

    /*pulseoxymeter_t update():
     *    Reads in the FIFO data. 
     *    Applies an Median and Mean to raw light reading values
     *    Apply DC Filters to the Light reading values
     *    Balance light intensities
     *    Apply Low-pass Filter
     *    Run adjusting frequencies sampling rate
     *    FFT filtering
     *    Calulate R-value
     *    Calulate pulse and SpO2
          Returns pulseoxymeter result*/
    pulseoximeter_t update();

//    void setLEDCurrents( byte redLedCurrent, byte IRLedCurrent );
    
    /* MAXM86161_StartSampling:
          Commands the MAXM86161 to begin collecting and buffering data.
          Returns true if successful, false if an error occured.*/
    bool MAXM86161_StartSampling();
    
    /* MAXM86161_StopSampling:
          Commands the MAXM86161 to stop collecting and buffering data.
          Returns true if successful, false if an error occured.*/
    bool MAXM86161_StopSampling();
    
    /* MAXM86161_EnableInterrupt:
          Enables the MAXM86161's INTB pin to trigger (falling edge) based on the FIFO threshold level.
          Can be used by application level code to trigger retrieval of data samples (instead of polling).
          Returns true if successful, false if an error occured.*/
    bool MAXM86161_EnableInterrupt();
    
    /* MAXM86161_DisableInterrupt:
          Disables the MAXM86161's INTB pin.
          Returns true if successful, false if an error occured.*/
    bool MAXM86161_DisableInterrupt();
    
    /* MAXM86161_TryReadSamples:
          Attempts to retrieve up to 'Count' complete sets of data samples from the MAXM86161's FIFO.
          'Data' must be an array of MAXM86161SampleData structs of at least 'Count' length.
          The number of data sets retrieved is returned.
          Note: Partial data sets can be returned in some cases - i.e. if the FIFO overflows.  Check validity flags.
          Be sure to call this function frequently enough to avoid overflow.  The FIFO can hold 32 complete sets of data.*/
    uint8_t MAXM86161_TryReadSamples( MAXM86161SampleData * Data, uint8_t Count );

    bool MAXM86161_TryReadSample( MAXM86161SampleData & Data );


  private:  
    bool WriteReg3( RegAddr RegAddress, const uint8_t * Data, uint8_t Length );
    bool WriteReg2( RegAddr RegAddress, uint8_t Data );
    bool ReadReg3( RegAddr RegAddress, uint8_t * Data, uint8_t Length );
    bool ReadReg2( RegAddr RegAddress, uint8_t & Data );
    uint8_t readRegister(RegAddr RegAddress);
    void balanceIntesities( float redLedDC, float IRLedDC );

    uint8_t redLEDCurrent;
    uint8_t IRLEDCurrent;
    float lastREDLedCurrentCheck;
};

#endif
