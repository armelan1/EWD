 /*
This library is for the  MAXM86161 Pulse Oximeter

Creation Date: 02/10/2021

Revision History
================
2020/June/18  Barebone working code, non-library written by Yuchong Li
2021/Feb/10   Changing code to be a library MAXM86161 to work on ESP32 Featherwing for ePoD by Kevin Nichols
2021/March/18 LED light intensity adjustment
2021/March/25 Finger dection enable
2021/April/19 Updates to yield clean signals
2021/June/07  Filters and formulation for SpO2/HRT switched to "pulseox" library; written by Yuchong Li
2021/June/16  Enable temperature readings
2021/June/27  Added a input validation filter to remover incoming zeros

*/
/* Attached libraries
------------------------------- */
#include "MAXM86161.h"
#include <RunningMedian.h>
#include "pulseox_alg.h"

/* Control Serial Print Variables
------------------------------- */
bool debugIntensity = false;       // Controls the display of Light Intensities reeading results
bool show_result = false;

/* Control Plot Variables
------------------------------- */
bool plot_IR = true;
bool plot_red = false;
bool plot_green = false;

bool show_Heart_Beat_Plot = false; // shows the when the SpO2 algorrithm detects a heart beat and the SpO2 filter IR signal
bool show_Raw_Plot = false;
bool show_Avg_Raw_Plot = false;
bool show_DC_Filter_Plot = false;
bool show_final_plot = false;

bool plot_PPG = false;

/* Stored Variables
------------------------------- */
double stored_ir_BPM;
double stored_ir_amp;
uint32_t stored_Beat_time;
double stored_red_SpO2;
double stored_red_ratioRMS;
uint32_t stored_red_time;
double stored_red_amp;
double stored_green_SpO2;
double stored_green_ratioRMS;
uint32_t stored_green_time;
double stored_green_amp;
double stored_temperature;
int stored_fps_Count;
int raw_IRLED;
int raw_RedLED;
int raw_GreenLED;
int raw_Ambient;

int raw_ir_prev, raw_red_prev, raw_green_prev;

/* Valid Time Lenghts
------------------------------- */
uint32_t last_BPM_Time = 0;
uint32_t last_red_SpO2_Time = 0;
uint32_t last_green_SpO2_Time = 0;

/* Runnning Median Variables
------------------------------- */
RunningMedian update_fps_averaging_filter = RunningMedian(101);
RunningMedian raw_IR_averaging_filter = RunningMedian(7);
RunningMedian raw_Red_averaging_filter = RunningMedian(7);
RunningMedian raw_Green_averaging_filter = RunningMedian(7);

/* Balance Intesities Variables
------------------------------- */
int current_Balance_Count;
int skipping_Balance_Count = 20;
int max_Balance_Count = 50;
bool Calibrated;

/* =========================================================================
*  pulseoxymeter_t MAXM86161::update():
   *    Reset result values before updating them  
   *    Reads in the FIFO data. 
   *    Applies an FIR moving average filter to raw light reading values
   *    Apply DC Filters to the Light reading values
   *    Balance light intensities
   *    Apply Low-pass Filter
   *    Run adjusting frequencies sampling rate
   *    FFT filtering
   *    Calulate R-value
   *    Calulate pulse and SpO2
        Returns pulseoxymeter result*/
#define MEASURE_FPS
pulseoximeter_t MAXM86161::update()
{
#ifdef MEASURE_FPS
  /* measure sample rate */
  static int prev_millis = 0;
  static int update_count = 0;
  if(millis() - prev_millis > 1000)
  {
//    Serial.print("update fps:");
//    Serial.println(update_count);
    update_fps_averaging_filter.add((int)update_count);
    int update_count_avg = update_fps_averaging_filter.getAverage();
    stored_fps_Count = update_count_avg;
    prev_millis = millis();
    update_count = 0;
  }
#endif

  static bool first_run = true;
  if(first_run)
  {
    first_run = false;
    Serial.println("*** pulseox_alg: initialize"); 
    pulseox_init();
  }

  /* Reset pulseoximeter result values*/
  pulseoximeter_t result = {
    /*float ir_BPM*/ stored_ir_BPM,       //Calculated BPM
    /*float ir_PPG*/ 0.0,       //Cleanup IR value for PPG graphing
    /*float red_PPG*/ 0.0,      //Cleanup Red value for PPG graphing
    /*float green_PPG*/ 0.0,    //Cleanup Green value for PPG graphing
    /*float red_SpO2*/ stored_red_SpO2,              //Calculated SpO2 percentege
    /*float ratioRMSred*/ stored_red_ratioRMS,      //RMS ratio value - sent out for data collection
    /*float greenSpO2*/ stored_green_SpO2,
    /*float ratioRMSgreen*/ stored_green_ratioRMS,
    /*float green_BPM*/ 0.0,
    /*float red_BPM*/ 0.0,
    /*float IR_amp*/  stored_ir_amp,
    /*float red_amp*/ stored_red_amp,
    /*float green_amp*/ stored_green_amp,
    /*float beat_time*/ stored_Beat_time,
    /*float SpO2_red_time*/ stored_red_time,
    /*float SpO2_green_time*/ stored_green_time,
    /*float temperature*/ stored_temperature,
    /*bool fingerPresent*/ false,
    /*bool bpm_Updated*/ false,
    /*bool red_SpO2_Updated*/ false,
    /*bool green_SpO2_Updated*/ false,
    /*int avg_fps*/ stored_fps_Count
  };

  result.avg_fps = stored_fps_Count;

  MAXM86161SampleData data;
  /*read max86161 sample*/

  MAXM86161SampleData buffer[32];
  uint8_t count = MAXM86161::MAXM86161_TryReadSamples(buffer,32);
  //Serial.print("FIFO count; ");Serial.println(count);
  for(int i = 0; i < count; i++){
    data = buffer[i];
  
//  if(MAXM86161::MAXM86161_TryReadSample(data) == true){
    raw_Ambient=data.Ambient;

    /*input validation filter to remover incoming zeros*/
    if(data.IRLED != 0) raw_IRLED = data.IRLED;
    if(data.RedLED != 0) raw_RedLED = data.RedLED;
    if(data.GreenLED != 0) raw_GreenLED = data.GreenLED;
    
    // Variables for pulseox_update
    float ir_value = raw_IRLED;
    float green_value = raw_GreenLED;
    float red_value = raw_RedLED;
    int bpm_valid = 0;
    float bpm, spo2_r, R_r, red_LED_DC, IR_LED_DC;
    int spo2_valid_r = 0;
    float spo2_g, R_g;
    int spo2_valid_g = 0;

    psx_dsp_t psx_dsp;
    psx_dsp.amp_valid = 0;

    pulseox_update(
        red_value, green_value, ir_value, stored_fps_Count, 
        &spo2_r, &R_r, &spo2_valid_r, 
        &spo2_g, &R_g, &spo2_valid_g,
        &bpm, &bpm_valid,
        &red_LED_DC, &IR_LED_DC, 
        &psx_dsp);

    if (Calibrated == false){
      balanceIntesities(red_LED_DC,IR_LED_DC);
    }

//    static int beatUpdateCount = 0;
//    if(psx_dsp.beat_detected) beatUpdateCount++;
//    Serial.println(beatUpdateCount);
    if (show_Heart_Beat_Plot){
      if(psx_dsp.beat_detected){
        Serial.print(200);
      }
      else{
        Serial.print (0);
      }
      Serial.print(",");
      Serial.println(psx_dsp.filterSpO2IRvalueOut);
    }

    if(bpm_valid)
    {
      stored_ir_BPM = bpm;
      stored_ir_amp=psx_dsp.ir_amp;
      result.bpm_Updated = true;
      uint32_t BPM_Diff = millis()- last_BPM_Time;
      stored_Beat_time = BPM_Diff;
      result.beat_time = stored_Beat_time;
      last_BPM_Time = millis();
    }

    if(spo2_valid_r)
    {
      if(spo2_r >0){
        stored_red_SpO2=spo2_r;
        result.red_SpO2 = stored_red_SpO2;
      }
      stored_red_ratioRMS=R_r;
      stored_red_amp=psx_dsp.red_amp;
      uint32_t SpO2_Diff = millis()- last_red_SpO2_Time;
      stored_red_time = SpO2_Diff;
      result.SpO2_red_time = stored_red_time;
      last_red_SpO2_Time = millis();
      result.red_SpO2_Updated = true;
    }

    result.ir_PPG=psx_dsp.ir_ac;
    result.red_PPG=psx_dsp.red_ac;
    result.green_PPG=psx_dsp.green_ac;
    
    result.IR_amp=stored_ir_amp;
    result.red_amp=stored_red_amp;
    result.green_amp=stored_green_amp;

    #ifdef MEASURE_FPS
      update_count++;
    #endif
  } // end of for loop - FIFO buffer reading

  raw_IR_averaging_filter.add(raw_IRLED); 
  float average_Raw_IR = raw_IR_averaging_filter.getAverage();
  
  raw_Red_averaging_filter.add(raw_RedLED);
  float average_Raw_Red = raw_Red_averaging_filter.getAverage();

  raw_Green_averaging_filter.add(raw_GreenLED);
  float average_Raw_Green = raw_Green_averaging_filter.getAverage();
//Serial.print("IR raw: ");
//Serial.println(average_Raw_IR);
//Serial.print("Red raw: ");
//Serial.println(average_Raw_Red);
  /* Check to that tissue is present --------------------------------------- */
  if(average_Raw_IR < 150000 && average_Raw_Red < 80000){
    result.fingerPresent=false;
    MAXM86161();        //Reset variables
    return result;
  }
  else{
    result.fingerPresent=true;
  }

  WriteReg2(RegAddr::DieTemperatureConfiguration, 0b00000001); // Enable one temperature measurement
  int8_t temp = (int8_t)readRegister(RegAddr::DieTemperatureInteger);
  unsigned long tempFractionInt = readRegister(RegAddr::DieTemperatureFraction);
  float tempFraction=(float)tempFractionInt*0.0625;
  stored_temperature = (float)temp + tempFraction;
  result.temperature = stored_temperature;
  WriteReg2(RegAddr::DieTemperatureConfiguration, 0b00000000); //Reset temperature

  result.ir_BPM = stored_ir_BPM;
  
  result.ratioRMSred = stored_red_ratioRMS;
  result.green_SpO2 = stored_green_SpO2;
  result.ratioRMSgreen = stored_green_ratioRMS;
  result.red_BPM = 0.0;
//  result.green_BPM = 0.0;

  if(show_result==true){
    Serial.println("******Result VALUES*****");
    Serial.print("----BPM Final:");
    Serial.println(stored_ir_BPM,0);
    Serial.print("----SpO2(red):");
    Serial.println(stored_red_SpO2);
    Serial.print("----SpO2(green):");
    Serial.println(stored_green_SpO2);
  }

  ////////////////////////////////////////////////////////
  if(plot_PPG == true){
    if(show_Raw_Plot==true){
      if(plot_IR == true){
        Serial.print(raw_IRLED);
      }
      if(plot_red == true && plot_IR == true){
        Serial.print(",");
      }
      if(plot_red == true){
        Serial.print(raw_RedLED);
      }
      if((plot_IR == true || plot_red == true) && (plot_green == true)){
        Serial.print(",");
      }
      if(plot_green == true){
        Serial.print(raw_GreenLED);
      }
      if(show_Avg_Raw_Plot == true||show_DC_Filter_Plot== true||show_final_plot == true){
        Serial.print(",");
      }
      else{
      Serial.println(" ");
      }
    } // end of raw plot controls
    
    //************************************
    if(show_Avg_Raw_Plot == true){
      if(plot_IR == true){
        Serial.print(average_Raw_IR);
      }
      if(plot_red == true && plot_IR == true){
        Serial.print(",");
      }
      if(plot_red == true){
        Serial.print(average_Raw_Red);
      }
      if((plot_IR == true || plot_red == true) && (plot_green == true)){
        Serial.print(",");
      }
      if(plot_green == true){
        Serial.print(average_Raw_Green);
      }
      if(show_Raw_Plot == true||show_DC_Filter_Plot== true||show_final_plot == true){
        Serial.print(",");
      }
      else{
        Serial.println(" ");
      }
    } // end of raw averaging plot controls
    
    //************************************
    if(show_final_plot==true){
      if(plot_IR == true){
        Serial.print(result.ir_PPG);
      }
      if(plot_IR == true && plot_red == true){
        Serial.print(",");
      }
      if(plot_red == true){
        Serial.print(result.red_PPG);
      }
      if((plot_IR == true || plot_red == true) && (plot_green == true)){
        Serial.print(",");
      }
      if(plot_green == true){
        Serial.print(result.green_PPG);
      }
      if(show_Raw_Plot == true||show_DC_Filter_Plot== true||show_Avg_Raw_Plot == true){
        Serial.print(",");
      }
      else{
        Serial.print(", ");
      }
    } // end of show final plot controls
  }
//  Serial.println(result.ir_PPG*1000);

  return result;
}


/* =========================================================================
   MAXM86161():
   *  Put MAXM86161 varibles in a initialize state.*/
MAXM86161::MAXM86161(){
  raw_IR_averaging_filter.clear();
  raw_Red_averaging_filter.clear();
  raw_Green_averaging_filter.clear();

  stored_ir_BPM = 0.0;
  stored_ir_amp = 0.0;
  stored_Beat_time = 0.0;
  stored_red_SpO2 = 0.0;
  stored_red_ratioRMS = 0.0;
  stored_red_time = 0.0;
  stored_red_amp = 0.0;
  stored_green_SpO2 = 0.0;
  stored_green_ratioRMS = 0.0;
  stored_green_time = 0.0;
  stored_green_amp = 0.0;

  if(show_Raw_Plot == true){
    plot_PPG = true;
    show_result = false;
  }
  delay(2);
  if(show_Avg_Raw_Plot == true){
    plot_PPG = true;
    show_result = false;
  }
  delay(2);
  if(show_DC_Filter_Plot == true){
    plot_PPG = true;
    show_result = false;
  }
  delay(2);
  if(show_final_plot == true){
    plot_PPG = true;
    show_result = false;
  }
  delay(2);

}


/* =========================================================================
*  MAXM86161_Init:
      Connect to the MAXM86161 via i2c (Wire Library) and initialize it.
      Configures the MAXM86161 to sample all three LEDs plus Ambient at 25Hz.
      MAXM86161 will remain in an idle (shutdown) state after initialization.
      Returns true if successful, false if an error occured.*/
bool MAXM86161::MAXM86161_Init(){ 
  uint8_t Value = 0;
  if ( !MAXM86161::ReadReg2(RegAddr::PartID,Value) || (Value != PartIDValue ) )
      return false;

  // Soft Reset
  if ( !MAXM86161::WriteReg2(RegAddr::SystemControl, 0b00000001) )
      return false;
  // Wait 1ms
  delay(1);
  // Shutdown
  if ( !MAXM86161::WriteReg2(RegAddr::SystemControl, 0b00000010) )
      return false;

  SamplingEnabled = false;

  // Clear Interrupts
  MAXM86161::ReadReg2(RegAddr::InterruptStatus1, Value);
  MAXM86161::ReadReg2(RegAddr::InterruptStatus2, Value);

  // Set ADC Range to 16384nA and Pulse Width to 123.8us
//  if ( !MAXM86161::WriteReg2(RegAddr::PPGConfiguration1, 0b10001111) )
  if ( !MAXM86161::WriteReg2(RegAddr::PPGConfiguration1, (ALC_DISABLE<<7)|(ADD_OFFSET<<6)|(PPG1_ADC_RGE<<2)|PPG_TINT) )
      return false;
  
      
  // Set Sample Averaging
  if ( !MAXM86161::WriteReg2(RegAddr::PPGConfiguration2, ( SAMPLING_RATE<<3) | SAMPLING_AVERAGE) )
      return false;
  
  // Set LED Settling Time to 12us (Disable Burst Mode, Filter = CDM)
//  if ( !MAXM86161::WriteReg2(RegAddr::PPGConfiguration3, 0b11000000) )
  if ( !MAXM86161::WriteReg2(RegAddr::PPGConfiguration3, (LED_SETLNG<<6)|(DIG_FILT_SEL<<5)|(BURST_RATE<<1)|BURST_EN) )
      return false;
    
  // Set Photo Diode Bias to 0~65pF
  if ( !MAXM86161::WriteReg2(RegAddr::PhotoDiodeBias, 0b00000001) )
      return false;
  
  // Set LED Current Range to 124mA
  if ( !MAXM86161::WriteReg2(RegAddr::LEDRange1, 0b00111111) )
      return false;
   
  // Set LED1 (Green) Drive Current to 15.36mA
  if ( !MAXM86161::WriteReg2(RegAddr::LED1PA, STARTING_GREEN_LED_CURRENT) )
      return false;
  
  // Set LED2 (IR) Drive Current to 15.36mA
  if ( !MAXM86161::WriteReg2(RegAddr::LED2PA, STARTING_IR_LED_CURRENT) )
      return false;
  
  // Set LED3 (Red) Drive Current to 15.36mA
  if ( !MAXM86161::WriteReg2(RegAddr::LED3PA, STARTING_RED_LED_CURRENT) )
      return false;
    
  // Disable Low Power Mode (Still Shutdown)
  if ( !MAXM86161::WriteReg2(RegAddr::SystemControl, 0b00000010) )
      return false;

  // Set FIFO Full Threshold to 16 (128-112)
  if ( !MAXM86161::WriteReg2(RegAddr::FIFOConfiguration1, 112) )
      return false;
      
  // Set FIFO Rollover and Status Clear
  if ( !MAXM86161::WriteReg2(RegAddr::FIFOConfiguration2, 0b00001010) )
      return false;

  // Setup LED Sequence (1 -> 2 -> 3 -> A)
  if ( !MAXM86161::WriteReg2(RegAddr::LEDSequenceRegister1, 0b00100001) )
      return false;
      
  if ( !MAXM86161::WriteReg2(RegAddr::LEDSequenceRegister2, 0b10010011) )
      return false;
      
  if ( !MAXM86161::WriteReg2(RegAddr::LEDSequenceRegister3, 0) )
      return false;

  if ( !MAXM86161::WriteReg2(RegAddr::DieTemperatureConfiguration, 0b00000001) )
      return false;
  // setup PPG Picket Fence Dectection and Replace
  if ( !MAXM86161::WriteReg2(RegAddr::PicketFence, (PF_ENABLE<<6)|(PF_ORDER<<5)|(IIR_TC<<4)|(IIR_INIT_VALUE<<2)|THRESHOLD_SIGMA_MULT) )
      return false;

  redLEDCurrent = (uint8_t)STARTING_RED_LED_CURRENT;
  lastREDLedCurrentCheck = 0;
  current_Balance_Count = 0;
  Calibrated = false;
  
  Initialized = true;
    return true;
}




/* =========================================================================
*  MAXM86161_StartSampling:
      Commands the MAXM86161 to begin collecting and buffering data.
      Returns true if successful, false if an error occured. */
bool MAXM86161::MAXM86161_StartSampling(){
    if ( !Initialized )
        return false;
    
    // Begin Sampling (Shutdown = 0)
    if ( !MAXM86161::WriteReg2(RegAddr::SystemControl, 0b00000100) )
        return false;
    
    SamplingEnabled = true;
    return true;
}



/* =========================================================================
*  MAXM86161_StopSampling:
      Commands the MAXM86161 to stop collecting and buffering data.
      Returns true if successful, false if an error occured. */
bool MAXM86161::MAXM86161_StopSampling(){
    if ( !Initialized )
        return false;

    // Shutdown
    if ( !MAXM86161::WriteReg2(RegAddr::SystemControl, 0b00000010) )
        return false;
        
    SamplingEnabled = false;
    return true;
}



/* =========================================================================
*  MAXM86161_EnableInterrupt:
      Enables the MAXM86161's INTB pin to trigger (falling edge) based on the FIFO threshold level.
      Can be used by application level code to trigger retrieval of data samples (instead of polling).
      Returns true if successful, false if an error occured.*/
bool MAXM86161::MAXM86161_EnableInterrupt(){
    // Enable FIFO Threshold Interrupt
    if ( !MAXM86161::WriteReg2(RegAddr::InterruptEnable1, 0b10000000) )
        return false;
    
    return true;
}



/* =========================================================================
*  MAXM86161_DisableInterrupt:
      Disables the MAXM86161's INTB pin.
      Returns true if successful, false if an error occured.*/
bool MAXM86161::MAXM86161_DisableInterrupt(){
    // Disable Interrupts
    if ( !MAXM86161::WriteReg2(RegAddr::InterruptEnable1, 0b00000000) )
        return false;
    
    return true;
}



/* =========================================================================
*  MAXM86161_TryReadSample:
      Attempts to retrieve a single complete set of data samples from the MAXM86161's FIFO.
      Returns true if a data set could be retrieved.
      Note: Partial data sets can be returned in some cases - i.e. if the FIFO overflows.  Check validity flags.
      Be sure to call this function frequently enough to avoid overflow.  The FIFO can
      hold 32 complete sets of data.*/
uint8_t MAXM86161::MAXM86161_TryReadSamples( MAXM86161SampleData * Data, uint8_t Count )
{
    if ( !Initialized || !SamplingEnabled )
        return 0;
    
    uint8_t FIFOCount = 0;
    if ( !MAXM86161::ReadReg2(RegAddr::FIFODataCounter,FIFOCount) )
        return 0;
    
    uint8_t SampleCount = FIFOCount / LEDSeqCount;
    if ( Count > SampleCount )
        Count = SampleCount;
    
    uint8_t FIFOIndex = 0;
    uint8_t Index = 0;
    while ( Index<Count )
    {
        Data[Index].ValidData = 0;
        bool SampleLoop = true;

        while(SampleLoop)
        {
            uint8_t FIFOData[3];
            if ( !MAXM86161::ReadReg3(RegAddr::FIFODataRegister,FIFOData,3) )
                return 0;

            uint8_t Tag = ((FIFOData[0] & 0b11111000) >> 3);
            uint32_t Sample = ((FIFOData[0] << 16) | (FIFOData[1] << 8) | (FIFOData[2])) & 0x7FFFF;
            switch((FIFOTag)Tag)
            {
                case FIFOTag::PPG1_LEDC1_DATA:
                    Data[Index].GreenLED = Sample;
                    Data[Index].ValidData |= 1;
                    SampleLoop = (LEDSeqCount != 1);
                    break;
               case FIFOTag::PPG1_LEDC2_DATA:
                    Data[Index].IRLED = Sample;
                    Data[Index].ValidData |= 2;
                    SampleLoop = (LEDSeqCount != 2);
                    break;
                case FIFOTag::PPG1_LEDC3_DATA:
                    Data[Index].RedLED = Sample;
                    Data[Index].ValidData |= 4;
                    SampleLoop = (LEDSeqCount != 3);
                    break;
                case FIFOTag::PPG1_LEDC4_DATA:
                    Data[Index].Ambient = Sample;
                    Data[Index].ValidData |= 8;
                    SampleLoop = (LEDSeqCount != 4);
                    break;
                default:
                    //Serial.printf("Unexpected Data Tag in FIFO: %u\r\n",Tag);
                    break;
            }

            FIFOIndex++;
            if ( FIFOIndex >= FIFOCount )
                break;
        }

        Index++;
        if ( (FIFOCount - FIFOIndex) < LEDSeqCount )
            break;
    }

    return Index;
}



/*  =========================================================================*/
bool MAXM86161::MAXM86161_TryReadSample( MAXM86161SampleData & Data )
{
    return MAXM86161_TryReadSamples(&Data,1);
}



/*  =========================================================================*/
bool MAXM86161::WriteReg3( RegAddr RegAddress, const uint8_t * Data, uint8_t Length ){
    Wire.beginTransmission(I2C_Address);
    Wire.write((uint8_t)RegAddress);
    Wire.write(Data,Length);
    return (Wire.endTransmission() == I2C_ERROR_OK);
}



/*  =========================================================================*/
bool MAXM86161::WriteReg2( RegAddr RegAddress, uint8_t Data )
{
    return MAXM86161::WriteReg3(RegAddress,&Data,1);
}



/*  =========================================================================*/
bool MAXM86161::ReadReg3( RegAddr RegAddress, uint8_t * Data, uint8_t Length )
{
    Wire.beginTransmission(I2C_Address);
    Wire.write((uint8_t)RegAddress);
    Wire.endTransmission(false);
    Wire.beginTransmission(I2C_Address);
    Wire.requestFrom(I2C_Address,Length);
    Wire.readBytes(Data,Length);
    return (Wire.endTransmission() == I2C_ERROR_OK);
}



/*  =========================================================================*/
bool MAXM86161::ReadReg2( RegAddr RegAddress, uint8_t & Data )
{
    return MAXM86161::ReadReg3(RegAddress,&Data,1);
}



/*  =========================================================================*/
uint8_t MAXM86161::readRegister(RegAddr RegAddress)
{
  Wire.beginTransmission(I2C_Address);
  Wire.write((uint8_t)RegAddress);
  Wire.endTransmission(false);
  Wire.requestFrom(I2C_Address, 1);

  return Wire.read();
}



/*  =========================================================================*/
void MAXM86161::balanceIntesities( float redLedDC, float IRLedDC ){
  if( millis() - lastREDLedCurrentCheck >= RED_LED_CURRENT_ADJUSTMENT_MS){
    if(current_Balance_Count <= skipping_Balance_Count){
      current_Balance_Count++;
      if(debugIntensity == true){
        Serial.print("Count: ");
        Serial.println(current_Balance_Count);
      }
    }
    else {
      current_Balance_Count++;
      if(debugIntensity == true){
        Serial.println("=========================");
        Serial.println("Done skipping");
        Serial.print("abs(IRdc - RedDC): ");
        Serial.println( abs(IRLedDC - redLedDC) );
        Serial.print("Red LED DC-filtered input: ");
        Serial.println(redLedDC);
        Serial.print("IR LED DC-filtered input: ");
        Serial.println(IRLedDC);
//        Serial.println("=========================");
      }

      if( abs(IRLedDC - redLedDC) < MIN_ACCEPTABLE_INTENSITY_DIFF_RED && redLEDCurrent < MAXM86161_LED_CURRENT_124MA ) 
      {
        current_Balance_Count--;
        redLEDCurrent--;
        MAXM86161::WriteReg2(RegAddr::LED3PA, redLEDCurrent);
//        setLEDCurrents( redLEDCurrent, IrLedCurrent );
                
        if(debugIntensity == true){ 
          Serial.println("Calibrating");
          Serial.println("RED LED Current - ");
          Serial.println(redLEDCurrent);
          Serial.println(IRLEDCurrent);
        } 
      }
      else if(abs(IRLedDC - redLedDC) > MAX_ACCEPTABLE_INTENSITY_DIFF_RED && redLEDCurrent > 0 ) 
      {
        current_Balance_Count--;
        redLEDCurrent++;
        MAXM86161::WriteReg2(RegAddr::LED3PA, redLEDCurrent);
//        setLEDCurrents( redLEDCurrent, IRLEDCurrent );
        if(debugIntensity == true){ 
          Serial.println("Calibrating");
          Serial.println("RED LED Current +");
          Serial.println(redLEDCurrent);
          Serial.println(IRLEDCurrent);
        }
      }

    } // end of else skipping Count

    if(current_Balance_Count == max_Balance_Count){
      Calibrated = true;
    }
    
    if(debugIntensity == true && current_Balance_Count > skipping_Balance_Count){
        Serial.print("Count: ");
        Serial.println(current_Balance_Count);
        Serial.println("=========================");
      }
    lastREDLedCurrentCheck = millis();
  }
}
