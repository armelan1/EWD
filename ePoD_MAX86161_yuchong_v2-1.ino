/*
 * ePoD Phase 2 - Gamma 1
 * - MAXM8616 and bno055 sensors
 * Written by: Kevin Nichols
 * Initial Written on 04/05/2021
 * ==============================
 * (Update revision date on line 68 to record info correctly)
 * Revisions:
 *  2021/June/07   Yuchong implementented filters and formulation to work with MAXM86161 sensor
 *  2021/June/17   Adjusted displays to work with updated libraries
 *  2022/March/21  Yuchong adjusted filters for the heart rate and SpO2
 *  2022/May/25    Added Device name to display and SD file. Adjusting output filters.
 *  2022/June      Cleaned up Pulse Oximeter filters and changed the data stored/transmitted
 *  
 *  
 * Code for Bluetooth to IOS App via ESP32 Created By: Tyler Goudie (May 12,2020)
Notes:  The Bluetooth running on the ESP32 does not run within the main loop of code,
    it is executed by the BLE chip within the ESP32. All that is required is the
    correct setup of the Bluetooth.  You can set the vaules of Characterisitics
    anywhere by using "pCharacteristic"->setValue(), for example if I wanted to
    change the time characterisitic I would write: pTimeCharacteristic->setValue(). 
    Once this is set The ESP32 will automatically set the specified characterisitc
    to this value and notify the client device.

    IMPORTANT|| The setValue() Function only accepts Char arrays and INT8 so you must
    convert whatever value you wish to send to a char array||. The Preference for
    Setting values Currently is to use INT8.
    
    Example of seting a float to char Array and updating Characteristic Value
    float iVarToCast = 123.456;
    char buffer[7]; // buffer size must equal size of value to be sent -- 123.456 to
    [1],[2],[3],[.],[4],[6],[0]
    dtostrf(iVarToCast, 7, 0, buffer);  
    pCharacterisitc->setValue(buffer);
*/

//=====================
// support libraries
#include <Arduino.h>
#include <Wire.h>               // for I2C communication
#include "MAXM86161.h"          // for MAXM86161 Pulse Oximeter Sensor
#include "bno055.h"             // for bon055 Sensor
#include "SPI.h"                // for SPI communication (OLED Featherwing & SD Card)
#include <SdFat.h>              // for reading and writing to an SD Card
#include "OLED_SSD1306.h"       // OLED Featherwing
#include <Adafruit_GFX.h>       // OLED Featherwing
#include "RTC_lib.h"            // Real Time Clock library from Adafruit
#include <BLEDevice.h>          // for Bluetooth Communication
#include <BLEUtils.h>           // for Bluetooth Communication
#include <BLEServer.h>          // for Bluetooth Communication

#include <RunningMedian.h>

//====================
// General Variables
bool plotRedppg = false;          // toggles on/off Serial Print/Plot of the Red ppg signal
bool plotIRppg = false;           // toggles on/off Serial Print/Plot of the IR ppg signal
bool plotPPG = false;             // the code will toggle on if a red/IR/green ppg and stop other Serial prints

bool CalibrationPrint = false;    // the code will toggle on if a red/green calibration and stop other Serial prints
bool redCalibration = false;      // toggles on/off Serial Print to display info for calibrating Red

//**SerialTesting will get turn off if plotting or calibrating
bool SerialTesting = false;        // toggles on/off Serial Print for testing

//#define I2C_SDA 21                // SDA at pin 21 on ESP32 v2
//#define I2C_SCL 20                // SCL at pin 20 on ESP32 v2
#define I2C_SDA 23                // SDA at pin 23 on HUZZAH32 – ESP32
#define I2C_SCL 22                // SCL at pin 22 on HUZZAH32 – ESP32

String LastRevisionDate = "6-8-2022";   // variable to store last version to storage on SD card/BLE
String dataString;                // variable of the gathered data tostore on SDcard/BLE
const int redLED = 13;            // HUZZAH32 – ESP32 red LED pin

//=========================
// Pulse Oximeter Variables
MAXM86161* pulseOx;
double ppgIR;
double BPMred, SpO2red, ppgRed, rValueRed;
int BPM, SpO2;

uint32_t lastPulseUpdate = 0;
uint32_t pulse_Time_Span = 0;       // variable to measure time between valid BPM updates
uint32_t red_SpO2_Time_Span = 0;    // variable to measure time between valid red SpO2 updates
uint32_t lastRedSpO2Update = 0;
double rawTemperatureIn;
double adjTemperature;
double adjValue = 23.0;
bool heart, redSpO2, grnSpO2 =false;

//===================
// SD card Variables
SdFat sd;
const int cardSelect=33;        // SD card select at pin 33 on HUZZAH32 – ESP32
SdFile file;

// File base name.  Must be six characters or less.
#define FILE_BASE_NAME "ePod_"
const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME)-1;
char filename[15] = FILE_BASE_NAME "0000.txt";

uint32_t lastRecordTime = 0;    // variable to measure time between updates
int recordTimeDuration = 50;   // 100ms to record data at 10Hz

//====================
// bno055 Variables
//#define bno055_SAMPLERATE_DELAY_MS (100)
bno055* BNO;
float xGavg, yGavg, zGavg;

//==========================
// Real Time Clock Variables
RTC_PCF8523 rtc;
String hrs, mins, secs, yr, Month, Day;

//===================
// OLED Variables
OLED_SSD1306 display = OLED_SSD1306(128, 32, &Wire);
#define BUTTON_A 15   // pin 15 on HUZZAH32 – ESP32 to OLED button 'A'
#define BUTTON_B 32   // pin 32 on HUZZAH32 – ESP32 to OLED button 'B'
#define BUTTON_C 14   // pin 14 on HUZZAH32 – ESP32 to OLED button 'C'
int A_State;          // Screen display 'A' is for all info
int B_PulseDetail;    // Screen display 'B' is for detail pulse information
int C_PulseOnly;      // Screen display 'C' is for Pulse and SpO2 Only

//========================
// Bluetooth Variables
const long bleUpdateTimeSpan = 50; // 20ms to update BLE data at 1about 50Hz
unsigned long lastBLEupdate = 0;

bool deviceConnected = false;
bool oldDeviceConnected = false;
String btSDfileName;    //sharing the SD file name via bluetooth
String DAQid;
String DAQ_number;

// Universaly Unique Identifiers These are constant and DO NOT change.
#define GENERAL_SERVICE_UUID "b2f9b8ba-6459-4268-85e2-95d182f18329"
#define GENERAL_CODENAME_CHARACTERISITC_UUID "dd42b023-c2c0-4511-a861-1dd7dae3ac9d"
#define GENERAL_CODECOMPILEDATE_CHARACTERISTIC "ba2a327a-cf06-4555-85c5-e79058cda88c"
#define GENERAL_SDSAVE_CHARACTERISITC_UUID "43e75f6d-63d3-49d2-9f3a-0f951738a2a1" //was "GENERAL_CHARACTERISITC_UUID"
#define SENSOR_SERVICE_UUID  "55cefee6-22b2-4b4a-b26d-49da9d0d724c" //was "ACCELERATION_SERVICE_UUID"
#define SENSOR_CHARACTERISTIC_UUID "6bcbdecf-3cba-44c5-8501-791d986d7236" //was "AX_CHARACTERISTIC_UUID"


// Initilize Services and Characteristics
BLEService *pGeneralService;
BLECharacteristic *pGeneralCharacteristic;
BLECharacteristic *pGeneralCodeNameCharacteristic;
BLECharacteristic *pGeneralCodeCompileDateCharacteristic;
BLECharacteristic *pGeneralSdSaveCharacteristic;

BLEService *pSensorService;
BLECharacteristic *pSensorCharacteristic;

int uniqueId = ESP.getEfuseMac(); // unique identifier
char buffer[5];
std::string temp =  itoa(uniqueId,buffer,10);
std::string deviceName = "ePoD" + temp ;// Creates a Unique Device Name Using the devices Chip ID

// Class for Server Callbacks Finds out if the server has been disconnected from.
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

// Class that recieves a callback if client side BLE updates any characteristic value
class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    BLEUUID CharacteristicUUID = pCharacteristic->getUUID();
    std::string value = pCharacteristic->getValue();
    Serial.println("==================================");
    Serial.println(value.c_str());
        
  }
};


//=====================================================
//=====================================================
void dateTime(uint16_t* date, uint16_t* time) {
  DateTime now = rtc.now();
  
  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(now.year(),now.month(),now.day());
  
  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(now.hour(), now.minute(), now.second());
  }

//=====================================================
//=====================================================
void error(int code)
{
  //do nothing
}
//=====================================================
//=====================================================

void setup() {
  // **Get code information
  String CodeName=(__FILE__);
  String CompliedDate=(__DATE__);
  String CompliedTime=(__TIME__);
  String simpleCodeName=(CodeName.substring((CodeName.indexOf(".")), (CodeName.lastIndexOf("\\")) + 1));
  
  Wire.begin();
//  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.begin(115200);
  delay(2000);      // delay to ensure serial port connection.

  pinMode(redLED, OUTPUT);

  // Defining pins for buttons users interface
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
  
  // Set the starting screen mode before user interface
  A_State=0;
  B_PulseDetail=0;
  C_PulseOnly=1;

  // **Initializing OLED display
  Serial.println("Initializing OLED Featherwing");
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  display.display();
  delay(2000);    // Delay to show introduction image buffer
  
  // **Initializing SD and see if there is an SD card present
  display.display();
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.print("Creating New SD File. Processing");
  display.display(); // actually display all of the above

  // Automatical check/balance Serial print/plot
  if((plotRedppg==true) || (plotIRppg==true)){
    plotPPG = true;
    SerialTesting = false;
    Serial.println("Change plotPPG variable to TRUE");
  }
  
  // Automatical check/balance Calibration Printing
  if(redCalibration==true){
    CalibrationPrint = true;
    SerialTesting = false;
    Serial.println("Change CalibrationPrint variable to TRUE");
  }

  if(plotPPG==true && CalibrationPrint==true){
    Serial.println("Serial plot and print conflict");
    delay(2000);
    error(7);
  }

  // **Check and initialize the real time clock
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    error(1);
  }
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  
  if(SerialTesting == true){
    delay(2000); // delay to ensure serial port connection.
    Serial.print("Code File Name: ");
    Serial.println(simpleCodeName);
    Serial.print("Complided on: ");
    Serial.println(CompliedDate);
    Serial.print("at this time: ");
    Serial.println(CompliedTime);
  }
  
  Serial.println("Initializing SD card..."); 
  if (!sd.begin(cardSelect, SD_SCK_MHZ(4))) {
    error(2);
  }
  // Find an unused sd file name.
  if (BASE_NAME_SIZE > 6) {
    Serial.println("FILE_BASE_NAME too long");
    error(2);
  }
  Serial.print("Creating File Name, please wait ");
  int i = 0;
  int j = 0;
  bool jump = true; // "jump" by 100s
  while (sd.exists(filename)) {
    if(j == 0 && jump == true){
      // This while loop is to minimums new file search time
      while (sd.exists(filename)){
        filename[BASE_NAME_SIZE + 1] = '0' + (j/100)%10;
        filename[BASE_NAME_SIZE + 2] = '9';
        filename[BASE_NAME_SIZE +3] = '9';
        j=j+100;
      }
    }
    if(jump==true){
      j=j-100;
      i=i+j;
      jump=false;
    }
    filename[BASE_NAME_SIZE ]= '0' + (i/1000)%10;
    filename[BASE_NAME_SIZE + 1] = '0' + (i/100)%10;
    filename[BASE_NAME_SIZE + 2] = '0' + (i/10)%10;
    filename[BASE_NAME_SIZE +3] = '0' + i%10;
    if (i%20 == 0){
        display.display();
        display.print("-");
        display.display();
        Serial.print("+");
    }
    if (i==10000) {
      error(3);
      Serial.println("May be time to cleanup SD card");
    }
    i++;    
  }

  if (!file.open(filename, O_WRONLY | O_CREAT | O_EXCL)) {
    error(3);
  }

  Serial.println(" ");
  Serial.print("SD card ready and Writing data to: "); 
  Serial.println(filename);
  btSDfileName = String(filename);
  
  // **Time/date stamp to SD file property
  SdFile::dateTimeCallback(dateTime);
  
  DateTime now = rtc.now();     // Get the current time/date from RTC
  
  display.display();
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.print("Device ID: ");
  display.setTextSize(2);
  display.println(deviceName.c_str());
  display.display(); // actually display all of the above  
  delay(1000);
  
  // Connnect to the MAXM86161 sensor
  pulseOx = new MAXM86161();

  if (!pulseOx->MAXM86161_Init()) {
    if(SerialTesting == true){
      Serial.println("PulsOx FAILED");
    }
    error(4);
  }
  else {
    if(SerialTesting == true){
      Serial.println("PulseOxSUCCESS");
    }
  }
  pulseOx->MAXM86161_StartSampling();
  delay(100);

  // Connnect to the bno055 sensor
  if (!BNO->begin()) {
    Serial.println("BNO FAILED");
    error(6);
  }
  else {
    Serial.println("BNO SUCCESS");
  }


  // Create BLE Device
  BLEDevice::init(deviceName);

  // Create BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  //Create BLE Service's
  pGeneralService = pServer->createService(GENERAL_SERVICE_UUID);
  pSensorService = pServer->createService(SENSOR_SERVICE_UUID);
  
  //Create BLE Characteristics and set Properties
  pGeneralCodeNameCharacteristic = pGeneralService->createCharacteristic(
                      GENERAL_CODENAME_CHARACTERISITC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE);
  pGeneralCodeCompileDateCharacteristic = pGeneralService->createCharacteristic(
                      GENERAL_CODECOMPILEDATE_CHARACTERISTIC,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE);
  pGeneralSdSaveCharacteristic = pGeneralService->createCharacteristic(
                      GENERAL_SDSAVE_CHARACTERISITC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE);
  pSensorCharacteristic = pSensorService->createCharacteristic(
                      SENSOR_CHARACTERISTIC_UUID, 
                      BLECharacteristic::PROPERTY_READ   |                    
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE);

  if(SerialTesting == true){
    Serial.println("Setting BLE characteristics set");
  }
  // Start Running Services
  pGeneralService->start();
  pSensorService->start();
  
  // Start Advertising
  BLEAdvertising *pGeneralAdvertising = pServer->getAdvertising();
  pGeneralAdvertising->addServiceUUID(GENERAL_SERVICE_UUID);
  pGeneralAdvertising->setScanResponse(true);
  pGeneralAdvertising->setMinPreferred(0x06); // Helps With IOS Connectivity
  pGeneralAdvertising->setMinPreferred(0x12);
  
  BLEDevice::startAdvertising();
  // updating Bluetooth Variable
  std::string BTcodeName = simpleCodeName.c_str();
  pGeneralCodeNameCharacteristic->setValue(BTcodeName);
  std::string codeDate = CompliedDate.c_str();
  std::string codeTime = CompliedTime.c_str();
  std::string codeInfo = codeDate + "_" + codeTime;
  pGeneralCodeCompileDateCharacteristic->setValue(codeInfo);
  std::string sdFileName = btSDfileName.c_str();
  pGeneralSdSaveCharacteristic->setValue(sdFileName);
  
  if(SerialTesting == true){
    Serial.println("Bluetooth Setup Completed!");
  }
 
  // **Open the SD file and write in header information
  file.open(filename, O_WRONLY | O_CREAT );
//  file.open(filename, O_WRONLY | O_APPEND);
  file.print("Code File Name: ");
  file.println(simpleCodeName);
  file.print("Complied Date: ");
  file.println(CompliedDate);
  file.print("Complied Time: ");
  file.println(CompliedTime);
  file.print("Version Information: ");
  file.println(LastRevisionDate);
  file.print("Device ID: ");
  file.println (deviceName.c_str());
  file.print("Date at Start of Recording: ");
  file.print(String(now.month()));
  file.print("/");
  file.print(String(now.day()));
  file.print("/");
  file.println(String(now.year()));
  file.print("Time at Start of Recording: ");
  file.print(String(now.hour()));
  file.print(":");
  file.print(String(now.minute()));
  file.print(":");
  file.println(String(now.second()));
  file.println("");
  // SD column header Titles
  file.println("Time(hr:min:sec); Pulse(bpm); SpO2(%); g(X-axis); g(Y-axis); g(Z-axis); PPG(IR); R ratio(RMS); Note; Temperature(F); PPG(Red); Pluse Time Span (ms); SpO2 Time Span (ms); Perfusion Index; Instant SpO2 (%)");
  file.close();
  delay(50);

  // **Code Information Text Display
  display.display();
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.println(simpleCodeName);
  display.println(LastRevisionDate);
  display.display(); // actually display all of the above  
  delay(2000);

    // **Code Information Text Display
  display.display();
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  display.print("File: ");
  display.setTextSize(2);
  display.println(filename);
//  display.println("OK, GO--------------*"); //dashes are to help determine full space acroos screen
  display.setCursor(0,0);
  display.display(); // actually display all of the above  
  delay(3000);

  lastPulseUpdate = millis();
  lastRedSpO2Update = millis();
  lastRecordTime = millis();
}

/*=======================================================
 ======================================================== */
void loop(){
  accelerometer_t Data = BNO->update();
  pulseoximeter_t result = pulseOx->update();

  // Check for button input
  if (! digitalRead(BUTTON_A)){
    A_State=1;
    B_PulseDetail=0;
    C_PulseOnly=0;
  }
  if (! digitalRead(BUTTON_B)){
    A_State=0;
    B_PulseDetail=1;
    C_PulseOnly=0;
  }
  if (! digitalRead(BUTTON_C)){
    A_State=0;
    B_PulseDetail=0;
    C_PulseOnly=1;
  }
  if(SerialTesting==true){
    if(result.fingerPresent==false){
      Serial.println("Pulse Ox making Contact: False");
    }
  }
    
  if(result.bpm_Updated ==true){
    display.display();
    display.setTextSize(2);
    if(A_State==1){
      display.setCursor(57,0);
    }
    if(B_PulseDetail==1){
      display.setCursor(57,0);
    }
    if(C_PulseOnly==1){
      display.setCursor(110,0);
    }
    
    display.cp437(true);
    display.write(3);
    display.display();
    lastPulseUpdate = millis();
  }
  if(result.bpm_Updated ==true){
    lastPulseUpdate = millis();
  }
  if(result.red_SpO2_Updated ==true){
    lastRedSpO2Update = millis();
  }

  pulse_Time_Span = (millis()-lastPulseUpdate)/1000;
  red_SpO2_Time_Span= (millis()-lastRedSpO2Update)/1000;
  


  // Check to see if it is time to record tothe SD card to update bluetooth info
  int recordDiff = millis()-lastRecordTime;
  int bleDiff = millis()-lastBLEupdate;
  
  if((recordDiff >= recordTimeDuration) || (bleDiff >= bleUpdateTimeSpan)){
    // Get RTC time and format time
    DateTime now = rtc.now();
    
    // This section adjust the time and date to keep the zero before numbers less than 10
    if (now.hour() < 10){
      hrs="0";
      hrs += String(now.hour(),DEC);
    }
    else{
      hrs = String(now.hour(),DEC);
    }
    if (now.minute() < 10){
      mins = "0";
      mins += String(now.minute(),DEC);
    }
    else{
      mins = String(now.minute(),DEC);
    }
    if(now.second() < 10){
      secs = "0";
      secs += String(now.second(),DEC);
    }
    else{
      secs = String(now.second(),DEC);
    }
    if (now.month() < 10){
      Month = "0";
      Month += String(now.month());
    }
    else{
    Month = String(now.month());
    }
    if (now.day() < 10){
      Day = "0";
      Day += String(now.day());
    }
    else{
    Day = String(now.day());
    }
    yr = String(now.year()-2000);
    
    ppgIR = result.ir_PPG;
    ppgRed = result.red_PPG;
    rValueRed = result.ratioRMSred;
    
    // **Get the Accelerometer sensor values:
    xGavg=Data.accelX;
    yGavg=Data.accelY;
    zGavg=Data.accelZ;
      
    SpO2 = result.red_SpO2;

    double BPM = (result.ir_BPM);
    BPM = (int)(BPM+0.5);     

    rawTemperatureIn=result.temperature;
    adjTemperature=(rawTemperatureIn*1.8)+32;  //Needs to be in Farenhiet for app
    adjTemperature =  adjTemperature + adjValue;
      
    String dataString = hrs;
    dataString += ":";
    dataString += mins;
    dataString += ":";
    dataString += secs;
    dataString += ";";
    
    dataString += String(BPM);
    dataString += ";";
    dataString += String(SpO2);
    dataString += ";";
    
    dataString += String(xGavg);
    dataString += ";";
    dataString += String(yGavg);
    dataString += ";";
    dataString += String(zGavg);
    dataString += ";";
    dataString += String(ppgIR);
    dataString += ";";
    dataString += String(rValueRed,4);
    dataString += ";";
    
    // Notes
    if(result.fingerPresent==true){
      dataString += "Skin Contact";
    }
    else if(result.fingerPresent==false){
      dataString += "No Skin Contact";
    }
    else{
      dataString += "-";
    }
    
    dataString += ";";
    dataString+=String(adjTemperature);
      
    dataString += ";";
    dataString += String(ppgRed);
    dataString += ";";
   
    dataString += (pulse_Time_Span);  // seconds between valid BPM data
    dataString += ";";
    dataString += (red_SpO2_Time_Span);  // seconds between valid red SpO2 data
    dataString += ";";
    dataString += String("n/a"); // To be the Perfusion Index    
    dataString += ";";
    dataString += String(Instant_SpO2,2); // Instant SpO2(%)
    dataString += ";";

    
    // Bluetooth comunication  
    if(bleDiff >= bleUpdateTimeSpan){
      char buffer [33];
      std::string msgtime = dataString.c_str();
      // Serial.println(dataString);  
      // updating Bluetooth Variable
      pSensorCharacteristic->setValue(msgtime);
      pSensorCharacteristic->notify();
      
      pGeneralCodeNameCharacteristic->notify();
      pGeneralCodeCompileDateCharacteristic->notify();
      pGeneralSdSaveCharacteristic->notify();
      
      lastBLEupdate = millis();
    }

    //=================================================== 
    //*Controls how offen data is displayed and saved to SD card*
    if(recordDiff >= recordTimeDuration){
      // Record data
      file.open(filename, O_WRONLY | O_APPEND);
      file.println(dataString);
      file.close();
      
      if(SerialTesting == true && plotPPG == false && CalibrationPrint == false){
        Serial.print(Month);
        Serial.print('/');
        Serial.print(Day);
        Serial.print('/');
        Serial.println(yr);
        Serial.print(hrs);
        Serial.print(':');
        Serial.print(mins);
        Serial.print(':');
        Serial.println(secs);
        Serial.print("Pulse(ir): ");
        Serial.println(BPM);
        Serial.print("Time between valid BPM (sec): ");
        Serial.println(pulse_Time_Span,1);
        Serial.print("SpO2(%): ");
        Serial.println(SpO2);
        Serial.print("SpO2(red): ");
        Serial.println(result.red_SpO2);
        Serial.print("r-value (red): ");
        Serial.println(result.ratioRMSred,4);
        Serial.print("Time between valid SpO2-red (sec): ");
        Serial.println(red_SpO2_Time_Span,1);
        Serial.print("X: "); Serial.print(xGavg); Serial.println(" g; ");
        Serial.print("Y: "); Serial.print(yGavg); Serial.println(" g; ");
        Serial.print("Z: "); Serial.print(zGavg); Serial.println(" g; ");
        Serial.print("Temperature: ");
        Serial.print(adjTemperature);
        Serial.println(" F");
        Serial.print("Perfusion Index: ");
        Serial.println("TBD");
        Serial.print("Instant SpO2(%): ");
        Serial.println("TBD");
        Serial.println(); // for screen spacing// Make sure to call update as fast as possible
      }
      
      if(A_State==1){
        display.display();
        display.clearDisplay();
        display.setCursor(0,0);
        display.setTextSize(1);
        display.print(hrs);
        display.print(':');
        display.print(mins);
        display.print(':');
        display.print(secs);
        display.setCursor(53,0);
        display.print("-- ");
        display.print(Month);
        display.print('/');
        display.print(Day);
        display.print('/');
        display.println(yr);
        display.print(BPM);
        display.print(" bpm");
        display.setCursor(72,8);
        display.print(SpO2);;
        display.println(" %");
        display.print("X:");
        display.print(xGavg);
        display.print(" g; ");
        display.print("Y:");
        display.print(yGavg);
        display.println(" g");
        display.print("Z:");
        display.print(zGavg);
        display.print(" g");
        display.display();
      }
      if(C_PulseOnly==1){
        display.display();
        display.clearDisplay();
        display.setCursor(0,0);
        display.setTextSize(2);
        if(result.fingerPresent==false){
          display.println("NOT making");
          display.println("contact");
        }
        else{
          display.print(BPM,0);
          display.print(" bpm");
          display.setTextSize(1);
          display.setCursor(70,8);
          display.print(" sec:");
          display.print(pulse_Time_Span);
          display.setTextSize(2);
          display.setCursor(0,18);
          display.print(SpO2);;
          display.print(" %");
          display.setTextSize(1);
          display.setCursor(60,24);
          display.print(" sec:");
          display.print(red_SpO2_Time_Span);

        }
        display.display();
      }
//================================================================      
      if(B_PulseDetail==1){
        display.display();
        display.clearDisplay();
        
        if(result.fingerPresent==false){
          display.setCursor(0,0);
          display.setTextSize(2);
          display.println("NOT making");
          display.print("contact");
        }
        else{
          display.setCursor(30,0);
          display.setTextSize(1);
          display.print("Red");
          display.setCursor(70,0);
          display.print("| fps");
          display.setCursor(0,8);
          display.print("SpO2: ");
          display.setCursor(30,8);
          display.print(result.red_SpO2,1);
          display.setCursor(70,8);
          display.print("| ");
          display.print(result.avg_fps); 
          display.setCursor(0,16);
          display.print("R:");
          display.setCursor(30,16);
          display.print(result.ratioRMSred,4);
          display.setCursor(70,16);
          display.print("| ");
          display.setCursor(0,24);
          display.print("sec: ");
          display.setCursor(30,24);
          display.print(red_SpO2_Time_Span,1);
          display.setCursor(70,24);
          display.print("| ");
          display.display();
        }
      } //end of display Pulse Detail
        
      if(CalibrationPrint == true){
        Serial.print("Pulse(ir): ");
        Serial.println(BPM);
        Serial.print("Time since last valid BPM (sec): ");
        Serial.println(pulse_Time_Span,1);
        if(redCalibration==true){
          Serial.print("SpO2(red): ");
          Serial.println(result.red_SpO2,2);
          Serial.print("R-Ratio(rms)red: ");
          Serial.println(result.ratioRMSred,5);
          Serial.print("Red Amplitude: ");
          Serial.println(result.red_amp,3);
          Serial.print("IR Amplitude: ");
          Serial.println(result.IR_amp,3);
          Serial.print("Time since last red-ir SpO2: ");
          Serial.println(red_SpO2_Time_Span,3);
          Serial.println("====================");
        }

        Serial.println(); // for screen spacing// Make sure to call update as fast as possible
      }

      lastRecordTime = millis();
    }
  }
  
  if(plotPPG == true){
    if(plotRedppg == true){
      Serial.print(ppgRed);
    }
    if(plotIRppg == true){
      if(plotRedppg == true){
        Serial.print(",");
      }
      Serial.print(ppgIR);
    }
    Serial.println(" ");
  }

}//end of void loop()


//=================================================================================
// Error Messages and Blinking LED Indicator
void error(uint8_t errno) {
  while(1) {
    switch (errno) {
      case 1:
        Serial.println("Couldn't find RTC");
        display.display();
        display.clearDisplay();
        display.setCursor(0,0);
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.println("Couldn't find RTC");
        display.setCursor(0,0);
        display.display(); // actually display all of the above
        break;

      case 2:
        Serial.println("SD Card init. failed!");
        display.display();
        display.clearDisplay();
        display.setCursor(0,0);
        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.println("SD Card failed!");
        display.setCursor(0,0);
        display.display(); // actually display all of the above
        break;

      case 3:
        Serial.print("Couldnt create "); 
        Serial.println(filename);
        display.display();
        display.clearDisplay();
        display.setCursor(0,0);
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.print("Couldn't create ");
        display.println(filename);
        display.setCursor(0,0);
        display.display(); // actually display all of the above
        break;
        
      case 4:
        Serial.println("ERROR: Failed to initialize pulse oximeter");
        display.display();
        display.clearDisplay();
        display.setCursor(0,0);
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.println("FAILED initilization of Pulse Ox!");
        display.println("Restart system after fixing issue");
        display.setCursor(0,0);
        display.display(); // actually display all of the above
        break;
        
      case 5:
        Serial.println("ERROR opening datalog file!");
        display.display();
        display.clearDisplay();
        display.setCursor(0,0);
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.print("Error opening datalog file!");
        display.setCursor(0,0);
        display.display(); // actually display all of the above
        
      case 6:
        Serial.println("ERROR: Failed to initialize BNO-055");
        display.display();
        display.clearDisplay();
        display.setCursor(0,0);
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.println("FAILED initilization of bno055!");
        display.println("Restart system after fixing issue");
        display.setCursor(0,0);
        display.display(); // actually display all of the above
        break;

      case 7:
        Serial.println("ERROR: Conflict between selected Serial print and plot");
        display.display();
        display.clearDisplay();
        display.setCursor(0,0);
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.println("ERROR: Conflict between selected Serial print and plot!");
        display.setCursor(0,0);
        display.display(); // actually display all of the above
        break;

      default:
          break;
    }
    
    for (int i=0; i<20; i++) {
      digitalWrite(redLED, HIGH);
      delay(500);
      digitalWrite(redLED, LOW);
      delay(500);
    }
  } 
}
