// Combined Pressure and Air Quality Sensors on Ardunio Mega 2560 //

/** Code developed from jeremycole/AllSensors_DLHR (DLLR-L10D-E1BD-C-NAV7 pressure sensor) and paulvha/sps30 (Sensirion SPS30 sensors)
 *  Pins 20 (SDA), 21 (SCL) --> DLLR-L10D-E1BD-C-NAV7, I2C (Pressure Sensor)
 *  Pins 16 (TX2), 17 (RX2) --> Sensirion SPS30, UART/Serial 2 (Air Quality Sensor)
 *  Pins 14 (TX3), 15 (RX3) --> Sensirion SPS30, UART/Serial 3 (Air Quality Sensor)
  */

// DLLR-L10D-E1BD-C-NAV7 Pressure Sensor //

#include <Wire.h>

#include <AllSensors_DLHR.h>

AllSensors_DLHR_L10D_7 gagePressure(&Wire);

// Sensirion SPS30 Sensors //

#include "sps30.h"

#define SPS30_COMMS2 Serial2   
#define SPS30_COMMS3 Serial3   

#define DEBUG2 0
#define DEBUG3 0

// create constructors
SPS30 sps302;
SPS30 sps303;

// save serial numbers
char serial[5][32];

void setup() {

  Serial.begin(115200);

// DLLR-L10D-E1BD-C-NAV7 Pressure Sensor //
  
  {
  
  digitalWrite(20,LOW);
  digitalWrite(21,LOW);
  Wire.begin();

  gagePressure.setPressureUnit(AllSensors_DLHR::PressureUnit::IN_H2O);

  }

// Sensirion SPS30 Sensors //
  
  {
  
  // setup SPS30 - 2 and 3  
  #ifdef SPS30_COMMS2
  setupSPS30(2);
  #endif
  #ifdef SPS30_COMMS3
  setupSPS30(3);
  #endif
  
  }

}

void loop() {

// DLLR-L10D-E1BD-C-NAV7 Pressure Sensor //
    
  gagePressure.startMeasurement();
  gagePressure.readData(true);
  if ((gagePressure.pressure > -10) && (gagePressure.pressure < 2)) // ignore pressure values well above or below expected range (in inH2O); include SPS30 measurements in 'if' statement
 
  { 
   
    Serial.print(((gagePressure.pressure)*25.4)+0.76); // output pressure and convert to mmH2O; adjust to offset of sensor (+0.76); change this offset each test day depending on sensor offset
    Serial.print(" ");
    Serial.print(gagePressure.temperature); // output temperature
    Serial.print(" ");

// Sensirion SPS30 Sensors //
  
  struct sps_values val;
  uint8_t ret;
  
  #ifdef SPS30_COMMS2
  read_all(2);
  #endif
  #ifdef SPS30_COMMS3
  read_all(3);
  #endif
  
  Serial.println(); // output SPS30 data values
  
  }
  
 delay(2000);

}

/* overall output = DLLR Differential Pressure (mmH2O), DLLR Temperature(deg C), SPS30 Serial 2 Data, SPS30 Serial 3 Data */

/*------------------------------------------------------------------------------------------------------------------------------------------------*/

// Sensirion SPS30 Sensors //

/**
 * @brief : read and display all values
 * @param dev : constructor
 * @param d : device number
 */
bool read_all(uint8_t d)
{

  static bool header = true;
  uint8_t ret, error_cnt = 0;
  struct sps_values val;

  // loop to get data
  do {
    if (d == 2) ret = sps302.GetValues(&val); 
    else if (d == 3) ret = sps303.GetValues(&val); 

  else {
    Serial.println("Invalid device to read");
    return false;
  }

    // data might not have been ready
    if (ret == ERR_DATALENGTH){

        if (error_cnt++ > 3) {
          ErrtoMess(d,(char *) "Error during reading values: ",ret);
        }
        delay(1000);
    }

    // if other error
    else if(ret != ERR_OK) {
      ErrtoMess(d,(char *) "Error during reading values: ",ret);
    }

  } while (ret != ERR_OK);

/* output of SPS30 Sensors
 *  Particle Mass (Conc. [μg/m3]), Number Count(Conc. [#/cm3]), Average Particle Size ([μm])
 *  PM1.0/PM2.5/PM4.0/PM10.0/NC0.5/NC1.0/NC2.5/NC4.0/NC10.0/PartSize
 */
  Serial.print(val.MassPM1);
  Serial.print("/");
  Serial.print(val.MassPM2);
  Serial.print("/");
  Serial.print(val.MassPM4);
  Serial.print("/");
  Serial.print(val.MassPM10);
  Serial.print("/");
  Serial.print(val.NumPM0);
  Serial.print("/");
  Serial.print(val.NumPM1);
  Serial.print("/");
  Serial.print(val.NumPM2);
  Serial.print("/");
  Serial.print(val.NumPM4);
  Serial.print("/");
  Serial.print(val.NumPM10);
  Serial.print("/");
  Serial.print(val.PartSize);
  Serial.print("/");

  return(true);
}

/*
 * Initialize SPS30
 * @param d   : device number
 */
void setupSPS30(uint8_t d)
{
  bool ret;


#ifdef SPS30_COMMS2 // for Serial 2
  if (d == 2)
  {
    // set driver debug level
    sps302.EnableDebugging(DEBUG2);

    // start channel
    SPS30_COMMS2.begin(115200);

    // Initialize SPS30 library
    if (!sps302.begin(&SPS30_COMMS2)) {
      Serial.println(F("\nCould not set communication channel for SPS30 - 2"));
      ErrorStop();
    }
  
    // check for SPS30 connection
    if (! sps302.probe()) {
      Serial.println(F("\nCould not probe / connect with SPS30 - 2"));
      ErrorStop();
    }
  
    // reset SPS30 connection
    if (! sps302.reset()){
      Serial.println(F("could not reset."));
      ErrorStop();
    }
  
    // read device info
    GetDeviceInfo(d, false);
    
    if (! sps302.start()) {
      Serial.println("\nCould NOT start measurement SPS30");
      ErrorStop();
    }
  
  }
#endif

#ifdef SPS30_COMMS3 // for Serial 3
  if (d == 3)
  {
    // set driver debug level
    sps303.EnableDebugging(DEBUG3);

    // start channel
    SPS30_COMMS3.begin(115200);

    // initialize SPS30 library
    if (!sps303.begin(&SPS30_COMMS3)) {
      Serial.println("\nCould not set communication channel for SPS30 - 3");
      ErrorStop();
    }
  
    // check for SPS30 connection
    if (! sps303.probe()) {
      Serial.println("\nCould not probe / connect with SPS30 - 3");
      ErrorStop();
    } 
  
    // reset SPS30 connection
    if (! sps303.reset()){
      Serial.println("could not reset.");
      ErrorStop();
    }
  
    // read device info
    GetDeviceInfo(d, false);
    
    if (! sps303.start()) {
      Serial.println("\nCould NOT start measurement SPS30");
      ErrorStop();
    }

  }
#endif

  if (d <1 || d > 5) {
      Serial.print("Invalid device number for setup :");
      Serial.println(d);
      ErrorStop();
  }
}

/**
 * @brief : read and display device info
 * @param d : device number
 * @param i2c : true if I2C / wire channel
 */
void GetDeviceInfo(uint8_t d, bool i2c)
{
  char buf[32];
  uint8_t ret;
  SPS30_version v;
  
  if (d == 2) ret = sps302.GetSerialNumber(buf, 32); 
  else if (d == 3) ret = sps303.GetSerialNumber(buf, 32); 

  else {
    Serial.println("Invalid device");
    return;
  }
 // try to read serial number
  if (ret == ERR_OK) {
  //  Serial.print(F("Serial number : "));
    if(strlen(buf) > 0) {
   //   Serial.print(buf);
   //   strcpy(&serial[d -1][0], buf); // save serial number
    }
    else {
      Serial.print("not available");
      serial[d-1][0] = 0;
    }
  }
  else
    ErrtoMess(d,(char *) "could not get serial number", ret);

  if (d == 2) ret = sps302.GetProductName(buf, 32);
  else if (d == 3) ret = sps303.GetProductName(buf, 32);

  // try to get product name

  if (ret == ERR_OK)  {
   // Serial.print(F(" Product name  : "));

    //if(strlen(buf) > 0) 
    //Serial.println(buf);
    //else Serial.println(F("not available"));
 }
  else
    ErrtoMess(d,(char *)"could not get product name.", ret);
    
   // try to get version info
  if (d == 2) ret = sps302.GetVersion(&v); 
  else if (d == 3) ret = sps303.GetVersion(&v); 

  if (ret != ERR_OK) {
    Serial.println("Can not read version info");
    return;
  }
  
  // if Wire channel : no SHDLC info
  if (i2c){
    Serial.println();
    return;
  }  

}

/**
 *  @brief : continued loop after fatal error
 *  @param d: device number
 *  @param mess : message to display
 *  @param r : error code
 *  if r is zero, it will only display the message
 */
void Errorloop(uint8_t d, char * mess, uint8_t r)
{
  if (r) ErrtoMess(d, mess, r);
  else Serial.println(mess);
  Serial.println("Program on hold");
  for(;;) delay(100000);
}

// just stop
void ErrorStop()
{
  Errorloop(1,(char *)"",0);
}

/**
 *  @brief : display error message
 *  @param d: device number
 *  @param mess : message to display
 *  @param r : error code
 *
 */
void ErrtoMess(uint8_t d, char * mess, uint8_t r)
{
  char buf[80];

  Serial.print(mess);
  if (d == 2) sps302.GetErrDescription(r, buf, 80); 
  else if (d == 3) sps303.GetErrDescription(r, buf, 80); 
  else    strcpy(buf, "unknown device number");

  Serial.println(buf);
}

/**
 * serialTrigger prints repeated message, then waits for enter to come in from the serial port.
 */
void serialTrigger(char * mess)
{
  Serial.println();

  while (!Serial.available()) {
    Serial.println(mess);
    delay(2000);
  }

  while (Serial.available())
    Serial.read();
}
