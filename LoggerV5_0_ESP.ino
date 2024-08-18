//Libraries. This flight computer uses an MPU6050 and BMP390 over I2C protocol, and SD reader over SPI.
#include <SdFat.h> //SdFat library runs way faster than the standard SD library
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>.
#include <Adafruit_MPU6050.h>
#include <DFRobot_BMP3XX.h>
#include "Buzzer.h"
#include "Serial_Debug.h"
#include "Control.h"

//Define constants
#define SEALEVELPRESSURE_HPA 1013.25 
#define droguepin 6
#define mainpin 5
#define RGBpin 21
#define mainalti 100 //altitude to activate mainpin on descent
#define flushtime 2500 //time between flushing SD card buffer
#define logtime 2 //set a minimum logging interval in ms.
#define bmprate 20 //time in ms between reading bmp sensor. Allows IMU and BMP to read and log at different rates.
#define LEDduration 300
//#define datadebugging

//Variables
float pressure, altitude, altioffset, correctedalt, maxaccel, TTA, ax, ay, az, gx, gy, gz; //max acceleration and time to apogee not currently implemented
float maxalt = 0;
byte droguecount = 0;
byte maincount = 0;
byte logtimeout = 0;
unsigned long timer, datatimer, bmptimer, LEDtimer, flushclock, apogeetime, liftofftime, timetoapogee; //some of these yet not implemented
bool bmpready = false;
bool LEDstate = false;
int logNumber = 0;

//BMP, MPU and SD things
DFRobot_BMP390L_I2C bmp(&Wire, bmp.eSDOVDD);
Adafruit_MPU6050 mpu;
SdFat SD;
SdFile logfile;
Buzzer Buzz(15);
void events(int state); //forward declaration of events function for control function, wouldn't compile properly without it
Control Control(droguepin, mainpin, 20, events); //set pins for ejection charges, safety lockout altitude (so ejection charges don't accidentally go off at ground level) and what function to callback to on state change
Serial_Debug Debug(115200);

void setup() {
//start buzzer library and serial
  neopixelWrite(21,0,0,0); // onboard LED Off
  Wire.begin(4,2,1000000);
  Buzz.begin();
  Debug.begin();

//setup begin tone
  neopixelWrite(21,0,0,64); // onboard LED Blue
  Buzz.startup();

 //setup pins
  Control.begin();
  Control.SetSafetyLock(20);
  Control.SetMainAltitude(100);
  pinMode(SS, OUTPUT);
  neopixelWrite(21,0,0,0); // onboard LED Off
  delay(500);
  
//start BMP390
  neopixelWrite(21,0,0,64); // onboard LED Blue
  int rslt;
  while (ERR_OK != (rslt = bmp.begin())){
        if(ERR_DATA_BUS == rslt){
        Debug.debugBMP(1, 0);
        Buzz.error();
        neopixelWrite(21,255,0,0); // onboard LED Red
        while(1);
        }
        else if(ERR_IC_VERSION == rslt){
               Debug.debugBMP(2, 0);
               Buzz.error();
               neopixelWrite(21,255,0,0); // onboard LED Red
               while(1);
               }
        }  
  bmp.setPWRMode(bmp.ePressEN | bmp.eTempEN | bmp.eNormalMode);
  bmp.setOSRMode(bmp.ePressOSRMode8 | bmp.eTempOSRMode1);
  bmp.setODRMode(BMP3XX_ODR_50_HZ);
  bmp.setIIRMode(BMP3XX_IIR_CONFIG_COEF_7);
  delay(1000);
  pressure = bmp.readPressPa()/100;
  altioffset = 44330.0 * (1.0 - pow(pressure / SEALEVELPRESSURE_HPA, 0.1903)); //Set an offest for height above ground. There's definitely a better way to do this.
  Debug.debugBMP(3, altioffset);
  neopixelWrite(21,0,0,0); // onboard LED Off
  delay(500);

//start MPU6050
  neopixelWrite(21,0,0,64); // Blue
  if (mpu.begin()) {
     mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
     mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
     mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
     Debug.debugIMU(2);
     }
  else {
       Debug.debugIMU(1);
       Buzz.error();
       neopixelWrite(21,255,0,0); // onboard LED Red
       while(1);
       }
  neopixelWrite(21,0,0,0); // onboard LED Off
  delay(500);

//start SD
  neopixelWrite(21,0,0,64); // onboard LED Blue
  if (SD.begin(14, SD_SCK_MHZ(16))) {     
     #define COMMA logfile.print(",");
     while (SD.exists("FltLog" + String(logNumber) + ".csv")) {
     logNumber++;
     }
     char filename[20];
     snprintf(filename, sizeof(filename), "FltLog%d.csv", logNumber);
     logfile.open(filename, O_WRITE | O_CREAT);     
     delay(2000); // Issues seem to happen without this short delay.
     logfile.println("~ LOGGER v5.0 ~");
     logfile.println("~ Last compiled: 31-7-24 ~");
     logfile.println();
     logfile.println("ms,ax,ay,az,gx,gy,gz,pres,alt,rel. alt,");
     logfile.sync(); 
     Debug.debugSD(2);   
     }
  else {
       Debug.debugSD(1);
       Buzz.error();
       neopixelWrite(21,255,0,0); // onboard LED Red
       while (1); //Stopper. Won't continue without the SD card.
       }
  neopixelWrite(21,0,0,0); // onboard LED Off
  delay(500);

//indicate setup is done.
  neopixelWrite(21,0,128,0); // onboard LED Green
  Buzz.success();
  delay(1000);
  neopixelWrite(21,0,0,0); // onboard LED Off
  delay(10000);
}

void readsensors (void) {
//Read IMU and apply calibration offset
//Calibration values need to be adjusted for each logger. 
//There is a better way to calibrate the accelerometer using a formula but offsets will do for now.
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  ax = a.acceleration.x -0.65; 
  ay = a.acceleration.y +0.19; 
  az = a.acceleration.z - 10.26; 
  gx = g.gyro.x + 0.08; 
  gy = g.gyro.y - 0.03; 
  gz = g.gyro.z - 0.02; 

//Read baro if it's time, call Control.Deployment and pass altitude and Z acceleration for deployment checks
  if (datatimer - bmptimer >= bmprate){
     bmptimer = datatimer;
     pressure = bmp.readPressPa()/100; // Read pressure once
     altitude = 44330.0 * (1.0 - pow(pressure / SEALEVELPRESSURE_HPA, 0.1903));
     correctedalt = altitude - altioffset;
     Control.Deployment(correctedalt, az); //az not currently implemented, function currently still asks for it though
     bmpready = true;
     }
}

void endlog (void) {
//footer for log file
  //TTA = apogeetime - liftofftime;
  logfile.println();
  logfile.print("max altitude: "); COMMA;
  logfile.print(Control.maxalt);
  logfile.print(" m, at: "); COMMA;
  logfile.println(Control.apogeetime);
// the following is not currently in use. Instead of tacking on all this to the log file, it would be beter to create a separate file with flight summaries.       
//  logfile.print("liftoff: "); COMMA;
//  logfile.print(liftofftime);
//  logfile.println(" ms");
//  logfile.print("Time to apogee:"); COMMA;
//  logfile.print(TTA);
//  logfile.println(" ms");
  logfile.println();
  logfile.print(" ~END OF LOG~");
  logfile.println();
  logfile.sync();
  logfile.close();
  neopixelWrite(21,0,128,0); //onboard LED Green
  while(1) {
           Buzz.ended(); //Ending tone 
           }
}

void events (int state) { 
//this is the function that is passed to the control library as the function to call back to on state change.
//allows user to set up some optional extra actions, like logging when apogee is detected or when the control function has disarmed the pyro channel.
//knowing when there was a state change in the control logic is useful for understanding what happened and figuring out why certain logic was triggered on a flight if something didn't work as expected.
  switch (state){
    case 0:
    COMMA
    logfile.print("armed");
    break;
    case 1:
    COMMA
    logfile.print("drogue fired");
    break;
    case 2:
    COMMA
    logfile.print("main fired");
    break;
    case 3:
    COMMA
    logfile.print("disarmed");
    break;
    default:
      //nothing to do here for states 4 and 5.
    break;
    }
}

void loop() {
  if (millis() - datatimer >= logtime){
     //run through the sensor reads and deployment checks
     datatimer = millis();
     timer = millis(); //check 
     readsensors();

     //log the readings to file. This is probably inefficient but will go in the bin if using onboard memory anyway.
     logfile.println();
     logfile.print(datatimer); COMMA;
     logfile.print(ax); COMMA;
     logfile.print(ay); COMMA;
     logfile.print(az); COMMA;
     logfile.print(gx); COMMA;
     logfile.print(gy); COMMA;
     logfile.print(gz);
     if (bmpready){
        COMMA;
        logfile.print(pressure); COMMA;
        logfile.print(altitude); COMMA;
        logfile.print(correctedalt);
        Control.Deployment(correctedalt, az); //pass altitude and acceleration to control
        bmpready = false;
        
        //Serial data printed only if defined   
        #ifdef datadebugging
        Debug.debugdata(datatimer, pressure, altitude, correctedalt, ax, ay, az, gx, gy, gz, Control.controlstate);
        #endif
        }

     //do this stuff only once per whatever the flushtime is set to.
     if ((timer - flushclock >= flushtime)){
        logfile.sync(); //sync in the SDfat library does the same thing as flush in the standard Arduino SD library.
        Buzz.running();
        neopixelWrite(21,96,0,255); //onboard LED Purple
        LEDstate = true;
        flushclock = timer;
        LEDtimer = timer;
        //end of flight timeout
        if (Control.controlstate == 5){
           endlog();
           }
     }

     //Do this more often. Don't want the LED on for very long.
     if (LEDstate && timer - LEDtimer >= LEDduration) {
        neopixelWrite(21,0,0,0);
        LEDtimer = timer;
        LEDstate = false;
        }
  }   
}
