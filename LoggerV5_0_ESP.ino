//Libraries. This flight computer uses an MPU6050 and BMP390over I2C, and SD reader over SPI.
#include <SdFat.h>
#include <Wire.h> //This can be ditched if everything can be put on an SPI bus to SPEED TOWN yyyyeeEEEAHHH DAWWWWG
#include <SPI.h>
#include <Adafruit_Sensor.h> //wtf does this one even do, idk.
#include <Adafruit_MPU6050.h>
#include <DFRobot_BMP3XX.h>
#include "Buzzer.h"
//#include "Backup_Data.h"
#include "Serial_Debug.h"
#include "Control.h"

//Define constants
#define SEALEVELPRESSURE_HPA 1013.25 
#define droguepin 6
#define mainpin 5
#define RGBpin 21
#define mainalti 100 //trigger altitude to activate mainpin
#define buzztime 3000
#define flushtime 5000
#define logtime 2 //set a minimum logging interval in ms.
#define bmprate 20 //time in ms between reading bmp sensor. Allows IMU and BMP to operate at different rates.
#define LEDduration 300
//#define datadebugging

//Variables
float pressure, altitude, altioffset, correctedalt, maxaccel, TTA, ax, ay, az, gx, gy, gz;
float maxalt = 0;
byte droguecount = 0;
byte maincount = 0;
byte logtimeout = 0;
long buzzclock;
unsigned long timer, datatimer, bmptimer, LEDtimer, flushclock, apogeetime, liftofftime, timetoapogee;
bool bmpready = false;
bool EEPROMenabled = false;
bool LEDstate = false;
int logNumber = 0;

//BMP, MPU and SD things
DFRobot_BMP390L_I2C bmp(&Wire, bmp.eSDOVDD);
Adafruit_MPU6050 mpu; //May need to be changed to 9-axis sensor
SdFat SD;
SdFile logfile;
Buzzer Buzz(15);
void events(int state); //forward declaration of events function
Control Control(droguepin, mainpin, 20, events);
//Backup_Data Backup(0, 4, 8); //backup logging object - takes EEPROM addresses for 3 data points - Not currently in use
Serial_Debug Debug(115200);

void setup() {
//start buzzer library and serial
  neopixelWrite(21,0,0,0); // Off / black
  Wire.begin(4,2,1000000);
  Buzz.begin();
  Debug.begin();

//setup begin tone
  neopixelWrite(21,0,0,64); // Blue
  Buzz.startup();

//setup pins
  Control.begin();
  Control.SetSafetyLock(50);
  Control.SetMainAltitude(100);
  pinMode(SS, OUTPUT);
  neopixelWrite(21,0,0,0); // Off
  delay(300);
  
//start BMP390
  neopixelWrite(21,0,0,64); // Blue
  int rslt;
  while (ERR_OK != (rslt = bmp.begin())){
        if(ERR_DATA_BUS == rslt){
        Debug.debugBMP(1, 0);
        Buzz.error();
        neopixelWrite(21,255,0,0); // Red
        while(1);
        }
        else if(ERR_IC_VERSION == rslt){
               Debug.debugBMP(2, 0);
               Buzz.error();
               neopixelWrite(21,255,0,0); // Red
               while(1);
               }
        }  
  bmp.setPWRMode(bmp.ePressEN | bmp.eTempEN | bmp.eNormalMode);
  bmp.setOSRMode(bmp.ePressOSRMode8 | bmp.eTempOSRMode1);
  bmp.setODRMode(BMP3XX_ODR_50_HZ);
  bmp.setIIRMode(BMP3XX_IIR_CONFIG_COEF_7);
  delay(1000);
  pressure = bmp.readPressPa()/100;
  altioffset = 44330.0 * (1.0 - pow(pressure / SEALEVELPRESSURE_HPA, 0.1903)); //Figure out an offest for height above ground. There's a better way to do this with the BMP probably.
  Debug.debugBMP(3, altioffset);
  neopixelWrite(21,0,0,0); // Off
  delay(300);

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
       neopixelWrite(21,255,0,0); // Red
       while(1);
       }
  neopixelWrite(21,0,0,0); // Off
  delay(300);

//start SD
  neopixelWrite(21,0,0,64); // Blue
  if (SD.begin(14, SD_SCK_MHZ(16))) {     
     #define COMMA logfile.print(",");
     while (SD.exists("FltLog" + String(logNumber) + ".csv")) {
     logNumber++;
     }
     char filename[20];
     snprintf(filename, sizeof(filename), "FltLog%d.csv", logNumber);
     logfile.open(filename, O_WRITE | O_CREAT);     
     delay(2000); //delays are yuck but this might be necessary because the SD card initialises so fast now. Issues seem to happen without it.
     logfile.println("~LOGGER v5.0~");
     logfile.println();
     logfile.println("ms,ax,ay,az,gx,gy,gz,pres,alt,rel. alt,");
     logfile.sync(); 
     Debug.debugSD(2);   
     }
  else {
       EEPROMenabled = true; //EEPROM has limited writes, so to save EEPROM wear, writes will only occur if EEPROMenabled is true (so like... when my dumb ass forgets the SD card...).
       Debug.debugSD(1);
       Buzz.error();
       neopixelWrite(21,255,0,0); // Red
       while (1); //stopper. This can be changed to a prompt once the EEPROM code is ready to allow logger to continue without the SD card.
       }
  neopixelWrite(21,0,0,0); // Off
  delay(300);

//indicate setup is done.
  neopixelWrite(21,0,128,0); // Green
  Buzz.success();
  delay(1000);
  neopixelWrite(21,0,0,0); // Off / black
  delay(3000);
}

void readsensors (void) {
//Read IMU and apply calibration offset
//Calibration values need to be adjusted for each logger. 
//There is a better way to calibrate the accelerometer using linear formula but offsets will do for now.
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  ax = a.acceleration.x - 0.74;
  ay = a.acceleration.y - 0.15;
  az = a.acceleration.z + 1.78;
  gx = g.gyro.x + 0.29;
  gy = g.gyro.y + 0.59;
  gz = g.gyro.z + 0.18;

  //Read baro if it's time, call Control.Deployment and pass altitude and Z acceleration for deployment checks
  if (datatimer - bmptimer >= bmprate){
     bmptimer = datatimer;
     pressure = bmp.readPressPa()/100; // Read pressure once
     altitude = 44330.0 * (1.0 - pow(pressure / SEALEVELPRESSURE_HPA, 0.1903));
     correctedalt = altitude - altioffset;
     Control.Deployment(correctedalt, az);
     bmpready = true;
     }
}

void endlog (void) {
//footer for log file
  //TTA = apogeetime - liftofftime;
  if (!EEPROMenabled){
     logfile.println();
     logfile.print("max altitude: "); COMMA;
     logfile.print(Control.maxalt);
     logfile.print(" m, at: "); COMMA;
     logfile.println(Control.apogeetime);
// the following is not currently in use. Instead of tacking on all this to the log file, it would be beter to create a separate file with flight summaries.       
//     logfile.print("liftoff: "); COMMA;
//     logfile.print(liftofftime);
//     logfile.println(" ms");
//     logfile.print("Time to apogee:"); COMMA;
//     logfile.print(TTA);
//     logfile.println(" ms");
     logfile.println();
     logfile.print(" ~END OF LOG~");
     logfile.println();
     logfile.sync();
     logfile.close();
     }
     neopixelWrite(21,0,128,0); // Green
//Write some minimal data to EEPROM
//Not currently in use
/*  if (EEPROMenabled){
     Backup.save(maxalt, maxaccel, TTA); //saves 3 data points - can add a couple more with library modification.
     }*/
//Ending tone 
  while(1) {
           Buzz.ended();
           }
}

void events (int state) {
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
     timer = millis(); 
     readsensors();

     //log the readings to file. v4.1 prints in a different way, might have to bring that across if it's more efficient
     if (!EEPROMenabled){
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
            Control.Deployment(correctedalt, az); //"hey control.h, here's the altitude and acceleration. have fun."
            bmpready = false;
            }
        }
    
     //Serial readouts   
     #ifdef datadebugging
     Debug.debugdata(datatimer, pressure, altitude, correctedalt, ax, ay, az, gx, gy, gz, Control.controlstate);
     #endif

     //sync in the SDfat library does the same thing as flush.
     if ((timer - flushclock >= flushtime)){
        logfile.sync();
        flushclock = timer;
        }

     //Tone indicating logging is happening. Pretty sure the "if" statement can be run as part of the function instead, but that also means calling the function every loop rather than only making it conditional. 
     //Could also just put it inside the SD sync statement or vice-versa, that way it's only ever checked when the other is checked as well, rather than being checked every loop. 
     if (timer - buzzclock >= buzztime) {
        Buzz.running();
        neopixelWrite(21,96,0,255); //Purple
        LEDtimer = timer;
        LEDstate = true;
        buzzclock = timer;
        }

     if (LEDstate && timer - LEDtimer >= LEDduration) {
        neopixelWrite(21,0,0,0);
        LEDtimer = timer;
        LEDstate = false;
        }

     //logging timeout check.
     if (Control.controlstate == 5){
        endlog();
        }
  }   
}

/* 05/05/2024 - this version is not yet complete and is intended to run on an ESP32-S3 based data logger, and has now been split off from the version that fits on an Arduino Nano.

Pin allocations are not set. This version of code is intended to be somewhat modular and has functionality for dual deployment.
-Uses BMP390 for the barometer and MPU6050 for the IMU.
-EEPROM Backup is commented out for now, may end up being removed completely.
-Buzzer tones are in their own library now
-Control library handles arming/disarming ejection charge pins, drogue deployment, main deployment and timeout notification on landing through the use of numbered "states"
-Each state corresponds with a different set of checks, when the parameters in one state is met, will change state and perform the new checks
-Control library can be given the deployment pins, arming altitude, main deployment altitude and a callback function that will be handed the control state before each state change.
-Serial debug library has a few different custom functions to assist with breadboarding and diagnosis.
-logging rate with an ESP32-S3 can reach over 500hz for the IMU, or over 1000hz with data debugging not defined, and at least 50hz for the baro although is currently not fully tested.
-Starting to make use of on-board RGB LED 

Done:
-Serial debugging now added as a library - added control state to the data debugging
-Deployment has now moved to a library but is untested.
-Added callbacks to main file from Control library, function added in main code to print events to the log file for post-flight analysis/diagnosis.
-Somehow this version is smaller than v4.1 when compiled for an Arduino Nano, yet has WAY more functionality?? What the?? (Probably lighter libraries...) (actually not sure if it still fits on a Nano... It might if EEPROM backup is removed...)(oh my god.... it does?!?!)
-Don't know how to feel about the fact that this even compiles after just slapping in all these changes in under a day. It's either really good or or it's about to be a major headache that will be hard to find and fix.
-Better slap this puppy on a breadboard and find out...
-Oh... it mostly worked the first time?!? Holy shit!

To do:
-Finish EEPROM backup one day, but this is low priority.
-Explore adding a 9-axis IMU in place of the MPU6050
-Servo control should be possible with the S3, but might roll that in to a separate version.
-Not even using multi-core task assignment yet either... at this point though it may not be necessary. 500hz on the IMU is enough to start see ringing and vibrations, rather than just a rough output of forces!
-Modularity is getting there... need to start putting in IFFDEFs and more class constructor options to make it easier to include/exclude things.
-Build a combined "configuration" function in Control library - setting arming, main and apogee detect sensitivity should all just be one function call, but maybe just leave the option for individual calls. 
-Change timestamps to use micros() instead of millis() for greater accuracy of IMU readings - currently ably to hit over 500hz so the resolution might be necessary.
-Change up the loop a bit to make use of a "data ready" system - this way the loop just skips doing anything until the corresponding flags are flipped. 
-E.g - does nothing until IMU ready = true, then it logs the IMU and checks BMPready. if BMPready then if statements are checked for logfile.sync, Buzz.running and Control.controlstate
*/