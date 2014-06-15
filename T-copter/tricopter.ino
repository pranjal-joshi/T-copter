// Project Infinity - The UAV T-copter drone
// Build for ATmega644PA (sanguino) (Flight Controller)
/*
Author  :  Pranjal Joshi
Date    :  30-4-2014
License :  GNU GPL v2 (Relased in public domain as open-source).
*/
#include <HMC5883L.h>
#include <TinyGPS++.h>
#include <RF24.h>
#include <SPI.h>
#include <Wire.h>
#include <PID_v1.h>
#include <I2Cdev.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <Adafruit_BMP085.h>
#include <Servo.h>
#include <printf.h>
#include <EEPROM.h>
#include <math.h>
#include <SoftwareSerial.h>
#include "printf.h"

// Serial_Debug ***************************************
#define IMU_DEBUG 0
#define PID_DEBUG 0
#define RADIO_DEBUG 0
#define THROTTLE_DEBUG 0

// General configuration **************************************
#define PAYLOADSIZE 10
#define ESC_MIN 1000
#define ESC_MAX 2000
#define YAW_LEFT 83
#define YAW_CENTER 113
#define YAW_RIGHT 143
#define YAW_TOLERANCE_DEGREES 2.5
#define THRESHOLD_RADIUS_METERS 5
#define TAIL_LIFTUP_COUNTERWEIGHT 0

// My left motor rotates slower due to manufacturing defects.
// adding constant to calculated signal provides compensation by
// increasing speed of defective motor than other, thus both
// motors rotates with almost same speed/rpm (equal thrust).
#define SLOW_MOTOR_COMPENSATION 3.33

// Hardware config ************************************
#define gpsled A7
#define lightsPin 12

// PID tunings ****************************************
double kpr = 0.75;      // roll
double kir = 0.5;
double kdr = 0.05;

double kpp = 0.75;      // pitch
double kip = 0.5;
double kdp = 0.05;

double kpy = 1.5;      // yaw
double kiy = 0.02;
double kdy = 0.01;

#define kpa 1.5      // altihold
#define kia 0.02
#define kda 0.01

#define PID_PITCH_MIN -50
#define PID_PITCH_MAX 50
#define PID_ROLL_MIN -30
#define PID_ROLL_MAX 30

#define INIT_ROLL 0.0
#define INIT_PITCH 0.0
#define INIT_YAW 0.22

// initial PID values. tested on stable platform / ground.

double initRoll = INIT_ROLL;
double initPitch = INIT_PITCH;
double initYaw = INIT_YAW;

// Global Variables ***********************************

int mx,my,mz;        // HMC5883L

double roll,pitch,yaw,throttle;
double oproll_left,oproll_right,oppitch_fwd,oppitch_rev,opyaw;   // PID op

double alti,temperature,opalti,givenAlti;          // BMP085

const uint64_t pipe = 0xE8E8F0F0E1LL;  // radio pipe
uint16_t radioFrame[PAYLOADSIZE/2];    // radio payload
double readThrottle, readYaw, readRoll, readPitch, readStabilize, readSpecialKeys;  //read form nRF

float flat,flon,x2lat,x2lon,diflat,diflon,dist,temp,heading,angle;      // GPS & waypoints
double northheading, initHeadings;              // angle to north
uint16_t fixedInitHeadings;

char BT_data[10];    // bluetooth framing
uint16_t addr = 0;    // EEPROM addr

byte isAltihold=0,isAutopilot=0,areLights=0;

uint8_t mpuIntStatus,devStatus;
uint16_t packetSize,fifoCount;
uint8_t fifoBuffer[64];
VectorFloat gravity;
float ypr[3] = {0.0, 0.0, 0.0};
Quaternion q;
double smoothVal;
volatile boolean mpuInterrupt = false;

// define instances/objects **************************
HMC5883L hmc;
MPU6050 mpu;
TinyGPSPlus gps;
RF24 radio(0,1);
Adafruit_BMP085 bmp;
Servo right;
Servo left;
Servo rear;
Servo yawservo;
SoftwareSerial gpsPort(3,4);
PID pid_roll_left(&roll, &oproll_left, &initRoll, kpr, kir, kdr, DIRECT);
PID pid_roll_right(&roll, &oproll_right, &initRoll, kpr, kir, kdr, REVERSE);
PID pid_pitch_fwd(&pitch, &oppitch_fwd, &initPitch, kpp, kip, kdp, DIRECT);
PID pid_pitch_rev(&pitch, &oppitch_rev, &initPitch, kpp, kip, kdp, REVERSE);
PID pid_yaw(&northheading, &opyaw, &initHeadings, kpy, kiy, kdy, DIRECT);
PID pid_altihold_up(&alti, &opalti, &givenAlti, kpa, kia, kda, DIRECT);
PID pid_altihold_down(&alti, &opalti, &givenAlti, kpa, kia, kda, REVERSE);


void setup()
{
  Serial.begin(57600);    // HC-05 default speed
  gpsPort.begin(9600);
  Wire.begin();
  TWBR = 24;      // i2c bus @ 400KHz
  pinMode(gpsled,OUTPUT);
  digitalWrite(gpsled,LOW);
  pinMode(lightsPin,OUTPUT);
  digitalWrite(lightsPin,LOW);
  Serial.println("\n\rUAV [ T-copter ] - Booting up");
  Serial.print("Free RAM (bytes) : ");
  Serial.println(freeRam());
  Serial.println("Initializing HC-05 Bluetooth..");
  loadDefaultSetpoints();
  initPID();
  initRadio();
  initIMU();
  stabilizeIMU();
  getStartupHeadings();
  fixedInitHeadings = initHeadings;
  yawservo.attach(A3);
  testYawServo();
  attachMotors();
  // check arming signal
  // wait untill not armed
  detachMotors();
}

void loop()
{
  readGPS();
  readRadio();
  readIMU();
  computePID();
  updateMotors();
}

void initPID()
{
  pid_pitch_fwd.SetMode(AUTOMATIC);
  pid_pitch_fwd.SetOutputLimits(PID_PITCH_MIN,PID_PITCH_MAX);
  pid_pitch_fwd.SetSampleTime(14);
  
  pid_pitch_rev.SetMode(AUTOMATIC);
  pid_pitch_rev.SetOutputLimits(PID_PITCH_MIN,PID_PITCH_MAX);
  pid_pitch_rev.SetSampleTime(14);
  
  pid_roll_left.SetMode(AUTOMATIC);
  pid_roll_left.SetOutputLimits(PID_ROLL_MIN,PID_ROLL_MAX);
  pid_roll_left.SetSampleTime(14);
  
  pid_roll_right.SetMode(AUTOMATIC);
  pid_roll_right.SetOutputLimits(PID_ROLL_MIN,PID_ROLL_MAX);
  pid_roll_right.SetSampleTime(14);
  
  pid_yaw.SetMode(AUTOMATIC);          // yaw servo
  pid_yaw.SetOutputLimits(-30,30);
  //pid_yaw.SetSampleTime(50);
  
  pid_altihold_up.SetMode(AUTOMATIC);
  pid_altihold_up.SetOutputLimits(ESC_MIN,ESC_MAX);
  //pid_altihold_up.SetSampleTime(5);
  
  pid_altihold_down.SetMode(AUTOMATIC);
  pid_altihold_down.SetOutputLimits(ESC_MIN,ESC_MAX);
  //pid_altihold_down.SetSampleTime(5);
}

void getAltitude()
{
  alti = bmp.readAltitude();
  temperature = bmp.readTemperature();
  
  #if IMU_DEBUG
    Serial.print(alti);
    Serial.print("\t");
  #endif
}

void getHeadings()
{
  hmc.getHeading(&mx, &my, &mz);
  northheading = atan2(my, mx);
  if(northheading < 0)
    northheading += 2 * M_PI;
  northheading *= 180/M_PI;
  
  #if IMU_DEBUG
    Serial.println(northheading);
  #endif
}

void readRadio()
{
  detachMotors();
  if(radio.available())
  {
    bool done = false;
    while (!done)
    {
      done = radio.read(radioFrame, sizeof(radioFrame));
    }
    attachMotors();
    readThrottle = (double)radioFrame[0];
    readYaw = (double)radioFrame[1];
    readPitch = (double)radioFrame[2];
    readRoll = (double)radioFrame[3];
    readSpecialKeys = (double)radioFrame[4];
    
    #if RADIO_DEBUG
      for(byte k=0;k<5;k++)
      {
        Serial.print(radioFrame[k]);
        Serial.print("    ");
      }
      Serial.println();
    #endif
    
    decodeRadio();
  }
  attachMotors();
}

void decodeRadio()
{
  isAltihold=0;
  isAutopilot=0;
  // TODO : add arming facility
  if(bitRead((uint16_t)readSpecialKeys,1))
  {
    isAutopilot = 0;
    navigation();
    // enable autopilot realted services
  }
  else
    isAutopilot = 1;
  if(!(bitRead((uint16_t)readSpecialKeys,2)))
  {
    isAltihold = 0;
    // enable altihold
    
    // *****....saftey shutdown for TESTING ONLY... *******
    while(1)
    {
      left.writeMicroseconds(1005);
      right.writeMicroseconds(1005);
      rear.writeMicroseconds(1005);
    }
  }
  else
    isAltihold = 1;
  if(bitRead((uint16_t)readSpecialKeys,3))
    digitalWrite(lightsPin,LOW);  // enable lights
  else
    digitalWrite(lightsPin,HIGH);
  
}

void navigation()
{
  float flat1 = flat;
  float flon1 = flon;
  
  diflat = radians(x2lat - flat1);
  flat1 = radians(flat1);
  x2lat = radians(x2lat);
  diflon = radians((x2lon)-(flon)); 
  
  dist = sin(diflat/2.0) * sin(diflat/2.0);
  temp = cos(flat1) * cos(x2lat) * ((sin(diflon/2.0)*(sin(diflon/2.0))));
  dist +=  temp;
  dist = 2*atan2(sqrt(dist),sqrt(1.0-dist));
  dist = dist * 6371000.0;    // converted into meters
  dist += (dist * 0.003947368421);     // error compensation to 1 meter
  
  flon = radians(flon);
  x2lon = radians(x2lon);
  
  heading = atan2((sin(x2lon-flon)*cos(x2lat)),((cos(flat)*sin(x2lat))-(sin(flat)*cos(x2lat)*cos(x2lon-flon)),2*M_PI));
  heading = heading * (180/M_PI);    // convert to degrees
  heading = (int)heading;      // cast to int as float is not required further
  if(heading < 0)
  {
    heading += 360;            // make heading positive
  }
  
  getHeadings();              // read compass
  angle = northheading - heading;
  
  if(angle >= -180 && angle <= 0)
  {
    // turn right
  }
  else if(angle < -180)
  {
    // turn left
  }
  else if(angle >= 0 && angle < 180)
  {
    // turn left
  }
  if(northheading == heading)
  {
    // go straight
  }
  
  if(dist < THRESHOLD_RADIUS_METERS)
  {
    // get next waypoint
    getNextWaypoint();
  }
  getAltitude(); // & TO-DO: maintain altitude
}

/*void serialEvent()
{
  // TODO : implement bluetooth reception
  Serial.println("Serial Interrupt detected");
}*/

void eepromWriteCoordinate(char * data, int addr)
{
  for (int i=0; i<9; i++)
    EEPROM.write(addr++, data[i]);
}

void eepromReadCoordinate(char* data, int addr)
{
  for(byte i=0;i<9;i++)
    data[i] = EEPROM.read(addr++);
}

void getNextWaypoint()
{
  /*
  logic set home point at 1 & next waypoints as 2,3,...
  set 9 as base addr [as co-ordinates are of 9 bytes]
  then use 9 * no.of waypoint to get respective addr in
  EEPROM location & read it.
  convert to float using atof() & return it to navigation
  controller & autopilot system for further processing.
  */
}

void initRadio()
{
  readPitch = 150;
  readYaw = 150;
  readRoll = 150;
  loadDefaultSetpoints();
  detachMotors();
  radio.begin();
  radio.setPayloadSize(10);
  radio.setPALevel(RF24_PA_MAX);
  radio.setCRCLength(RF24_CRC_8);
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1,pipe);
  radio.startListening();
  Serial.println("Intializing nRF24L01+PA+LNA Radio interface...");
  
  #if RADIO_DEBUG
    printf_begin();
    radio.printDetails();
  #endif
}

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

void attachMotors()
{
  right.attach(A0);
  left.attach(A1);
  rear.attach(A2);
  yawservo.attach(A3);
}

void detachMotors()
{
  right.detach();
  left.detach();
  rear.detach();
  yawservo.detach();
}

void initIMU()
{
  if(!mpu.testConnection())
    Serial.println("*** MPU6050 failed. Flight Abort.");
  else
  {
    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    mpu.setFullScaleGyroRange(3);
    mpu.setFullScaleAccelRange(3);
    mpu.setXGyroOffset(-17);
    mpu.setYGyroOffset(-17);
    mpu.setZGyroOffset(-15);
    mpu.setXAccelOffset(-2205-285-39-37);
    mpu.setYAccelOffset(1500+820+95+10);
    mpu.setZAccelOffset(1688);
    if(devStatus == 0)
    {
      mpu.setDMPEnabled(true);
      attachInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
      packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
      Serial.print("MPU6050 DMP failed. Error = ");
      Serial.println(devStatus);
    }
  }
  if(!hmc.testConnection())
    Serial.println("*** HMC5883L failed.");
  else
    hmc.initialize();
  if(!bmp.begin())
    Serial.println("*** BMP085 failed.");
  else
    Serial.println("GY-88 IMU initialized.");
  Serial.println("Initializing MTK3329 GPS...");
}

void dmpDataReady()
{
  mpuInterrupt = true;
}

void testYawServo()
{
  byte k;
  yawservo.write(YAW_CENTER);
  delay(200);
  for(k=YAW_CENTER;k<YAW_RIGHT;k++)
  {
    yawservo.write(k);
    delay(25);
  }
  for(k=YAW_RIGHT;k>YAW_LEFT;k--)
  {
    yawservo.write(k);
    delay(25);
  }
}

void readIMU()
{
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if((mpuIntStatus & 0x10) || fifoCount == 1024)
    mpu.resetFIFO();
  else if(mpuIntStatus & 0x02)
  {
    while(fifoCount < packetSize)
      fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    ypr[0] *= 180/M_PI;  // convert to degrees
    ypr[1] *= 180/M_PI;
    ypr[2] *= 180/M_PI;
    yaw = ypr[0];      // save ypr values to pid input variables
    pitch = ypr[1];
    roll = ypr[2];
    
    #if IMU_DEBUG
      Serial.print(ypr[0]);
      Serial.print("\t");
      Serial.print(ypr[1]);
      Serial.print("\t");
      Serial.print(ypr[2]);
      Serial.print("\t");
    #endif
    
    getAltitude();
    getHeadings();
  }
}

void computePID()
{ 
  if(readPitch > 150)          // change pitch setpoint according to RC TX
  {
    initPitch = 15;
    kpp = 1.9;
    kip = 0.3;
    kdp = 0.85;
  }
  else if(readPitch < 150)
  {
    initPitch = -15;
    kpp = 1.9;
    kip = 0.3;
    kdp = 0.85;
  }
  else if(readPitch == 150)
  {
    initPitch = INIT_PITCH;
    loadDefaultSetpoints();
  }
  
  if(readRoll > 150)          // change roll setpoint according to RC TX
  {
    initRoll = 15;
    kpr = 1.8;
    kir = 0.45;
    kdr = 0.85;
  }
  else if(readRoll < 150)
  {
    initRoll = -15;
    kpr = 1.8;
    kir = 0.45;
    kdr = 0.85;
  }
  else if(readRoll == 150)
  {
    initRoll = INIT_ROLL;
    loadDefaultSetpoints();
  }
  
  if(readYaw > 150)
  {
    yawservo.write(YAW_RIGHT);
    getHeadings();
    initHeadings = northheading;
  }
  else if(readYaw < 150)
  {
    yawservo.write(YAW_LEFT);
    getHeadings();
    initHeadings = northheading;
  }
  else
  {
    loadDefaultSetpoints();
    pid_yaw.Compute();
  }
  
  pid_roll_left.Compute();
  pid_roll_right.Compute();
  pid_pitch_fwd.Compute();
  pid_pitch_rev.Compute();
  
  #if PID_DEBUG
    Serial.print("P - fwd/rev:\t");
    Serial.print(oppitch_fwd);
    Serial.print("   ");
    Serial.print(oppitch_rev);
    Serial.print("\tR - left/right:\t");
    Serial.print(oproll_left);
    Serial.print("   ");
    Serial.print(oproll_right);
    Serial.print("\tYaw:\t");
    Serial.println(opyaw);
  #endif
}

void updateMotors()
{
  #define START_PID_CONTROLLERS_AT 1168
    
  if((readThrottle > START_PID_CONTROLLERS_AT) && (readThrottle < 2002))
  {
    left.writeMicroseconds(readThrottle + oppitch_fwd + oproll_left + SLOW_MOTOR_COMPENSATION);
    right.writeMicroseconds(readThrottle + oppitch_fwd + oproll_right);
    rear.writeMicroseconds(readThrottle + oppitch_rev - TAIL_LIFTUP_COUNTERWEIGHT);
  }
  else if(readThrottle < START_PID_CONTROLLERS_AT && readThrottle > 1034)
  {
    left.writeMicroseconds(readThrottle);
    right.writeMicroseconds(readThrottle);
    rear.writeMicroseconds(readThrottle);
  }
  else
  {
    left.writeMicroseconds(1005);
    right.writeMicroseconds(1005);
    rear.writeMicroseconds(1005);
  }
  if(readYaw == 150)
    yawservo.write(YAW_CENTER + opyaw);
  
  #if THROTTLE_DEBUG
    Serial.print("left:\t");
    Serial.print(readThrottle + oppitch_fwd + oproll_left + SLOW_MOTOR_COMPENSATION);
    Serial.print("\tright:\t");
    Serial.print(readThrottle + oppitch_fwd + oproll_right);
    Serial.print("\trear:\t");
    Serial.print(readThrottle + oppitch_rev - TAIL_LIFTUP_COUNTERWEIGHT);
    Serial.print("\tyaw:\t");
    Serial.println(YAW_CENTER + opyaw); //opyaw_right
  #endif
}

void readGPS()
{
  if(gpsPort.available())
  {
    char c = gpsPort.read();
    if(gps.encode(c))
    {
      if(gps.location.isValid())
      {
        digitalWrite(gpsled,HIGH);
        flat = gps.location.lat();
        flon = gps.location.lng();
        // get loc & waypoint calculations
      }
    }
   }
}

void getStartupHeadings()
{
  byte cnt;
  initHeadings = 0;
  northheading = 0;
  for(cnt=0;cnt<20;cnt++)
  {
    getHeadings();
    initHeadings += northheading;
  }
  initHeadings /= 20;
  Serial.print("Initial north headings for yaw correction: ");
  Serial.print(initHeadings);
  Serial.println(" degrees.");
}

void loadDefaultSetpoints()
{
  double kpr = 2.5;      // roll
  double kir = 0.5;
  double kdr = 1.1;

  double kpp = 2.6;      // pitch
  double kip = 0.55;
  double kdp = 1.23;

  double kpy = 0.65;      // yaw
  double kiy = 0.25;
  double kdy = 0.095;
}

void stabilizeIMU()
{
  readIMU();
  while((ypr[1] > 1.0 || ypr[1] < -1.0) || (ypr[2] > 1.0 || ypr[2] < -1.0))
  {
    digitalWrite(gpsled,HIGH);
    delay(100);
    digitalWrite(gpsled,LOW);
    delay(100);
    readIMU();
  }
}
