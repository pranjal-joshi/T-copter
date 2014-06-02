// Yaw mechanism testing code
// Guide to build yaw mechanism is available at:
// http://pranjalnrobotics.blogspot.com

#include <Servo.h> 
 
Servo BLDC;
Servo yaw;

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define MOTOR_PIN 8
#define SERVO_PIN 7
 
int pos = 83,speed=0;

/*
For my setup, Servo angles are as follow. It may have different values for diffrent configurations.
Central position	:	113
Maximum Left		:	83
Maximum Right		:	143
*/
 
void setup() 
{ 
  Serial.begin(57600);
  Serial.println("Press any key to start & stop BLDC motor!");
  while(!Serial.available());
  BLDC.attach(MOTOR_PIN);
  yaw.attach(SERVO_PIN);
  arm();
}

void loop()
{
  for(pos = 83; pos < 144; pos ++)
  {
    yaw.write(pos);
    delay(20);
    updateBLDC();
  }
  for(pos = 143; pos > 82; pos --)
  {
    yaw.write(pos);
    delay(20);
    updateBLDC();
  }
  
}

void arm()
{
  int a;
  for(a=995;a<1041;a++)
  {
    BLDC.writeMicroseconds(a);
    delay(30);
  }
}

void updateBLDC()
{
  if(Serial.available())
  {
    char c = (char)Serial.read();
    if(c == '1')
    {
      Serial.println("ON - Lowest speed");
      speed = 1045;
    }
    if(c == '2')
    {
      Serial.println("ON - Medium speed");
      speed = 1300;
    }
    if(c == '3')
    {
      Serial.println("ON - Full speed");
      speed = 2000;
    }
    if(c == '0')
    {
      Serial.println("OFF");
      speed = 1020;
    }
    Serial.flush();
    BLDC.writeMicroseconds(speed);
  }
}
