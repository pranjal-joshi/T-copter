// Yaw mechanism testing code
// Guide to build yaw mechanism is available at: http://pranjalnrobotics.blogspot.com

#include <Servo.h> 
 
Servo BLDC1;
Servo BLDC2;
Servo BLDC3;
Servo yaw;

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
 
int pos,speed=0;

/*
For my setup, Servo angles are as follow. It may have different values for diffrent configurations.
Central position	:	113
Maximum Left		:	83
Maximum Right		:	143
*/
 
void setup() 
{ 
  Serial.begin(57600);
  Serial.println("0 = OFF\n\r1 = LOW\n\r2 = MEDIUM\n\r3 = HIGH");;
  while(!Serial.available());
  BLDC1.attach(8);
  BLDC2.attach(7);
  BLDC3.attach(6);
  yaw.attach(9);
  yaw.write(113);
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
    BLDC1.writeMicroseconds(a);
    BLDC2.writeMicroseconds(a);
    BLDC3.writeMicroseconds(a);
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
      speed = 1000;
    }
    Serial.flush();
    BLDC1.writeMicroseconds(speed);
    /*
    my BLDC2 motor move slower than other 2.
    dont know why! May be manufacturing error.
    hence adding compensation to maintain speed equal to
    other motors.
    */
    BLDC2.writeMicroseconds(speed + 20);
    BLDC3.writeMicroseconds(speed);
  }
}
