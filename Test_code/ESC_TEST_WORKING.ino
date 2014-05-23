#include <Servo.h> 
 
Servo s;

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define MOTOR_PIN 8
 
int pos = 0;
 
void setup() 
{ 
  Serial.begin(9600);
  Serial.println("Press any key to start & stop BLDC motor!");
  while(!Serial.available());
  s.attach(MOTOR_PIN);
  arm();
}

void loop()
{
  int speed;
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
  }
  s.writeMicroseconds(speed);
}

void arm()
{
  int a;
  for(a=995;a<1041;a++)
  {
    s.writeMicroseconds(a);
    delay(30);
  }
}
