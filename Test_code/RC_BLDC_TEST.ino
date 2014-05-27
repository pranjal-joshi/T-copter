#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
/*-----( Declare Constants and Pin Numbers )-----*/
#define CE_PIN   9
#define CSN_PIN 10

// NOTE: the "LL" at the end of the constant is "LongLong" type
const uint64_t pipe = 0xE8E8F0F0E1LL; // Define the transmit pipe


/*-----( Declare objects )-----*/
RF24 radio(CE_PIN, CSN_PIN); // Create a Radio
Servo s;
/*-----( Declare Variables )-----*/
int joystick[5];  // 2 element array holding Joystick readings

void setup()   /****** SETUP: RUNS ONCE ******/
{
  Serial.begin(9600);
  s.attach(7);
  arm();
  s.detach();
  Serial.println("Nrf24L01 Receiver Starting");
  radio.begin();
  radio.setPayloadSize(10);
  radio.setPALevel(RF24_PA_LOW);
  radio.setCRCLength(RF24_CRC_8);
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1,pipe);
  radio.startListening();
}//--(end setup )---


void loop()   /****** LOOP: RUNS CONSTANTLY ******/
{
  bool motorON = false;
  s.detach();
  if ( radio.available() )
  {
    // Read the data payload until we've received everything
    bool done = false;
    while (!done)
    {
      // Fetch the data payload
      done = radio.read( joystick, sizeof(joystick) );
      for(byte k=0;k<5;k++)
      {
        Serial.print(joystick[k]);
        Serial.print("   ");
      }
      Serial.println();
      if(joystick[0] == 0)
        joystick[0] = 1035;
      if(joystick[4] & 0x01 == 0x00)
        motorON = true;
      else
        motorON = false;
      //joystick[0] = map(joystick[0],1035,2000,0,180);
      //if(motorON)
      {
        s.attach(7);
        s.writeMicroseconds(joystick[0]);
      }
      Serial.println(joystick[0]);
    }
  }
  s.attach(7);
}//--(end main loop )---

void arm()
{
  int a;
  for(a=1000;a<1040;a++)
  {
    s.writeMicroseconds(a);
    delay(10);
  }
  s.writeMicroseconds(1035);
}