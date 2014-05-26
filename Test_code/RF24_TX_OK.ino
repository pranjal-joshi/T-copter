#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
/*-----( Declare Constants and Pin Numbers )-----*/
#define CE_PIN   9
#define CSN_PIN 10

const uint64_t pipe = 0xE8E8F0F0E1LL; // Define the transmit pipe

RF24 radio(CE_PIN, CSN_PIN); // Create a Radio

int i[5] = {1,2,3,4,5};

void setup()
{
  Serial.begin(9600);
  radio.begin();
  radio.setRetries(15,15);
  radio.setPayloadSize(10);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(pipe);
}

void loop()
{
  i[0]++;
  i[1]++;
  i[2]++;
  i[3]++;
  i[4]++;
  rp();
}

void rp()
{
  radio.write(i,sizeof(i));
}
