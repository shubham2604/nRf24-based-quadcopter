#include<SPI.h>
#include"RF24.h"

#define throttle_pin A1
#define yaw_pin A0
#define pitch_pin A3
#define roll_pin A2

boolean hashardware=true; //set true when connecting joysticks

RF24 tx(7,8);
byte addresses[][6]={"tx","rx"};

struct packet
{
  int throttle;
  int yaw;
  int roll;
  int pitch;
}packet_tx;

void setup() {
  Serial.begin(115200);
  pinMode(throttle_pin,INPUT);
  pinMode(yaw_pin,INPUT);
  pinMode(pitch_pin,INPUT);
  pinMode(roll_pin,INPUT);
  tx.begin();
  tx.setChannel(115);
  tx.setAutoAck(0);
  tx.setDataRate(RF24_250KBPS);
  tx.setPALevel(RF24_PA_MAX);
  tx.openWritingPipe(addresses[0]);
  tx.openReadingPipe(1,addresses[1]);
tx.setCRCLength(RF24_CRC_16);
//tx.disableCRC();
}
void loop() {
  if(hashardware){
  packet_tx.throttle = analogRead(throttle_pin);
  packet_tx.yaw = map(analogRead(yaw_pin),0,1023,1023,0);
  packet_tx.pitch = analogRead(pitch_pin);
  packet_tx.roll = map(analogRead(roll_pin),0,1023,1023,0);
  }
  else{
  packet_tx.throttle = 512;
  packet_tx.yaw = 414;
  packet_tx.pitch = 454;
  packet_tx.roll = 768;
  }
  Serial.println();
  Serial.println("Transmitting");
  Serial.print("  throttle : " );
  Serial.print(packet_tx.throttle);
  Serial.print("  yaw : " );
  Serial.print(packet_tx.yaw);
  Serial.print("  pitch : " );
  Serial.print(packet_tx.pitch);
  Serial.print("  roll : " );
  Serial.print(packet_tx.roll);
  
  
  if(!tx.write(&packet_tx,sizeof(packet_tx))){
    Serial.println("Transmission failed");
  }
  
}
