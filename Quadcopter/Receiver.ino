#include<SPI.h>



byte addresses[][6]={"tx","rx"};

struct packet
{
  int throttle;
  int yaw;
  int roll;
  int pitch;
}packet_rx;

void rx_Initialize(){
//  Serial.begin(115200); used for debugging
 
  rx.begin();
  rx.setChannel(115);
  rx.setDataRate(RF24_250KBPS);
  rx.setPALevel(RF24_PA_LOW);
  rx.openWritingPipe(addresses[1]);
  rx.openReadingPipe(1,addresses[0]);
  rx.startListening();
  rx.setAutoAck(0);
  rx.setCRCLength(RF24_CRC_16);

}


void receiver_update(){

  if(rx.available())
  {
    while(rx.available())
      {rx.read(&packet_rx,sizeof(packet_rx));
      lastRecvTime = millis();
  }
     

  }
  
   receiver_input_channel_1 = map( packet_rx.roll, 0, 1023, 1000, 2000);
   receiver_input_channel_2 = map( packet_rx.pitch, 0, 1023, 1000, 2000);
   receiver_input_channel_3 = map( packet_rx.throttle, 0, 1023, 1000, 2000);
   receiver_input_channel_4 = map( packet_rx.yaw, 0, 1023, 1000, 2000);
  
  
/*    
  Serial.print(receiver_input_channel_3);
  Serial.print("\n");
  Serial.print(receiver_input_channel_4);
  Serial.print("\n");
  Serial.print(receiver_input_channel_1);
  Serial.print("\n");
  Serial.print(receiver_input_channel_2);
  Serial.println("\n");
  */  
}




void signal_loss(){
  
  start = 0;
  
}
