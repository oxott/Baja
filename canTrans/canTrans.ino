#define lowByte(w) (((uint8_t) (w)) & B00011111)
#define highByte(w) ((uint8_t) ((w) >> 5))

#include <SPI.h>          //Library for using SPI Communication 
#include <mcp2515.h>      //Library for using CAN Communication
#include <Thread.h>
#include <ThreadController.h>

struct can_frame canMsg;
MCP2515 mcp2515(10);

ThreadController cpu;

Thread threadSendCan1;

void setup(){
  while (!Serial);
  Serial.begin(115200);
  
  SPI.begin();               //Begins SPI communication
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS,MCP_8MHZ); //Sets CAN at speed 500KBPS and Clock 8MHz
  mcp2515.setNormalMode();

  threadSendCan1.setInterval(500);
  threadSendCan1.onRun(send1);

  cpu.add(&threadSendCan1);
}

void loop() 
{
  cpu.run();  
}

void send1(){
  int pot1 = analogRead(0);
  int pot2 = analogRead(5);
  canMsg.can_id  = 0x036;           //CAN id as 0x036
  canMsg.can_dlc = 8;               //CAN data length as 8
  canMsg.data[0] = 0x022; 
  canMsg.data[1] = highByte(pot1);               //Update humidity value in [0]
  canMsg.data[2] = lowByte(pot1);               //Update temperature value in [1]
  canMsg.data[3] = 0x023;
  canMsg.data[4] = highByte(pot2);
  canMsg.data[5] = lowByte(pot2);
  canMsg.data[6] = 0x000;
  canMsg.data[7] = 0x000;
  mcp2515.sendMessage(&canMsg);     //Sends the CAN message  
  Serial.println("Sending...");
  Serial.println(String(pot1) + ", " + String(pot2));
  
}
