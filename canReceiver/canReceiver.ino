#include <SPI.h>              //Library for using SPI Communication 
#include <mcp2515.h>          //Library for using CAN Communication

struct can_frame canMsg; 
MCP2515 mcp2515(10);                 // SPI CS Pin 10 

void setup(){
  SPI.begin();                       //Begins SPI communication
  
  Serial.begin(115200);                //Begins Serial Communication at 9600 baudrate 
  
  mcp2515.reset();                          
  mcp2515.setBitrate(CAN_500KBPS,MCP_8MHZ); //Sets CAN at speed 500KBPS and Clock 8MHz 
  mcp2515.setNormalMode();                  //Sets CAN at normal mode
}

void loop(){
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) // To receive data (Poll Read)
  {
     int canId = canMsg.can_id;   
     int med1 = restaura(canMsg.data[0], canMsg.data[1]);
     Serial.println("Recebido de " + String(canId, HEX) + " = "  + String(med1));
    }
}

int restaura(byte b0, byte b1){
  return (b0<<5) + b1;
}
