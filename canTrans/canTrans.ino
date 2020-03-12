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

struct med {
  int cod;
  int meas;
  uint8_t MSB;
  uint8_t LSB;
};
typedef struct med Med;

void setup() {
  while (!Serial);
  Serial.begin(115200);

  SPI.begin();               //Begins SPI communication

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); //Sets CAN at speed 500KBPS and Clock 8MHz
  mcp2515.setNormalMode();

  threadSendCan1.setInterval(1);
  threadSendCan1.onRun(send1);

  cpu.add(&threadSendCan1);
}

void loop() {
  cpu.run();
}

void send1() {
  Med S1 = {22, analogRead(0), 0, 0};
  createMeasurement(S1.cod, S1.meas, &S1.MSB, &S1.LSB);
  Med S2 = {24, analogRead(5), 0, 0};
  createMeasurement(S2.cod, S2.meas, &S2.MSB, &S2.LSB);
  Med S3 = {0, 0};
  Med S4 = {0, 0};
  canMsg.can_id  = 0x036;           //CAN id as 0x036
  canMsg.can_dlc = 8;               //CAN data length as 8
  canMsg.data[0] = S1.MSB;
  canMsg.data[1] = S1.LSB;               //Update humidity value in [0]
  canMsg.data[2] = S2.MSB;               //Update temperature value in [1]
  canMsg.data[3] = S2.LSB;
  canMsg.data[4] = S3.MSB;
  canMsg.data[5] = S3.LSB;
  canMsg.data[6] = S4.MSB;
  canMsg.data[7] = S4.LSB;
  mcp2515.sendMessage(&canMsg);     //Sends the CAN message
  Serial.println("Sending...");
  Serial.println("S1 = " + String(S1.cod) + " = " + String(S1.meas));
  Serial.println("S2 = " + String(S2.cod) + " = " + String(S2.meas));
}

void createMeasurement(int type, int measurement, uint8_t* MSB, uint8_t* LSB) {
  *MSB = (uint8_t)((type << 2) | ((measurement & 768) >> 8));
  *LSB = (uint8_t)(measurement & 255);
}
