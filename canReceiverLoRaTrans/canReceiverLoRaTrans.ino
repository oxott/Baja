String KEY = "f8da0ab3"

#define CAN_CS 10             //Definição do Pino de Chip Select do Can
#define LORA_CS 9             //Definição do Pino de Chip Select do LoRa
#define LORA_RST 0            //Definição do Pino de Reset do LoRa
#define LORA_D0 4             //Definição do Pino D0 do LoRa



#include <SPI.h>              //Biblioteca para SPI
#include <mcp2515.h>          //Biblioteca para o CAN
#include <LoRa.h>             //Biblioteca do LoRa
#include <Thread.h>           //Biblioteca para a criação de Threads
#include <ThreadController.h>

String outgoing;              //Mensagem de saída do LoRa
byte localAddress = 0xFF;     //Endereço desse dispositivo
byte destination = 0xBB;      //Endereço de destino no Box

struct can_frame canMsg;      //Struct com o frame recebido pelo CAN

MCP2515 mcp2515(CAN_CS);      //Configuração do Pino de Chip Select do CAN

Thread threadSendBox;
ThreadController cpu;

struct med {
  int cod;
  int meas;
  uint8_t MSB;
  uint8_t LSB;
};
typedef struct med Med;
Med S1, S2, S3, S4;

int med1 = 0;
int med1Data = 0;

void setup() {
  //Begins SPI communication
  SPI.begin();
  //Configuração dos pinos da LoRa
  LoRa.setPins(LORA_CS, LORA_D0, LORA_RST);
  //Início da Comunicação Serial
  Serial.begin(115200);

  //Configurações iniciais do CAN, resetando o modulo
  mcp2515.reset();
  //Setando a velocidade de 500KBPS e o clock de 8 MHz
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  //Setando o CAN para modo de operação normal
  mcp2515.setNormalMode();

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  threadSendBox.setInterval(50);
  threadSendBox.onRun(sendBox);

  cpu.add(&threadSendBox);
}

void loop() {
  cpu.run();
  //Desabilita o LoRa e ativa o CAN
  digitalWrite(CAN_CS, LOW);
  digitalWrite(LORA_CS, HIGH);
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) { // To receive data (Poll Read){
    int canId = canMsg.can_id;
    S1 = {0, 0, canMsg.data[0], canMsg.data[1]};
    recoverMeasurement(&S1.cod, &S1.meas, S1.MSB, S1.LSB);
    S2 = {0, 0, canMsg.data[2], canMsg.data[3]};
    recoverMeasurement(&S2.cod, &S2.meas, S2.MSB, S2.LSB);
    S3 = {26, 666, 0, 0};
    createMeasurement(S3.cod, S3.meas, &S3.MSB, &S3.LSB);
    S4 = {28, 694, 0, 0};
    createMeasurement(S4.cod, S4.meas, &S4.MSB, &S4.LSB);
    Serial.println("Received");
    //Serial.println(S1.MSB);
    //Serial.println(S1.LSB);
  }
}


void recoverMeasurement(int* type, int* measurement, uint8_t MSB, uint8_t LSB) {
  *type = (int)((MSB & 252) >> 2);
  *measurement = (int)(((MSB & 3) << 8) + LSB);
}

void createMeasurement(int type, int measurement, uint8_t* MSB, uint8_t* LSB) {
  *MSB = (uint8_t)((type << 2) | ((measurement & 768) >> 8));
  *LSB = (uint8_t)(measurement & 255);
}

void sendBox() {
  Serial.println(KEY.length());
  digitalWrite(CAN_CS, HIGH);
  digitalWrite(LORA_CS, LOW);
  LoRa.beginPacket();
  LoRa.print("MINHAPICA");
  LoRa.write(S1.MSB);
  LoRa.write(S1.LSB);
  LoRa.write(S2.MSB);
  LoRa.write(S2.LSB);
  LoRa.write(S3.MSB);
  LoRa.write(S3.LSB);
  LoRa.write(S4.MSB);
  LoRa.write(S4.LSB);

  LoRa.endPacket();
  Serial.println("Enviado LoRa");
}
