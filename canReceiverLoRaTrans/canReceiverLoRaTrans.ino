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
  
  //Inicializando o Lora a 915 MHz
  if (!LoRa.begin(915E6)) {             
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }
  Serial.println("LoRa init succeeded.");

  threadSendBox.setInterval(500);
  threadSendBox.onRun(&sendBox);
  cpu.add(&threadSendBox);
}

void loop() {
  cpu.run();
  //Desabilita o LoRa e ativa o CAN
  digitalWrite(CAN_CS, LOW);
  digitalWrite(LORA_CS, HIGH);
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) // To receive data (Poll Read)
  {
    int canId = canMsg.can_id;
    int med1 = canMsg.data[0];
    int med2 = canMsg.data[3];
    int med1Data = restaura(canMsg.data[1], canMsg.data[2]);
    int med2Data = restaura(canMsg.data[4], canMsg.data[5]);
    Serial.println(String(med1Data) + ", " + String(med2Data));
  }
}

int restaura(byte b0, byte b1) {
  return (b0 << 5) + b1;
}

void sendBox(){
  digitalWrite(CAN_CS, HIGH);
  digitalWrite(LORA_CS, LOW);
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(med1);
  LoRa.print(med1Data);                     // add payload
  LoRa.endPacket();                     // finish packet and send it
  Serial.println("Enviei: " + String(med1) + " = " + String(med1Data));
  Serial.println("");
}
