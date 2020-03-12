/*
  LoRa Duplex communication

  Sends a message every half second, and polls continually
  for new incoming messages. Implements a one-byte addressing scheme,
  with 0xFF as the broadcast address.

  Uses readString() from Stream class to read payload. The Stream class'
  timeout may affect other functuons, like the radio's callback. For an

  created 28 April 2017
  by Tom Igoe
*/
#define LORA_CS 9
#define LORA_RST 0
#define LORA_D0 4

#include <SPI.h>              // include libraries
#include <LoRa.h>

struct med {
  int cod;
  int meas;
  uint8_t MSB;
  uint8_t LSB;
};
typedef struct med Med;
Med S1, S2, S3, S4;

void setup() {
  Serial.begin(115200);                   // initialize serial
  while (!Serial);

  Serial.println(420);            //Inicio

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(LORA_CS, LORA_D0, LORA_RST);// set CS, reset, IRQ pin

  if (!LoRa.begin(915E6)) {             // initialize ratio at 915 MHz
    Serial.println(666);                //LoRa Fail
    while (true);                       // if failed, do nothing
  }

  Serial.println(333);                  //LoRa God
}

void loop() {
  onReceive(LoRa.parsePacket());
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return
  String incoming = "";
  
  if(LoRa.available()) {
    for(int i=0; i<9; i++)
      incoming += (char)LoRa.read();
  }
  
  S1.MSB = LoRa.read();
  S1.LSB = LoRa.read();
  S2.MSB = LoRa.read();
  S2.LSB = LoRa.read();
  S3.MSB = LoRa.read();
  S3.LSB = LoRa.read();
  S4.MSB = LoRa.read();
  S4.LSB = LoRa.read();


  recoverMeasurement(&S1.cod, &S1.meas, S1.MSB, S1.LSB);
  recoverMeasurement(&S2.cod, &S2.meas, S2.MSB, S2.LSB);
  recoverMeasurement(&S3.cod, &S3.meas, S3.MSB, S3.LSB);
  recoverMeasurement(&S4.cod, &S4.meas, S4.MSB, S4.LSB);
  /*
    Serial.println(String(S1.cod) + " = " + String(S1.meas));
    Serial.println(String(S2.cod) + " = " + String(S2.meas));
    Serial.println(String(S3.cod) + " = " + String(S3.meas));
    Serial.println(String(S4.cod) + " = " + String(S4.meas));
    Serial.println("RSSI: " + String(LoRa.packetRssi()));
    Serial.println("Snr: " + String(LoRa.packetSnr()));
    Serial.println("");

    Serial.println(S1.cod);
    Serial.println(S1.meas);
    Serial.println(S2.cod);
    Serial.println(S2.meas);
    Serial.println(S3.cod);
    Serial.println(S3.meas);
    Serial.println(S4.cod);
    Serial.println(S4.meas);
  */
  Serial.println(incoming);
  Serial.println(String(S1.cod) + ";" + String(S1.meas) + ";" + String(S2.cod) + ";" + String(S2.meas) + ";" +
                 String(S3.cod) + ";" + String(S3.meas) + ";" + String(S4.cod) + ";" + String(S4.meas) + ";" +
                 String(LoRa.packetRssi()) + ";" + String(LoRa.packetSnr()));
}

void recoverMeasurement(int* type, int* measurement, uint8_t MSB, uint8_t LSB) {
  *type = (int)((MSB & 252) >> 2);
  *measurement = (int)(((MSB & 3) << 8) + LSB);
}
