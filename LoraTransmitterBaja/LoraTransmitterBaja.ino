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
#include <SPI.h>              // include libraries
#include <LoRa.h>
#include <Thread.h>
#include <ThreadController.h>

const int csPin = 10;          // LoRa radio chip select
const int resetPin = 0;       // LoRa radio reset
const int irqPin = 2;         // change for your board; must be a hardware interrupt pin
const int interval = 500;

String outgoing;              // outgoing message

byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xFF;     // address of this device
byte destination = 0xBB;      // destination to send to
byte pot1 = 0x24;
byte pot2 = 0x25;
long lastSendTime = 0;        // last send time

Thread ThreadPot1;
Thread ThreadPot2;
ThreadController cpu;
void setup() {
  Serial.begin(115200);                   // initialize serial
  while (!Serial);

  Serial.println("LoRa Duplex");

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  if (!LoRa.begin(915E6)) {             // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }
  Serial.println("LoRa init succeeded.");

  ThreadPot1.setInterval(500);
  ThreadPot1.onRun(sendPot1);
  ThreadPot2.setInterval(500);
  ThreadPot2.onRun(sendPot2);

  cpu.add(&ThreadPot1);
  cpu.add(&ThreadPot2);
}

void loop() {
  cpu.run();
}

void sendPot1(){
  Serial.print("Pot1: ");
  sendMessage(String(analogRead(0)), pot1);
}
void sendPot2(){
  Serial.print("Pot2: ");
  sendMessage(String(analogRead(2)), pot2);
}

void sendMessage(String outgoing, byte description) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(description);
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  Serial.println(outgoing + " 0x" + String(description, HEX));
}
