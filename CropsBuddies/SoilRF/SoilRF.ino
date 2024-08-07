#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include "nRF24L01.h"

RF24 radio(8,10);

uint8_t address[][6] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05}; 
bool radioNumber = 0; // 0 usa address[0] para transmitir, 1 usa address[1] para transmitir
String state;
char beacon[100];

void setup() {
  // put your setup code here, to run once: 
  Serial.begin(115200);
  
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  
  if (!radio.begin()) {
    Serial.println("Radio hardware error");
    while (1) {} // hold in infinite loop
  }
  radioSetup();
}

void loop() {
    // put your main code here, to run repeatedly:
  if (radio.available()) {
    radio.read(&beacon, sizeof(beacon));
    String state = String(beacon);
    Serial.println(beacon);
    if (String('M')==state.substring(0,1)) {
      unsigned int currentMillis = millis();
      unsigned int referenceMillis = currentMillis;
      while(currentMillis - referenceMillis < wait(radioNumber)) {
      }
      toTx();
      dataIn();
      beaconOut(); 
    }
  }
}

void toRx() {
  // Transición a estado RX
  radio.txStandBy();
  radio.flush_tx();
  radio.openReadingPipe(1, address[!radioNumber]);
  radio.startListening();
}

void toTx() {
  // Retorno a estado TX
  radio.flush_rx();
  radio.closeReadingPipe(1);
  radio.stopListening();
  radio.openWritingPipe(address[radioNumber]);
}

unsigned int wait(int radioNumber) {
  unsigned int wait;
  wait = 1000 + 1000*radioNumber;
  return wait;
}

void dataIn() {
  state = "H:";
  state+=  "H1=" + String(analogRead(A0)) + ","; 
  state+=  "H2=" + String(analogRead(A1));
  Serial.println(state);
}

void beaconOut() {
  state.toCharArray(beacon, sizeof(beacon));
  radio.write(&beacon, sizeof(beacon));
}

void radioSetup() {
  // Establecer configuración de potencia de la radio
  radio.setPALevel(RF24_PA_HIGH); // {RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX}, RF24_PA_MAX is default.
  // Configuraciones adicionales para deshabilitar elementos del protocolo ESB por defecto
  radio.setDataRate(RF24_2MBPS); // Se fija la máxima tasa de transmisión posible
  // Se establece el tamaño de payload justo y necesario
  radio.setPayloadSize(sizeof(beacon));
  // Apertura del pipe de escritura: "escribo en mi propio pipe"
  radio.openWritingPipe(address[radioNumber]);
  // Apertura del pipe de lectura: "leo en el pipe de escritura de la otra radio"
  radio.openReadingPipe(1, address[!radioNumber]);
  // Dependiendo del tipo de nodo, preparo para primera escritura o próxima lectura
  radio.startListening();  // put radio in TX mode
}
