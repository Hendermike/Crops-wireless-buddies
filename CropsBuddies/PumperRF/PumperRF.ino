class Relay {
  private:
    byte pin;
  public:
    Relay(byte pin) {
      this->pin = pin;
      init();
    }

    void init() {
      pinMode(pin, OUTPUT);
      off();
    }

    void on() {
      digitalWrite(pin, LOW); 
    }

    void off(){
      digitalWrite(pin, HIGH);
    }
};


long DURATION;
int DISTANCE;
float US_MEASURE = 0;
float US_MIN_REFERENCE;
float US_MAX_REFERENCE;
class Ultrasonido {
  // defines constant
  private:
  byte trig_pin;
  byte echo_pin;
  float us_ref;
  public:
    Ultrasonido(byte trig_pin, byte echo_pin) {
      this->trig_pin = trig_pin;
      this->echo_pin = echo_pin;
    }
    void setup() {
      pinMode(trig_pin, OUTPUT); // Sets the TRIG_PIN as an Output
      pinMode(echo_pin, INPUT);
    }

    void loop() {
    }

    float distancia() {
    // Clears the TRIG_PIN
    digitalWrite(trig_pin, LOW);
    delayMicroseconds(2);

    DURATION = 0;
    //for (int i=0; i<5; i++) {
      // Sets the TRIG_PIN on HIGH state for 10 micro seconds
      digitalWrite(trig_pin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trig_pin, LOW);
      // Reads the ECHO_PIN, returns the sound wave travel time in microseconds
      DURATION = pulseIn(echo_pin, HIGH);
    //}
    //DURATION = DURATION/5; 
    
    // Calculating the DISTANCE
    float d = DURATION*0.034/2;
    return d;
    }

    void sample() {
      //float n = US_REF - distancia();
      US_MEASURE = distancia();  
    }
    
};

/* RELAY PINS */
#define WATER_IN_PIN 3
#define WATER_OUT_PIN 4
#define WATER_CYCLE_PIN 5
/* ULTRASONIC PINS */
#define TRIG_PIN 9
#define ECHO_PIN 8

Relay WATER_IN_CHANNEL(WATER_IN_PIN);
Relay WATER_OUT_CHANNEL(WATER_OUT_PIN);
Relay WATER_CYCLE_CHANNEL(WATER_CYCLE_PIN);

Ultrasonido US(TRIG_PIN, ECHO_PIN);

#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include "nRF24L01.h"

RF24 radio(8,10);

uint8_t address[][6] = {"1Node", "2Node"};
bool radioNumber = 1; // 0 usa address[0] para transmitir, 1 usa address[1] para transmitir
char beacon[100];
int masterNumber;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  if (!radio.begin()) {
    while (1) {} // hold in infinite loop
  }
  radioSetup();
  WATER_IN_CHANNEL.init();
  WATER_OUT_CHANNEL.init();
  WATER_OUT_CHANNEL.init();
  US.setup();
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
        if(radio.available()) {
          radio.read(&beacon, sizeof(beacon));
          String msg = String(beacon);
          if (String('H')==msg.substring(0,1)) {
            int H1 = msg.substring(5,8).toInt();
            int H2 = msg.substring(12,15).toInt();
          } else if (String('S')==msg.substring(0,1)) {
            int pH = msg.substring(5,8).toInt();
            int EC = msg.substring(12,15).toInt(); 
          }
        }
      }
    }
  }

  // Accionamiento de bombas
  
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
  radio.openReadingPipe(1, address[masterNumber]);
  // Dependiendo del tipo de nodo, preparo para primera escritura o próxima lectura
  radio.startListening();  // put radio in TX mode
}
