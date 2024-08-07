class PeristalticPump {
  private:
    byte a_pin;
    byte b_pin;
    byte stby_pin;
    byte pwm_pin;
    unsigned long interval;
    unsigned long currentPumpMillis; 
    unsigned long lastPumpMillis;
  public:      
    PeristalticPump(byte a_pin, byte b_pin, int stby_pin, int pwm_pin, unsigned long interval) {
      this->a_pin = a_pin;
      this->b_pin = b_pin;
      this->stby_pin = stby_pin;
      this->pwm_pin = pwm_pin;
      this->interval = interval;
      init();
    }

    void init() {
      pinMode(a_pin, OUTPUT);
      pinMode(b_pin, OUTPUT);
      pinMode(stby_pin, OUTPUT);
      pinMode(pwm_pin, OUTPUT);
      off();
    }

    void off() {
      digitalWrite(a_pin, LOW);
      digitalWrite(b_pin, LOW);
      analogWrite(stby_pin, 0);
      analogWrite(pwm_pin, 0);
    }

    void wakeUp() {
      digitalWrite(stby_pin, HIGH);
    }

    void sleep() {
      digitalWrite(stby_pin, LOW);
    }

    void pumpCycle() {
      digitalWrite(a_pin, HIGH);
      digitalWrite(b_pin, LOW);
      int value = 0;
      int max_value = 200;   
      /*  ACCELERATION REGIME  */
      while (value < max_value) { //ACELERANDO  
        analogWrite(pwm_pin, value);
        
        lastPumpMillis = 0;
        currentPumpMillis = millis();
        while(lastPumpMillis < 5) {
          lastPumpMillis = millis() - currentPumpMillis;
        }
        
        value = value + 5;
      }
      /*  STATIONARY REGIME  */
      analogWrite(pwm_pin, max_value);
      
      lastPumpMillis = 0;
      currentPumpMillis = millis();
      while(lastPumpMillis < interval) {
         lastPumpMillis = millis() - currentPumpMillis;
      }

      /*  DECELERATION REGIME  */
      for (int x = 0; x <= max_value - 1; x++) {
        value--;
        analogWrite(pwm_pin, value);
        
        lastPumpMillis = 0;
        currentPumpMillis = millis();
        while(lastPumpMillis < 5) {
          lastPumpMillis = millis() - currentPumpMillis;
        }  
      }
      analogWrite(pwm_pin, 0);
      off();
    }
};

/* PERISTALTIC PUMPS PINS */
#define STBY_1 A4

#define PAI1_1_PIN 2
#define PAI2_1_PIN 3
#define PWMA_1 A0

#define PBI1_2_PIN 4
#define PBI2_2_PIN 5
#define PWMB_1 A3

#define STBY_2 A5

#define PAI1_3_PIN 6
#define PAI2_3_PIN 7
#define PWMA_2 A2

#define PBI1_4_PIN 0
#define PBI2_4_PIN 1
#define PWMB_2 A1

/* TIME CONSTANTS AND REFERENCES */
const unsigned long PUMP_INTERVAL = 1000; //2 segundos

/* PERISTALTIC PUMPS INITIALIZATION */
PeristalticPump PUMP_PH_UP(PAI1_1_PIN, PAI2_1_PIN, STBY_1, PWMA_1, PUMP_INTERVAL); //PH
PeristalticPump PUMP_PH_DOWN(PBI1_2_PIN, PBI2_2_PIN, STBY_2, PWMB_1, PUMP_INTERVAL); //EC
PeristalticPump PUMP_CALMAG(PAI1_3_PIN, PAI2_3_PIN, STBY_2, PWMA_2, PUMP_INTERVAL);
PeristalticPump PUMP_NUTRI(PBI1_4_PIN, PBI2_4_PIN, STBY_1, PWMB_2, PUMP_INTERVAL);

#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include "nRF24L01.h"
#include <stdlib.h>

RF24 radio(8,10);

double EC;
double pH;
bool pHCTRL = false;
bool ECCTRL = false;

unsigned long currentMillis;
unsigned long pHReferenceMillis = 0;
unsigned long ECReferenceMillis = 0;
const unsigned long MIX_TIME = 30000;

uint8_t address[][6] = {"HNode", "PNode", "SNode", "DNode", "MNode"};
bool radioNumber = 1; // 0 usa address[0] para transmitir, 1 usa address[1] para transmitir
char beacon[100];
int masterNumber;

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(115200);
  if (!radio.begin()) {
    //Serial.print("Hardware error");
    while (1) {} // hold in infinite loop
  }
  radioSetup();
  PUMP_PH_UP.init();
  PUMP_PH_DOWN.init();
  PUMP_CALMAG.init();
  PUMP_NUTRI.init();
}

void loop() {
  // put your main code here, to run repeatedly:
  currentMillis = millis();
  pH=-1;
  EC=-1;  
  if (radio.available()) {
    radio.read(&beacon, sizeof(beacon));
    String state = String(beacon);
    //Serial.println(beacon);
    if (String('S')==state.substring(0,1)) { 
    }
    
  // Get pH value
  int startIndex;
  int stopIndex;
  for (int i = 0; i < state.length(); i++) {
    //Serial.println(state.substring(i, i + 3));
    if(state.substring(i, i + 3) == "pH=") {
       /*Serial.println("Found!");
       Serial.println(state.substring(i, i + 3));*/
       startIndex = i + 3;
    }
    if(state.substring(i, i + 1) == ",") {
       /*Serial.println("Found!");
       Serial.println(state.substring(i, i + 1));*/
       stopIndex = i;
    }
  }
  //Serial.println("pH = " + String(state.substring(startIndex, stopIndex)));
  pH = state.substring(startIndex, stopIndex).toDouble();
  // Get EC value
  for (int i = 0; i < state.length(); i++) {
    //Serial.println(state.substring(i, i + 3));
    if(state.substring(i, i + 3) == "EC=") {
       /*Serial.println("Found!");
       Serial.println(state.substring(i, i + 3));*/
       startIndex = i + 3;
    }
  }
  //Serial.println("EC = " + String(state.substring(startIndex)));
  EC = state.substring(startIndex).toDouble();
  
    /*if (String('M')==state.substring(0,1)) {
      unsigned int currentMillis = millis();
      unsigned int referenceMillis = currentMillis;
      while(currentMillis - referenceMillis < wait(radioNumber)) {
        if(radio.available()) {
          radio.read(&beacon, sizeof(beacon));
          String msg = String(beacon);
          if (String('S')==msg.substring(0,1)) {
            pH = atof(msg.substring(5,8).c_str());
            EC = atof(msg.substring(12,15).c_str()); 
          }
        }
      }
    }*/
  }
  //Serial.println("pH : " + String(pH) + ", " + "EC : " + String(EC));
  /*pH = 5.5;
  EC = 1300;*/
  periCTRLRoutine();
  //pumpTest();
  delay(3000);
}

void periCTRLRoutine() {
  if (EC==-1 && pH == -1) {
    //Todavía no hay datos de los sensores
    //Serial.println("No hay nuevo dato");
  } else {
    //Serial.println("Hay nuevo dato");
    // Accionamiento de peristálticas
    //Serial.println("Bombas peristálticas");
    if ((pH - 6.0) >= 0) {
      if(currentMillis - pHReferenceMillis > MIX_TIME) { //Aplicación solo cada 30 segundos
        //Serial.println("pH > 6: bombeando");
        PUMP_PH_DOWN.wakeUp();
        PUMP_PH_DOWN.pumpCycle();
        PUMP_PH_DOWN.off();
        pHReferenceMillis = millis();
        pHCTRL = false;
      } else {
        //Serial.println("Esperando que se asiente la solución");
      }
    } else if ((pH - 5.0) <= 0) {
      if(currentMillis - pHReferenceMillis > MIX_TIME) {
        PUMP_PH_UP.wakeUp();
        PUMP_PH_UP.pumpCycle();
        PUMP_PH_UP.off();
        pHReferenceMillis = millis();
        pHCTRL = false;
      } else {
        
      }
    } else {
      pHCTRL = true;
    }
    
    //pHCTRL=true;
    if (pHCTRL) {
      //Serial.println("Control EC");
      delay(1000);
      if ((EC - 1440) <= 0) {
        //Serial.println(currentMillis - ECReferenceMillis);
        if(currentMillis - ECReferenceMillis > MIX_TIME) { //Aplicación solo cada 30 segundos
          //Serial.println("Pump Nutri");
          PUMP_NUTRI.wakeUp();
          PUMP_NUTRI.pumpCycle();
          PUMP_NUTRI.off();
          ECReferenceMillis = millis();
          ECCTRL = false;
        } else {
          //Serial.println("Esperando que se asiente la solución");
        }
      } else if ((EC - 1700) >= 0) {
        cleanRoutineRequest();
        ECCTRL = false;
      } else {
        ECCTRL = true;
      }
    }
  }
}

void cleanRoutineRequest() {
  toTx();
  String state = "D:R:CLEAN";
  state.toCharArray(beacon, sizeof(beacon));
  radio.write(&beacon, sizeof(beacon));
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

void pumpTest() {
  PUMP_NUTRI.wakeUp();
  PUMP_NUTRI.pumpCycle();
  PUMP_NUTRI.off();
  delay(1000);
  PUMP_PH_UP.wakeUp();
  PUMP_PH_UP.pumpCycle();
  PUMP_PH_UP.off();
  delay(1000);
  PUMP_PH_DOWN.wakeUp();
  PUMP_PH_DOWN.pumpCycle();
  PUMP_PH_DOWN.off();
  delay(1000);
  PUMP_CALMAG.wakeUp();
  PUMP_CALMAG.pumpCycle();
  PUMP_CALMAG.off();
  delay(1000);
}

void radioSetup() {
  // Establecer configuración de potencia de la radio
  radio.setPALevel(RF24_PA_HIGH); // {RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX}, RF24_PA_MAX is default.
  // Configuraciones adicionales para deshabilitar elementos del protocolo ESB por defecto
  radio.setDataRate(RF24_2MBPS); // Se fija la máxima tasa de transmisión posible
  // Se establece el tamaño de payload justo y necesario
  radio.setPayloadSize(sizeof(beacon));
  // Apertura del pipe de escritura: "escribo en mi propio pipe"
  radio.openWritingPipe(address[4]);
  // Apertura del pipe de lectura: "leo en el pipe de escritura de la otra radio"
  radio.openReadingPipe(1, address[3]);
  // Dependiendo del tipo de nodo, preparo para primera escritura o próxima lectura
  radio.startListening();  // put radio in TX mode
}
