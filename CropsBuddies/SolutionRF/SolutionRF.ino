/* Convenient sensor's class definition */
double EC_MEASURE;
unsigned long int EC_RAW_MEASURE;
int EC_MIN_REFERENCE = 57; // Valor promedio para solución de EC 1.413 ms/cm
int EC_MAX_REFERENCE = 462; // Valor promedio para solución de EC 12.880 ms/cm
class EC {
   /*  MEASUREMENT AND MODEL RELATED VARIABLES DEFINITION  */
  double EC_MODEL_SLOPE = (12.880 - 1.413)/double(EC_MAX_REFERENCE - EC_MIN_REFERENCE);
  double EC_MODEL_COEF = 1.413 - double(EC_MIN_REFERENCE)*EC_MODEL_SLOPE;
  unsigned long int EC_SAMPLES_SUM; 
  int EC_BUFFER[10],EC_TEMPORAL; 
  /*  PHYSICAL ADDRESS RELATED VARIABLES  */  
  const int EC_PIN = A0;
  public:
    void setup() {
      pinMode(EC_PIN, INPUT);
    }

    void loop() {
    }

    void sample(String MODE) {
      for(int i=0;i<10;i++){ 
        EC_BUFFER[i]=analogRead(EC_PIN);
        delay(10);
        }
 
      for(int i=0;i<9;i++){
        for(int j=i+1;j<10;j++){
          if(EC_BUFFER[i]>EC_BUFFER[j]){
            EC_TEMPORAL=EC_BUFFER[i];
            EC_BUFFER[i]=EC_BUFFER[j];
            EC_BUFFER[j]=EC_TEMPORAL;
            }
          }
       }
       
      EC_SAMPLES_SUM=0;
      for(int i=2;i<8;i++)
      EC_SAMPLES_SUM+=EC_BUFFER[i];

      if (MODE == "CALIBRATION") {
        EC_RAW_MEASURE = EC_SAMPLES_SUM/6;
      } else if (MODE == "MEASURE") {     
        EC_MEASURE = EC_MODEL_SLOPE*(EC_SAMPLES_SUM/6) + EC_MODEL_COEF;
      }
    
    }
    
};


float PH_MEASURE;
unsigned long int PH_RAW_MEASURE;
int PH_MIN_REF = 417; // Valor promedio para solución de pH 4
int PH_MAX_REF = 310; // Valor promedio para solución de pH 7
class pH {
  /*  MEASUREMENT AND MODEL RELATED VARIABLES DEFINITION  */
  float PH_MODEL_SLOPE = 3/float(PH_MAX_REF - PH_MIN_REF);
  float PH_MODEL_COEF = 7 - float(PH_MAX_REF)*PH_MODEL_SLOPE;
  int PH_BUFFER[10],PH_TEMPORAL;
  unsigned long int PH_SAMPLES_SUM;
  /*  PHYSICAL ADDRESS RELATED VARIABLES  */  
  const int PH_PIN = A1; 
  
  public:
    void setup() {
      pinMode(PH_PIN, INPUT);
    }

       void loop() {
      //PH_MEASURE = UPDATE(); 
      }

    float sample(String MODE) {
      for(int i=0;i<10;i++){ 
        PH_BUFFER[i]=analogRead(PH_PIN);
        delay(10);
        }
 
      for(int i=0;i<9;i++){
        for(int j=i+1;j<10;j++){
          if(PH_BUFFER[i]>PH_BUFFER[j]){
            PH_TEMPORAL=PH_BUFFER[i];
            PH_BUFFER[i]=PH_BUFFER[j];
            PH_BUFFER[j]=PH_TEMPORAL;
            }
          }
      }
      
      PH_SAMPLES_SUM=0;
      for(int i=2;i<8;i++)
      PH_SAMPLES_SUM+=PH_BUFFER[i];
 
      /* OPCIONES DE MEDICIÓN */
      float PH_MODEL_OUTPUT = PH_MODEL_SLOPE*(PH_SAMPLES_SUM/6) + PH_MODEL_COEF;
      /* OPCIONES DE CALIBRACIÓN */
       //float PH_MODEL_OUTPUT = (PH_SAMPLES_SUM/6);
      
      if (MODE == "CALIBRATION") {
        PH_RAW_MEASURE = PH_SAMPLES_SUM/6;
      } else if (MODE == "MEASURE") {     
        PH_MEASURE = PH_MODEL_OUTPUT;
      }
    }

};

/* Includes */

#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include "nRF24L01.h"

/* UI related initialiations and definitions */

const uint8_t RF_MODE = 0x00;
const uint8_t PH_CAL_MODE = 0x01;
const uint8_t EC_CAL_MODE = 0x02;
volatile uint8_t CURRENT_MODE = RF_MODE;

int EC_LED = 2;
int PH_LED = 3;
int MODE_BTN = 4;
int LOW_BTN = 5;
int HIGH_BTN = 6;

/* RF and sensors related initialiations and definitions */

RF24 radio(8,10);
EC EC;
pH pH;

uint8_t address[][6] = {"HNode", "PNode", "SNode", "DNode", "MNode"};
int radioNumber = 3; // 0 usa address[0] para transmitir, 1 usa address[1] para transmitir
int masterNumber = 4;
String state;
char beacon[100];

/* Time managment variables and constants */

const unsigned long samplingInterval = 5000;
unsigned long currentMillis;
unsigned long referenceMillis;

void setup() {
  // put your setup code here, to run once: 
  Serial.begin(115200);

  UIsetup(); 
  
  pH.setup();
  EC.setup();
  
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A3,INPUT);

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  
  if (!radio.begin()) {
    Serial.println("Radio hardware error");
    while (1) {} // hold in infinite loop
  }
  radioSetup();

  toTx();
}

void loop() {
  // put your main code here, to run repeatedly:
 currentMillis = millis();
 if (digitalRead(MODE_BTN)) {
    if (CURRENT_MODE == RF_MODE) {
      CURRENT_MODE+=0x01;
      digitalWrite(PH_LED, HIGH);
    } else if (CURRENT_MODE == PH_CAL_MODE) {
      CURRENT_MODE+=0x01;
      digitalWrite(PH_LED, LOW);
      digitalWrite(EC_LED, HIGH);
    } else if (CURRENT_MODE == EC_CAL_MODE) {
      CURRENT_MODE=0x00;
      digitalWrite(EC_LED, LOW);
    }
  } else {
    if (CURRENT_MODE == RF_MODE) {
      // RF_MODE Operation
      RF_MODE_ROUTINE();
    } else if (CURRENT_MODE == PH_CAL_MODE) {
      if (digitalRead(LOW_BTN)) {
        Serial.println("Setting low pH reference");
        setPhL();
      } else if (digitalRead(HIGH_BTN)) {
        Serial.println("Setting high pH reference");
        setPhH();
      }
    } else if (CURRENT_MODE == EC_CAL_MODE) {
      if (digitalRead(LOW_BTN)) {
        Serial.println("Setting low EC reference");
        setEcL();
      } else if (digitalRead(HIGH_BTN)) {
        Serial.println("Setting high EC reference");
        setEcH();
      }
    }  
  }
  delay(1000);
  /*if (radio.available()) {
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
  }*/ 
}

void RF_MODE_ROUTINE(void) {
  if (currentMillis - referenceMillis > samplingInterval) {
    EC.sample("MEASURE");
    pH.sample("MEASURE");
    toTx();
    dataIn();
    beaconOut();
    referenceMillis = millis();  
  }
}

/* UI related functions */

void UIsetup(void) {
  pinMode(EC_LED, OUTPUT);
  pinMode(PH_LED, OUTPUT);

  pinMode(MODE_BTN, INPUT);
  pinMode(LOW_BTN, INPUT);
  pinMode(HIGH_BTN, INPUT);

  digitalWrite(EC_LED, LOW);
  digitalWrite(PH_LED, LOW);
  digitalWrite(MODE_BTN, LOW);
  digitalWrite(LOW_BTN, LOW);
  digitalWrite(HIGH_BTN, LOW);
}

void toggle(int pin, int d) {
  int toggler = 0;
  for (int i = 0; i < 10; i++) {
    if (toggler == 0) {
      digitalWrite(pin, HIGH);
      toggler = 1;
    } else {
      digitalWrite(pin, LOW);
      toggler = 0; 
    }
    delay(d);
  }  
}

void setPhH(void) {
  toggle(PH_LED, 100);
  pH.sample("CALIBRATION");
  PH_MAX_REF = PH_RAW_MEASURE;
  digitalWrite(PH_LED, HIGH);
}
  
void setPhL(void) {
  toggle(PH_LED, 250);
  pH.sample("CALIBRATION");
  PH_MIN_REF = PH_RAW_MEASURE;
  digitalWrite(PH_LED, HIGH);
}

void setEcH(void) {
  toggle(EC_LED, 100);
  EC.sample("CALIBRATION");
  EC_MAX_REFERENCE = EC_RAW_MEASURE;
  digitalWrite(EC_LED, HIGH);
}

void setEcL(void) {
  toggle(EC_LED, 250);
  EC.sample("CALIBRATION");
  EC_MIN_REFERENCE = EC_RAW_MEASURE;
  digitalWrite(EC_LED, HIGH);
}

/* Radio link related functions */

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
  radio.openWritingPipe(address[3]);
}

unsigned int wait(int radioNumber) {
  unsigned int wait;
  wait = 1000 + 1000*radioNumber;
  return wait;
}

void dataIn() {
  state = "S:";
  state+=  "pH=" + String(PH_MEASURE) + ","; 
  state+=  "EC=" + String(EC_MEASURE);
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
  radio.openWritingPipe(address[3]);
  // Apertura del pipe de lectura: "leo en el pipe de escritura de la otra radio"
  radio.openReadingPipe(1, address[masterNumber]);
  // Dependiendo del tipo de nodo, preparo para primera escritura o próxima lectura
  radio.startListening();  // put radio in TX mode
}
