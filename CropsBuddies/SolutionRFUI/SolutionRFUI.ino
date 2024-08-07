#include <SPI.h>
#include "printf.h"

const uint8_t RF_MODE = 0x00;
const uint8_t PH_CAL_MODE = 0x01;
const uint8_t EC_CAL_MODE = 0x02;
volatile uint8_t CURRENT_MODE = RF_MODE;

int EC_LED = 2;
int PH_LED = 3;
int MODE_BTN = 4;
int LOW_BTN = 5;
int HIGH_BTN = 6;

void setup() {
  // put your setup code here, to run once: 
  Serial.begin(115200);
  
  UIsetup();
  
}

void loop() {
  // put your main code here, to run repeatedly:
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
    delay(1000);
  } else {
    if (CURRENT_MODE == RF_MODE) {
      // RF_MODE Operation
    } else if (CURRENT_MODE == PH_CAL_MODE) {
      if (digitalRead(LOW_BTN)) {
        Serial.println("Setting low pH reference");
        setPhL();
      } else if (digitalRead(HIGH_BTN)) {
        Serial.println("Setting high pH reference");
        setPhH();
      }
      delay(1000);
    } else if (CURRENT_MODE == EC_CAL_MODE) {
      if (digitalRead(LOW_BTN)) {
        Serial.println("Setting low EC reference");
        setEcL();
      } else if (digitalRead(HIGH_BTN)) {
        Serial.println("Setting high EC reference");
        setEcH();
      }
      delay(1000);
    }
      
  }
}

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
  digitalWrite(PH_LED, HIGH);
}
  
void setPhL(void) {
  toggle(PH_LED, 250);
  digitalWrite(PH_LED, HIGH);
}

void setEcH(void) {
  toggle(EC_LED, 100);
  digitalWrite(EC_LED, HIGH);
}

void setEcL(void) {
  toggle(EC_LED, 250);
  digitalWrite(EC_LED, HIGH);
}
