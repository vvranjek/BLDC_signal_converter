#include <Arduino.h>

#define PIN_RC_PWM_IN PB2
#define PIN_DIR_OUT_F PB4
#define PIN_DIR_OUT_R PB3
#define PIN_PWM_ANALOG_OUT PB0
#define PIN_LED PB1
#define DEADZONE_US 40

unsigned long RC_ts;
unsigned long PWM_time;
float speed = 0.0;

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_RC_PWM_IN, INPUT);
  pinMode(PIN_DIR_OUT_F, OUTPUT);
  pinMode(PIN_DIR_OUT_R, OUTPUT);
  pinMode(PIN_PWM_ANALOG_OUT, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
}

void led_on() {
  digitalWrite(PIN_LED, HIGH);
}

void led_off() {
  digitalWrite(PIN_LED, LOW);
}


void loop() {
/*
  while (1) {

    //Fading the LED
    for(int i=0; i<255; i++){
      analogWrite(PWM_ANALOG_OUT, i);
      delay(20);
    }
    for(int i=255; i>0; i--){
      analogWrite(PWM_ANALOG_OUT, i);
      delay(20);
    }
  }
*/



  // put your main code here, to run repeatedly:

  if (digitalRead(PIN_RC_PWM_IN)) {
    led_on();

    //analogWrite(PWM_ANALOG_OUT, 100);


    RC_ts = micros();
    while(digitalRead(PIN_RC_PWM_IN)) {}
    
    PWM_time = micros()-RC_ts;
    int16_t PWM_value = (int16_t)1500-PWM_time;
    
    if (abs(PWM_value) < DEADZONE_US) {
      speed = 0;
    }
    else {
      speed = (float)((PWM_value-DEADZONE_US)/400.0);
    }

    if (speed > 1.0) speed = 1.0;
    if (speed < -1.0) speed = -1.0;



    led_off();
  }



  if (speed > 0.0) {
    digitalWrite(PIN_DIR_OUT_F, HIGH);
    digitalWrite(PIN_DIR_OUT_R, LOW);
  }
  else {
    digitalWrite(PIN_DIR_OUT_F, LOW);
    digitalWrite(PIN_DIR_OUT_R, HIGH);
  }

  analogWrite(PIN_PWM_ANALOG_OUT, (uint8_t)(fabs(speed)*255));


  
}

