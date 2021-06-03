#include <Arduino.h>

#define PIN_RC_PWM_IN       PD2
#define PIN_DIR_OUT_F       PD3
#define PIN_DIR_OUT_R       PD4
#define PIN_BRAKE           PD5
#define PIN_PWM_ANALOG_OUT  PD6
#define PIN_LED             PB5
#define PIN_DEADZONE_ADJ    PC0
#define PIN_TRIM_ADJ        PC1
#define DEADZONE_US         50
#define PWM_MAX_PERIOD      100

unsigned long RC_ts;
unsigned long Safety_ts;
float speed = 0.0;
int16_t PWM_value = 0;
volatile unsigned long PWM_time = 0;
volatile unsigned long PWM_rise_time = 0;
bool PWM_complete = false;
bool failsafe = false;
uint16_t deadzone_adjust = 0;
int16_t trim_adjust = 0;

void PWM_rising();
void PWM_falling();
void led_on();
void led_off();
void brake_set();
void brake_release();

void setup() {
    // put your setup code here, to run once:
    pinMode(PIN_RC_PWM_IN, INPUT);
    pinMode(PIN_DEADZONE_ADJ, INPUT);
    pinMode(PIN_TRIM_ADJ, INPUT);
    pinMode(PIN_DIR_OUT_F, OUTPUT);
    pinMode(PIN_DIR_OUT_R, OUTPUT);
    pinMode(PIN_PWM_ANALOG_OUT, OUTPUT);
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_BRAKE, OUTPUT);

    attachInterrupt(0, PWM_rising, RISING);
}

// Raising PWM interrupt
void PWM_rising() {
    PWM_rise_time = micros();
    attachInterrupt(0, PWM_falling, FALLING);

    led_on();
}

// Falling PWM interrupt
void PWM_falling() {
    PWM_time = micros() - PWM_rise_time;
    attachInterrupt(0, PWM_rising, RISING);

    led_off();
    PWM_complete = true;
}

void loop() {

    // Get pot readings 
    deadzone_adjust = analogRead(PIN_DEADZONE_ADJ)/6.82; // 0-150
    trim_adjust = (analogRead(PIN_TRIM_ADJ) - 512)/3.41; // +-150

    if (PWM_complete) {
        PWM_complete = false;
        Safety_ts = millis();

        // Sanity check (PWM time)
        if (PWM_time < 500 || PWM_time > 2500) {
            PWM_value = 0;
            led_on();
        }
        // Get PWM value (-1000...1000)
        else {
            PWM_value = (int16_t)1500 - PWM_time + trim_adjust;
        }

        // Deadzone
        if (abs(PWM_value) < deadzone_adjust) {
            speed = 0;
            brake_set();
        } else {
            speed = (float)((PWM_value - deadzone_adjust) / 400.0);
            brake_release();
        }

        // Speed limit
        if (speed > 1.0)
            speed = 1.0;
        if (speed < -1.0)
            speed = -1.0;
    }

    // Safety/failsafe
    if (millis() - Safety_ts > PWM_MAX_PERIOD) {
      failsafe = true;
    }
    else {
      failsafe = false;
    }

    // Set outputs
    if (!failsafe) {
      if (speed > 0.0) {
        digitalWrite(PIN_DIR_OUT_F, HIGH);
        digitalWrite(PIN_DIR_OUT_R, LOW);
      } else {
          digitalWrite(PIN_DIR_OUT_F, LOW);
          digitalWrite(PIN_DIR_OUT_R, HIGH);
      }

      analogWrite(PIN_PWM_ANALOG_OUT, (uint8_t)(fabs(speed) * 255));
    }
    else {
      analogWrite(PIN_PWM_ANALOG_OUT, 0);
      brake_set();
      led_on();
    }

}

void brake_set() {
    digitalWrite(PIN_BRAKE, LOW);
}

void brake_release() {
    digitalWrite(PIN_BRAKE, HIGH);
}

void led_on() {
    digitalWrite(PIN_LED, HIGH);
}

void led_off() {
    digitalWrite(PIN_LED, LOW);
}