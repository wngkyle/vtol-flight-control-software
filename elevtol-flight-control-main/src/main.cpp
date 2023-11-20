#include <Arduino.h>
#include <Wire.h>

// Setup
#define built_in_LED 13
#define start_calibration_LED 41
#define end_calibration_LED 40
#define buzzer_pin 8

void test_print();
void setup_feedback();
void check_battery_voltage();

void setup() {
  Serial.begin(57600);

  pinMode(built_in_LED, OUTPUT);
  pinMode(start_calibration_LED, OUTPUT);
  pinMode(end_calibration_LED, OUTPUT);

  // setup_feedback();
}

void loop() {
  delay(500);
}

void test_print() {
  Serial.println("Hello World!");
  delay(500);
}

void setup_feedback() {
  digitalWrite(built_in_LED, HIGH);
  digitalWrite(start_calibration_LED, HIGH);

  delay(4000);

  digitalWrite(start_calibration_LED, LOW);
  digitalWrite(end_calibration_LED, HIGH);

  tone(buzzer_pin, 1050);
  delay(300);
  noTone(buzzer_pin);
  delay(250);
  tone(buzzer_pin, 1050);
  delay(300);
  noTone(buzzer_pin);
}

