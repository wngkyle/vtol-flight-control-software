#include <Arduino.h>

#define built_in_LED 13
#define start_calibration_LED 41
#define end_calibration_LED 40
#define buzzer_pin 8
#define battery_monitor_pin 15

float voltage_per_cell;
float total_battery_voltage;

void setup_feedback();
void check_battery_voltage();

void setup() {
  pinMode(built_in_LED, OUTPUT);
  pinMode(start_calibration_LED, OUTPUT);
  pinMode(end_calibration_LED, OUTPUT);
  pinMode(battery_monitor_pin, INPUT);

  setup_feedback();
}

void loop() {
 
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

void check_battery_voltage() {
  voltage_per_cell = analogRead(battery_monitor_pin);
  voltage_per_cell = voltage_per_cell / 1023 * 3.3;
  total_battery_voltage = voltage_per_cell * 5;
  Serial.print("Total battery voltage: ");
  Serial.println(total_battery_voltage);
  Serial.print("Cell voltage: ");
  Serial.println(voltage_per_cell);
}