#include <Arduino.h>
#include <Wire.h>

// Setup
#define built_in_LED 13
#define start_calibration_LED 41
#define end_calibration_LED 40
#define buzzer_pin 8

// Battery Monitoring
float measured_voltage;
float total_battery_voltage;
float average_voltage_per_cell;
#define num_of_cell 3
#define battery_monitor_pin 15


void test_print();
void setup_feedback();
void check_battery_voltage();

void setup() {
  Serial.begin(57600);

  pinMode(built_in_LED, OUTPUT);
  pinMode(start_calibration_LED, OUTPUT);
  pinMode(end_calibration_LED, OUTPUT);
  pinMode(battery_monitor_pin, INPUT);

  // setup_feedback();
}

void loop() {
  check_battery_voltage();
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

void check_battery_voltage() {
  measured_voltage = analogRead(battery_monitor_pin);
  measured_voltage = measured_voltage / 1023 * 3.3;
  Serial.print("Measured Voltage: ");
  Serial.print(measured_voltage);
  total_battery_voltage = measured_voltage * 5;
  Serial.print("    Total Battery Voltage: ");
  Serial.print(total_battery_voltage);
  average_voltage_per_cell = total_battery_voltage / num_of_cell;
  Serial.print("    Average Voltage Per Cell: ");
  Serial.println(average_voltage_per_cell);
}

