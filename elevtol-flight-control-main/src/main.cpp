#include <Arduino.h>
#include <Wire.h>
#include <PulsePosition.h>

// Setup
#define built_in_LED 13
#define start_calibration_LED 41
#define end_calibration_LED 40
#define buzzer_pin 8

// Reading Receiver Input
PulsePositionInput receiver_input;
int num_of_channels = 0;
float receiver_values[] = {0,0,0,0,0,0,0,0};

void test_print();
void setup_feedback();
void read_receiver_input();
void print_receiver_input();
void throttle_safety();

void setup() {
  Serial.begin(57600);

  // Setup
  pinMode(built_in_LED, OUTPUT);
  pinMode(start_calibration_LED, OUTPUT);
  pinMode(end_calibration_LED, OUTPUT);

  // Receiver Setup
  // Available pin on Teensy 4.1 for PPM : 6, 9, 10, 11, 12, 13, 14, 15, 18, 19
  receiver_input.begin(14);

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

void read_receiver_input() {
  num_of_channels = receiver_input.available();
  if (num_of_channels > 0) {
    for (int i = 0; i <= num_of_channels; i++) {
      receiver_values[i] = receiver_input.read(i);
    }
  }
}

void print_receiver_input() {
  Serial.print("Roll: ");
  Serial.print(receiver_values[0]);
  Serial.print("  Pitch: ");
  Serial.print(receiver_values[1]);
  Serial.print("  Roll: ");
  Serial.print(receiver_values[2]);
  Serial.print("  Yaw: ");
  Serial.println(receiver_values[3]);
  delay(50);
}

void throttle_safety() { 
  do {
    read_receiver_input();
    delay(10);
  } while(receiver_values[2] < 1020); // check throttle value if below safety position
}