#include <Arduino.h>
#include <PulsePosition.h>

PulsePositionInput input(RISING);

float value[] = {0, 0, 0, 0, 0, 0, 0, 0};
int num_of_channel = 4;

void read_receiver();

void setup() {
  Serial.begin(57600);
  input.begin(14);
  // Available pin on Teensy 4.1 for PPM : 6, 9, 10, 11, 12, 13, 14, 15, 18, 19
}

void loop() {
  read_receiver();
  Serial.print("Roll: ");
  Serial.print(value[0]);
  Serial.print("  Pitch: ");
  Serial.print(value[1]);
  Serial.print("  Roll: ");
  Serial.print(value[2]);
  Serial.print("  Yaw: ");
  Serial.println(value[3]);
  delay(50);
}

void read_receiver() {
  num_of_channel = input.available();
  if (num_of_channel > 0) {
    for (int i = 0; i <= num_of_channel; i++) {
      value[i] = input.read(i);
    }
  }
}

