#include <Arduino.h>
#include <PulsePosition.h>

PulsePositionInput input(RISING);

float input_throttle; // Between 1000 ms and 2000 ms
float value[] = {0, 0, 0, 0, 0, 0, 0, 0};
int num_of_channel = 4;

void read_receiver();

void setup() {
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  input.begin(14);
  analogWriteFrequency(1, 250); // Set PWM frequency on pin 9 to 1 kHz
  analogWriteResolution(12); // Set the resolution to 12 bits
  delay(250);

  // Ensure the throttle stick is at the bottom position (for safety purposes)
  while (value[2] < 1020 || value[2] > 10150) {
    read_receiver();
    delay(10);
  }
}

void loop() {
  read_receiver();
  input_throttle = value[2];
  analogWrite(1, 1024 * input_throttle);
}

void read_receiver() {
  num_of_channel = input.available();
  if (num_of_channel > 0) {
    for (int i = 0; i <= num_of_channel; i++) {
      value[i] = input.read(i);
    }
  }
}
