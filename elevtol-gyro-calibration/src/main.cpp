#include <Arduino.h>
#include <Wire.h>

#define calibration_loop 2000

float rate_roll, rate_pitch, rate_yaw;
float rate_calibration_roll, rate_calibration_pitch, rate_calibration_yaw;
float calibration_num;

void gyro_signals(void) {
  // Setting low pass filter for filtering high frequency signals
  Wire.beginTransmission(0x68); 
  Wire.write(0x1A); // Low pass filter
  Wire.write(0x05);
  Wire.endTransmission();

  // Setting the sensitivty scale factor
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  // Preparing to read sensor output
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);

  int16_t gyro_x = Wire.read() << 8 | Wire.read(); 
  int16_t gyro_y = Wire.read() << 8 | Wire.read();
  int16_t gyro_z = Wire.read() << 8 | Wire.read();

  rate_roll = (float) gyro_x / 65.5;
  rate_pitch = (float) gyro_y / 65.5;
  rate_yaw = (float) gyro_z / 65.5;
}

void setup() {
  for(int i = 0; i < calibration_loop; i++) {
    gyro_signals();
    rate_calibration_roll += rate_roll;
    rate_calibration_pitch += rate_pitch;
    rate_calibration_yaw += rate_yaw;
    delay(1);
  }

  rate_calibration_roll /= 2000;
  rate_calibration_pitch /= 2000;
  rate_calibration_yaw /= 2000;
}

void loop() {
  gyro_signals();
  rate_roll -= rate_calibration_roll;
  rate_pitch -= rate_calibration_pitch;
  rate_yaw -= rate_calibration_yaw;
  Serial.print("Roll rate: ");
  Serial.print(rate_roll);
  Serial.print("  Pitch rate: ");
  Serial.print(rate_pitch);
  Serial.print("  Yaw rate: ");
  Serial.println(rate_yaw);
  delay(50);
}
