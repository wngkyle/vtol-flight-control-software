#include <Arduino.h>
#include <Wire.h>

#define built_in_LED 13
#define start_calibration_LED 41
#define end_calibration_LED 40
#define buzzer_pin 8
#define battery_monitor_pin 15

float voltage_per_cell;
float total_battery_voltage;
float rate_roll;
float rate_pitch;
float rate_yaw;
unsigned int gyro_x = 0;
unsigned int gyro_y = 0;
unsigned int gyro_z = 0;
int gyro_axis_count = 0;

void setup_feedback();
void check_battery_voltage();
void calibrate_gyro();
void read_gyro_signals();

void setup() {
  Serial.begin(57600);
  pinMode(built_in_LED, OUTPUT);
  pinMode(start_calibration_LED, OUTPUT);
  pinMode(end_calibration_LED, OUTPUT);
  pinMode(battery_monitor_pin, INPUT);

  calibrate_gyro();

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

void calibrate_gyro() {
  Wire.setClock(400000); // set clock speed of the I2C protocol to 400kHz
  Wire.begin(); // initialize the Wire library and join the I2C bus
  delay(300); // delay 300msec to give MPU time to start

  Wire.beginTransmission(0x68); // slave address of MPU-6050 : 1101000 = 0x68

  Wire.write(0x6B); // register address for power management register 1
  Wire.write(0x00); // set everything to zero to deactivate sleep mode, and start the sensor
  Wire.endTransmission(false); 

  Wire.write(0x1A); // register address for configuring digital low pass filter (DLPF) for both gyroscope and accelerometer
  Wire.write(0x05); // set to 5, activating DLPF to bandwidth of 10Hz, 5 = 0x05
  Wire.endTransmission(false); // passing false sends a restart (repeated start) condition

  Wire.write(0x1B); // register address for gyroscope configuration
  Wire.write(0x08); // set sensitivity scale factor to 65.5 LSB / (deg/s) and full scale range to +/- 500 deg/s, 0b00001000 = 0x08
  Wire.endTransmission(true); // passing true to send a stop condition
}

void read_gyro_signals() {
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(false);

  Wire.requestFrom(0x68, 6, true);
  while(Wire.available()) {
    if ( 0 <= gyro_axis_count < 2) {
      gyro_x |= Wire.read();
      if (gyro_axis_count == 0) {
        gyro_x << 8;
      }
    } else if (2 <= gyro_axis_count < 4) {
      gyro_y |= Wire.read();
      if (gyro_axis_count == 2) {
        gyro_y << 8;
      }
    } else {
      gyro_z |= Wire.read();
      if (gyro_axis_count == 4) {
        gyro_z << 8;
      } 
    }
    gyro_axis_count = gyro_axis_count + 1;
  }
  Wire.endTransmission(true);

  gyro_axis_count = 0;

  rate_roll = gyro_x / 65.5;
  rate_pitch = gyro_y / 65.5;
  rate_yaw = gyro_z / 65.5;

  Serial.print("Rate roll: ");
  Serial.print(rate_roll);
  Serial.print("    Rate pitch: ");
  Serial.print(rate_pitch);
  Serial.print("    Rate yaw: ");
  Serial.println(rate_yaw);

  delay(50); // delay 50msec
}