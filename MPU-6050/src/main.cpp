#include <Arduino.h>
#include <Wire.h>
#include <elevtol_mpu_6050.h>   

// MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;

struct MyData {
  byte X;
  byte Y;
  byte Z;
};

MyData data;

void mpu_setup() {
    Serial.begin(9600);
    Wire.begin();
    mpu.initialize();
}

void mpu_loop() {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    data.X = map(ax, -17000, 17000, 0, 255 ); // X axis data
    data.Y = map(ay, -17000, 17000, 0, 255); 
    data.Z = map(az, -17000, 17000, 0, 255);  // Y axis data
    delay(500);
    Serial.print("Axis X = ");
    Serial.print(data.X);
    Serial.print("  ");
    Serial.print("Axis Y = ");
    Serial.print(data.Y);
    Serial.print("  ");
    Serial.print("Axis Z  = ");
    Serial.println(data.Z);
}

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

void gyro_signal(void) {
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);
    Wire.write(0x05); // low pass filter
    Wire.endTransmission();

    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x8); // scale factor
    Wire.endTransmission();

    Wire.beginTransmission(0x68);
    Wire.write(0x43); // read measurement
    Wire.endTransmission();

    Wire.requestFrom(0x68, 6);
    int16_t gyro_x = Wire.read() << 8 | Wire.read();
    int16_t gyro_y = Wire.read() << 8 | Wire.read();
    int16_t gyro_z = Wire.read() << 8 | Wire.read();
    RateRoll = (float) gyro_x / 65.5;
    RatePitch = (float) gyro_y / 65.5;
    RateYaw = (float) gyro_z / 65.5;

    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;
    Serial.print("MPU-6050    Roll Rate: ");
    Serial.print(RateRoll);
    Serial.print("  Pitch Rate: ");
    Serial.print(RatePitch);
    Serial.print("  Yaw Rate: ");
    Serial.println(RateYaw);
    delay(50);
}

void gyro_signal_setup() {
    Serial.begin(57600);

    Wire.setClock(400000);
    Wire.begin();
    delay(250);

    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00); // power on
    Wire.endTransmission();
    for(RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
        gyro_signal();
        RateCalibrationRoll += RateRoll;
        RateCalibrationPitch += RatePitch;
        RateCalibrationYaw += RateYaw;
        delay(1);
    }
    RateCalibrationRoll /= 2000;
    RateCalibrationPitch /= 2000;
    RateCalibrationYaw /= 2000;
}

MPU_6050 mpu_6050;
float rate_roll, rate_pitch, rate_yaw;
sensors_event_t a, g, temp;

void mpu_6050_setup() {
    // Set the clock speed to 400kHz
    Serial.begin(115200);
    while (!Serial) { // pause until serial monitor open
        delay(10);
    }
    if (!mpu_6050.begin()) {
        Serial.println("MPU_6050 Error : Cannot find chip");
        while(1) {
            delay(10);
        }
    }
    Serial.println("MPU_6050 Connected...");
    mpu_6050.setAccelerometerRange(MPU_6050_ACCE_RANGE_8_G);
    mpu_6050.setGyroscopeRange(MPU_6050_GYRO_RANGE_500_DEG);
    mpu_6050.setDLPFBandWidth(MPU_6050_BANDWIDTH_10_HZ);
    delay(100);

    for (int i = 0; i < 2000; i ++) {
        mpu_6050.getEvent(&a, &g, &temp);
        rate_roll += g.gyro.x;
        rate_pitch += g.gyro.y;
        rate_yaw += g.gyro.z;
        delay(1);
    }

    rate_roll /= 2000;
    rate_pitch /= 2000;
    rate_yaw /= 2000;

    delay(10);
}

void mpu_6050_loop() {
    mpu_6050.getEvent(&a, &g, &temp);
    Serial.print(a.acceleration.x);
    Serial.print("  ");
    Serial.print(a.acceleration.y);
    Serial.print("  ");
    Serial.print(a.acceleration.z);
    Serial.print("                  ");
    Serial.print(g.gyro.x-rate_roll);
    Serial.print("  ");
    Serial.print(g.gyro.y-rate_pitch);
    Serial.print("  ");
    Serial.print(g.gyro.z-rate_yaw);
    Serial.print("                  ");
    Serial.println(temp.temperature);
    delay(300);
}

void setup() {
    mpu_6050_setup();
}

void loop() {
    mpu_6050_loop();
}

