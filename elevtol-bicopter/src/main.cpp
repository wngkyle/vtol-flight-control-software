#include <Arduino.h>
#include <Servo.h>
#include <elevtol_mpu_6050.h>
#include <elevtol_pid_control.h>

#define gyro_scale_factor 131
#define acce_scale_factor 16384
#define gravitational_constant 9.80665

float rate_roll, rate_pitch, rate_yaw;
float rate_acceX, rate_acceY, rate_acceZ;
float curr_time, pwd_left, pwm_right;
uint16_t raw_gyroX, raw_gyroY, raw_gyroZ;
uint16_t raw_acceX, raw_acceY, raw_acceZ;
sensors_event_t a, g;

Servo right_motor;
Servo left_motor;
MPU_6050 mpu;
PID_Control pid;

void mpu_reading();


void setup() {
    Serial.begin(250000);
    while(!Serial) {
        delay(10);
    }

    // Setting up MPU, configuring the setting
    if (!mpu.begin()) {
        Serial.println("MPU_6050 Error : Cannot find chip");
        while(1) {
            delay(10);
        }
    }
    Serial.println("MPU_6050 Connected...");
    mpu.setAccelerometerRange(MPU_6050_ACCE_RANGE_2_G);
    mpu.setGyroscopeRange(MPU_6050_GYRO_RANGE_500_DEG);
    mpu.setDLPFBandWidth(MPU_6050_BANDWIDTH_10_HZ);
    delay(100);

    // Gyroscope calibration
    for(int i = 0; i < 2000; i++) {
        mpu.getGyroEvent(&g);
        rate_roll += g.gyro.x;
        rate_pitch += g.gyro.y;
        rate_yaw += g.gyro.z;
        delay(1);
    }
    // Add
    rate_roll /= 2000;
    rate_pitch /= 2000;
    rate_yaw /= 2000;
    delay(100);

    // Accelerometer calibration
    for(int i = 0; i < 2000; i++) {
        mpu.getAcceEvent(&a);
        rate_acceX += a.acceleration.x;
        rate_acceY += a.acceleration.y;
        rate_acceZ = gravitational_constant - a.acceleration.z;
        delay(1);
    }
    // Subtract
    rate_acceX /= 2000;
    rate_acceY /= 2000;
    rate_acceZ /= 2000;
    delay(100);

    // Setting PID constants
    pid.setPGain(0.001);
    // I and D gains are default to 0

    // Assign motor pins
    right_motor.attach(4); // right motor use pin 4
    left_motor.attach(5); // left motor use pin 5

    delay(2000);
}

void loop() {
    // mpu.getAcceEvent(&a);
    // mpu.getGyroEvent(&g);

}

void mpu_reading() {
    mpu.getGyroEvent(&g);
    mpu.getAcceEvent(&a);
    Serial.print(a.acceleration.x - rate_acceX);
    Serial.print("  ");
    Serial.print(a.acceleration.y - rate_acceY);
    Serial.print("  ");
    Serial.print(a.acceleration.z - rate_acceZ);
    Serial.print("                  ");
    Serial.print(g.gyro.x - rate_roll);
    Serial.print("  ");
    Serial.print(g.gyro.y - rate_pitch);
    Serial.print("  ");
    Serial.println(g.gyro.z - rate_yaw);
}