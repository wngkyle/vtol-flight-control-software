#include <Arduino.h>
#include <Servo.h>
#include <elevtol_mpu_6050.h>

Servo right_motor;
Servo left_motor;
MPU_6050 mpu;

#define gyro_scale_factor 131
#define acce_scale_factor 16384

float p_gain = 0;
float i_gain = 0;
float d_gain = 0;
float p_term, i_term, d_term;
float rate_roll, rate_pitch, rate_yaw;
float pid, pwd_left, pwm_right;
float time, prev_time, elapsed_time, error, prev_error;
uint16_t raw_gyroX, raw_gyroY, raw_gyroZ;
uint16_t raw_acceX, raw_acceY, raw_acceZ;
sensors_event_t a, g;

float calculate_p_term();
float calculate_i_term();
float calculate_d_term();

void setup() {
    Serial.begin(250000);
    while(!Serial) {
        delay(10);
    }
    if (!mpu.begin()) {
        Serial.println("MPU_6050 Error : Cannot find chip");
        while(1) {
            delay(10);
        }
    }
    Serial.println("MPU_6050 Connected...");
    mpu.setAccelerometerRange(MPU_6050_ACCE_RANGE_8_G);
    mpu.setGyroscopeRange(MPU_6050_GYRO_RANGE_500_DEG);
    mpu.setDLPFBandWidth(MPU_6050_BANDWIDTH_10_HZ);
    delay(100);
    for (int i = 0; i < 2000; i ++) {
        mpu.getGyroEvent(&g);
        rate_roll += g.gyro.x;
        rate_pitch += g.gyro.y;
        rate_yaw += g.gyro.z;
        delay(1);
    }

    rate_roll /= 2000;
    rate_pitch /= 2000;
    rate_yaw /= 2000;

    right_motor.attach(4);
    left_motor.attach(5);

    delay(2000);
    
    time = millis();
}

void loop() {
    prev_time = time;
    time = millis();
    elapsed_time = time - prev_time;

    mpu.getAcceEvent(&a);
    mpu.getGyroEvent(&g);


}

float calculate_p_term() {

}

float calculate_i_term() {

}

float calculate_d_term() {

}