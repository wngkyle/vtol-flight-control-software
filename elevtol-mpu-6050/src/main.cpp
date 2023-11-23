#include <Arduino.h>
#include <Wire.h>
#include <elevtol_mpu_6050.h>   

int16_t ax, ay, az;
int16_t gx, gy, gz;

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
    // Setting Accelerometer Range :
        // MPU_6050_ACCE_RANGE_2_G
        // MPU_6050_ACCE_RANGE_4_G
        // MPU_6050_ACCE_RANGE_8_G
        // MPU_6050_ACCE_RANGE_16_G

    mpu_6050.setGyroscopeRange(MPU_6050_GYRO_RANGE_500_DEG);
    // Setting Gyroscope Range : 
        // MPU_6050_GYRO_RANGE_250_DEG
        // MPU_6050_GYRO_RANGE_500_DEG
        // MPU_6050_GYRO_RANGE_1000_DEG
        // MPU_6050_GYRO_RANGE_2000_DEG

    mpu_6050.setDLPFBandWidth(MPU_6050_BANDWIDTH_10_HZ);
    // Setting Digital Low Pass Filter :
        // MPU_6050_BANDWIDTH_260_HZ
        // MPU_6050_BANDWIDTH_184_HZ
        // MPU_6050_BANDWIDTH_94_HZ
        // MPU_6050_BANDWIDTH_44_HZ
        // MPU_6050_BANDWIDTH_21_HZ
        // MPU_6050_BANDWIDTH_10_HZ
        // MPU_6050_BANDWIDTH_5_HZ

    delay(100);

    // Gyroscope calibration
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

void mpu_6050_print_all() {
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
    delay(100);
}

void mpu_6050_print_temp() {
    mpu_6050.getTempEvent(&temp);
    Serial.println(temp.temperature);
    delay(100);
}

void mpu_6050_print_acce() {
    mpu_6050.getAcceEvent(&a);
    Serial.print(a.acceleration.x);
    Serial.print("  ");
    Serial.print(a.acceleration.y);
    Serial.print("  ");
    Serial.println(a.acceleration.z);
    delay(100);
}

void mpu_6050_print_gyro() {
    mpu_6050.getGyroEvent(&g);
    Serial.print(g.gyro.x-rate_roll);
    Serial.print("  ");
    Serial.print(g.gyro.y-rate_pitch);
    Serial.print("  ");
    Serial.print(g.gyro.z-rate_yaw);
    delay(100);
}

void setup() {
    mpu_6050_setup();
}

void loop() {
    mpu_6050_print_all();
}

