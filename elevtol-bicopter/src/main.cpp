#include <Arduino.h>
#include <Servo.h>
#include <elevtol_mpu_6050.h>
#include <elevtol_pid_control.h>

#define gyro_scale_factor 131
#define acce_scale_factor 16384
#define gravitational_constant 9.80665
#define rad_to_deg 57.2957795131
#define alpha 0.35
// Pairs:
// @ delay = 100ms, alpha = 0.35
// @ delay = 0ms, alpha = 0.75, none

float rate_roll, rate_pitch, rate_yaw;
float rate_acceX, rate_acceY, rate_acceZ;
float acceX, acceY, acceZ;
float gyroX, gyroY, gyroZ;
float acce_angle[2], filtered_angle[2];
float pwm_left, pwm_right;
float total_pid;
sensors_event_t a, g;

Servo right_motor;
Servo left_motor;
MPU_6050 mpu;

void mpu_reading();
void print_acce_angle();
void print_filtered_angle();
void gyroscope_calibration();
void accelerometer_calibration();
void calibrate_esc();
void arming_esc();
void done_calibration();
void calculate_acce_angle();
void complementary_filter();

float temp_time;
bool first_loop = true;
float curr_time = 0;
float prev_time = 0;
float time_elapsed;
float curr_error;
float curr_error_sum;
float prev_error;
float _desired_value = 0;
float measurement;
float p_gain = 6.5;
float i_gain = 0;
float d_gain = 1.3;
float p_term, i_term, d_term;
float total;

void setup() {
    Serial.begin(250000);
    while(!Serial) {
        delay(10);
    }

    if (!mpu.begin()) {
        Serial.println("MPU-6050 Error : Cannot find chip");
        while(1) {
            delay(10);
        }
    }
    Serial.println("MPU_6050 Connected...");
    mpu.setAccelerometerRange(MPU_6050_ACCE_RANGE_2_G);
    mpu.setGyroscopeRange(MPU_6050_GYRO_RANGE_500_DEG);
    mpu.setDLPFBandWidth(MPU_6050_BANDWIDTH_10_HZ);
    delay(100);
    Serial.println("Calibrating MPU...");
    gyroscope_calibration();
    delay(100);
    accelerometer_calibration();
    delay(100);
    done_calibration();
    acce_angle[0] = 0;
    acce_angle[1] = 0;
    filtered_angle[0] = 0;
    filtered_angle[1] = 0;
    delay(3000);

    right_motor.attach(4); // right motor use pin 4
    left_motor.attach(5); // left motor use pin 5
    delay(1000);
    arming_esc();
    delay(1000);

    Serial.println("All Done...");
    delay(3000);
}

void loop() {
    prev_time = curr_time;
    curr_time = millis();
    if(first_loop == true) {
        time_elapsed = 0;
        first_loop = false;
    } else {
        time_elapsed = (curr_time - prev_time) / 1000;
    }

    mpu.getGyroEvent(&g);
    mpu.getAcceEvent(&a);
    gyroX = g.gyro.x - rate_roll;
    gyroY = g.gyro.y - rate_pitch;
    gyroZ = g.gyro.z - rate_yaw;
    acceX = a.acceleration.x - rate_acceX;
    acceY = a.acceleration.y - rate_acceY;
    acceZ = a.acceleration.z + rate_acceZ;

    calculate_acce_angle();
    complementary_filter();

    // P Term
    curr_error = _desired_value - filtered_angle[1];
    p_term = p_gain * curr_error;

    // I Term
    if (-3 < curr_error && curr_error < 3) {
        curr_error_sum = (prev_error + curr_error) * time_elapsed / 2;
        curr_error_sum *= i_gain;
        i_term += curr_error_sum;
    }

    // D Term
    float error_diff = curr_error - prev_error;
    d_term = d_gain * (error_diff / time_elapsed);

    prev_error = curr_error;

    total = p_term + i_term + d_term;

    if (total < -1000) {
        total = -1000;
    }
    if (total > 1000) {
        total = 1000;
    }

    pwm_right = 1080 - total;
    pwm_left = 1050 + total;

    Serial.println(total);

    if(pwm_right < 1000) {
        pwm_right= 1000;
    }
    if(pwm_right > 2000) {
        pwm_right=2000;
    }
    if(pwm_left < 1000) {
        pwm_left= 1000;
    }
    if(pwm_left > 2000) {
        pwm_left=2000;
    }

    right_motor.writeMicroseconds(pwm_right);
    left_motor.writeMicroseconds(pwm_left);

    // right_motor.writeMicroseconds(1077);
    // left_motor.writeMicroseconds(1050);
}

void arming_esc() {
    Serial.println("Start Arming ESC After 5 Seconds...");
    delay(1000);
    Serial.println("5");
    delay(1000);
    Serial.println("4");
    delay(1000);
    Serial.println("3");
    delay(1000);
    Serial.println("2");
    delay(1000);
    Serial.println("1");
    delay(1000);
    Serial.println("Arming ESC...");
    temp_time = millis();
    while (millis() - temp_time < 9000) {
        right_motor.writeMicroseconds(1020);
        left_motor.writeMicroseconds(1020);
    }
    Serial.println("ESC Arming Completed...");
}

void calibrate_esc() {
    Serial.println("Start Calibrating ESC After 7 Seconds...");
    delay(7000);
    Serial.println("Calibrating ESC...");
    temp_time = millis();
    while (millis() - temp_time < 7000) {
        right_motor.writeMicroseconds(2000);
        left_motor.writeMicroseconds(2000);
    }

    while (millis() - temp_time < 9000) {
        right_motor.writeMicroseconds(1000);
        left_motor.writeMicroseconds(1000);
    }
    right_motor.writeMicroseconds(1000);
    left_motor.writeMicroseconds(1000);
    Serial.println("ESC Calibration Completed...");
}

void calculate_acce_angle() {
    acce_angle[0] = atan(acceY / sqrt(pow(acceX, 2) + pow(acceZ, 2))); // roll
    acce_angle[1] = atan(-1 * acceX / sqrt(pow(acceY, 2) + pow(acceZ, 2))); // pitch
}

void complementary_filter() {
    filtered_angle[0] = alpha * (filtered_angle[0] + (gyroY * time_elapsed)) + ((1 - alpha) * acce_angle[0]); // roll
    filtered_angle[1] = alpha * (filtered_angle[1] + (gyroX * time_elapsed)) + ((1 - alpha) * acce_angle[1]); // pitch
}

void gyroscope_calibration() {
    // Gyroscope calibration
    for(int i = 0; i < 2000; i++) {
        mpu.getGyroEvent(&g);
        rate_roll += g.gyro.x;
        rate_pitch += g.gyro.y;
        rate_yaw += g.gyro.z;
        delay(1);
    }
    // add, add, add
    rate_roll /= 2000;
    rate_pitch /= 2000;
    rate_yaw /= 2000;
    delay(100);
}

void accelerometer_calibration() {
    // Accelerometer calibration
    for(int i = 0; i < 2000; i++) {
        mpu.getAcceEvent(&a);
        rate_acceX += a.acceleration.x;
        rate_acceY += a.acceleration.y;
        rate_acceZ += gravitational_constant - a.acceleration.z;
        delay(1);
    }
    // subtract, subtract, add
    rate_acceX /= 2000;
    rate_acceY /= 2000;
    rate_acceZ /= 2000;
    delay(100);
}

void done_calibration() {
    Serial.println("Done MPU Calibration...");
    Serial.print("Rate roll: ");
    Serial.print(rate_roll);
    Serial.print("    Rate pitch: ");
    Serial.print(rate_pitch);
    Serial.print("    Rate yaw: ");
    Serial.println(rate_yaw);
    
    Serial.print("Rate acceX: ");
    Serial.print(rate_acceX);
    Serial.print("    Rate acceY: ");
    Serial.print(rate_acceY);
    Serial.print("    Rate acceZ: ");
    Serial.println(rate_acceZ);
}

void mpu_reading() {
    // mpu.getGyroEvent(&g);
    // mpu.getAcceEvent(&a);
    Serial.print(gyroX);
    Serial.print("  ");
    Serial.print(gyroY);
    Serial.print("  ");
    Serial.print(gyroZ);
    Serial.print("                  ");
    Serial.print(acceX);
    Serial.print("  ");
    Serial.print(acceY);
    Serial.print("  ");
    Serial.println(acceZ);
}

void print_acce_angle() {
    Serial.print(acce_angle[0] * rad_to_deg);
    Serial.print("      ");
    Serial.println(acce_angle[1] * rad_to_deg);
}

void print_filtered_angle() {
    Serial.print(filtered_angle[0] * rad_to_deg);
    Serial.print("      ");
    Serial.println(filtered_angle[1] * rad_to_deg);
}
