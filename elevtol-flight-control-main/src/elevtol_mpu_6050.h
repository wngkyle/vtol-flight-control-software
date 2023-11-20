// header gaurd 
// alternative : #pragma once
// Pragma once request the compiler to gaurd the header instead of relying on user
#ifndef ELEVTOL_MPU_6050_H
#define ELEVTOL_MPU_6050_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>

#define MPU_6050_DEFAULT_ADDR 0x68 // default I2C address when AD0 is 0
#define MPU_6050_DEVICE_ID 0x68 // device address, WHO_AM_I value

// Didn't include: 
// self test x, y, z reg
// interrupt pin config reg
// interrupt enable config reg
// interrupt status reg
// motion detection bits
// motion duration counter
#define MPU_6050_SAMPLE_RATE_DIVIDER 0x19 
#define MPU_6050_GENERAL_CONFIG 0x1A
#define MPU_6050_GYRO_CONFIG 0x1B
#define MPU_6050_ACCE_CONFIG 0x1C
#define MPU_6050_WHO_AM_I 0x75
#define MPU_6050_USER_CONTORL 0x6A
#define MPU_6050_PRIMARY_POWER 0x6B
#define MPU_6050_SECONDARY_POWER 0x6C
#define MPU_6050_TEMP_HIGH 0x41
#define MPU_6050_TEMP_LOW 0x42
#define MPU_6050_ACCE_BASE 0x3B
#define MPU_6050_SIGNAL_RESET 0x68

// FSYNC = External frame synchronization, aligning a device's internal 
//         operation like data aquisition, data processing with an input
//         external reference signal 
typedef enum fsync_out {
    MPU_6050_OUT_DISABLED,
    MPU_6050_OUT_TEMP,
    MPU_6050_OUT_GYRO_X,
    MPU_6050_OUT_GYRO_Y,
    MPU_6050_OUT_GYRO_Z,
    MPU_6050_OUT_ACCE_X,
    MPU_6050_OUT_ACCE_Y,
    MPU_6050_OUT_ACCE_Z,
} mpu_6050_fysnc_out;

// dlpf = Digital low pass filter
typedef enum dlpf_bandwidth {
    MPU_6050_BANDWIDTH_260_HZ,
    MPU_6050_BANDWIDTH_184_HZ,
    MPU_6050_BANDWIDTH_94_HZ,
    MPU_6050_BANDWIDTH_44_HZ,
    MPU_6050_BANDWIDTH_21_HZ,
    MPU_6050_BANDWIDTH_10_HZ,
    MPU_6050_BANDWIDTH_5_HZ,
} mpu_6050_dlpf_bandwidth;

// PLL = Phase-locked loop, controlling or synchronizing the timing 
//       of the internal clock using a certain signal. In other words,
//       control system that generates an output signal whose phase of 
//       an input reference signal
typedef enum clock_source {
    MPU_6050_INTERNAL_8MHz,
    MPU_6050_PLL_GYRO_X,
    MPU_6050_PLL_GYRO_Y,
    MPU_6050_PLL_GYRO_Z,
    MPU_6050_PLL_EXT_32kHz,
    MPU_6050_PLL_EXT_19MHz,
} mpu_6050_clock_source;

typedef enum gyro_range {
    MPU_6050_GYRO_RANGE_250_DEG,
    MPU_6050_GYRO_RANGE_500_DEG,
    MPU_6050_GYRO_RANGE_1000_DEG,
    MPU_6050_GYRO_RANGE_2000_DEG,
} mpu_6050_gyro_range;

typedef enum acce_range {
    MPU_6050_ACCE_RANGE_2_G,
    MPU_6050_ACCE_RANGE_4_G,
    MPU_6050_ACCE_RANGE_8_G,
    MPU_6050_ACCE_RANGE_16_G,
} mpu_6050_acce_range;

typedef enum cycle_rate {
    MPU_6050_CYCLE_RATE_1_25_HZ,
    MPU_6050_CYCLE_RATE_5_HZ,
    MPU_6050_CYCLE_RATE_20_HZ,
    MPU_6050_CYCLE_RATE_40_HZ,
} mpu_6050_cycle_rate;

// typdef didn't include dlhp = digital low high pass filter config

class MPU_6050;

class MPU_6050_temp : public Adafruit_Sensor {
    public: 
        MPU_6050_temp(MPU_6050 *parent);
        bool getEvent(sensors_event_t *);
        void getSensor(sensor_t *);
    private:
        MPU_6050 *_mpu_6050 = NULL;
        int _sensorID = 0x65; 
};


class MPU_6050_acce : public Adafruit_Sensor {
    public:
        MPU_6050_acce(MPU_6050 *parent);
        bool getEvent(sensors_event_t *);
        void getSensor(sensor_t *);
    private:
        MPU_6050 *_mpu_6050 = NULL;
        int _sensorID = 0x651;
};

class MPU_6050_gyro : public Adafruit_Sensor {
    public:
        MPU_6050_gyro(MPU_6050 *parent);
        bool getEvent(sensors_event_t *);
        void getSensor(sensor_t *);
    private:
        MPU_6050 *_mpu_6050 = NULL;
        int _sensorID = 0x652;
};

class MPU_6050 final {
    public:
        MPU_6050();
        ~MPU_6050();
        bool getEvent(sensors_event_t *acce, sensors_event_t *gyro, sensors_event_t *temp);
        bool begin(uint8_t i2c_address = MPU_6050_DEFAULT_ADDR, TwoWire *wire = &Wire, int32_t sensorID = 0);
        bool enableSleep(bool enable);
        void reset(void);

        // Friend classes
        friend class MPU_6050_temp;
        friend class MPU_6050_acce;
        friend class MPU_6050_gyro;
        
        // Sensor getter
        Adafruit_Sensor *getTemperatureSensor(void);
        Adafruit_Sensor *getAccelerometerSensor(void);
        Adafruit_Sensor *getGyroscopeSensor(void);
        // Accelerometer getter and setter
        mpu_6050_acce_range getAccelerometerRange(void);
        void setAccelerometerRange(mpu_6050_acce_range range);
        // Gyroscope getter and setter
        mpu_6050_gyro_range getGyroscopeRange(void);
        void setGyroscopeRange(mpu_6050_gyro_range range);
        // Frame Synchronization getter and setter
        mpu_6050_fysnc_out getFrameSynchronization(void);
        void setFrameSynchronization(mpu_6050_fysnc_out reference);
        // DLPF Bandwidth getter and setter
        mpu_6050_dlpf_bandwidth getDLPFBandWidth(void);
        void setDLPFBandWidth(mpu_6050_dlpf_bandwidth bandwidth);
        // Clock Source getter and setter
        mpu_6050_clock_source getClockSource(void);
        void setClockSource(mpu_6050_clock_source source);
        // Cycle Rate getter and setter
        mpu_6050_cycle_rate getCycleRate(void);
        void setCycleRate(mpu_6050_cycle_rate rate);
        // Sample Rate Divider getter and setter
        uint8_t getSampleRateDivider(void);
        void setSampleRateDivider(uint8_t divider);

    private:
        int16_t rawTemp;
        int16_t rawAcceX, rawAcceY, rawAcceZ;
        int16_t rawGyroX, rawGyroY, rawGyroZ;

        void _getRawSensorDate(void);
        void _scaleSensorDate(void);
        void fullTempEvent(sensors_event_t *temp, uint32_t timestamp);
        void fullAcceEvent(sensors_event_t *acce, uint32_t timestamp);
        void fullGyroEvent(sensors_event_t *gyro, uint32_t timestamp);
    protected:
        float temperature;
        float acceX, acceY, acceZ;
        float gyroX, gyroY, gyroZ;
        uint16_t _temp_sensorID, _acce_sensorID, _gyro_sensorID;

        Adafruit_I2CDevice *i2c_device = NULL;
        MPU_6050_temp *temp_sensor = NULL;
        MPU_6050_acce *acce_sensor = NULL;
        MPU_6050_gyro *gyro_sensor = NULL;
 
        void _read(void);
        virtual bool _init(int32_t sensor_id);
};

#endif