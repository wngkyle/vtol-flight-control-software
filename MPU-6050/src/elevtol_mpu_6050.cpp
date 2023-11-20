#include <Arduino.h>
#include <elevtol_mpu_6050.h>

MPU_6050_temp::MPU_6050_temp(MPU_6050 *parent) {
    _mpu_6050 = parent;
}

bool MPU_6050_temp::getEvent(sensors_event_t *event) {
    _mpu_6050->_read();
    _mpu_6050->fullTempEvent(event, millis());

    return true;
}

void MPU_6050_temp::getSensor(sensor_t *sensor) {
    memset(sensor, 0, sizeof(sensor_t)); // reseting the value in sensor_t object

    // insert sensor name here
    strncpy(sensor->name, "MPU_6050_T", sizeof(sensor->name) - 1);
    sensor->name[sizeof(sensor->name)-1] = 0; // eliminates the null character
    sensor->version = 1;
    sensor->sensor_id = _sensorID;
    sensor->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
    sensor->min_delay = 0;
    sensor->min_value = -40;
    sensor->max_value = 105;
    sensor->resolution = 0.00294;
}

MPU_6050_acce::MPU_6050_acce(MPU_6050 *parent) {
    _mpu_6050 = parent;
}

bool MPU_6050_acce::getEvent(sensors_event_t *event) {
    _mpu_6050->_read();
    _mpu_6050->fullAcceEvent(event, millis());

    return true;
}

void MPU_6050_acce::getSensor(sensor_t *sensor) {
    memset(sensor, 0, sizeof(sensor_t)); // reseting the value in sensor_t object

    // insert sensor name here
    strncpy(sensor->name, "MPU_6050_A", sizeof(sensor->name) - 1); // eliminates the null character
    sensor->name[sizeof(sensor->name)-1] = 0;
    sensor->version = 1;
    sensor->sensor_id = _sensorID;
    sensor->type = SENSOR_TYPE_ACCELEROMETER;
    sensor->min_delay = 0;
    sensor->min_value = -156.9094F;
    sensor->max_value = 156.90964F;
    sensor->resolution = 0.061;
}

MPU_6050_gyro::MPU_6050_gyro(MPU_6050 *parent) {
    _mpu_6050 = parent;
}

bool MPU_6050_gyro::getEvent(sensors_event_t *event) {
    _mpu_6050->_read();
    _mpu_6050->fullGyroEvent(event, millis());

    return true;
}

void MPU_6050_gyro::getSensor(sensor_t *sensor) {
    memset(sensor, 0, sizeof(sensor_t)); // reseting the value in sensor_t object

    // insert sensor name here 
    strncpy(sensor->name, "MPU_6050_A", sizeof(sensor->name) - 1); // eliminates the null character
    sensor->name[sizeof(sensor->name)-1] = 0;
    sensor->version = 1;
    sensor->sensor_id = _sensorID;
    sensor->type = SENSOR_TYPE_GYROSCOPE;
    sensor->min_delay = 0;
    sensor->min_value = -34.91;
    sensor->max_value = 34.91;
    sensor->resolution = 1.332e-4;
}

MPU_6050::MPU_6050(void) {}

MPU_6050::~MPU_6050(void) {
    if (temp_sensor) {
        delete temp_sensor;
    } 
    if (acce_sensor) {
        delete acce_sensor;
    }
    if (gyro_sensor) {
        delete gyro_sensor;
    }
    if (i2c_device) {
        delete i2c_device;
    }
}

bool MPU_6050::getEvent(sensors_event_t *acce, sensors_event_t *gyro, sensors_event_t *temp) {
    uint32_t timestamp = millis();
    _read();
    // fill accelerometer, gyroscope, and temperature event
    fullAcceEvent(acce, timestamp);
    fullGyroEvent(gyro, timestamp);
    fullTempEvent(temp, timestamp);
    return true;
}

bool MPU_6050::begin(uint8_t i2c_address, TwoWire *wire, int32_t sensorID) {
    if (i2c_device) { // delete old i2c_device pointer
        delete i2c_device;
    }

    i2c_device = new Adafruit_I2CDevice(MPU_6050_DEFAULT_ADDR, wire); // create new instance
 
    bool mpu_connected = false;
    for (uint8_t i = 0; i < 5; i++) { // check if mpu is connected sucessfully, run a for loop
        mpu_connected = i2c_device->begin();
        if (mpu_connected) {
            break;
        }
        delay(10);
    }
    if (mpu_connected == false) {
        Serial.println("MPU-6050 Error : Failed to connect");
        return false;
    }

    Adafruit_BusIO_Register chip_id = Adafruit_BusIO_Register(i2c_device, MPU_6050_WHO_AM_I);

    if (chip_id.read() != MPU_6050_DEVICE_ID) { // check if it's talking to the right chip
        return false;
    }
    
    return _init(sensorID);
}

bool MPU_6050::enableSleep(bool enable) {
    Adafruit_BusIO_Register power_management_1 = Adafruit_BusIO_Register(i2c_device, MPU_6050_PRIMARY_POWER);
    uint32_t val = power_management_1.read(); // retreive previous value
    val &= ~(1 << 6);
    val |= (1 << 6); // SLEEP is the 7th bit, setting SLEEP to 1 put the device in low power sleep mode
    power_management_1.write(val);
    return true;
}

void MPU_6050::reset(void) { // this reset function executes a full device reset
    Adafruit_BusIO_Register power_management_1 = Adafruit_BusIO_Register(i2c_device, MPU_6050_PRIMARY_POWER);
    Adafruit_BusIO_Register signal_reset = Adafruit_BusIO_Register(i2c_device, MPU_6050_SIGNAL_RESET);
    uint32_t val = power_management_1.read();
    val &= ~(1 << 7); 
    val |= (1 << 7); // device reset at 8th bit of the power management 1 register
    while ((power_management_1.read() >> 7) != 0) { // the 8th bit (device reset bit) automatically clear to 0 after reset
        delay(1); // check if reset is completed continuously using the while loop
    }
    delay(100);

    signal_reset.write(0x07); // gyro, acce, and temp reset at the first three bits of the signal path reset register

    delay(100);
}

void MPU_6050::fullTempEvent(sensors_event_t *temp, uint32_t timestamp) {
    // reset the temp sensors_event_t variable
    // sensors_event_t is a data type declared using typedef struct in Adafruit_Sensor.h
    memset(temp, 0, sizeof(sensors_event_t));
    // setting sensor version to the size of sensors_event_t because sensor_event_t contains a union
    // union is a special class type that can hold only one of its non-static data members at a time
    // so the size of sensors_event_t can change based on this
    temp->version = sizeof(sensors_event_t);
    temp->sensor_id = _temp_sensorID;
    temp->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
    temp->timestamp = timestamp;
    temp->temperature = temperature;
}

void MPU_6050::fullAcceEvent(sensors_event_t *acce, uint32_t timestamp) {
    // reset the acce sensors_event_t variable
    // sensors_event_t is a data type declared using typedef struct in Adafruit_Sensor.h
    memset(acce, 0, sizeof(sensors_event_t));
    acce->version = 1;
    acce->sensor_id = _acce_sensorID;
    acce->type = SENSOR_TYPE_ACCELEROMETER;
    acce->timestamp = timestamp;
    acce->acceleration.x = acceX * SENSORS_GRAVITY_STANDARD;
    acce->acceleration.y = acceY * SENSORS_GRAVITY_STANDARD;
    acce->acceleration.z = acceZ * SENSORS_GRAVITY_STANDARD;
}

void MPU_6050::fullGyroEvent(sensors_event_t *gyro, uint32_t timestamp) {
    // reset the gyro sensors_event_t variable
    // sensors_event_t is a data type declared using typedef struct in Adafruit_Sensor.h
    memset(gyro, 0, sizeof(sensors_event_t));
    gyro->version = 1;
    gyro->sensor_id = _gyro_sensorID;
    gyro->type = SENSOR_TYPE_GYROSCOPE;
    gyro->timestamp = timestamp;
    gyro->gyro.x = gyroX * SENSORS_DPS_TO_RADS; // SENSORS_DPS_TO_RADS is degree/s to radian/s multiplier
    gyro->gyro.y = gyroY * SENSORS_DPS_TO_RADS;
    gyro->gyro.z = gyroZ * SENSORS_DPS_TO_RADS;
}

// main read function
void MPU_6050::_read(void) {
    // Data register starts at 0x3B (MPU_6050_ACCE_BASE) 
    Adafruit_BusIO_Register data_and_measurements = Adafruit_BusIO_Register(i2c_device, MPU_6050_ACCE_BASE, 14);

    uint8_t buffer[14];
    bool read_and_store = data_and_measurements.read(buffer, 14);
    if(!read_and_store) {
        Serial.println("MPU-6050 Error : Failed to read MPU measurement and data register");
        Serial.println("End program, and fix the bug");
        cli();
        while(1);
    }

    // Register Address: 
    // 0x3B ~ 0x3C = acceX_out
    // 0x3D ~ 0x3E = acceY_out
    // 0x3F ~ 0x40 = acceZ_out
    rawAcceX = buffer[0] << 8 | buffer[1];
    rawAcceY = buffer[2] << 8 | buffer[3];
    rawAcceZ = buffer[4] << 8 | buffer[5];
    // 0x41 ~ 0x42 = temp_out
    rawTemp = buffer[6] << 8 | buffer[7];
    // 0x43 ~ 0x44 = gyroX_out
    // 0x45 ~ 0x46 = gyroY_out
    // 0x47 ~ 0x4* = gyroZ_out
    rawGyroX = buffer[8] << 8 | buffer[9];
    rawGyroY = buffer[10] << 8 | buffer[11];
    rawGyroZ = buffer[12] << 8 | buffer[13];

    float acce_scale;
    mpu_6050_acce_range acce_range = getAccelerometerRange();
    if (acce_range == MPU_6050_ACCE_RANGE_2_G) {
        acce_scale = 16384;
    } else if (acce_range == MPU_6050_ACCE_RANGE_4_G) {
        acce_scale = 8192;
    } else if (acce_range == MPU_6050_ACCE_RANGE_8_G) {
        acce_scale = 4096;
    } else { // MPU_6050_ACCE_RANGE_16_G
        acce_scale = 2048;
    }

    // acce_scale LSB/g
    acceX = ((float)rawAcceX) / acce_scale;
    acceY = ((float)rawAcceY) / acce_scale;
    acceZ = ((float)rawAcceZ) / acce_scale;

    // 340 is the scale factor
    // 36.53 is the offset value for calibrating raw temperature values
    temperature = ((float)rawTemp) / 340 + 36.53;

    float gyro_scale;
    mpu_6050_gyro_range gyro_range = getGyroscopeRange();
    if (gyro_range == MPU_6050_GYRO_RANGE_250_DEG) {
        gyro_scale = 131;
    } else if (gyro_range == MPU_6050_GYRO_RANGE_500_DEG) {
        gyro_scale = 65.5;
    } else if (gyro_range == MPU_6050_GYRO_RANGE_1000_DEG) {
        gyro_scale = 32.8;
    } else { // MPU_6050_GYRO_RANGE_2000_DEG
        gyro_scale = 16.4;
    }

    // gyro_scale LSB/(degree/s)
    gyroX = ((float)rawGyroX) / gyro_scale;
    gyroY = ((float)rawGyroY) / gyro_scale;
    gyroZ = ((float)rawGyroZ) / gyro_scale;
}

bool MPU_6050::_init(int32_t sensor_id) {
    _acce_sensorID = sensor_id; // default setting of sensor_id is 0 if not specified
    _gyro_sensorID = sensor_id + 1;
    _temp_sensorID = sensor_id + 2;

    // before configuring anything, do a device reset
    reset();

    // sample rate divider register
    // this register the divider of the gyroscope output rate that is used to generate sample output rate
    // sensor register output, FIFO output, and DMP 
    // sample_rate = gyroscope_output_rate / (1 + sample_divider)
    // gyroscope output is 8kHz when dlpf is disabled, and 1kHz when dlpf is enabled
    // accelerometer output rate is fixed at 1kHz
    setSampleRateDivider(0);
    setDLPFBandWidth(MPU_6050_BANDWIDTH_260_HZ);
    setGyroscopeRange(MPU_6050_GYRO_RANGE_500_DEG);
    setAccelerometerRange(MPU_6050_ACCE_RANGE_2_G);
    
    Adafruit_BusIO_Register power_management_1 = Adafruit_BusIO_Register(i2c_device, MPU_6050_PRIMARY_POWER);

    power_management_1.write(0x01); // phase locked (PLL) sample output rate to gyro_x

    delay(100);

    // delete old sensor pointers if exist
    if (temp_sensor) {
        delete temp_sensor;
    } 
    if (acce_sensor) {
        delete acce_sensor;
    }
    if (gyro_sensor) {
        delete gyro_sensor;
    }

    // initialize new sensor pointer
    temp_sensor = new MPU_6050_temp(this);
    acce_sensor = new MPU_6050_acce(this);
    gyro_sensor = new MPU_6050_gyro(this);
    
    return true;
}

mpu_6050_acce_range MPU_6050::getAccelerometerRange(void) {
    Adafruit_BusIO_Register acce_reg = Adafruit_BusIO_Register(i2c_device, MPU_6050_ACCE_CONFIG);
    uint8_t val = acce_reg.read();
    val &= 0x18; // clean bits
    val >>= 3;
    return (mpu_6050_acce_range)val;
}
void MPU_6050::setAccelerometerRange(mpu_6050_acce_range range) {
    Adafruit_BusIO_Register acce_reg = Adafruit_BusIO_Register(i2c_device, MPU_6050_ACCE_CONFIG);
    uint8_t val = acce_reg.read();
    val &= 0xE7; // clear bits
    uint8_t bits = static_cast<uint8_t>(range);
    bits <<= 3; // set up mask
    val |= bits;
    acce_reg.write(val);
}

mpu_6050_gyro_range MPU_6050::getGyroscopeRange(void) {
    Adafruit_BusIO_Register gyro_reg = Adafruit_BusIO_Register(i2c_device, MPU_6050_GYRO_CONFIG);
    uint8_t val = gyro_reg.read();
    val &= 0x18; // clean bits
    val >>= 3; 
    return (mpu_6050_gyro_range)val;
}
void MPU_6050::setGyroscopeRange(mpu_6050_gyro_range range) {
    Adafruit_BusIO_Register gyro_reg = Adafruit_BusIO_Register(i2c_device, MPU_6050_GYRO_CONFIG);
    uint8_t val = gyro_reg.read();
    val &= 0xE7; // clear bits
    uint8_t bits = static_cast<uint8_t>(range);
    bits <<= 3; // set up mask
    val |= bits;
    gyro_reg.write(val);
}

mpu_6050_fysnc_out MPU_6050::getFrameSynchronization(void) {
    Adafruit_BusIO_Register fsync_out_reg = Adafruit_BusIO_Register(i2c_device, MPU_6050_GENERAL_CONFIG);
    uint8_t val = fsync_out_reg.read();
    val &= 0x38; // clean bits
    val >>= 3;
    return (mpu_6050_fysnc_out)val;
}
void MPU_6050::setFrameSynchronization(mpu_6050_fysnc_out reference) {
    Adafruit_BusIO_Register fsync_out_reg = Adafruit_BusIO_Register(i2c_device, MPU_6050_GENERAL_CONFIG);
    uint8_t val = fsync_out_reg.read();
    val &= 0xC7; // clear bits
    uint8_t bits = static_cast<uint8_t>(reference);
    bits <<= 3; // set up mask
    val |= bits;
    fsync_out_reg.write(val);
}

mpu_6050_dlpf_bandwidth MPU_6050::getDLPFBandWidth(void) {
    Adafruit_BusIO_Register dlpf_bandwidth_reg = Adafruit_BusIO_Register(i2c_device, MPU_6050_GENERAL_CONFIG);
    uint8_t val = dlpf_bandwidth_reg.read();
    val &= 0x07; // clean bits
    return (mpu_6050_dlpf_bandwidth)val;
}
void MPU_6050::setDLPFBandWidth(mpu_6050_dlpf_bandwidth bandwidth) {
    Adafruit_BusIO_Register dlpf_bandwidth_reg = Adafruit_BusIO_Register(i2c_device, MPU_6050_GENERAL_CONFIG);
    uint8_t val = dlpf_bandwidth_reg.read();
    val &= 0xF8;
    uint8_t bits = static_cast<uint8_t>(bandwidth);
    val |= bits;
    dlpf_bandwidth_reg.write(val);
}

mpu_6050_clock_source MPU_6050::getClockSource(void) {
    Adafruit_BusIO_Register clock_soure_reg = Adafruit_BusIO_Register(i2c_device, MPU_6050_PRIMARY_POWER);
    uint8_t val = clock_soure_reg.read();
    val &= 0x07;
    return (mpu_6050_clock_source)val;
}
void MPU_6050::setClockSource(mpu_6050_clock_source source) {
    Adafruit_BusIO_Register clock_source_reg = Adafruit_BusIO_Register(i2c_device, MPU_6050_PRIMARY_POWER);
    uint8_t val = clock_source_reg.read();
    val &= 0xF8;
    uint8_t bits = static_cast<uint8_t>(source);
    val |= bits;
    clock_source_reg.write(val);
}

mpu_6050_cycle_rate MPU_6050::getCycleRate(void) {
    Adafruit_BusIO_Register cycle_rate_reg = Adafruit_BusIO_Register(i2c_device, MPU_6050_SECONDARY_POWER);
    uint8_t val = cycle_rate_reg.read();
    val &= 0xC0;
    return (mpu_6050_cycle_rate)val;
}
void MPU_6050::setCycleRate(mpu_6050_cycle_rate rate) {
    Adafruit_BusIO_Register cycle_rate_reg = Adafruit_BusIO_Register(i2c_device, MPU_6050_SECONDARY_POWER);
    uint8_t val = cycle_rate_reg.read();
    val &= 0x3F;
    uint8_t bits = static_cast<uint8_t>(rate);
    bits <<= 6;
    val |= bits;
    cycle_rate_reg.write(val);
}

uint8_t MPU_6050::getSampleRateDivider(void) {
    Adafruit_BusIO_Register sample_divider_reg = Adafruit_BusIO_Register(i2c_device, MPU_6050_SAMPLE_RATE_DIVIDER);
    uint8_t val = sample_divider_reg.read();
    return val;
}
void MPU_6050::setSampleRateDivider(uint8_t divider) {
    Adafruit_BusIO_Register sample_divider_reg = Adafruit_BusIO_Register(i2c_device, MPU_6050_SAMPLE_RATE_DIVIDER);
    sample_divider_reg.write(divider);
}

Adafruit_Sensor *MPU_6050::getTemperatureSensor(void) {
    return temp_sensor;
}
Adafruit_Sensor *MPU_6050::getAccelerometerSensor(void) {
    return acce_sensor;
}
Adafruit_Sensor *MPU_6050::getGyroscopeSensor(void) {
    return gyro_sensor;
}
