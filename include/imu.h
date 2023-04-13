// Header file for the IMU class. The IMU handles reading the IMU data, calibration, and running an AHRS algorithm.

#ifndef IMU_H
#define IMU_H

// IMU includes
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <Preferences.h>

// Type for saving and loading calibration parameters to/from EEPROM
const int IMU_CALIBRATION_DATA_LEN = (9+3+3+3) * sizeof(float);
typedef union {
    struct __attribute__((packed)){
        float zero_electric_angle;

        // Eccentricity LUT
        float mag_soft_iron[9];
        float mag_hard_iron[3];
        float gyro_bias[3];
        float accel_bias[3];
    };

    uint8_t raw[IMU_CALIBRATION_DATA_LEN];
} imu_calibration_data_t;


// Helper function for calculating the CRC16 checksum
extern uint16_t crc16_update(uint16_t crc, uint8_t a);
extern void serial_print_motioncal(sensor_event_t &accel_event, sensor_event_t &gyro_event, sensor_event_t &mag_event);

class Imu {
public:
    Imu();

    // Init IMU sensor, with default settings. Use saved calibration if available
    void init();

    // Init IMU, with option to calibrate or not
    void init(bool shouldCalibrate);

    void loop();


private:
    // IMU sensor devices
    Adafruit_LSM6DSOX lsm6ds;
    Adafruit_LIS3MDL lis3mdl;

    // Buffers for reading in magnetic calibration data
    float offsets[16];
    byte caldata[68]; // buffer to receive magnetic calibration data
    byte calcount=0;

    // Event objects to store IMU data
    sensors_event_t accel_event, gyro_event, mag_event, temp;

    // Function for reading IMU data
    void read();

    // Function for performing IMU calibration
    // This will first do magnetic calibration using motioncal, followed by accelerometer and gyroscope calibration
    void calibrate();

    // Reads and parses calibration data sent over serial from MotionCal
    bool receiveCalibration();




};



#endif // DRIVE_BASE_H