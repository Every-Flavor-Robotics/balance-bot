// Header file for the IMU class. The IMU handles reading the IMU data, calibration, and running an AHRS algorithm.

#ifndef IMU_H
#define IMU_H

// IMU includes
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <Preferences.h>
#include <Adafruit_AHRS.h>


// Open imu namespace
namespace Imu {

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
extern void serial_print_motioncal(sensors_event_t &accel_event, sensors_event_t &gyro_event, sensors_event_t &mag_event);
extern Adafruit_LSM6DSOX lsm6ds;
extern Adafruit_LIS3MDL lis3mdl;

class Imu {
public:
    Imu();

    // Init IMU sensor, with default settings. Use saved calibration if available
    void init();

    // Init IMU, with option to calibrate or not
    void init(bool shouldCalibrate);

    void loop();

    float getRoll();
    float getPitch();
    float getYaw();



private:
    // IMU sensor devices


    // Buffers for reading in magnetic calibration data
    float offsets[16];
    byte caldata[68]; // buffer to receive magnetic calibration data
    byte calcount=0;
    imu_calibration_data_t calibrationData;

    // Filter
    Adafruit_Madgwick filter;
    // Last filter update time
    unsigned long lastUpdate;


    // Event objects to store IMU data
    sensors_event_t accel_event, gyro_event, mag_event, temp;

    // Function for reading IMU data
    void read();

    // Function for performing IMU calibration
    // This will first do magnetic calibration using motioncal, followed by accelerometer and gyroscope calibration
    void calibrate();

    // Reads and parses calibration data sent over serial from MotionCal
    bool receiveCalibration();

    // Apply calibration and change units
    void applyCalibration();

};
} // namespace imu


#endif // IMU_H