// Header file for the DriveBase class. The drive base handles controlling both left and right motors.

#ifndef DRIVE_BASE_H
#define DRIVE_BASE_H

#include <SimpleFOC.h>

// Encoder libs
#include <sensors/MagneticSensorI2C.h>
#include "mt6701_sensor.h"
#include "encoders/calibrated/CalibratedSensor.h"
#include <Wire.h>

// TODO: These are global because the SimpleFOC commander API doesn't support lambdas to save state in for the callback
// Need to decide if this is the best solution, or if there is something better we can do
extern Commander command;
extern BLDCMotor motor_left;
extern BLDCMotor motor_right;


class DriveBase {
public:
    DriveBase();

    // Init motors and encoders, calibration is automatically loaded and FOCStudio is disabled
    void init();

    // Init motors and encoders, optionally calibrating and/or enabling FOCStudio
    void init(bool shouldCalibrate, bool enableFocStudio);

    void loop();

    // void doTargetLeft(char* cmd);
    // void doTargetRight(char* cmd);


private:
    // Left Motor and Encoder pins
    const int k_left_enc_sda = 15;
    const int k_left_enc_scl = 32;
    const int k_left_enc_cs  = 14;
    const int k_left_gpio_uh = 18;
    const int k_left_gpio_ul =  5;
    const int k_left_gpio_vh = 17;
    const int k_left_gpio_vl = 16;
    const int k_left_gpio_wh =  4;
    const int k_left_gpio_wl =  0;

    // Right Motor and Encoder pins
    const int k_right_enc_sda = 15;
    const int k_right_enc_scl = 32;
    const int k_right_enc_cs  = 12;
    const int k_right_gpio_uh = 23;
    const int k_right_gpio_ul = 22;
    const int k_right_gpio_vh = 33;
    const int k_right_gpio_vl = 25;
    const int k_right_gpio_wh = 21;
    const int k_right_gpio_wl = 19;

    // Motor and Encoder parameters
    const float k_voltage_power_supply = 9.0;
    const float k_voltage_limit = 9.0;
    const float k_current_limit = 1.0;
    const float k_velocity_limit = 100.0;
    const float k_voltage_calibration = 2.0;

    bool shouldCalibrate;
    bool enableFocStudio;

    // Encoder, motor, and driver instances
    MT6701Sensor encoder_left;
    CalibratedSensor sensor_calibrated_left;
    BLDCDriver6PWM driver_left;

    MT6701Sensor encoder_right;
    CalibratedSensor sensor_calibrated_right;
    BLDCDriver6PWM driver_right;



    void initHelper(BLDCMotor& motor, BLDCDriver6PWM& driver, CalibratedSensor& sensor_calibrated, MT6701Sensor& encoder, const char* name);
};



#endif // DRIVE_BASE_H